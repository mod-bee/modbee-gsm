// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#include "GsmAtEngine.h"

#include <new>
#include <type_traits>
#include "GsmParse.h"
#include "GsmNoExcept.h"

namespace {

bool startsWith(const char* line, size_t length, const char* prefix) {
    if (prefix == nullptr || prefix[0] == '\0') { return false; }
    for (size_t i = 0; prefix[i] != '\0'; i++) {
        if (i >= length || line[i] != prefix[i]) { return false; }
    }
    return true;
}

bool equals(const char* line, size_t length, const char* other) {
    size_t i = 0;
    for (; i < length; i++) {
        if (other[i] == '\0' || line[i] != other[i]) { return false; }
    }
    return other[i] == '\0';
}

size_t copyBounded(char* destination, size_t capacity, const char* source) {
    if (source == nullptr) { destination[0] = '\0'; return 0; }
    size_t i = 0;
    for (; i + 1 < capacity && source[i] != '\0'; i++) { destination[i] = source[i]; }
    destination[i] = '\0';
    return i;
}

} // namespace

GsmAtEngine::GsmAtEngine() {
    _active.deferredPrefix[0] = '\0';
}

GsmAtEngine::~GsmAtEngine() {
    release();
}

void GsmAtEngine::release() {
    gsm_freeLarge(_slots);
    gsm_freeLarge(_text);
    _slots = nullptr;
    _text = nullptr;
    _longText = nullptr;
    _longInUse = false;
    _allocated = 0;
    _active.text = nullptr;
    _active.own = nullptr;
}

bool GsmAtEngine::reserve() {
    if (_slots != nullptr && _allocated == _capacity) { return true; }
    // A new size waits until nothing is queued or on the wire: nothing in
    // flight is ever moved.
    if (_slots != nullptr && (_queued != 0 || _state != State::Idle)) { return true; }
    release();

    static_assert(std::is_trivially_destructible<Slot>::value,
                  "the slots are released without running destructors");
    size_t slotBytes = 0;
    size_t textBytes = 0;
    if (!gsm_multiply(_capacity, sizeof(Slot), slotBytes)) { return false; }
    if (!gsm_multiply(_capacity + 1, GSM_AT_COMMAND_MAX, textBytes)) { return false; }
    if (textBytes > (size_t)-1 - GSM_AT_COMMAND_LONG_MAX) { return false; }
    textBytes += GSM_AT_COMMAND_LONG_MAX;

    void* slots = gsm_allocLarge(slotBytes);
    char* text = (char*)gsm_allocLarge(textBytes);
    if (slots == nullptr || text == nullptr) {
        gsm_freeLarge(slots);
        gsm_freeLarge(text);
        return false;
    }
    _slots = (Slot*)slots;
    _text = text;
    for (size_t i = 0; i < _capacity; i++) {
        new (&_slots[i]) Slot();
        _slots[i].own = _slots[i].text = text + i * GSM_AT_COMMAND_MAX;
        _slots[i].text[0] = '\0';
        _slots[i].deferredPrefix[0] = '\0';
    }
    _active.own = _active.text = text + _capacity * GSM_AT_COMMAND_MAX;
    _active.text[0] = '\0';
    _longText = text + (_capacity + 1) * GSM_AT_COMMAND_MAX;
    _longText[0] = '\0';
    _longInUse = false;
    _allocated = _capacity;
    return true;
}

// The long buffer goes back when the command holding it is done with.
void GsmAtEngine::releaseText(Slot& slot) {
    if (slot.text == _longText && _longText != nullptr) { _longInUse = false; }
    slot.text = slot.own;
    if (slot.text != nullptr) { slot.text[0] = '\0'; }
    slot.textLength = 0;
}

bool GsmAtEngine::begin(GsmTransport* transport) {
    end();
    if (!reserve()) { return false; }
    _transport = transport;
    if (_transport == nullptr) { return false; }
    if (!_transport->begin()) { _transport = nullptr; return false; }

    _parser.reset();
    _state = State::Idle;
    _cancelPrompt = false;
    _settling = false;
    _counters = Counters();
    return true;
}

// Retires every queued command accepted before `boundary`, oldest first.
//
// BY ID, NOT BY SLOT. A handler run from here may submit, and its command
// lands in whichever slot is free - ahead of the loop or behind it. Walking the
// slots retired the ones that landed behind and missed the ones that landed
// ahead, so the same act concluded one command at once and left another in the
// queue uncounted. The id says when a command was accepted, whatever slot it
// is in, so only those that were outstanding when this started are touched.
//
// Oldest first, too, so callers hear about their commands in the order they
// made them - the same promise the queue makes when it is running.
void GsmAtEngine::retireOlderThan(uint32_t boundary, GsmResult result) {
    // One retirement per pass, and never more passes than there are slots:
    // a handler can add commands, but never one older than the boundary.
    for (size_t pass = 0; pass < _allocated; pass++) {
        size_t oldest = _allocated;
        uint32_t oldestAge = 0;
        for (size_t i = 0; i < _allocated; i++) {
            if (!_slots[i].inUse) { continue; }
            // How long before the boundary this was accepted. Zero, or a
            // wrapped "negative", is a command accepted since - leave it.
            const uint32_t age = (uint32_t)(boundary - _slots[i].opId);
            if (age == 0 || age >= 0x80000000u) { continue; }
            if (oldest == _allocated || age > oldestAge) {
                oldest = i;
                oldestAge = age;
            }
        }
        if (oldest == _allocated) { return; }
        retire(oldest, result);
    }
}

void GsmAtEngine::abandonAll(GsmStatus status) {
    // Whatever was accepted from here on is for the module that comes back.
    const uint32_t boundary = _nextOpId;

    // The link is put back to a clean state BEFORE any handler runs, not
    // after. A handler may queue the first command for the new instance; a
    // reset afterwards would be resetting underneath it. And a cancel owed to
    // a prompt is owed to a module that has since restarted, so it goes too.
    _parser.reset();
    _cancelPrompt = false;
    _settling = false;

    if (_state != State::Idle) { conclude(status); }
    retireOlderThan(boundary, status);
}

void GsmAtEngine::end() {
    // THE LINK GOES FIRST, so that a handler submitting from inside one of the
    // conclusions below is refused rather than accepted. Accepted, its command
    // would sit in a slot of an engine that is no longer sending anything -
    // concluded only if somebody happened to call begin() again.
    GsmTransport* const transport = _transport;
    const uint32_t boundary = _nextOpId;
    _transport = nullptr;
    _parser.reset();
    _cancelPrompt = false;
    _settling = false;

    // EVERYTHING OUTSTANDING IS CONCLUDED, including what is on the wire.
    // Anything less leaves a caller waiting forever for a link that has gone.
    if (_state != State::Idle) {
        conclude(GsmStatus::Cancelled);
    }
    retireOlderThan(boundary, GsmStatus::Cancelled);

    if (transport != nullptr) { transport->end(); }
    _state = State::Idle;
}

// =============================================================================
// SUBMIT
// =============================================================================

GsmResult GsmAtEngine::submit(const GsmCommand& command) {
    if (_transport == nullptr) {
        _counters.refused++;
        return GsmStatus::NotInitialised;
    }
    if (command.text == nullptr || command.text[0] == '\0') {
        _counters.refused++;
        return GsmStatus::InvalidArgument;
    }

    // A payload that will not fit in the slot's declaration is refused here
    // rather than half-sent later. See the header: entering prompt mode
    // without being able to finish wedges the AT channel.
    if (command.payloadLength != 0 && command.payload == nullptr) {
        _counters.refused++;
        return GsmStatus::InvalidArgument;
    }

    // A command that does not fit is refused rather than truncated. A truncated
    // AT command is not a shorter command - it is a different one, and the
    // modem's answer to it would be matched against the wrong expectation.
    const char* source = command.text;
    size_t sourceLength = 0;
    while (source[sourceLength] != '\0') { sourceLength++; }
    if (sourceLength >= GSM_AT_COMMAND_LONG_MAX) {
        _counters.refused++;
        return GsmStatus::TooLarge;
    }

    // AND REFUSED IF THE MODULE ITSELF COULD NOT HOLD IT. The two families
    // differ by more than five times - 3071 characters against 559 - so a
    // command that is comfortable on one is over the line buffer on the other,
    // where the module truncates it and answers a DIFFERENT command.
    //
    // Zero means the driver has not declared a limit, and then only our own
    // buffer bounds it.
    if (_atLineLimit != 0 && sourceLength > _atLineLimit) {
        _counters.refused++;
        return GsmStatus::TooLarge;
    }

    // --- Supersede ---
    //
    // A later command with the same key REPLACES an earlier one that has not
    // yet been sent, and the earlier caller is told. Polling signal strength
    // every scan then sends one query rather than a backlog of identical ones.
    //
    // Only queued commands, never the one in flight: that one has already
    // reached the modem, and rewriting it would mean the answer coming back
    // belongs to a command nobody asked for.
    //
    // FOUND HERE, RETIRED ONLY ONCE THIS COMMAND IS CERTAINLY ACCEPTED. The
    // replacement used to happen first, before the checks that can refuse -
    // so a replacement that was then TooLarge or QueueFull had already retired
    // the command it was meant to replace. The earlier caller heard
    // "Superseded", the later one heard "refused", and nothing went to the
    // modem at all: a signal poll that quietly stopped polling.
    size_t victim = _allocated;
    if (command.supersedeKey != 0) {
        for (size_t i = 0; i < _allocated; i++) {
            if (_slots[i].inUse && _slots[i].supersedeKey == command.supersedeKey) {
                victim = i;
                break;
            }
        }
    }

    // A slot: a free one, or failing that the one being replaced - a full
    // queue still takes a replacement, because the queue does not grow.
    size_t free = _allocated;
    for (size_t i = 0; i < _allocated; i++) {
        if (!_slots[i].inUse) { free = i; break; }
    }
    if (free == _allocated && victim == _allocated) {
        _counters.refused++;
        return GsmStatus::QueueFull;
    }

    // Longer than a slot holds: the one long buffer, when it is free - or
    // when the command being replaced is what holds it. Checked last, after
    // every other refusal, so a refused command never holds it.
    const bool needsLong = sourceLength >= GSM_AT_COMMAND_MAX;
    if (needsLong && _longInUse &&
        !(victim != _allocated && _slots[victim].text == _longText)) {
        _counters.refused++;
        return GsmStatus::QueueFull;
    }

    // --- Accepted. Nothing below refuses. ---
    //
    // The replaced command leaves the queue now, but its handler runs LAST,
    // once this command is in place: a handler that submits must find the
    // queue as it will be, not a half-filled slot.
    GsmDoneFn victimDone = nullptr;
    void* victimContext = nullptr;
    const bool superseding = victim != _allocated;
    if (superseding) {
        withdraw(victim, victimDone, victimContext);
        _counters.superseded++;
        if (free == _allocated) { free = victim; }
    }

    Slot& slot = _slots[free];

    if (needsLong) {
        _longInUse = true;
        slot.text = _longText;
        slot.textLength = copyBounded(slot.text, GSM_AT_COMMAND_LONG_MAX, source);
    } else {
        slot.text = slot.own;
        slot.textLength = copyBounded(slot.text, GSM_AT_COMMAND_MAX, source);
    }

    copyBounded(slot.deferredPrefix, sizeof slot.deferredPrefix, command.deferredPrefix);

    slot.inlinePayloadPrefix = command.inlinePayloadPrefix;
    slot.payload = command.payload;
    slot.payloadLength = command.payloadLength;
    // FLOORED BY HOW OFTEN WE ARE ACTUALLY LOOKED AT.
    //
    // Three service intervals, so a reply landing one poll before the deadline
    // is still seen. An application polling every 100 ms cannot enforce a 50 ms
    // timeout, and pretending otherwise turns its own scheduling into modem
    // faults. See GsmModem::serviceIntervalUs().
    slot.timeoutMs = command.timeoutMs;
    {
        const uint32_t floorMs = (uint32_t)(((_serviceUs + 999) / 1000) * 3);
        if (slot.timeoutMs < floorMs) { slot.timeoutMs = floorMs; }
    }
    slot.deferredTimeoutMs = command.deferredTimeoutMs;
    slot.supersedeKey = command.supersedeKey;
    slot.onReply = command.onReply;
    slot.onDone = command.onDone;
    slot.context = command.context;
    slot.opId = _nextOpId++;
    if (_nextOpId == 0) { _nextOpId = 1; }    // never zero; zero means "unused"
    slot.inUse = true;

    _queued++;
    _counters.submitted++;

    if (superseding && victimDone != nullptr) {
        victimDone(victimContext, GsmStatus::Superseded);
    }
    return GsmStatus::Queued;
}

// =============================================================================
// POLL
// =============================================================================

void GsmAtEngine::poll() {
    if (_transport == nullptr) { return; }

    serviceTransmit();
    servicePoll();

    // A handler run from servicePoll() may have called end() - "send the
    // alarm, then shut down" written in a completion callback. The transport
    // is gone, and nothing below may touch it.
    if (_transport == nullptr) { return; }

    // Timeouts. Checked after reading, so a reply that arrived in the same
    // poll as its deadline is honoured rather than raced.
    //
    // IN EVERY STATE, including the two where we are still writing. Those
    // were once exempt, on the theory that a write always finishes - and a
    // link that refuses every write (CTS held off, a buffer that never drains)
    // left the command half-sent for ever, with the whole queue behind it.
    if (_state != State::Idle && !gsm_isFuture(gsm_millis(), _deadline)) {
        _counters.timeouts++;

        // THE MODULE MAY BE SITTING AT ITS PROMPT. A prompt that comes late,
        // or a payload that stopped part-way, leaves it waiting for data - and
        // the next command would be taken as that data. So it is cancelled
        // before anything else is sent. See serviceTransmit().
        //
        // A command abandoned while its TEXT was still going out is left
        // alone: a CR now would run whatever prefix of it had been written,
        // which is a different command. The module discards the fragment
        // when the next one arrives and answers that with ERROR - one wasted
        // command, against running a truncated one.
        if (_state == State::AwaitingPrompt || _state == State::SendingPayload) {
            _cancelPrompt = true;
        }

        // The parser may be mid-payload on a command that is now over. Its
        // bytes are never coming; leaving it waiting would eat the next
        // reply as payload and the link would never recover.
        _parser.abandon();
        conclude(GsmStatus::Timeout);
    }

    if (_state == State::Idle) { startNext(); }

    // Starting a command writes its first bytes in the same poll, so the
    // modem sees it without waiting for the next one.
    serviceTransmit();
}

void GsmAtEngine::servicePoll() {
    uint8_t buffer[64];
    size_t budget = _maxBytesPerPoll;

    while (budget > 0) {
        const size_t want = budget < sizeof buffer ? budget : sizeof buffer;
        const size_t taken = _transport->read(buffer, want);
        if (taken == 0) { break; }

        budget -= taken;
        _counters.bytesReceived += (uint32_t)taken;

        size_t offset = 0;
        while (offset < taken) {
            size_t used = 0;
            GsmAtParser::Event event;
            const bool got = _parser.consume(buffer + offset, taken - offset, used, event);
            offset += used;
            if (!got) { break; }

            switch (event.kind) {
                case GsmAtEventKind::Line:
                    handleLine(event.text(), event.length, event.truncated);
                    break;

                case GsmAtEventKind::Prompt:
                    if (_state == State::AwaitingPrompt) {
                        _state = State::SendingPayload;
                        _sentCursor = 0;
                    }
                    break;

                case GsmAtEventKind::PayloadChunk:
                case GsmAtEventKind::PayloadComplete:
                    // Payloads coming FROM the modem belong to a socket read,
                    // which drives the parser itself. The engine carries them
                    // to the reply handler untouched, as bytes rather than as
                    // a line - it never inspects what a socket is carrying.
                    if (_active.onReply != nullptr &&
                        event.kind == GsmAtEventKind::PayloadChunk) {
                        _active.onReply(_active.context, (const char*)event.data,
                                        event.length);
                    }
                    break;

                case GsmAtEventKind::None:
                    break;
            }

            // A handler ended the engine. The rest of this buffer belongs to a
            // session that is over, and the transport it came from is gone.
            if (_transport == nullptr) { return; }
        }
    }
}

void GsmAtEngine::serviceTransmit() {
    // THE CANCEL GOES BEFORE ANYTHING ELSE. A prompt is ended by Ctrl-Z or
    // abandoned by ESC - "<CTRL-Z/ESC>" [A76XX §9.2.13 AT+CMGS]. Sent as soon
    // as the link will take it, and no command text is written until it has
    // gone and the quiet spell after it has passed - so a command is never
    // written into a prompt that may still be open.
    //
    // THE NEXT COMMAND STARTS ANYWAY, and waits here inside its own deadline.
    // Holding startNext() back instead meant that a link refusing writes kept
    // the cancel owed for ever, and every command queued behind it waited
    // for ever with no deadline running - the very wedge the timeouts above
    // exist to prevent. This way each of them times out, honestly.
    //
    // BEST EFFORT, AND SAID SO. A send that declared its length counts bytes
    // rather than watching for ESC, and may take this one as data; it then
    // waits for the rest, and the command after it is what times out. That is
    // still a link that recovers, which a command written into the prompt is
    // not.
    const bool lineClear = _state == State::Idle ||
                           (_state == State::SendingCommand && _sentCursor == 0);
    if (_cancelPrompt && lineClear) {
        const uint8_t escape = 0x1B;
        if (_transport->write(&escape, 1) == 0) { return; }    // next poll
        _counters.bytesSent++;
        _cancelPrompt = false;
        _settling = true;
        _settleUntil = (gsm_time_t)(gsm_millis() + GSM_PROMPT_CANCEL_SETTLE_MS);
    }

    if (_state == State::SendingCommand && _sentCursor == 0) {
        if (_cancelPrompt) { return; }
        if (_settling) {
            if (gsm_isFuture(gsm_millis(), _settleUntil)) { return; }
            _settling = false;
        }
    }

    if (_state == State::SendingCommand) {
        // The text, then a CR. Built as one write where it fits so a modem
        // never sees a command line arrive in pieces with a gap in the middle.
        while (_sentCursor < _active.textLength + 1) {
            const bool atTerminator = (_sentCursor == _active.textLength);
            const uint8_t byte = atTerminator ? (uint8_t)'\r'
                                              : (uint8_t)_active.text[_sentCursor];
            const size_t written = _transport->write(&byte, 1);
            if (written == 0) { return; }        // no room; resume next poll
            _sentCursor++;
            _counters.bytesSent++;
        }

        _sentCursor = 0;
        if (_active.payloadLength != 0) {
            _state = State::AwaitingPrompt;
            _parser.expectPrompt(_promptCharacter);
        } else {
            _state = State::AwaitingResult;
        }
        return;
    }

    if (_state == State::SendingPayload) {
        while (_sentCursor < _active.payloadLength) {
            const size_t written = _transport->write(_active.payload + _sentCursor,
                                                     _active.payloadLength - _sentCursor);
            if (written == 0) { return; }        // no room; resume next poll
            _sentCursor += written;
            _counters.bytesSent += (uint32_t)written;
        }
        _state = State::AwaitingResult;
    }
}

// =============================================================================
// LINES
// =============================================================================

void GsmAtEngine::handleLine(const char* line, size_t length, bool truncated) {
    // THE ORDERING THAT MATTERS, AND IT IS ONE LINE OF CODE.
    //
    // URCs first, always. A URC can land in the middle of a reply, and an
    // engine that matched the in-flight command first would read
    // "+CMTI: \"SM\",3" as the answer to a signal query - reporting nonsense
    // and losing the SMS in the same stroke.
    if (_router != nullptr && _router->route(line, length)) { return; }

    // An empty line is framing, not content. Modems emit them liberally.
    if (length == 0) { return; }

    // A truncated line cannot be matched against anything, because what is
    // missing may be exactly what would have distinguished it. Passed to the
    // reply handler so nothing is lost, never treated as a final result.
    if (truncated) {
        if (_active.onReply != nullptr) { _active.onReply(_active.context, line, length); }
        return;
    }

    if (_state == State::AwaitingDeferred) {
        if (startsWith(line, length, _active.deferredPrefix)) {
            if (_active.onReply != nullptr) { _active.onReply(_active.context, line, length); }
            conclude(GsmStatus::Ok);
            return;
        }
        // Anything else while waiting for a deferred answer is informational.
        if (_active.onReply != nullptr) { _active.onReply(_active.context, line, length); }
        return;
    }

    // A RESULT CAN ARRIVE WHILE THE PAYLOAD IS STILL GOING OUT. The module
    // gave up on the send part-way - and said so. Treated as an orphan, that
    // answer was thrown away and the command reported a Timeout minutes later
    // instead of the reason it was given.
    if (_state != State::AwaitingResult && _state != State::AwaitingPrompt &&
        _state != State::SendingPayload) {
        // NOTHING IS IN FLIGHT AND NO HANDLER WANTED IT. This is the one
        // place a line is genuinely orphaned, so this is where it is counted.
        //
        // A final result arriving here is ordinary and not worth reporting:
        // a command that already timed out may still have its OK on the way,
        // and counting those would bury the lines that matter.
        GsmResult ignored = GsmStatus::Ok;
        if (_router != nullptr && !matchFinalResult(line, length, ignored)) {
            _router->noteUnhandled(line, length);
        }
        return;
    }

    GsmResult final = GsmStatus::Ok;
    if (matchFinalResult(line, length, final)) {
        // THE DEFERRED CASE. A successful first answer is not the conclusion
        // when a second one is coming - AT+CIPOPEN returns OK and then, up to
        // two minutes later, the result that actually says whether the socket
        // opened.
        if (final.isOk() && _active.deferredPrefix[0] != '\0') {
            _state = State::AwaitingDeferred;
            const uint32_t window = _active.deferredTimeoutMs != 0
                                  ? _active.deferredTimeoutMs : _active.timeoutMs;
            _deadline = (gsm_time_t)(gsm_millis() + window);
            return;
        }
        conclude(final);
        return;
    }

    // An ordinary reply line: the content the caller asked for.
    if (_active.onReply != nullptr) { _active.onReply(_active.context, line, length); }
}

bool GsmAtEngine::matchFinalResult(const char* line, size_t length, GsmResult& out) {
    // The universal set, from V.25ter and 3GPP TS 27.007. These mean the same
    // on every modem ever made, so the engine may know them - a vendor's own
    // extras go through the hook below, which is the driver's business.
    if (equals(line, length, "OK")) { out = GsmStatus::Ok; return true; }
    if (equals(line, length, "ERROR")) { out = GsmStatus::Error; return true; }

    if (startsWith(line, length, "+CME ERROR")) {
        GsmField rest;
        uint32_t code = 0;
        if (GsmParse::afterPrefix(line, length, "+CME ERROR", rest) &&
            GsmParse::toUnsigned(GsmParse::trim(rest), code)) {
            out = GsmResult::fromModule(GsmErrorSpace::Cme, (uint16_t)code);
        } else {
            // AT+CMEE can be set to report these as text rather than a number.
            // We pin it numeric during bring-up for exactly this reason, but a
            // modem that was left in verbose mode must still conclude the
            // command rather than hang it.
            out = GsmStatus::Error;
        }
        return true;
    }

    if (startsWith(line, length, "+CMS ERROR")) {
        GsmField rest;
        uint32_t code = 0;
        if (GsmParse::afterPrefix(line, length, "+CMS ERROR", rest) &&
            GsmParse::toUnsigned(GsmParse::trim(rest), code)) {
            out = GsmResult::fromModule(GsmErrorSpace::Cms, (uint16_t)code);
        } else {
            out = GsmStatus::Error;
        }
        return true;
    }

    // Whatever the module invents on top. The driver owns that table.
    if (_finalHook != nullptr &&
        _finalHook(_finalHookContext, line, length, out)) {
        return true;
    }

    return false;
}

// =============================================================================
// CONCLUSIONS
// =============================================================================

void GsmAtEngine::conclude(GsmResult result) {
    if (_state == State::Idle) { return; }

    GsmDoneFn done = _active.onDone;
    void* context = _active.context;

    // THE RECORD IS CLEARED BEFORE THE HANDLER RUNS.
    //
    // A completion handler is application code, and the natural thing for one
    // to do is queue the next command. If the active record were still live
    // when that happened, a handler that concluded something would re-enter
    // this function against a record already being retired, and the command
    // would conclude twice. Clearing first makes the depth bounded however the
    // application behaves.
    _state = State::Idle;
    _sentCursor = 0;
    _active.onDone = nullptr;
    _active.onReply = nullptr;
    _active.payload = nullptr;
    _active.payloadLength = 0;
    _active.deferredPrefix[0] = '\0';
    _active.inUse = false;
    releaseText(_active);

    // A prompt that is no longer wanted is not waited for. A command refused
    // instead of prompted - ERROR where "> " was expected - left the parser
    // armed, and with nothing queued behind it to reset it, the next "> " at
    // the start of any line was swallowed as a prompt nobody asked for.
    if (_parser.mode() == GsmAtMode::Prompt) { _parser.abandon(); }

    _counters.concluded++;

    if (done != nullptr) { done(context, result); }
}

void GsmAtEngine::retire(size_t index, GsmResult result) {
    GsmDoneFn done = nullptr;
    void* context = nullptr;
    if (!withdraw(index, done, context)) { return; }
    if (done != nullptr) { done(context, result); }
}

// Takes a queued command out of the queue and counts it concluded, handing
// back the handler that must now be told - exactly once, by the caller.
bool GsmAtEngine::withdraw(size_t index, GsmDoneFn& done, void*& context) {
    Slot& slot = _slots[index];
    if (!slot.inUse) { return false; }

    done = slot.onDone;
    context = slot.context;

    // Removed from the queue BEFORE its handler runs, for the same reason
    // conclude() clears first: the handler is allowed to submit, and a submit
    // must not find this slot still occupied or still eligible to supersede.
    slot.inUse = false;
    slot.onDone = nullptr;
    slot.onReply = nullptr;
    slot.payload = nullptr;
    slot.payloadLength = 0;
    slot.supersedeKey = 0;
    releaseText(slot);
    if (_queued > 0) { _queued--; }

    _counters.concluded++;
    return true;
}

void GsmAtEngine::startNext() {
    if (_state != State::Idle || _queued == 0) { return; }

    // Oldest first. What is accepted first leaves first - anything that
    // carries its own ordering depends on it, and a modem's own state machine
    // certainly does.
    size_t oldest = _allocated;
    uint32_t oldestId = 0;
    for (size_t i = 0; i < _allocated; i++) {
        if (!_slots[i].inUse) { continue; }
        // Wrapping comparison, like the clock: after 2^32 submits a plain `<`
        // would put opId 1 ahead of 0xFFFFFFFF and run the newer one first.
        // Queued ids are never 2^31 apart, so the top bit of the difference
        // is the order.
        if (oldest == _allocated ||
            (uint32_t)(_slots[i].opId - oldestId) >= 0x80000000u) {
            oldest = i;
            oldestId = _slots[i].opId;
        }
    }
    if (oldest == _allocated) { _queued = 0; return; }

    // The command takes its text with it - the slot's buffer, or the long
    // one - and the slot takes the buffer the last command on the wire had.
    // So a handler queueing more work can never overwrite what is being sent.
    char* spare = _active.own;
    _active = _slots[oldest];
    _slots[oldest].own = spare;
    _slots[oldest].text = spare;
    _slots[oldest].textLength = 0;
    if (spare != nullptr) { spare[0] = '\0'; }
    _slots[oldest].inUse = false;
    _slots[oldest].onDone = nullptr;
    _slots[oldest].payload = nullptr;
    if (_queued > 0) { _queued--; }

    _state = State::SendingCommand;
    _sentCursor = 0;
    _deadline = (gsm_time_t)(gsm_millis() + _active.timeoutMs);

    // The parser is put back into line mode for the new command. Whatever the
    // last one left behind - a half-read payload after a timeout - is not this
    // command's business.
    _parser.abandon();

    // ARMED AFTER the reset, not before. abandon() clears the arming, so a
    // caller that armed the parser at submit time would find it quietly undone
    // by the time its command actually ran.
    if (_active.inlinePayloadPrefix != nullptr) {
        _parser.expectInlinePayload(_active.inlinePayloadPrefix);
    }
}
