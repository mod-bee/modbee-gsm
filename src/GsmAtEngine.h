// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#pragma once

// The command queue. EVERY ACCEPTED COMMAND REACHES EXACTLY ONE CONCLUSION -
// success, refusal, timeout, superseded or queue-full. Never silently dropped.
//
// ONE COMMAND IN FLIGHT, because both manuals require it: "you need to wait
// the final response ... of last AT Command you entered before you enter the
// next AT Command" [A76XX 1.5.2 item 5, SIM707x 1.4.5]. Two at once produce
// interleaved replies no parser can untangle.
//
//   Stacked      nothing blocks; nothing is sent inside the submitting call.
//   Ordered      accepted first, leaves first.
//   Superseded   a queued, unsent command is REPLACED and its caller told.
//   Concluded    exactly once.
//
// TWO-STAGE CONCLUSIONS. AT+CIPOPEN answers "OK" and then, up to 120 s later,
// "+CIPOPEN: <link>,<err>" with the real outcome; AT+NETOPEN likewise. OK is
// not the conclusion, so a command may declare a DEFERRED PREFIX: it concludes
// on that line, under its own deadline.
//
// AND IT HOLDS THE QUEUE WHILE IT WAITS. This used to claim otherwise, and the
// code never did it. Letting the next command go while "+CIPOPEN:" is still
// owed would need the engine to tell two outstanding answers with the same
// prefix apart - two sockets opening at once both wait for "+CIPOPEN:" - and
// that is the driver's knowledge, not the engine's. A wrong guess concludes the
// wrong socket. So the wait is honest and serial: a slow open delays the
// commands behind it, and nothing is ever matched to the wrong caller.

#include "GsmPlatform.h"
#include "GsmResult.h"
#include "GsmTransport.h"
#include "GsmAtParser.h"
#include "GsmUrcRouter.h"

// Commands held at once, BY DEFAULT. Past it a command is REFUSED with
// QueueFull rather than dropped - a caller told "accepted" that gets nothing is
// the failure this whole file exists to prevent. setCapacity() changes it
// before begin(); GsmDevice::setCapacity() and GSM_MODEM's AT_QUEUE reach it.
#ifndef GSM_QUEUE_CAPACITY
#define GSM_QUEUE_CAPACITY 12
#endif

// The room each queued command has.
//
// Modules allow far more - A76XX 3071 characters, SIM707x 559 - but almost
// nothing here approaches it. A socket open carrying a full hostname is under
// 120. 192 leaves room and keeps the queue at a size a board can hold.
#ifndef GSM_AT_COMMAND_MAX
#define GSM_AT_COMMAND_MAX 192
#endif

// The longest command this library builds, and the one buffer that holds it.
//
// The SIM707x bearer set-up carries the APN, user name and password in one
// line (AT+CNCFG): 3GPP allows a 100-octet APN [TS 23.003 9.1] and the module
// takes 127 characters each of user name and password [SIM7070 AT 7.2.2,
// "+CNCFG: ...,150,127,127"] - about 380 characters. Rather than make every
// slot that long, a command past GSM_AT_COMMAND_MAX takes this ONE buffer;
// a second long command while it is held is refused with QueueFull and is
// accepted once the first has gone. Nothing longer is ever built.
#ifndef GSM_AT_COMMAND_LONG_MAX
#define GSM_AT_COMMAND_LONG_MAX 384
#endif

// The longest deferred-result prefix, e.g. "+CIPOPEN:".
#ifndef GSM_DEFERRED_PREFIX_MAX
#define GSM_DEFERRED_PREFIX_MAX 20
#endif

// How long the link is left quiet after an ESC has cancelled an abandoned
// prompt, before the next command goes.
//
// A module may answer the cancellation, and the manuals do not agree on how.
// Sent straight after the ESC, the next command would be concluded by that
// answer - an "OK" credited to a query that never ran. A tenth of a second is
// far longer than a module takes to answer at any baud rate it runs at, and
// this only happens after a send has already failed.
#ifndef GSM_PROMPT_CANCEL_SETTLE_MS
#define GSM_PROMPT_CANCEL_SETTLE_MS 100
#endif

// How much the engine drains from the link in one poll(), by default.
//
// Bounded so that poll() does a predictable amount of work whatever the modem
// is doing. A link that keeps talking is serviced again on the next poll, and
// because the driver's receive buffer fills in the background nothing is lost
// by not reading it all at once.
//
// This is the DEFAULT; setMaxBytesPerPoll() changes it at run time, which is
// what an integrator with a tight scan cycle needs and what a compile-time
// constant cannot give them.
#ifndef GSM_POLL_BYTE_BUDGET
#define GSM_POLL_BYTE_BUDGET 256
#endif

// Called for each reply line that is neither a URC nor the final result.
typedef void (*GsmReplyFn)(void* context, const char* line, size_t length);

// Called exactly once, when the command concludes.
//
// A plain function pointer and a context, not a std::function. This runs on
// the poll path, it must not allocate, and generated IEC 61131 function-block
// code cannot register a lambda at all - the context pointer is how a C++
// caller gets its own object back without one.
typedef void (*GsmDoneFn)(void* context, GsmResult result);

// Lets a driver recognise a final result its module invents, on top of the
// universal OK / ERROR / +CME ERROR / +CMS ERROR that the engine knows. Return
// true when the line concludes the command.
typedef bool (*GsmFinalResultFn)(void* context, const char* line, size_t length,
                                 GsmResult& out);

// What a caller submits.
struct GsmCommand {
    // The command text, WITHOUT the trailing CR. Copied into the queue slot,
    // so the caller's buffer may go out of scope the moment submit() returns.
    const char* text = nullptr;

    // Bytes to send after the modem answers with its prompt. Used by socket
    // sends and by SMS.
    //
    // BORROWED, NOT COPIED, and this is the one ownership rule in the library:
    // it must stay valid until the command concludes. Copying it would mean a
    // twelve-slot queue carrying twelve 1500-byte payloads, which is 18 kB a
    // board does not have. The socket layer owns its transmit buffer and does
    // not release it until the completion runs.
    const uint8_t* payload = nullptr;
    size_t payloadLength = 0;

    // How long the modem has to produce a final result. From the driver, per
    // command, because the spread is enormous: a bare AT is a second, a socket
    // open is 120, and an operator scan is 180 - and nothing is wrong when the
    // scan takes all of it. A single global timeout is either far too short
    // for the scan or so long that a wedged modem is undetectable.
    uint32_t timeoutMs = 1000;

    // A second answer that arrives after the first. Empty for the usual case.
    const char* deferredPrefix = nullptr;
    uint32_t deferredTimeoutMs = 0;

    // For a module whose payload begins PART-WAY THROUGH its header line,
    // after a comma. Null for the ordinary shape.
    //
    // Carried on the command rather than armed by the caller, because the
    // parser is reset when a command STARTS - which is long after it was
    // submitted. Arming it at submit time and having the reset quietly undo it
    // was a real defect: the payload was then scanned as text, and the first
    // line ending inside it cut the reply in half.
    const char* inlinePayloadPrefix = nullptr;

    // Two commands with the same non-zero key supersede one another while both
    // are still queued and unsent. Zero means "never supersede".
    //
    // The key is the CALLER's namespace, not a global one - a socket uses its
    // own index, the network layer uses a constant - so unrelated commands
    // cannot collide.
    uint16_t supersedeKey = 0;

    GsmReplyFn onReply = nullptr;
    GsmDoneFn onDone = nullptr;
    void* context = nullptr;
};

class GsmAtEngine {
public:
    GsmAtEngine();
    ~GsmAtEngine();
    GsmAtEngine(const GsmAtEngine&) = delete;
    GsmAtEngine& operator=(const GsmAtEngine&) = delete;

    // How many commands may wait at once. 0 is GSM_QUEUE_CAPACITY. Takes
    // effect at the next reserve() - begin() makes one - while nothing is
    // queued; until then the storage already taken serves.
    void setCapacity(size_t commands) { _capacity = commands != 0 ? commands : GSM_QUEUE_CAPACITY; }

    // Takes the queue's storage now: a slot and a GSM_AT_COMMAND_MAX buffer
    // per command, one for the command on the wire, and the one long buffer.
    // False when there is no room - begin() then fails too, and an owner can
    // say why rather than every command later being refused.
    bool reserve();
    bool reserved() const { return _slots != nullptr; }

    // The transport is the caller's and must outlive the engine. False when
    // the transport will not open, or reserve() fails.
    bool begin(GsmTransport* transport);

    // Concludes everything outstanding as Cancelled, so nothing is left
    // waiting, and releases the link.
    //
    // A handler that submits while end() is concluding things is REFUSED, by
    // the return value, with NotInitialised. The link is going; accepting the
    // command would leave it in a slot of an engine that will never send it.
    void end();

    // The whole contract. Reads what has arrived, routes URCs, concludes what
    // finished, sends what is next, and times out what never came back.
    //
    // Never blocks, and never for longer than it takes to move the bytes that
    // are already there.
    void poll();

    // --- Submitting ---

    // Returns Queued on acceptance, or the reason it was refused.
    //
    // A refusal is reported through the RETURN VALUE and never through the
    // handler, because the caller has not yet recorded what it is waiting for.
    // Running its completion before its own call has returned is how a caller
    // concludes an operation it does not know it made.
    GsmResult submit(const GsmCommand& command);

    // The application's measured poll interval, pushed down by GsmModem.
    //
    // Used as a FLOOR under every command timeout: a command is not called
    // late until the application has had at least a few chances to notice the
    // reply. Without it, a caller polling slower than a timeout manufactures
    // timeouts out of its own scheduling.
    // The longest command line this MODULE will accept, from its profile.
    // Zero until a driver has been chosen. See submit().
    void setAtLineLimit(size_t characters) { _atLineLimit = characters; }
    size_t atLineLimit() const { return _atLineLimit; }

    void setServiceIntervalUs(gsm_time_t microseconds) { _serviceUs = microseconds; }
    gsm_time_t serviceIntervalUs() const { return _serviceUs; }

    // Bytes taken off the transport in one poll(). Bounds the worst-case work
    // in a single call, which is what an integrator with a scan-cycle deadline
    // actually needs - a wall-clock promise cannot be made across five
    // architectures, and a work budget can.
    //
    // The same knob as ModbusLink::setMaxBytesPerPoll(). Surplus stays in the
    // transport and is taken next call rather than being dropped.
    void setMaxBytesPerPoll(size_t bytes) { _maxBytesPerPoll = bytes ? bytes : 1; }
    size_t maxBytesPerPoll() const { return _maxBytesPerPoll; }

    // --- What the driver plugs in ---

    void setUrcRouter(GsmUrcRouter* router) { _router = router; }
    GsmUrcRouter* urcRouter() const { return _router; }

    // For final results the module invents beyond the 3GPP set.
    void setFinalResultHook(GsmFinalResultFn hook, void* context) {
        _finalHook = hook;
        _finalHookContext = context;
    }

    // The prompt character a module uses before a payload. '>' everywhere so
    // far, but it is a driver's business rather than an assumption.
    void setPromptCharacter(uint8_t character) { _promptCharacter = character; }

    // Conclude everything outstanding WITHOUT tearing the link down.
    //
    // For the case where the module is still on the other end of the wire but
    // is no longer the module we were talking to - it restarted, and anything
    // in flight died with the boot. end() would be wrong here: the transport
    // is fine and the caller wants to carry straight on configuring the new
    // instance.
    //
    // Every abandoned command still reaches exactly one conclusion, which is
    // the whole contract - a restart is not a licence to drop callers.
    //
    // WHAT A HANDLER SUBMITS DURING IT IS KEPT. Only what was outstanding when
    // abandonAll() was called is abandoned; a command a handler queues for the
    // module that has just come back is the caller carrying straight on, which
    // is what this exists for. It stays queued, is counted, and runs.
    void abandonAll(GsmStatus status);

    // --- State ---

    bool isIdle() const { return _state == State::Idle && _queued == 0; }
    size_t queued() const { return _queued; }
    size_t capacity() const { return _allocated != 0 ? _allocated : _capacity; }
    bool isBusy() const { return _state != State::Idle; }

    // Since begin(). For a diagnostic page and for tests that assert about
    // traffic rather than content.
    struct Counters {
        uint32_t submitted = 0;
        uint32_t concluded = 0;
        uint32_t timeouts = 0;
        uint32_t superseded = 0;
        uint32_t refused = 0;
        uint32_t bytesSent = 0;
        uint32_t bytesReceived = 0;
    };
    const Counters& counters() const { return _counters; }

    GsmAtParser& parser() { return _parser; }

    // The link, so the socket layer can ask what it can hold before deciding
    // how much to request from the modem. See GsmTransport::receiveCapacity():
    // on a link with no flow control, asking for more than can be drained
    // desynchronises the AT stream rather than merely dropping bytes.
    const GsmTransport* transport() const { return _transport; }

private:
    // Where the in-flight command has got to.
    //
    // PayloadOut exists as its own state rather than as a flag because of a
    // rule from the manual: "When the total size of the inputted data reaches
    // <len>, TA will report the following code. Otherwise, the serial port
    // will be blocked." (A76XX §19.2.14). A command that enters prompt mode
    // and then cannot finish its payload wedges the AT channel, so the whole
    // payload is in hand before the command is ever sent.
    enum class State : uint8_t {
        Idle = 0,
        SendingCommand,
        AwaitingPrompt,
        SendingPayload,
        AwaitingResult,
        AwaitingDeferred,
    };

    // `text` is `own` - the slot's GSM_AT_COMMAND_MAX buffer - or, for a
    // long command, the one long buffer. Buffers move with a command when it
    // starts (see startNext()), so a slot never shares one with the wire.
    struct Slot {
        char*    text = nullptr;
        char*    own = nullptr;
        size_t   textLength = 0;
        char     deferredPrefix[GSM_DEFERRED_PREFIX_MAX];
        const char* inlinePayloadPrefix = nullptr;
        const uint8_t* payload = nullptr;
        size_t   payloadLength = 0;
        uint32_t timeoutMs = 1000;
        uint32_t deferredTimeoutMs = 0;
        uint16_t supersedeKey = 0;
        GsmReplyFn onReply = nullptr;
        GsmDoneFn onDone = nullptr;
        void* context = nullptr;
        uint32_t opId = 0;
        bool inUse = false;
    };

    void servicePoll();
    void serviceTransmit();
    void handleLine(const char* line, size_t length, bool truncated);
    bool matchFinalResult(const char* line, size_t length, GsmResult& out);
    void startNext();
    void conclude(GsmResult result);
    void retire(size_t index, GsmResult result);
    bool withdraw(size_t index, GsmDoneFn& done, void*& context);
    void retireOlderThan(uint32_t boundary, GsmResult result);

    GsmTransport* _transport = nullptr;
    GsmUrcRouter* _router = nullptr;
    GsmAtParser   _parser;

    GsmFinalResultFn _finalHook = nullptr;
    void* _finalHookContext = nullptr;
    uint8_t _promptCharacter = '>';

    void release();
    void releaseText(Slot& slot);

    Slot*  _slots = nullptr;           // _allocated of them, taken by reserve()
    size_t _capacity = GSM_QUEUE_CAPACITY;
    size_t _allocated = 0;
    char*  _text = nullptr;            // every command buffer, in one allocation
    char*  _longText = nullptr;        // the long buffer, inside _text
    bool   _longInUse = false;
    size_t _queued = 0;
    uint32_t _nextOpId = 1;

    // The command in flight is slot 0's contents, moved out of the queue when
    // it starts. Held separately so that a completion handler may queue more
    // work without disturbing what is on the wire.
    Slot   _active;
    State  _state = State::Idle;
    size_t _sentCursor = 0;          // how much of the text or payload has gone
    gsm_time_t _deadline = 0;

    // A send was abandoned while the module may be sitting at its prompt -
    // the prompt timed out, or the payload stopped part-way. Until an ESC has
    // been written, the next command would be taken as payload bytes. See
    // serviceTransmit().
    bool _cancelPrompt = false;

    // And after the ESC, a short quiet spell before the next command: the
    // module may answer the cancellation, and that answer must reach nobody
    // rather than conclude the command that follows.
    bool       _settling = false;
    gsm_time_t _settleUntil = 0;

    gsm_time_t _serviceUs = 0;
    size_t     _atLineLimit = 0;
    size_t     _maxBytesPerPoll = GSM_POLL_BYTE_BUDGET;

    Counters _counters;
};
