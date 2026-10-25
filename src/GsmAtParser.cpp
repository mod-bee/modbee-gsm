// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#include "GsmAtParser.h"
#include "GsmNoExcept.h"

namespace {
constexpr size_t kMaxInlinePayload = GSM_MAX_INLINE_PAYLOAD;
}

void GsmAtParser::reset() {
    _mode = GsmAtMode::Line;
    _payloadRemaining = 0;
    _promptCharacter = '>';
    _inlinePrefix = nullptr;
    _inlinePending = false;
    _pendingInlinePayload = 0;
    _line[0] = '\0';
    startLine();
    _linesSeen = 0;
    _truncatedLines = 0;
}

void GsmAtParser::startLine() {
    // DELIBERATELY DOES NOT TOUCH _line. A Line event points at it, and this
    // runs immediately after one is emitted - clearing the first byte would
    // hand the caller an empty string with its original length.
    //
    // The buffer is overwritten naturally as the next line's bytes arrive, and
    // the NUL is written at emit time. That is exactly the documented
    // contract: an event is valid until the next call to consume().
    _lineLength = 0;
    _lineTruncated = false;
    _sawAnyOnLine = false;
}

void GsmAtParser::expectPayload(size_t bytes) {
    _payloadRemaining = bytes;
    _mode = GsmAtMode::PayloadIn;

    // Whatever was half-assembled on the current line is NOT part of the
    // payload and is not a line either. It is the tail of the header we were
    // just told about, and keeping it would prepend rubbish to the next real
    // line.
    startLine();
}

void GsmAtParser::expectInlinePayload(const char* prefix) {
    _inlinePrefix = prefix;
    _inlinePending = false;
    _pendingInlinePayload = 0;
}

void GsmAtParser::expectPrompt(uint8_t promptCharacter) {
    _promptCharacter = promptCharacter;
    _mode = GsmAtMode::Prompt;
    startLine();
}

void GsmAtParser::abandon() {
    _mode = GsmAtMode::Line;
    _payloadRemaining = 0;
    _inlinePrefix = nullptr;
    _inlinePending = false;
    _pendingInlinePayload = 0;
    startLine();
}

bool GsmAtParser::consume(const uint8_t* data, size_t length,
                          size_t& consumed, Event& out) {
    consumed = 0;
    out = Event();

    if (data == nullptr || length == 0) { return false; }

    // An inline header fired on the previous call: the bytes that follow it,
    // in this very buffer, are payload rather than the rest of a line.
    if (_inlinePending) {
        _inlinePending = false;
        _inlinePrefix = nullptr;
        expectPayload(_pendingInlinePayload);
        _pendingInlinePayload = 0;
    }

    // =========================================================================
    // PAYLOAD - count, do not look
    // =========================================================================
    if (_mode == GsmAtMode::PayloadIn) {
        // A zero-length payload is legal. The modem said it had nothing, which
        // is an answer rather than an error, and it completes without taking a
        // byte.
        if (_payloadRemaining == 0) {
            _mode = GsmAtMode::Line;
            out.kind = GsmAtEventKind::PayloadComplete;
            return true;
        }

        // THE ONE LINE THAT MATTERS. Never take more than was declared, even
        // when more is sitting in the buffer - what follows a payload is the
        // modem's "OK", and swallowing it into the payload loses the reply and
        // corrupts the data in the same stroke.
        size_t take = length;
        if (take > _payloadRemaining) { take = _payloadRemaining; }

        _payloadRemaining -= take;
        consumed = take;

        out.kind = GsmAtEventKind::PayloadChunk;
        out.data = data;                 // points into the CALLER's buffer
        out.length = take;
        out.remaining = _payloadRemaining;

        // THE MODE IS NOT CLEARED HERE. Completion is reported on the NEXT
        // call so a caller never handles two events from one return. Clearing
        // it here skips the completion event whenever the payload ends exactly
        // on a read boundary, which dribbled UART reads make common.
        //
        // PayloadIn with nothing remaining is the honest state: bytes
        // delivered, conclusion not yet reported.
        return true;
    }

    // =========================================================================
    // PROMPT - one character at the start of a line; whole lines before it
    // are still lines
    // =========================================================================
    //
    // CR, LF and spaces precede the prompt on most modules - the reply is
    // "\r\n> " - and are discarded.
    //
    // A WHOLE LINE BEFORE THE PROMPT IS NOT NOISE. It used to be: everything
    // ahead of the prompt character was swallowed, and that included any URC
    // the module sent while a send waited for its prompt. A bearer loss that
    // arrives mid-send, "+CIPEVENT: NETWORK CLOSED UNEXPECTEDLY", lands exactly
    // there - so the bearer loss was never seen, the modem believed it had a
    // data connection for ever, and every socket opened after it failed with
    // nothing to say why. A +CMTI there loses an SMS the same way.
    //
    // So a line that ends before the prompt is reported as a line, and the
    // parser stays waiting for the prompt. The engine routes it like any other:
    // a URC to its handler, an ERROR to the command that was refused - which
    // now concludes at once instead of timing out. Only a prompt character at
    // the START of a line is the prompt, so a '>' inside a line is text.
    if (_mode == GsmAtMode::Prompt) {
        for (size_t i = 0; i < length; i++) {
            const uint8_t byte = data[i];
            consumed = i + 1;

            bool blank = true;
            for (size_t b = 0; b < _lineLength; b++) {
                if (_line[b] != ' ') { blank = false; break; }
            }

            if (byte == _promptCharacter && blank) {
                _mode = GsmAtMode::Line;
                startLine();
                out.kind = GsmAtEventKind::Prompt;
                return true;
            }

            if (byte == '\n') {
                if (blank) {
                    startLine();
                    continue;
                }
                const bool truncated = _lineTruncated;
                _line[_lineLength] = '\0';
                _linesSeen++;
                if (truncated) { _truncatedLines++; }
                out.kind = GsmAtEventKind::Line;
                out.data = _line;
                out.length = _lineLength;
                out.truncated = truncated;
                startLine();
                return true;
            }

            if (byte == '\r') { continue; }

            if (_lineLength < GSM_AT_LINE_MAX) {
                _line[_lineLength++] = byte;
            } else {
                _lineTruncated = true;
            }
            _sawAnyOnLine = true;
        }
        return false;
    }

    // =========================================================================
    // LINE - CRLF delimited, bounded, and tolerant of either ending
    // =========================================================================
    for (size_t i = 0; i < length; i++) {
        const uint8_t byte = data[i];
        consumed = i + 1;

        if (byte == '\n') {
            // A line ends. It may be empty - modems emit "\r\n" liberally
            // between replies - and an empty line is reported rather than
            // swallowed, because the engine counts them when deciding whether
            // a module is idle.
            const bool truncated = _lineTruncated;
            const size_t produced = _lineLength;

            _line[produced] = '\0';
            _linesSeen++;
            if (truncated) { _truncatedLines++; }

            out.kind = GsmAtEventKind::Line;
            out.data = _line;
            out.length = produced;
            out.truncated = truncated;

            startLine();
            return true;
        }

        if (byte == '\r') {
            // Dropped rather than treated as a terminator. A bare CR with no
            // LF appears inside some modules' informational text, and ending
            // the line on it would split one reply into two that neither match
            // nor mean anything.
            continue;
        }

        // An armed inline payload: the comma after the length ends the header
        // and everything after it is opaque bytes. See expectInlinePayload().
        if (byte == ',' && _inlinePrefix != nullptr && !_inlinePending) {
            bool matches = true;
            size_t p = 0;
            for (; _inlinePrefix[p] != '\0'; p++) {
                if (p >= _lineLength || _line[p] != (uint8_t)_inlinePrefix[p]) {
                    matches = false;
                    break;
                }
            }
            if (matches && _lineLength > p) {
                // A LENGTH ARRIVES FROM THE MODEM AND IS BELIEVED, SO IT IS
                // BOUNDED HERE.
                //
                // This accumulated a size_t from up to GSM_AT_LINE_MAX digits
                // with no overflow guard and no ceiling, then handed it to
                // expectPayload(). One corrupt header - a line of garbage that
                // happens to start with the driver's inline prefix - would
                // park the parser in payload mode for an arbitrary count, and
                // only a command timeout calling abandon() would ever get it
                // out. Every byte in between would be swallowed as payload.
                //
                // Nothing the modem sends can legitimately exceed the largest
                // fetch this library will ever ask for, which is what the
                // profile's maxRecvChunk bounds. kMaxInlinePayload is that
                // ceiling with room to spare, and anything above it is a
                // corrupt line rather than a big one.
                size_t value = 0;
                bool numeric = true;
                bool digits = false;
                for (size_t d = p; d < _lineLength; d++) {
                    const uint8_t digit = _line[d];
                    if (digit == ' ') { continue; }
                    if (digit < '0' || digit > '9') { numeric = false; break; }
                    digits = true;
                    value = value * 10 + (size_t)(digit - '0');
                    if (value > kMaxInlinePayload) { numeric = false; break; }
                }
                if (numeric && digits) {
                    // Emit the header now; the payload begins with the next
                    // byte, which the top of the next consume() switches to.
                    _line[_lineLength] = '\0';
                    _linesSeen++;

                    out.kind = GsmAtEventKind::Line;
                    out.data = _line;
                    out.length = _lineLength;
                    out.truncated = false;

                    _pendingInlinePayload = value;
                    _inlinePending = true;
                    startLine();
                    return true;
                }
            }
        }

        if (_lineLength < GSM_AT_LINE_MAX) {
            _line[_lineLength++] = byte;
        } else {
            // Past the cap. Keep counting to the terminator so the line still
            // ENDS, but stop storing. See the header for why neither growing
            // nor dropping is acceptable.
            _lineTruncated = true;
        }
        _sawAnyOnLine = true;
    }

    return false;
}
