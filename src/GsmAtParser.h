// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#pragma once

// The AT channel is in one of four modes at any instant, and the parser must
// be told which:
//
//   Line         text, CRLF delimited. Commands, replies, URCs.
//   Prompt       waiting for a bare '>' before a payload may be sent.
//   PayloadIn    exactly N raw bytes the modem is writing to us.
//   PayloadOut   exactly N raw bytes we are writing to the modem.
//
// In payload mode it counts bytes and inspects nothing. A payload is opaque
// and a TLS record contains CR, LF and NUL within its first few hundred bytes,
// so a line-scanning parser would cut it apart. The length comes from the
// header the driver parsed, never from a guess.
//
// Reads the REPLY direction only. [ITU-T V.25ter] a command ends with bare CR,
// a reply with CRLF, so a lone CR here is dropped rather than terminating -
// some modules emit one inside informational text. Anything reading COMMANDS
// rather than replies must frame on CR itself.
//
// No clock and no I/O: bytes in, events out. Timeouts and meaning belong one
// layer up, which is what makes this fuzzable in isolation.
//
// A Line event points into an internal buffer, since a line is assembled
// across reads. A PayloadChunk points into the CALLER'S buffer - payloads are
// passed through, not copied.
#include "GsmPlatform.h"

// The longest reply line held in full. Past this the line is TRUNCATED and
// flagged, never grown and never dropped.
//
// Growing is not an option on a controller: a modem at the wrong baud rate
// emits megabytes of rubbish with no CRLF anywhere in it, and an unbounded
// accumulator is an out-of-memory reset. Dropping is not an option either -
// the engine still needs to know a line ended, or it waits forever for a
// reply that already came and went.
//
// 512 covers every reply this library parses. The long ones - an AT+COPS=?
// operator scan, an AT+CPSI? with every field populated - are informational,
// and a truncated one with a flag is a usable answer where a lost one is not.
#ifndef GSM_AT_LINE_MAX
#define GSM_AT_LINE_MAX 512
#endif

// The largest inline payload length this parser will believe from a modem.
//
// A header line carries the count of bytes that follow it. That count is the
// modem's word, not ours, and a corrupt line that happens to match a driver's
// inline prefix would otherwise put the parser into payload mode for as many
// bytes as its digits spell. Both families cap a single fetch at 1500, so
// anything past this is not a large read - it is a line that is not a header.
#ifndef GSM_MAX_INLINE_PAYLOAD
#define GSM_MAX_INLINE_PAYLOAD 2048
#endif

enum class GsmAtMode : uint8_t {
    Line = 0,     // the default and the resting state
    Prompt,       // waiting for the send prompt
    PayloadIn,    // taking exactly N bytes from the modem
};

enum class GsmAtEventKind : uint8_t {
    None = 0,
    Line,             // a complete line, CRLF stripped. May be empty.
    Prompt,           // the prompt character arrived
    PayloadChunk,     // part of a payload; points into the caller's buffer
    PayloadComplete,  // the declared byte count has been delivered
};

class GsmAtParser {
public:
    struct Event {
        GsmAtEventKind kind = GsmAtEventKind::None;

        // Line: the text, NUL terminated, CRLF stripped.
        // PayloadChunk: raw bytes, NOT terminated and NOT text.
        const uint8_t* data = nullptr;
        size_t length = 0;

        // Line only. The line was longer than GSM_AT_LINE_MAX and what is here
        // is its beginning. The engine treats a truncated line as unmatchable
        // rather than as a short line that happens not to match.
        bool truncated = false;

        // PayloadChunk only. How many bytes of the payload are still to come.
        size_t remaining = 0;

        const char* text() const { return (const char*)data; }
    };

    GsmAtParser() { reset(); }

    // Back to line mode with an empty buffer. Called when a link opens, and
    // after any fault that leaves the mode in doubt.
    void reset();

    // --- Telling the parser what to expect ---

    // Take exactly `bytes` and emit them as PayloadChunk events, then
    // PayloadComplete. Zero is legal and completes immediately - a modem may
    // legitimately answer that it has nothing.
    void expectPayload(size_t bytes);

    // Wait for a bare prompt character. '>' on every module this library has
    // met, but it is a parameter because that is a driver's business.
    void expectPrompt(uint8_t promptCharacter = '>');

    // A PAYLOAD THAT BEGINS PART-WAY THROUGH A LINE.
    //
    // Not every module puts its received bytes on a line of their own. One
    // family answers a fetch with
    //
    //     +CIPRXGET: 2,0,6,0<CR><LF>WORLD!<CR><LF>OK
    //
    // - a header line, then the bytes - while another answers
    //
    //     +CARECV: 6,WORLD!<CR><LF>OK
    //
    // with the length and the payload on ONE line, separated by a comma. The
    // second shape cannot be handled by waiting for a line end: the payload is
    // opaque, so the first CR or LF inside it would cut the line short and the
    // remainder would be read as replies. A TLS record contains one within a
    // few hundred bytes essentially always.
    //
    // Armed before such a command, this watches the line being assembled. When
    // it starts with `prefix` and a comma arrives, the digits before that
    // comma are the length: the header is emitted as a Line event and the
    // parser switches straight to payload mode for exactly that many bytes.
    //
    // Cleared automatically once it fires, so it never affects a later reply.
    void expectInlinePayload(const char* prefix);

    // Abandon whatever was expected and return to line mode. Used when a
    // command times out mid-payload: the bytes that never came must not be
    // waited for forever, and the next reply must be read as a line.
    void abandon();

    GsmAtMode mode() const { return _mode; }
    size_t payloadRemaining() const { return _payloadRemaining; }

    // --- Feeding it ---

    // Consumes from `data` until one event is ready or the input runs out.
    //
    // Returns true when `out` holds an event; `consumed` always says how many
    // bytes were taken. Call it in a loop until it returns false:
    //
    //     size_t offset = 0;
    //     while (offset < length) {
    //         size_t used = 0;
    //         GsmAtParser::Event event;
    //         const bool got = parser.consume(data + offset, length - offset,
    //                                         used, event);
    //         offset += used;
    //         if (got) { handle(event); } else { break; }
    //     }
    //
    // Stopping at each event is what makes the mode switch safe: the driver
    // reads a "+CIPRXGET: 2,0,100,0" header, calls expectPayload(100), and the
    // very next byte is treated as payload - even when the header and the
    // first payload byte arrived in the same read.
    bool consume(const uint8_t* data, size_t length, size_t& consumed, Event& out);

    // --- Counters, for diagnostics rather than logic ---

    uint32_t linesSeen() const { return _linesSeen; }
    uint32_t truncatedLines() const { return _truncatedLines; }

private:
    void startLine();

    uint8_t  _line[GSM_AT_LINE_MAX + 1];
    size_t   _lineLength = 0;
    bool     _lineTruncated = false;
    bool     _sawAnyOnLine = false;

    GsmAtMode _mode = GsmAtMode::Line;
    size_t    _payloadRemaining = 0;
    uint8_t   _promptCharacter = '>';

    // Inline payload arming. See expectInlinePayload().
    const char* _inlinePrefix = nullptr;
    size_t      _pendingInlinePayload = 0;
    bool        _inlinePending = false;

    uint32_t _linesSeen = 0;
    uint32_t _truncatedLines = 0;
};
