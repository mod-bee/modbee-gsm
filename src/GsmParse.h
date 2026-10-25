// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#pragma once

// =============================================================================
// READING WHAT A MODEM SAID
// =============================================================================
//
// Free functions over (pointer, length). No allocation, no clock, no I/O, and
// no knowledge of which module is talking. Fuzzable directly, which matters
// because everything here parses bytes we did not produce, over a link that
// can corrupt them, at a baud rate that might be wrong.
//
// --- THE RULE: REFUSE, NEVER GUESS ---
//
// Every function here returns false rather than a default when the input is
// not what it expected. That is not fastidiousness, it is the difference
// between a diagnosis and a wrong alarm:
//
//   "+CSQ: ,99" with a missing first field must not read as 0. Zero is a VALID
//   signal level meaning -113 dBm - so a parser that returned zero on failure
//   would report a dead modem as a real reading at the bottom of the scale,
//   and every alarm built on it fires for the wrong reason. The caller needs
//   to know the difference between "no signal" and "no answer".
//
// A caller that genuinely wants a default writes it at the call site, where
// somebody reading the code can see the decision.

#include "GsmPlatform.h"

// A borrowed slice of somebody else's buffer. Never owns, never allocates, and
// never outlives the line it points into.
struct GsmField {
    const char* data = nullptr;
    size_t length = 0;

    bool isEmpty() const { return length == 0; }

    // Exact comparison against a C string.
    bool equals(const char* other) const;

    // Case-insensitive, for the keywords modules spell inconsistently -
    // "OK", "ok", "Ok" all appear in real captures from the same module.
    bool equalsIgnoringCase(const char* other) const;

    bool startsWith(const char* prefix) const;

    // Copies into a caller's buffer and NUL terminates. Returns false if it
    // will not fit, rather than truncating - a half-copied ICCID that looks
    // plausible is worse than none.
    bool copyTo(char* out, size_t capacity) const;
};

namespace GsmParse {

// --- Structure ---

// Whatever follows "+PREFIX:" in `line`, with leading spaces removed, or an
// empty field when the line does not carry that prefix.
//
// Matching is exact and case-sensitive on purpose: "+CREG" and "+CGREG" differ
// by one character and mean different things, and a loose match between them
// is a registration state read from the wrong radio access technology.
bool afterPrefix(const char* line, size_t length, const char* prefix, GsmField& out);

// --- What a line of an identity reply is ---
//
// ATI is documented as free-form and the manuals show no fixed shape, so the
// only honest source for this is a capture. A real A7670G answers with four
// LABELLED lines (the IMEI here is an example, not the module's):
//
//     Manufacturer: SIMCOM INCORPORATED
//     Model: A7670G-LABE
//     Revision: V1.11.2
//     IMEI: 490154203237518
//
// while other modules answer with the bare model alone. Both are legal, and
// the difference decides whether a driver recognises the module at all.
//
// THIS RULE LIVES HERE BECAUSE IT WAS WRITTEN THREE TIMES AND WAS WRONG IN
// ALL THREE. Three separate readers of ATI each kept "the first line of four
// characters or more", which on the labelled form is the MANUFACTURER - so a
// healthy module was refused by every one of them. One rule, one place, and
// every reader calls it.
enum class GsmIdentityLine : uint8_t {
    Model,        // labelled "Model:" - authoritative, overrides anything else
    OtherLabel,   // a label that is NOT the model; never a candidate
    Unlabelled,   // free-form; the model if nothing better turns up
};

// Classifies one line of an ATI or AT+CGMM reply. `out` carries the value
// with any label and surrounding space removed.
GsmIdentityLine identityLine(const char* line, size_t length, GsmField& out);

// The n-th comma-separated field, counting from zero.
//
// Quoted fields are respected: a comma INSIDE quotes does not separate. That
// matters more than it looks - an operator name can legitimately contain one,
// and an AT+COPS? reply carrying "AT&T, Inc." would otherwise split into two
// fields and shift every field after it.
//
// The quotes themselves are stripped. An absent field - "1,,3" - yields an
// empty field and true, because the modem did say it; a field PAST the end
// yields false, because it did not.
bool field(const char* text, size_t length, size_t index, GsmField& out);

// How many comma-separated fields the text holds.
size_t fieldCount(const char* text, size_t length);

// --- Numbers ---
//
// Both refuse on overflow, on trailing rubbish, and on an empty field. A
// modem reporting "+CSQ: 3x" is a modem that is not well, and reading it as 3
// hides that.

bool toUnsigned(const GsmField& in, uint32_t& out);
bool toSigned(const GsmField& in, int32_t& out);

// A number that must also fall inside a stated range, which is how most AT
// parameters are defined. Saves every caller writing the same two comparisons
// and forgetting one of them.
bool toUnsignedInRange(const GsmField& in, uint32_t low, uint32_t high, uint32_t& out);

// Hexadecimal, for the cell and tracking-area identifiers that modules report
// in hex without saying so.
bool toHex(const GsmField& in, uint32_t& out);

// --- Domain conversions, where the manual defines a mapping ---

// AT+CSQ's <rssi> to dBm.
//
// [A76XX §3.2.3, 3GPP TS 27.007 8.5] 0 is "-113 dBm or less", 1 is -111, 2..30
// span -109..-53 in 2 dB steps, 31 is "-51 dBm or greater", and 99 is "not
// known or not detectable".
//
// So the mapping is -113 + 2*rssi, and 99 is NOT a signal level - it is the
// modem saying it does not know. Returning false for it is the whole reason
// this is a function rather than arithmetic at the call site.
bool rssiToDbm(uint32_t rssi, int& dbm);

// A timestamp as AT+CCLK? reports it.
//
// [A76XX §3.2.7] Format "yy/MM/dd,hh:mm:ss+zz", and THE ZONE IS IN QUARTER
// HOURS, not hours - the manual's own example is "08/05/06,14:28:10+32" for
// GMT+8. A parser that read 32 as hours would place a device eight time zones
// wrong and every timestamp with it. Range is -96..96.
struct GsmDateTime {
    uint16_t year = 0;        // full year, so 2008 rather than 8
    uint8_t  month = 0;       // 1..12
    uint8_t  day = 0;         // 1..31
    uint8_t  hour = 0;
    uint8_t  minute = 0;
    uint8_t  second = 0;
    int16_t  zoneQuarterHours = 0;   // as reported: quarter hours from GMT
    int      zoneMinutes() const { return zoneQuarterHours * 15; }
};

bool toDateTime(const GsmField& in, GsmDateTime& out);

// A dotted-quad, as modules report their own address.
//
// Returned as four octets rather than a uint32_t so there is no byte-order
// question to get wrong between a board and a host.
bool toIpv4(const GsmField& in, uint8_t out[4]);

// Whether a field looks like an IPv6 literal. Deliberately a recogniser rather
// than a parser: this library carries addresses as text and hands them to the
// modem as text, so nothing needs the sixteen bytes.
bool looksLikeIpv6(const GsmField& in);

// --- Text ---

// Strips one layer of surrounding double quotes, if present. Leaves an
// unquoted field alone rather than refusing it, because modules are
// inconsistent about quoting the same field between firmware revisions.
GsmField unquote(const GsmField& in);

// Leading and trailing spaces and tabs removed.
GsmField trim(const GsmField& in);

} // namespace GsmParse
