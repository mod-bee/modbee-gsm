// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#include "GsmParse.h"
#include "GsmNoExcept.h"

namespace {

bool isSpace(char c) { return c == ' ' || c == '\t'; }
bool isDigit(char c) { return c >= '0' && c <= '9'; }

char lower(char c) {
    return (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c;
}

int hexValue(char c) {
    if (c >= '0' && c <= '9') { return c - '0'; }
    if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
    if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
    return -1;
}

// Two digits, exactly. Used by the timestamp parser, where "8" where "08" was
// expected means the format is not what we think it is and every field after
// it is misaligned.
bool twoDigits(const char* p, uint8_t& out) {
    if (!isDigit(p[0]) || !isDigit(p[1])) { return false; }
    out = (uint8_t)((p[0] - '0') * 10 + (p[1] - '0'));
    return true;
}

} // namespace

// =============================================================================
// GsmField
// =============================================================================

bool GsmField::equals(const char* other) const {
    if (other == nullptr) { return false; }
    size_t i = 0;
    for (; i < length; i++) {
        if (other[i] == '\0' || data[i] != other[i]) { return false; }
    }
    return other[i] == '\0';
}

bool GsmField::equalsIgnoringCase(const char* other) const {
    if (other == nullptr) { return false; }
    size_t i = 0;
    for (; i < length; i++) {
        if (other[i] == '\0' || lower(data[i]) != lower(other[i])) { return false; }
    }
    return other[i] == '\0';
}

bool GsmField::startsWith(const char* prefix) const {
    if (prefix == nullptr) { return false; }
    for (size_t i = 0; prefix[i] != '\0'; i++) {
        if (i >= length || data[i] != prefix[i]) { return false; }
    }
    return true;
}

bool GsmField::copyTo(char* out, size_t capacity) const {
    if (out == nullptr || capacity == 0) { return false; }
    if (length + 1 > capacity) { return false; }   // refuse, never truncate
    for (size_t i = 0; i < length; i++) { out[i] = data[i]; }
    out[length] = '\0';
    return true;
}

// =============================================================================
// Structure
// =============================================================================

namespace GsmParse {

namespace {

// A label is only a label when its colon is there. Without that check a module
// legitimately named something starting with one of these words would be
// discarded as a label rather than read as the model.
bool hasLabel(const char* line, size_t length, const char* label) {
    size_t n = 0;
    while (label[n] != '\0') { n++; }
    if (length <= n || line[n] != ':') { return false; }
    for (size_t i = 0; i < n; i++) {
        if (line[i] != label[i]) { return false; }
    }
    return true;
}

} // namespace

GsmIdentityLine identityLine(const char* line, size_t length, GsmField& out) {
    out = GsmField();
    if (line == nullptr) { return GsmIdentityLine::OtherLabel; }

    if (hasLabel(line, length, "Model")) {
        afterPrefix(line, length, "Model", out);
        return GsmIdentityLine::Model;
    }

    // Everything a module is known to put in front of a line that is NOT the
    // model. Captured, not guessed - see the header.
    static const char* const kOtherLabels[] = {
        "Manufacturer", "Revision", "IMEI", "IMEISV", "SubEdition", "+GCAP",
    };
    for (size_t i = 0; i < sizeof kOtherLabels / sizeof kOtherLabels[0]; i++) {
        if (hasLabel(line, length, kOtherLabels[i])) {
            afterPrefix(line, length, kOtherLabels[i], out);
            return GsmIdentityLine::OtherLabel;
        }
    }

    out.data = line;
    out.length = length;
    return GsmIdentityLine::Unlabelled;
}

bool afterPrefix(const char* line, size_t length, const char* prefix, GsmField& out) {
    out = GsmField();
    if (line == nullptr || prefix == nullptr) { return false; }

    size_t i = 0;
    for (; prefix[i] != '\0'; i++) {
        if (i >= length || line[i] != prefix[i]) { return false; }
    }

    // A prefix may or may not be followed by its colon depending on how the
    // caller wrote it - "+CSQ" and "+CSQ:" are both reasonable to ask for.
    if (i < length && line[i] == ':') { i++; }

    while (i < length && isSpace(line[i])) { i++; }

    out.data = line + i;
    out.length = length - i;
    return true;
}

bool field(const char* text, size_t length, size_t index, GsmField& out) {
    out = GsmField();
    if (text == nullptr) { return false; }

    size_t start = 0;
    size_t current = 0;
    bool inQuotes = false;

    for (size_t i = 0; i <= length; i++) {
        const bool atEnd = (i == length);
        const char c = atEnd ? ',' : text[i];

        if (!atEnd && c == '"') {
            // A comma inside quotes does not separate. An operator name may
            // contain one - "AT&T, Inc." - and splitting on it would shift
            // every field after it by one.
            inQuotes = !inQuotes;
            continue;
        }

        if (c == ',' && !inQuotes) {
            if (current == index) {
                GsmField raw;
                raw.data = text + start;
                raw.length = i - start;
                out = unquote(trim(raw));
                return true;
            }
            current++;
            start = i + 1;
        }
    }

    // Asked for a field past the end. The modem did not say it, so this is
    // false rather than an empty field - the two are different, and a caller
    // that cannot tell them apart reads a short reply as a complete one.
    return false;
}

size_t fieldCount(const char* text, size_t length) {
    if (text == nullptr || length == 0) { return 0; }

    size_t count = 1;
    bool inQuotes = false;
    for (size_t i = 0; i < length; i++) {
        if (text[i] == '"') { inQuotes = !inQuotes; }
        else if (text[i] == ',' && !inQuotes) { count++; }
    }
    return count;
}

// =============================================================================
// Numbers
// =============================================================================

bool toUnsigned(const GsmField& in, uint32_t& out) {
    if (in.data == nullptr || in.length == 0) { return false; }

    uint32_t value = 0;
    for (size_t i = 0; i < in.length; i++) {
        const char c = in.data[i];
        if (!isDigit(c)) { return false; }          // trailing rubbish is a refusal

        const uint32_t digit = (uint32_t)(c - '0');
        // Overflow check BEFORE the multiply. A modem that reported a length
        // of 4294967300 would otherwise wrap to 4 and the caller would ask for
        // four bytes of a payload that is not four bytes long.
        if (value > (0xFFFFFFFFu - digit) / 10u) { return false; }
        value = value * 10u + digit;
    }
    out = value;
    return true;
}

bool toSigned(const GsmField& in, int32_t& out) {
    if (in.data == nullptr || in.length == 0) { return false; }

    size_t i = 0;
    bool negative = false;
    if (in.data[0] == '-' || in.data[0] == '+') {
        negative = (in.data[0] == '-');
        i = 1;
        if (in.length == 1) { return false; }       // a bare sign is not a number
    }

    GsmField digits;
    digits.data = in.data + i;
    digits.length = in.length - i;

    uint32_t magnitude = 0;
    if (!toUnsigned(digits, magnitude)) { return false; }

    // Asymmetric on purpose: INT32_MIN has no positive counterpart, and a
    // check written as `magnitude > 2147483647` would refuse a legitimate
    // -2147483648.
    if (negative) {
        if (magnitude > 2147483648u) { return false; }
        out = (magnitude == 2147483648u) ? (int32_t)(-2147483647 - 1)
                                         : -(int32_t)magnitude;
    } else {
        if (magnitude > 2147483647u) { return false; }
        out = (int32_t)magnitude;
    }
    return true;
}

bool toUnsignedInRange(const GsmField& in, uint32_t low, uint32_t high, uint32_t& out) {
    uint32_t value = 0;
    if (!toUnsigned(in, value)) { return false; }
    if (value < low || value > high) { return false; }
    out = value;
    return true;
}

bool toHex(const GsmField& in, uint32_t& out) {
    if (in.data == nullptr || in.length == 0 || in.length > 8) { return false; }

    uint32_t value = 0;
    for (size_t i = 0; i < in.length; i++) {
        const int digit = hexValue(in.data[i]);
        if (digit < 0) { return false; }
        value = (value << 4) | (uint32_t)digit;
    }
    out = value;
    return true;
}

// =============================================================================
// Domain conversions
// =============================================================================

bool rssiToDbm(uint32_t rssi, int& dbm) {
    // 99 is the modem saying it does not know, and it is NOT a level. Reading
    // it as one puts a number on a page that nobody can act on.
    if (rssi == 99) { return false; }
    if (rssi > 31) { return false; }

    // [A76XX §3.2.3] 0 is "-113 dBm or less", 31 is "-51 dBm or greater", and
    // everything between steps by 2 dB. The endpoints are bounds rather than
    // measurements, which is worth knowing when a reading sits at either.
    dbm = -113 + 2 * (int)rssi;
    return true;
}

bool toDateTime(const GsmField& in, GsmDateTime& out) {
    // "yy/MM/dd,hh:mm:ss+zz" - exactly 20 characters, and every separator in
    // its place. Length-checked first so no index below can run off the end
    // regardless of what the modem sent.
    const GsmField text = unquote(in);
    if (text.data == nullptr || text.length != 20) { return false; }

    const char* p = text.data;
    if (p[2] != '/' || p[5] != '/' || p[8] != ',' ||
        p[11] != ':' || p[14] != ':') { return false; }
    if (p[17] != '+' && p[17] != '-') { return false; }

    uint8_t yy = 0, mm = 0, dd = 0, hh = 0, mi = 0, ss = 0, zz = 0;
    if (!twoDigits(p + 0,  yy)) { return false; }
    if (!twoDigits(p + 3,  mm)) { return false; }
    if (!twoDigits(p + 6,  dd)) { return false; }
    if (!twoDigits(p + 9,  hh)) { return false; }
    if (!twoDigits(p + 12, mi)) { return false; }
    if (!twoDigits(p + 15, ss)) { return false; }
    if (!twoDigits(p + 18, zz)) { return false; }

    if (mm < 1 || mm > 12 || dd < 1 || dd > 31) { return false; }
    if (hh > 23 || mi > 59 || ss > 60) { return false; }   // 60 for a leap second

    // [A76XX §3.2.7] "-96 to 96", in QUARTER HOURS. The manual's own example
    // is "+32" for GMT+8. Anything outside that range is not a time zone.
    if (zz > 96) { return false; }

    // Two-digit years, windowed at 2000. These modules did not exist before
    // 2000 and this code will not be running in 2100.
    out.year = (uint16_t)(2000 + yy);
    out.month = mm;
    out.day = dd;
    out.hour = hh;
    out.minute = mi;
    out.second = ss;
    out.zoneQuarterHours = (p[17] == '-') ? (int16_t)-(int16_t)zz : (int16_t)zz;
    return true;
}

bool toIpv4(const GsmField& in, uint8_t out[4]) {
    const GsmField text = unquote(in);
    if (text.data == nullptr || text.length == 0) { return false; }

    uint32_t octet = 0;
    size_t digits = 0;
    size_t index = 0;

    for (size_t i = 0; i <= text.length; i++) {
        const bool atEnd = (i == text.length);
        const char c = atEnd ? '.' : text.data[i];

        if (c == '.') {
            if (digits == 0 || digits > 3 || octet > 255 || index > 3) { return false; }
            out[index++] = (uint8_t)octet;
            octet = 0;
            digits = 0;
            continue;
        }
        if (!isDigit(c)) { return false; }
        octet = octet * 10 + (uint32_t)(c - '0');
        digits++;
        if (octet > 255) { return false; }
    }

    return index == 4;
}

bool looksLikeIpv6(const GsmField& in) {
    const GsmField text = unquote(in);
    if (text.data == nullptr || text.length < 2) { return false; }

    // A recogniser, not a parser: at least one colon, and nothing that is not
    // hex, a colon or a dot - the last for the v4-mapped form.
    bool sawColon = false;
    for (size_t i = 0; i < text.length; i++) {
        const char c = text.data[i];
        if (c == ':') { sawColon = true; continue; }
        if (c == '.') { continue; }
        if (hexValue(c) < 0) { return false; }
    }
    return sawColon;
}

// =============================================================================
// Text
// =============================================================================

GsmField unquote(const GsmField& in) {
    if (in.data != nullptr && in.length >= 2 &&
        in.data[0] == '"' && in.data[in.length - 1] == '"') {
        GsmField out;
        out.data = in.data + 1;
        out.length = in.length - 2;
        return out;
    }
    // Left alone rather than refused: modules are inconsistent about quoting
    // the same field between firmware revisions, and a parser that insisted
    // would break on a firmware update that changed nothing else.
    return in;
}

GsmField trim(const GsmField& in) {
    GsmField out = in;
    while (out.length > 0 && isSpace(out.data[0])) { out.data++; out.length--; }
    while (out.length > 0 && isSpace(out.data[out.length - 1])) { out.length--; }
    return out;
}

} // namespace GsmParse
