// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#include "GsmSmsPdu.h"
#include "GsmNoExcept.h"

namespace {

// [TS 23.038 6.2.1] The GSM 7-bit default alphabet, as the Unicode code point
// each septet stands for - restricted here to what fits in one byte, which is
// every entry a controller realistically sends.
//
// THIS TABLE IS NOT ASCII AND THE DIFFERENCES ARE NOT DECORATIVE:
//
//   0x00 is '@', not NUL
//   0x01 is the pound sign, where ASCII has a control character
//   0x02 is the dollar sign
//   0x24 is the currency sign, where ASCII has '$'
//   0x40 is an inverted exclamation mark, where ASCII has '@'
//   0x5B..0x60 and 0x7B..0x7E are accented letters, not brackets
//
// So sending ASCII straight down the wire turns a pound sign into a hash and
// a backslash into an o-umlaut. 0xFF marks a position this table does not
// claim to represent.
const uint8_t kGsm7ToLatin[128] = {
    '@', 0xA3, '$', 0xA5, 0xE8, 0xE9, 0xF9, 0xEC,   // 0x00
    0xF2, 0xC7, '\n', 0xD8, 0xF8, '\r', 0xC5, 0xE5,
    0xFF, '_',  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0x10 - Greek, mostly
    0xFF, 0xFF, 0xFF, 0xFF, 0xC6, 0xE6, 0xDF, 0xC9,
    ' ', '!', '"', '#', 0xA4, '%', '&', '\'',       // 0x20
    '(', ')', '*', '+', ',', '-', '.', '/',
    '0', '1', '2', '3', '4', '5', '6', '7',         // 0x30
    '8', '9', ':', ';', '<', '=', '>', '?',
    0xA1, 'A', 'B', 'C', 'D', 'E', 'F', 'G',        // 0x40
    'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O',
    'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W',         // 0x50
    'X', 'Y', 'Z', 0xC4, 0xD6, 0xD1, 0xDC, 0xA7,
    0xBF, 'a', 'b', 'c', 'd', 'e', 'f', 'g',        // 0x60
    'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o',
    'p', 'q', 'r', 's', 't', 'u', 'v', 'w',         // 0x70
    'x', 'y', 'z', 0xE4, 0xF6, 0xF1, 0xFC, 0xE0,
};

// [TS 23.038 6.2.1.1] Ten characters live in an extension table and are sent
// as ESC (0x1B) followed by the code below. Without this, a message
// containing a brace or a Euro sign is silently mangled.
struct Extension { char ascii; uint8_t code; };
const Extension kExtensions[] = {
    { '\f', 0x0A }, { '^', 0x14 }, { '{', 0x28 }, { '}', 0x29 },
    { '\\', 0x2F }, { '[', 0x3C }, { '~', 0x3D }, { ']', 0x3E }, { '|', 0x40 },
};
const size_t kExtensionCount = sizeof kExtensions / sizeof kExtensions[0];

const uint8_t kEscape = 0x1B;

int hexDigit(char c) {
    if (c >= '0' && c <= '9') { return c - '0'; }
    if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
    if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
    return -1;
}

char hexChar(uint8_t value) {
    return (char)((value < 10) ? ('0' + value) : ('A' + value - 10));
}

// Finds a character's septet in the default alphabet, or reports that it needs
// the escape, or that it cannot be represented at all.
bool gsm7Septet(char character, uint8_t& septet, bool& needsEscape) {
    needsEscape = false;
    const uint8_t wanted = (uint8_t)character;

    for (size_t i = 0; i < kExtensionCount; i++) {
        if (kExtensions[i].ascii == character) {
            septet = kExtensions[i].code;
            needsEscape = true;
            return true;
        }
    }
    for (uint8_t i = 0; i < 128; i++) {
        if (kGsm7ToLatin[i] == wanted && kGsm7ToLatin[i] != 0xFF) {
            septet = i;
            return true;
        }
    }
    return false;
}

} // namespace

namespace GsmSmsPdu {

// =============================================================================
// HEX
// =============================================================================

size_t hexToOctets(const char* hex, size_t length, uint8_t* out, size_t capacity) {
    if (hex == nullptr || out == nullptr) { return 0; }

    // An odd length is not a PDU with a missing nibble - it is evidence the
    // reply was cut short, and decoding what arrived would produce a message
    // that looks real.
    if ((length & 1) != 0) { return 0; }
    if (length / 2 > capacity) { return 0; }

    for (size_t i = 0; i < length; i += 2) {
        const int high = hexDigit(hex[i]);
        const int low = hexDigit(hex[i + 1]);
        if (high < 0 || low < 0) { return 0; }
        out[i / 2] = (uint8_t)((high << 4) | low);
    }
    return length / 2;
}

size_t octetsToHex(const uint8_t* octets, size_t length, char* out, size_t capacity) {
    if (octets == nullptr || out == nullptr) { return 0; }
    if (length * 2 + 1 > capacity) { return 0; }

    for (size_t i = 0; i < length; i++) {
        out[i * 2] = hexChar((uint8_t)(octets[i] >> 4));
        out[i * 2 + 1] = hexChar((uint8_t)(octets[i] & 0x0F));
    }
    out[length * 2] = '\0';
    return length * 2;
}

// =============================================================================
// SEPTET PACKING
// =============================================================================

size_t packSeptets(const uint8_t* septets, size_t count, uint8_t* out, size_t capacity) {
    if (septets == nullptr || out == nullptr) { return 0; }

    // Seven characters occupy eight octets. The ceiling matters: 8 septets
    // need 7 octets, and rounding the wrong way either loses the last
    // character or appends a spurious one.
    const size_t needed = (count * 7 + 7) / 8;
    if (needed > capacity) { return 0; }

    for (size_t i = 0; i < needed; i++) { out[i] = 0; }

    // Bit-at-a-time rather than the usual shift-and-carry loop.
    //
    // Slower, and chosen deliberately: the compact form has an off-by-one at
    // the septet that straddles the eighth octet which is invisible in short
    // messages and corrupts long ones. This version cannot have it, and an
    // SMS is at most 160 characters so the cost is nothing.
    for (size_t index = 0; index < count; index++) {
        const uint8_t value = (uint8_t)(septets[index] & 0x7F);
        for (uint8_t bit = 0; bit < 7; bit++) {
            if ((value & (1u << bit)) == 0) { continue; }
            const size_t position = index * 7 + bit;
            out[position / 8] |= (uint8_t)(1u << (position % 8));
        }
    }
    return needed;
}

size_t unpackSeptets(const uint8_t* octets, size_t octetCount, size_t septetCount,
                     uint8_t* out, size_t capacity) {
    if (octets == nullptr || out == nullptr) { return 0; }
    if (septetCount > capacity) { return 0; }

    size_t produced = 0;
    for (size_t index = 0; index < septetCount; index++) {
        uint8_t value = 0;
        for (uint8_t bit = 0; bit < 7; bit++) {
            const size_t position = index * 7 + bit;

            // A DECLARED SEPTET COUNT IS NOT A PROMISE. A truncated or hostile
            // PDU can claim more characters than its octets can hold, and
            // reading past them is the classic parser defect in this format.
            if (position / 8 >= octetCount) { return produced; }

            if ((octets[position / 8] & (1u << (position % 8))) != 0) {
                value |= (uint8_t)(1u << bit);
            }
        }
        out[produced++] = value;
    }
    return produced;
}

// =============================================================================
// ALPHABET
// =============================================================================

bool asciiToGsm7(const char* text, size_t length, uint8_t* out, size_t capacity,
                 size_t& produced) {
    produced = 0;
    if (text == nullptr || out == nullptr) { return false; }

    for (size_t i = 0; i < length; i++) {
        uint8_t septet = 0;
        bool needsEscape = false;
        if (!gsm7Septet(text[i], septet, needsEscape)) {
            // Not representable in the default alphabet. Refused rather than
            // substituted: a message where one character silently became
            // something else is worse than one that was not sent, because
            // nobody finds out.
            return false;
        }

        // An escaped character costs TWO septets, which is why a message of
        // braces fits far fewer characters than one of letters - and why the
        // length check has to happen here rather than on the input length.
        const size_t cost = needsEscape ? 2u : 1u;
        if (produced + cost > capacity) { return false; }

        if (needsEscape) { out[produced++] = kEscape; }
        out[produced++] = septet;
    }
    return true;
}

size_t gsm7ToAscii(const uint8_t* septets, size_t count, char* out, size_t capacity) {
    if (septets == nullptr || out == nullptr || capacity == 0) { return 0; }

    size_t produced = 0;
    for (size_t i = 0; i < count && produced + 1 < capacity; i++) {
        const uint8_t septet = (uint8_t)(septets[i] & 0x7F);

        if (septet == kEscape) {
            if (i + 1 >= count) { break; }
            const uint8_t next = (uint8_t)(septets[++i] & 0x7F);
            bool found = false;
            for (size_t e = 0; e < kExtensionCount; e++) {
                if (kExtensions[e].code == next) {
                    out[produced++] = kExtensions[e].ascii;
                    found = true;
                    break;
                }
            }
            // [TS 23.038 6.2.1.1] An unknown escape is to be treated as a
            // space, not dropped - so the message keeps its shape.
            if (!found) { out[produced++] = ' '; }
            continue;
        }

        const uint8_t latin = kGsm7ToLatin[septet];
        out[produced++] = (latin == 0xFF) ? '?' : (char)latin;
    }
    out[produced] = '\0';
    return produced;
}

// =============================================================================
// ADDRESSES
// =============================================================================

size_t encodeAddress(const char* number, uint8_t* out, size_t capacity) {
    if (number == nullptr || out == nullptr) { return 0; }

    // A leading plus means international, which is a different type-of-address
    // octet. [TS 23.040 9.1.2.5] 0x91 is international E.164, 0x81 national.
    bool international = (number[0] == '+');
    const char* digits = international ? number + 1 : number;

    size_t count = 0;
    while (digits[count] != '\0') {
        if (digits[count] < '0' || digits[count] > '9') { return 0; }
        count++;
    }
    if (count == 0 || count > 20) { return 0; }

    const size_t pairs = (count + 1) / 2;
    if (capacity < pairs + 2) { return 0; }

    // The length is in DIGITS, not octets - the one field in the whole format
    // that is not a byte count.
    out[0] = (uint8_t)count;
    out[1] = international ? 0x91 : 0x81;

    // SEMI-OCTETS WITH THE PAIRS SWAPPED. "447700900123" becomes
    // 44 77 00 90 01 32 F1. Encoding it without the swap produces a number
    // that looks entirely plausible and reaches nobody.
    for (size_t i = 0; i < pairs; i++) {
        const uint8_t low = (uint8_t)(digits[i * 2] - '0');
        const uint8_t high = (i * 2 + 1 < count)
            ? (uint8_t)(digits[i * 2 + 1] - '0')
            : 0x0F;                       // odd count is padded with F
        out[2 + i] = (uint8_t)((high << 4) | low);
    }
    return pairs + 2;
}

bool decodeAddress(const uint8_t* pdu, size_t length, size_t& offset,
                   char* out, size_t capacity) {
    if (pdu == nullptr || out == nullptr || capacity == 0) { return false; }
    if (offset + 2 > length) { return false; }

    const uint8_t digitCount = pdu[offset];
    const uint8_t type = pdu[offset + 1];
    const size_t pairs = (digitCount + 1u) / 2u;

    if (offset + 2 + pairs > length) { return false; }
    if ((size_t)digitCount + 2 > capacity) { return false; }

    size_t produced = 0;
    if ((type & 0x70) == 0x10) { out[produced++] = '+'; }   // international

    for (size_t i = 0; i < pairs; i++) {
        const uint8_t octet = pdu[offset + 2 + i];
        const uint8_t low = (uint8_t)(octet & 0x0F);
        const uint8_t high = (uint8_t)(octet >> 4);

        if (produced + 1 < capacity) { out[produced++] = (char)('0' + low); }
        if ((i * 2 + 1) < digitCount && produced + 1 < capacity) {
            out[produced++] = (char)('0' + high);
        }
    }
    out[produced] = '\0';
    offset += 2 + pairs;
    return true;
}

// =============================================================================
// SUBMIT
// =============================================================================

size_t encodeSubmit(const char* number, const char* text,
                    uint8_t* out, size_t capacity, size_t& tpduOctets) {
    tpduOctets = 0;
    if (number == nullptr || text == nullptr || out == nullptr) { return 0; }

    size_t textLength = 0;
    while (text[textLength] != '\0') { textLength++; }

    uint8_t septets[200];
    size_t septetCount = 0;
    if (!asciiToGsm7(text, textLength, septets, sizeof septets, septetCount)) {
        return 0;
    }

    // [TS 23.040 9.2.3.16] One message carries at most 160 septets. Longer
    // needs concatenation, which is a different PDU shape - refused here
    // rather than silently truncated.
    if (septetCount > 160) { return 0; }

    size_t position = 0;

    // [TS 23.040 9.2.3.1] The service centre address. A single zero octet
    // means "use the one stored in the SIM", which is what every network
    // expects and what avoids needing the SMSC number at all.
    //
    // It is also the octet that AT+CMGS's length parameter EXCLUDES, which is
    // why tpduOctets is counted from here on rather than from zero.
    if (position >= capacity) { return 0; }
    out[position++] = 0x00;

    const size_t tpduStart = position;

    // [TS 23.040 9.2.2.2] The SMS-SUBMIT first octet.
    //   bits 1-0  message type indicator: 01 = SUBMIT
    //   bits 4-3  validity period format: 10 = relative, one octet
    // 0x11 is those two together and nothing else - no reply path, no user
    // data header, no status report requested.
    if (position >= capacity) { return 0; }
    out[position++] = 0x11;

    // [TS 23.040 9.2.3.6] Message reference. Zero lets the module choose,
    // which is what it does best - it keeps its own sequence across reboots.
    if (position >= capacity) { return 0; }
    out[position++] = 0x00;

    const size_t addressLength = encodeAddress(number, out + position, capacity - position);
    if (addressLength == 0) { return 0; }
    position += addressLength;

    // [TS 23.040 9.2.3.9] Protocol identifier: ordinary point-to-point.
    if (position >= capacity) { return 0; }
    out[position++] = 0x00;

    // [TS 23.038 4] Data coding scheme: the default alphabet.
    if (position >= capacity) { return 0; }
    out[position++] = 0x00;

    // [TS 23.040 9.2.3.12] Relative validity period. 0xAA is 4 days, which is
    // long enough that a device out of coverage overnight still receives, and
    // short enough that a stale command does not arrive next week.
    if (position >= capacity) { return 0; }
    out[position++] = 0xAA;

    // [TS 23.040 9.2.3.16] THE USER DATA LENGTH IS IN SEPTETS for the default
    // alphabet, and in OCTETS for every other coding scheme. Writing the octet
    // count here is the commonest single defect in SMS code: the message is
    // delivered with its last characters missing, and only for text long
    // enough that the two differ noticeably.
    if (position >= capacity) { return 0; }
    out[position++] = (uint8_t)septetCount;

    const size_t packed = packSeptets(septets, septetCount,
                                      out + position, capacity - position);
    if (packed == 0 && septetCount != 0) { return 0; }
    position += packed;

    tpduOctets = position - tpduStart;
    return position;
}

// =============================================================================
// DELIVER
// =============================================================================

bool decodeDeliver(const uint8_t* pdu, size_t length, GsmSmsMessage& out) {
    if (pdu == nullptr || length < 2) { return false; }

    size_t offset = 0;

    // [TS 23.040 9.2.3.1] The service centre address, which a received PDU
    // does carry. Its length is in OCTETS here, unlike the destination
    // address's length which is in digits - the format is not consistent with
    // itself and assuming it is walks the parser off the end.
    const uint8_t smscLength = pdu[offset];
    offset += 1 + smscLength;
    if (offset >= length) { return false; }

    // [TS 23.040 9.2.3.1] First octet. Bit 6 is the user-data-header
    // indicator, which is how concatenation announces itself.
    const uint8_t firstOctet = pdu[offset++];
    const bool hasUserDataHeader = (firstOctet & 0x40) != 0;

    if (!decodeAddress(pdu, length, offset, out.sender, sizeof out.sender)) {
        return false;
    }

    if (offset + 2 > length) { return false; }
    offset++;                               // protocol identifier
    const uint8_t codingScheme = pdu[offset++];

    // [TS 23.038 4] Bits 3-2 of the coding scheme choose the alphabet.
    const uint8_t alphabetBits = (uint8_t)((codingScheme >> 2) & 0x03);
    out.alphabet = (alphabetBits == 0) ? GsmSmsAlphabet::Gsm7Bit
                 : (alphabetBits == 1) ? GsmSmsAlphabet::Binary
                                       : GsmSmsAlphabet::Ucs2;

    // [TS 23.040 9.2.3.11] The service centre timestamp: seven semi-octet
    // pairs, swapped like an address, and the last is the time zone in
    // QUARTER HOURS with its sign in the low bit of the swapped value.
    if (offset + 7 > length) { return false; }
    {
        uint8_t field[7];
        for (size_t i = 0; i < 7; i++) {
            const uint8_t octet = pdu[offset + i];
            field[i] = (uint8_t)(((octet & 0x0F) * 10) + (octet >> 4));
        }
        out.timestamp.year = (uint16_t)(2000 + field[0]);
        out.timestamp.month = field[1];
        out.timestamp.day = field[2];
        out.timestamp.hour = field[3];
        out.timestamp.minute = field[4];
        out.timestamp.second = field[5];

        // The sign lives in bit 3 of the ORIGINAL high nibble, which the swap
        // above has moved into the units digit. Recovered from the raw octet
        // rather than from the swapped value.
        const uint8_t rawZone = pdu[offset + 6];
        const bool negative = (rawZone & 0x08) != 0;
        const uint8_t quarters =
            (uint8_t)((((rawZone & 0x07)) * 10) + (rawZone >> 4));
        out.timestamp.zoneQuarterHours =
            negative ? (int16_t)-(int16_t)quarters : (int16_t)quarters;
        out.hasTimestamp = true;
        offset += 7;
    }

    if (offset >= length) { return false; }
    const uint8_t userDataLength = pdu[offset++];

    const uint8_t* userData = pdu + offset;
    size_t userDataOctets = length - offset;

    size_t headerSeptets = 0;
    if (hasUserDataHeader) {
        if (userDataOctets < 1) { return false; }
        const uint8_t headerLength = userData[0];
        if ((size_t)headerLength + 1 > userDataOctets) { return false; }

        // [TS 23.040 9.2.3.24.1] Concatenation, eight-bit reference, and
        // 9.2.3.24.8 for the sixteen-bit form. Both appear in the wild.
        size_t p = 1;
        while (p + 1 < (size_t)headerLength + 1) {
            const uint8_t id = userData[p];
            const uint8_t len = userData[p + 1];
            if (p + 2 + len > (size_t)headerLength + 1) { break; }

            if (id == 0x00 && len == 3) {
                out.isPart = true;
                out.reference = userData[p + 2];
                out.partCount = userData[p + 3];
                out.partIndex = userData[p + 4];
            } else if (id == 0x08 && len == 4) {
                out.isPart = true;
                out.reference = (uint16_t)((userData[p + 2] << 8) | userData[p + 3]);
                out.partCount = userData[p + 4];
                out.partIndex = userData[p + 5];
            }
            p += 2 + len;
        }

        // THE HEADER OCCUPIES WHOLE SEPTETS, ROUNDED UP. Seven bits per
        // character means a header of n octets consumes ceil((n+1)*8/7)
        // septet positions, and the text begins at the next septet boundary -
        // not at the next octet. Getting this wrong shifts every character of
        // every concatenated message by a bit or two, which produces fluent
        // nonsense rather than an obvious fault.
        const size_t headerOctets = (size_t)headerLength + 1;
        headerSeptets = (headerOctets * 8 + 6) / 7;

        userData += headerOctets;
        userDataOctets -= headerOctets;
    }

    if (out.alphabet == GsmSmsAlphabet::Gsm7Bit) {
        uint8_t septets[200];
        const size_t wanted = (userDataLength > headerSeptets)
                            ? (size_t)userDataLength - headerSeptets : 0;

        // Re-unpack from the ORIGINAL octet boundary, then skip the header's
        // septets - the packing runs across the header, so unpacking from
        // after it would start mid-character.
        const uint8_t* packedStart = hasUserDataHeader
            ? (pdu + offset) : userData;
        const size_t packedOctets = hasUserDataHeader
            ? (length - offset) : userDataOctets;

        const size_t total = unpackSeptets(packedStart, packedOctets,
                                           (size_t)userDataLength,
                                           septets, sizeof septets);
        const size_t skip = (total > wanted) ? total - wanted : 0;
        out.textLength = gsm7ToAscii(septets + skip,
                                     (total > skip) ? total - skip : 0,
                                     out.text, sizeof out.text);
        return true;
    }

    if (out.alphabet == GsmSmsAlphabet::Ucs2) {
        // Big-endian sixteen-bit. Only the Latin-1 range is rendered; anything
        // else becomes '?' rather than being silently dropped, so the message
        // keeps its length and the loss is visible.
        size_t produced = 0;
        for (size_t i = 0; i + 1 < userDataOctets && produced + 1 < sizeof out.text; i += 2) {
            const uint16_t code = (uint16_t)((userData[i] << 8) | userData[i + 1]);
            out.text[produced++] = (code < 256) ? (char)code : '?';
        }
        out.text[produced] = '\0';
        out.textLength = produced;
        return true;
    }

    // Binary. Carried as it came; the caller decides what it means.
    size_t produced = 0;
    for (size_t i = 0; i < userDataOctets && produced + 1 < sizeof out.text; i++) {
        out.text[produced++] = (char)userData[i];
    }
    out.text[produced] = '\0';
    out.textLength = produced;
    return true;
}

} // namespace GsmSmsPdu
