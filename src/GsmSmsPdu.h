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
// SMS, IN THE FORM THE NETWORK ACTUALLY CARRIES
// =============================================================================
//
// A pure codec: bytes in, bytes out. No clock, no I/O, no modem. That is what
// makes it exhaustively testable against known PDUs and directly fuzzable,
// which matters because every byte it parses arrived from somewhere else.
//
// --- WHY PDU MODE AND NOT TEXT MODE ---
//
// Text mode is simpler and this library does not use it. It cannot carry
// anything outside the basic alphabet, it cannot do concatenation reliably,
// and its reply formatting differs between modules and between firmware
// revisions of the same module. The simpler-looking option is the one that
// breaks in the field, on the message that mattered.
//
// --- WHERE THIS COMES FROM ---
//
// 3GPP TS 23.040 defines the PDU structures; TS 23.038 defines the alphabet
// and the packing. Both are cited per function below.
//
// THESE TWO DOCUMENTS ARE NOT THE MODULE MANUALS, and the module manuals do
// not reproduce them. So this file is verified a second way: against
// known-good PDUs, checked in both directions - each one decoded to its
// fields and those fields encoded back to the same octets. Where a manual
// would be the authority, a vector is.

#include "GsmPlatform.h"
#include "GsmParse.h"

// The longest single message: 140 octets of user data plus headers.
#ifndef GSM_SMS_PDU_MAX
#define GSM_SMS_PDU_MAX 180
#endif

// [TS 23.038 4] The data coding scheme, reduced to what a controller meets.
enum class GsmSmsAlphabet : uint8_t {
    Gsm7Bit = 0,   // the default alphabet, seven bits per character
    Binary,        // 8-bit data, carried untouched
    Ucs2,          // 16-bit, for anything outside the default alphabet
};

struct GsmSmsMessage {
    char sender[24] = {0};          // or the recipient, when encoding
    char text[161] = {0};           // 160 septets is one message's worth
    size_t textLength = 0;

    GsmParse::GsmDateTime timestamp;
    bool hasTimestamp = false;

    GsmSmsAlphabet alphabet = GsmSmsAlphabet::Gsm7Bit;

    // [TS 23.040 9.2.3.24.1] Concatenation, where the message is one part of
    // several. Carried rather than reassembled here: reassembly needs storage
    // and a policy about incomplete sets, which belongs a layer up.
    bool     isPart = false;
    uint16_t reference = 0;
    uint8_t  partCount = 0;
    uint8_t  partIndex = 0;         // 1-based, as the standard numbers them
};

namespace GsmSmsPdu {

// --- Encoding ---

// Builds an SMS-SUBMIT for `text` to `number`.
//
// Returns the number of PDU octets written, or 0 on refusal - a number that
// will not fit, text too long for one message, a buffer too small. Never
// truncates: half an SMS that looks plausible is worse than none.
//
// `tpduOctets` receives what AT+CMGS needs, which is NOT the same as the
// return value. [TS 27.005 3.5.1] The length parameter counts the PDU
// EXCLUDING the service centre address, while the bytes handed to the modem
// INCLUDE it. Getting that wrong is the classic first-SMS failure: the modem
// waits for bytes that never come, or rejects a PDU it considers malformed.
size_t encodeSubmit(const char* number, const char* text,
                    uint8_t* out, size_t capacity, size_t& tpduOctets);

// --- Decoding ---

// Reads an SMS-DELIVER, as a module hands one back.
bool decodeDeliver(const uint8_t* pdu, size_t length, GsmSmsMessage& out);

// The hexadecimal text a module actually prints, converted to octets.
//
// A module reports a PDU as ASCII hex, two characters per octet. Refuses an
// odd length or a non-hex character rather than decoding half of it.
size_t hexToOctets(const char* hex, size_t length, uint8_t* out, size_t capacity);

// The reverse, for handing a PDU to the modem.
size_t octetsToHex(const uint8_t* octets, size_t length, char* out, size_t capacity);

// --- The pieces, exposed because they are independently testable ---

// [TS 23.038 6.1.2.1.1] Packs septets into octets, seven bits at a time,
// least significant bit first.
//
// THE PACKING IS NOT A BYTE OPERATION and that is where implementations go
// wrong: seven characters occupy eight octets, and a character's bits are
// split across an octet boundary six times out of seven.
size_t packSeptets(const uint8_t* septets, size_t count, uint8_t* out, size_t capacity);

size_t unpackSeptets(const uint8_t* octets, size_t octetCount, size_t septetCount,
                     uint8_t* out, size_t capacity);

// [TS 23.038 6.2.1] The default alphabet is NOT ASCII. Its first 128 entries
// include the Greek capitals, several currency symbols and a handful of
// accented letters, and ten ASCII characters are reachable only through an
// escape. A codec that assumed ASCII would send a pound sign as a hash.
bool asciiToGsm7(const char* text, size_t length, uint8_t* out, size_t capacity,
                 size_t& produced);
size_t gsm7ToAscii(const uint8_t* septets, size_t count, char* out, size_t capacity);

// [TS 23.040 9.1.2.5] Telephone numbers travel as semi-octets with the digits
// of each pair swapped, padded with 0xF. "+44 7700 900123" becomes
// 44 77 00 90 01 32 F1 - and reading it without the swap gives a number that
// looks plausible and reaches nobody.
size_t encodeAddress(const char* number, uint8_t* out, size_t capacity);
bool decodeAddress(const uint8_t* pdu, size_t length, size_t& offset,
                   char* out, size_t capacity);

} // namespace GsmSmsPdu
