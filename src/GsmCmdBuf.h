// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#pragma once

// A bounded builder for one command line.
//
// Drivers assemble commands from fragments - a keyword, a socket number, a
// quoted hostname, a port. Done with sprintf into a raw array, that is one
// buffer overflow per driver waiting for a long hostname; done with std::string
// it allocates on the poll path, which a controller cannot afford.
//
// So: a fixed buffer that REFUSES rather than truncates. A truncated AT command
// is not a shorter command, it is a different one - "AT+CIPOPEN=0,\"TCP\",\"ho"
// is a syntax error at best and a connection to the wrong place at worst - and
// the engine turns an overflowed buffer into a named refusal before anything
// reaches the modem.

#include "GsmPlatform.h"
#include "GsmAtEngine.h"   // GSM_AT_COMMAND_LONG_MAX

class GsmCmdBuf {
public:
    GsmCmdBuf() { clear(); }

    void clear() {
        _length = 0;
        _overflowed = false;
        _text[0] = '\0';
    }

    // Once overflowed, every later add() is a no-op and the flag stays set.
    // The failure is reported once, at the end, rather than producing a
    // plausible-looking command with a hole in the middle.
    bool add(const char* text) {
        if (text == nullptr) { return !_overflowed; }
        for (size_t i = 0; text[i] != '\0'; i++) {
            if (!addChar(text[i])) { return false; }
        }
        return true;
    }

    bool add(char character) { return addChar(character); }

    bool add(uint32_t value) {
        char digits[11];
        size_t count = 0;
        if (value == 0) { digits[count++] = '0'; }
        while (value != 0) { digits[count++] = (char)('0' + (value % 10)); value /= 10; }
        while (count > 0) { if (!addChar(digits[--count])) { return false; } }
        return true;
    }

    bool add(int32_t value) {
        if (value < 0) {
            if (!addChar('-')) { return false; }
            // Negated as unsigned so INT32_MIN does not overflow on the way.
            return add((uint32_t)(-(int64_t)value));
        }
        return add((uint32_t)value);
    }

    // Wraps in double quotes, as every AT string parameter is written.
    //
    // Does NOT escape anything, because AT has no escape mechanism inside a
    // quoted string. A hostname containing a quote is not representable, so it
    // is refused here rather than producing a command that means something
    // else - which is the whole class of injection this would otherwise be.
    //
    // CONTROL CHARACTERS TOO, NOT ONLY CR AND LF. The command line is edited
    // by the module as it arrives: backspace (S5) rubs out what came before
    // it, Ctrl-Z and ESC end or abort a prompt. A host or APN carrying them
    // could rewrite the command it sits in, and no legitimate value does.
    bool addQuoted(const char* text) {
        if (!addChar('"')) { return false; }
        if (text != nullptr) {
            for (size_t i = 0; text[i] != '\0'; i++) {
                const unsigned char c = (unsigned char)text[i];
                if (c == '"' || c < 0x20 || c == 0x7F) {
                    _overflowed = true;     // unrepresentable, so refuse the lot
                    return false;
                }
                if (!addChar(text[i])) { return false; }
            }
        }
        return addChar('"');
    }

    // EMPTY WHEN ANYTHING WAS REFUSED, and that is the safety property.
    //
    // A truncated AT command is not a shorter command - it is a DIFFERENT and
    // usually malformed one. "AT+CGDCONT=1,"IP","someverylongapn" with the
    // closing quote lost is a syntax error at best; a truncated host or number
    // is worse, because it can succeed against the wrong thing.
    //
    // The engine is handed a const char*, so it cannot ask a buffer whether it
    // overflowed - it can only see that the text is empty, which it already
    // refuses with InvalidArgument. Returning empty here is what makes that
    // one existing guard cover all twenty-one submit sites instead of only the
    // handful that remembered to check.
    const char* text() const { return _overflowed ? "" : _text; }
    size_t length() const { return _overflowed ? 0 : _length; }
    bool isEmpty() const { return _overflowed || _length == 0; }

    // True when anything was refused. Callers that want to say WHY rather
    // than just fail can still ask.
    bool overflowed() const { return _overflowed; }

private:
    bool addChar(char character) {
        if (_overflowed) { return false; }
        if (_length + 1 >= sizeof _text) { _overflowed = true; return false; }
        _text[_length++] = character;
        _text[_length] = '\0';
        return true;
    }

    char   _text[GSM_AT_COMMAND_LONG_MAX];   // the longest command built; see GsmAtEngine.h
    size_t _length = 0;
    bool   _overflowed = false;
};
