// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#pragma once

// The data context: which APN, whether it is up, and what address it got.
//
// REGISTERED AND HAVING A BEARER ARE DIFFERENT THINGS, and conflating them is
// what "full bars, no internet" looks like from the inside. Registration is
// the network agreeing to talk to the module; a bearer is a PDP context
// carrying packets. One can be true for minutes while the other is not.
//
// Holds no sequencing. GsmModem walks the driver's bearerUpStep/bearerDownStep
// and tells this what happened.

#include "GsmPlatform.h"
#include "GsmTypes.h"

class GsmBearer {
public:
    void apn(const GsmApn& wanted) { _apn = wanted; }
    const GsmApn& apn() const { return _apn; }

    bool isUp() const { return _up; }
    // Each bearer brought up is a NEW one, numbered. A socket remembers the
    // number it was opened on, so one opened on a bearer that has since gone -
    // down and back up between two looks, or down by a path that sent no
    // event - is known to be dead without anyone having to say so.
    void markUp() {
        if (!_up) { _epoch++; }
        _up = true;
    }
    uint32_t epoch() const { return _epoch; }

    // Returns true if this was a LOSS rather than a no-op, so the caller can
    // count it. A bearer that was already down has not just gone.
    bool markDown() {
        const bool wasUp = _up;
        _up = false;
        _address[0] = '\0';
        return wasUp;
    }

    const char* address() const { return _address; }

    void address(const char* text, size_t length) {
        if (text == nullptr) { return; }
        size_t i = 0;
        while (i + 1 < sizeof _address && i < length && text[i] != '\0') {
            _address[i] = text[i];
            i++;
        }
        _address[i] = '\0';
    }

    void reset() { _up = false; _address[0] = '\0'; }

private:
    GsmApn _apn;
    char   _address[46] = {0};      // long enough for IPv6 in full
    bool   _up = false;
    uint32_t _epoch = 0;
};
