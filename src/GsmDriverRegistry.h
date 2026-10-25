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
// WHICH MODEM IS THIS?
// =============================================================================
//
// Asked at run time, from what the module says about itself. NEVER with the
// preprocessor.
//
// That one decision is the largest single difference between this library and
// the one everybody uses. TinyGSM selects its module with a #define before the
// include, so one firmware drives one family chosen at compile time - and a
// host build that adapts to whatever is on /dev/ttyUSB0 cannot be written at
// all. Here every registered driver is linked, the module is asked, and the
// one that recognises it is installed. A board that only ever meets one module
// registers only that one, and the linker drops the rest.
//
// --- IDENTIFICATION AND REVISION, NOT JUST IDENTIFICATION ---
//
// Two facts in a profile vary by FIRMWARE rather than by part: a TLS session
// count, and whether a socket closure is pushed as a URC or must be polled. So
// claims() is given both, and a driver may refuse a revision it knows it
// cannot drive.

#include "GsmDriver.h"

#ifndef GSM_DRIVERS_MAX
#define GSM_DRIVERS_MAX 6
#endif

class GsmDriverRegistry {
public:
    // Drivers are the caller's and must outlive the registry. Order matters
    // only as a tie-break: the first that claims a module wins, so a specific
    // driver should be registered before a general one.
    bool add(GsmDriver* driver);

    // The driver that recognises this module, or null.
    //
    // Either argument may be empty - a module that refused the query still
    // gets offered, because a driver may recognise it by the other.
    GsmDriver* select(const char* identification, const char* revision) const;

    size_t count() const { return _count; }
    GsmDriver* at(size_t index) const {
        return index < _count ? _drivers[index] : nullptr;
    }

    void clear() { _count = 0; }

private:
    GsmDriver* _drivers[GSM_DRIVERS_MAX] = {nullptr};
    size_t _count = 0;
};
