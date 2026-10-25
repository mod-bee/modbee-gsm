// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#include "GsmDriverRegistry.h"
#include "GsmNoExcept.h"

bool GsmDriverRegistry::add(GsmDriver* driver) {
    if (driver == nullptr || _count >= GSM_DRIVERS_MAX) { return false; }

    // ALREADY HERE IS NOT AN ADDITION. The registry holds a fixed number of
    // drivers, and a repeated call - a begin() run twice, a driver added by
    // both a library and its application - would quietly spend a slot on a
    // duplicate and report success. The second entry can never be selected
    // either, because select() stops at the first driver that claims the
    // module, which is the first copy.
    for (size_t i = 0; i < _count; i++) {
        if (_drivers[i] == driver) { return false; }
    }

    _drivers[_count++] = driver;
    return true;
}

GsmDriver* GsmDriverRegistry::select(const char* identification,
                                     const char* revision) const {
    const char* id = (identification != nullptr) ? identification : "";
    const char* rev = (revision != nullptr) ? revision : "";

    for (size_t i = 0; i < _count; i++) {
        if (_drivers[i]->claims(id, rev)) { return _drivers[i]; }
    }

    // NOTHING IS GUESSED. A module nobody recognises gets no driver, and the
    // modem reports that it could not identify itself rather than driving an
    // unknown part with somebody else's dialect - which would fail in ways
    // that look like a hardware fault.
    return nullptr;
}
