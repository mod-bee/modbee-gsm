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
// LOCATION
// =============================================================================
//
// Power the receiver, ask it where it is, and report the answer WITH ITS
// QUALITY AND ITS AGE.
//
// That last part is the whole design. A receiver that lost the sky five
// minutes ago keeps reporting its last position perfectly happily - so an
// application handed a bare latitude and longitude has no way to tell a live
// fix from a stale one, and a tracked asset quietly becomes one that never
// moved. The quality and the age travel with the numbers rather than beside
// them, and ageMillis() is the number an alarm should actually be built on.
//
// The two modules this library ships drivers for report position in DIFFERENT
// FIELD ORDERS and DIFFERENT COORDINATE FORMATS - one in NMEA degrees-minutes
// with the hemisphere in its own field, the other in signed decimal degrees.
// Neither is converted here; each driver parses its own and hands back the
// same structure.

#include "GsmPlatform.h"
#include "GsmModem.h"

class GsmGnss {
public:
    void begin(GsmModem* modem);

    // Turns the receiver on or off. Queued; hasFix() says when it found
    // something. A cold start with no almanac can legitimately take minutes.
    GsmResult power(bool on);

    // Whether this MODULE actually has a receiver, as opposed to whether its
    // FAMILY generally does.
    //
    // The profile's hasGnss is a family fact, and that is not fine-grained
    // enough: an A7670G answers ERROR to every GNSS command including the
    // test forms, while other parts driven by the same driver answer them
    // happily. Its own datasheet lists LBS - network location from cell
    // towers - and no GNSS at all.
    //
    // So this is latched from what the module SAYS the first time it is
    // asked, which is the house rule applied one level further down than the
    // driver: detected at run time, never assumed from a family name.
    bool isSupported() const;
    bool isPowered() const { return _powered; }

    // Asks for the current fix. Superseded, so polling every scan costs one
    // query rather than a backlog.
    GsmResult refresh();

    bool hasFix() const { return _fix.hasPosition && _fix.quality == GsmQuality::Good; }
    const GsmFix& fix() const { return _fix; }

    // How long ago this library last accepted a fix, in milliseconds.
    //
    // THE NUMBER TO BUILD AN ALARM ON. A position with no age attached cannot
    // be judged, and every GNSS receiver ever made will hand you an old one
    // without complaint.
    gsm_time_t ageMillis() const;

private:
    static void onReply(void* context, const char* line, size_t length);
    static void onPowerDone(void* context, GsmResult result);

    GsmModem* _modem = nullptr;
    GsmFix    _fix;
    // Latched when the module refuses its own GNSS power command. Three-state
    // in effect: untested, supported, refused - so a single transient error
    // does not permanently disable a receiver that is merely busy.
    bool _refused = false;
    bool      _powered = false;
    bool      _wantPowered = false;
};
