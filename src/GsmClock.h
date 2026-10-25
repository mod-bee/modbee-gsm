// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#pragma once

// Network time, from the module's own clock.
//
// The network sets that clock during registration, so it needs no NTP, no
// socket and no bearer - which makes it the only time source available to a
// device that is registered but has no data.
//
// It is NOT read automatically. Asking costs a command, the answer is only as
// good as the last thing the network said, and most applications do not want
// it. refresh() asks; time() reports what came back.

#include "GsmPlatform.h"
#include "GsmParse.h"
#include "GsmResult.h"

class GsmModem;

class GsmClock {
public:
    void begin(GsmModem* modem) { _modem = modem; }

    // Queued, non-blocking. The answer lands on a later poll().
    GsmResult refresh();

    // Whether the module has given us a time, and what it was.
    //
    // False until refresh() has completed once. An unset module clock reads as
    // a date in 1980 or 2004 depending on family, so hasTime() tracks whether
    // the answer parsed rather than whether the fields are non-zero.
    bool hasTime() const { return _has; }
    const GsmParse::GsmDateTime& time() const { return _time; }

    // Milliseconds since the reading was taken. The module's clock runs on,
    // but the OFFSET from real time only changes when the network says so, so
    // an old reading is still useful - and the age says how much to trust it.
    gsm_time_t ageMillis() const;

private:
    static void onReply(void* context, const char* line, size_t length);
    static void onDone(void* context, GsmResult result);

    GsmModem* _modem = nullptr;
    GsmParse::GsmDateTime _time;
    gsm_time_t _stamp = 0;
    bool _has = false;
    bool _pending = false;
};
