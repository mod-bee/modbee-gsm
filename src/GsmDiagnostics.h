// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#pragma once

// What an operator needs on a diagnostic page: how good the signal is, what
// the radio is set to, and what has been going wrong.
//
// Separated from GsmModem because none of it decides anything. The state
// machine does not branch on signal strength or on a counter, so keeping them
// beside the sequencing only makes the sequencing harder to read.
//
// Quality is JUDGED, not merely reported. Unknown means nobody has asked;
// Bad means asked and hopeless. An alarm list that conflates the two sends
// somebody to the wrong end of a site.

#include "GsmPlatform.h"
#include "GsmTypes.h"

class GsmDiagnostics {
public:
    // Where the dBm bands sit. Defaults are for a cellular link where -95 is
    // workable and -105 is not; a deployment with a different antenna or a
    // different expectation can move them.
    struct Thresholds {
        int badAtOrBelowDbm = -105;
        int uncertainAtOrBelowDbm = -95;
    };

    void thresholds(const Thresholds& bands) { _bands = bands; }
    const Thresholds& thresholds() const { return _bands; }

    // Take a fresh reading and judge it.
    void signal(const GsmSignal& reading) {
        _signal = reading;
        if (!_signal.hasRssi) {
            _signal.quality = GsmQuality::Unknown;
        } else if (_signal.rssiDbm <= _bands.badAtOrBelowDbm) {
            _signal.quality = GsmQuality::Bad;
        } else if (_signal.rssiDbm <= _bands.uncertainAtOrBelowDbm) {
            _signal.quality = GsmQuality::Uncertain;
        } else {
            _signal.quality = GsmQuality::Good;
        }
    }

    const GsmSignal& signal() const { return _signal; }

    GsmRadioConfig& radio() { return _radio; }
    const GsmRadioConfig& radio() const { return _radio; }

private:
    Thresholds     _bands;
    GsmSignal      _signal;
    GsmRadioConfig _radio;
};
