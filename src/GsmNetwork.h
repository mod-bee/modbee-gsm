// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#pragma once

// Getting onto a network, and what happened when it did not work.
//
// Holds the registration state, what the application asked the radio to do,
// and the ESCALATION LADDER used when automatic selection never settles:
//
//   1. clear the SIM's forbidden-network list   - cheap, and survives a power
//                                                 cycle, so it goes first
//   2. scan, and try each operator on air in turn
//   3. release back to automatic and report a refusal
//
// Every manual selection is temporary and always released, so this is MORE
// connecting than leaving it automatic, not a pin to one network.
//
// GsmModem still owns the sequencing - this owns the state and decides what
// the next rung is.

#include "GsmPlatform.h"
#include "GsmTypes.h"
#include "GsmPowerControl.h"   // GsmTristate

class GsmNetwork {
public:
    // What the escalation should do next. GsmModem turns these into commands.
    enum class Escalation : uint8_t {
        ClearForbidden = 0,   // rung 1
        Scan,                 // rung 2
        TryOperator,          // rung 3, repeated
        GiveUp,               // released to automatic, reported as a refusal
    };

    // --- What the application asked for --------------------------------
    void preference(GsmRadioPreference wanted) { _preference = wanted; }
    GsmRadioPreference preference() const { return _preference; }

    void operatorCode(const char* numeric) { _operatorCode = numeric; }
    const char* operatorCode() const { return _operatorCode; }

    void bands(const GsmBandList& catM, const GsmBandList& nbIot) {
        _catM = catM;
        _nbIot = nbIot;
    }
    const GsmBandList& catMBands() const { return _catM; }
    const GsmBandList& nbIotBands() const { return _nbIot; }

    // --- What the network said -----------------------------------------
    void registration(GsmRegistration state) {
        // A better answer never loses to a worse one inside one rotation of
        // CEREG, CGREG and CREG: being told "searching" by the third does not
        // undo the first having said "home".
        if (gsm_isRegistered(state) || !gsm_isRegistered(_registration)) {
            _registration = state;
        }
        if (state == GsmRegistration::Denied) { _denials++; _everDenied = true; }
    }
    GsmRegistration registration() const { return _registration; }
    bool isRegistered() const { return gsm_isRegistered(_registration); }

    // --- Which register said what ---------------------------------------
    //
    // The answer above is the BEST one, deliberately sticky, because it is
    // what decides that registration has been reached. It cannot also decide
    // that registration has been LOST: a sticky "home" never loses to
    // "searching", so a module that dropped off its cell went on reading as
    // registered for ever, and Ready - which promised to drop back to
    // Registering - never did.
    //
    // So each register's latest word is kept on its own as well, and losing
    // registration is judged from these.
    enum : uint8_t {
        kRegisterEps = 0,     // +CEREG, 3GPP TS 27.007 10.1.20
        kRegisterPacket,      // +CGREG, 3GPP TS 27.007 10.1.19
        kRegisterCircuit,     // +CREG,  3GPP TS 27.007 7.2
        kRegisterCount,
    };

    void report(uint8_t which, GsmRegistration state) {
        if (which < kRegisterCount) { _reported[which] = state; }
    }
    GsmRegistration reported(uint8_t which) const {
        return which < kRegisterCount ? _reported[which]
                                      : GsmRegistration::NotRegisteredNotSearching;
    }

    // Whether either PACKET register still says registered.
    //
    // ONLY THE TWO THAT ANNOUNCE THEMSELVES. Both drivers ask for unsolicited
    // +CEREG and +CGREG and never for +CREG, so CREG's last answer is whatever
    // a poll during Registering happened to hear, and it can be an hour old.
    // Letting it vote would let a stale "registered" outvote a fresh "lost".
    // It is also the circuit-switched register, which carries no data - and
    // data is what this library brings a module up for.
    //
    // EITHER ONE IS ENOUGH. An LTE-only module carries everything on EPS and
    // may leave GPRS at "not registered" permanently; a module that fell back
    // to 2G is the other way round. Losing the one that is carrying traffic
    // only counts once the other is not carrying it either.
    bool packetRegistered() const {
        return gsm_isRegistered(_reported[kRegisterEps]) ||
               gsm_isRegistered(_reported[kRegisterPacket]);
    }

    // The network has taken registration away. Not through the sticky setter,
    // which would refuse: this is not a worse answer inside one rotation, it
    // is the news. Counted as a denial if it was one.
    void lose(GsmRegistration state) {
        _registration = state;
        if (state == GsmRegistration::Denied) { _denials++; _everDenied = true; }
    }

    void technology(GsmTechnology what) { _technology = what; }
    GsmTechnology technology() const { return _technology; }

    GsmTristate hasForbiddenNetworks() const { return _forbidden; }
    void forbiddenNetworks(bool any) {
        _forbidden = any ? GsmTristate::Yes : GsmTristate::No;
    }

    GsmOperatorList& operators() { return _operators; }
    uint8_t denials() const { return _denials; }

    // Whether the network has refused us AT ANY POINT this bring-up, not just
    // since the last rung. The running count is cleared as the ladder climbs;
    // this is what decides whether the ending is a refusal or a timeout - and
    // those send whoever reads them to different places.
    bool everDenied() const { return _everDenied; }
    void clearDenials() { _denials = 0; }

    // --- The escalation ladder -----------------------------------------
    Escalation nextEscalation() {
        if (!_forbiddenCleared) { _forbiddenCleared = true; return Escalation::ClearForbidden; }
        if (!_scanned) { _scanned = true; _scanIndex = 0; return Escalation::Scan; }
        if (_scanIndex < _operators.count) { return Escalation::TryOperator; }
        return Escalation::GiveUp;
    }

    // Valid immediately after nextEscalation() answered TryOperator.
    const char* takeNextOperator() {
        if (_scanIndex >= _operators.count) { return nullptr; }
        return _operators.codes[_scanIndex++];
    }

    bool anyOperatorsFound() const { return _operators.count > 0; }

    // Which registration register the next poll should ask. Rotates, because
    // no established modem library stakes registration on a single one.
    uint8_t nextRegisterQuery() { return _query++; }
    void wrapRegisterQuery() { _query = 0; }

    // Back to the bottom of the ladder, keeping what is known about the SIM.
    //
    // THE LADDER IS PER ATTEMPT, NOT PER PROCESS. It used to be climbed once
    // and then remembered: a bring-up that ended in a refusal left "cleared"
    // and "scanned" set, so every later bring-up found nothing left to try and
    // gave up on its first registration window. A SIM whose forbidden list
    // filled up again an hour later would never have it emptied again.
    void restartLadder() {
        _forbiddenCleared = false;
        _scanned = false;
        _scanIndex = 0;
        _denials = 0;
        _everDenied = false;
        _query = 0;
        _operators.clear();
    }

    // Everything the network told us, forgotten. For a new bring-up and for a
    // module that restarted underneath us: what it believed about the network
    // went with it, and so must what we believed about it.
    void reset() {
        _registration = GsmRegistration::NotRegisteredNotSearching;
        _technology = GsmTechnology::Unknown;
        _forbidden = GsmTristate::Unknown;
        for (uint8_t i = 0; i < kRegisterCount; i++) {
            _reported[i] = GsmRegistration::NotRegisteredNotSearching;
        }
        restartLadder();
    }

private:
    GsmRadioPreference _preference = GsmRadioPreference::Automatic;
    const char*        _operatorCode = nullptr;
    GsmBandList        _catM;
    GsmBandList        _nbIot;

    GsmRegistration _registration = GsmRegistration::NotRegisteredNotSearching;
    GsmTechnology   _technology = GsmTechnology::Unknown;
    GsmTristate     _forbidden = GsmTristate::Unknown;
    GsmRegistration _reported[kRegisterCount] = {
        GsmRegistration::NotRegisteredNotSearching,
        GsmRegistration::NotRegisteredNotSearching,
        GsmRegistration::NotRegisteredNotSearching,
    };

    GsmOperatorList _operators;
    uint8_t _scanIndex = 0;
    uint8_t _denials = 0;
    uint8_t _query = 0;
    bool    _forbiddenCleared = false;
    bool    _scanned = false;
    bool    _everDenied = false;
};
