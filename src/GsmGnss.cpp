// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#include "GsmGnss.h"
#include "GsmCmdBuf.h"
#include "GsmNoExcept.h"

namespace {
// The supersede key for the position query, in this file's own namespace.
const uint16_t kKeyFix = 0xF010;
} // namespace

void GsmGnss::begin(GsmModem* modem) {
    _modem = modem;
    _fix = GsmFix();
    _powered = false;
}

GsmResult GsmGnss::power(bool on) {
    if (_modem == nullptr || _modem->driver() == nullptr) {
        return GsmStatus::NotInitialised;
    }
    if (!_modem->driver()->profile().hasGnss || _refused) {
        // Refused by name rather than attempted. A module without a receiver
        // answers an error that reads like a fault, and the application would
        // retry it forever.
        //
        // _refused is the same answer learned from the module rather than
        // from the profile - see isSupported(). The profile says whether the
        // FAMILY has a receiver; only the module knows whether THIS part does.
        return GsmStatus::NotSupportedByModule;
    }

    GsmCmdBuf command;
    _modem->driver()->gnssPowerCommand(on, command);
    if (command.isEmpty() || command.overflowed()) {
        return GsmStatus::NotSupportedByModule;
    }

    _wantPowered = on;

    GsmCommand request;
    request.text = command.text();
    request.timeoutMs = 15000;
    request.onDone = &GsmGnss::onPowerDone;
    request.context = this;
    return _modem->engine().submit(request);
}

bool GsmGnss::isSupported() const {
    if (_modem == nullptr || _modem->driver() == nullptr) { return false; }
    return _modem->driver()->profile().hasGnss && !_refused;
}

void GsmGnss::onPowerDone(void* context, GsmResult result) {
    GsmGnss* self = (GsmGnss*)context;

    // A MODULE THAT REFUSES ITS OWN POWER COMMAND HAS NO RECEIVER.
    //
    // Latched so the application is told once, plainly, instead of being left
    // to wait for a fix that can never arrive.
    //
    // IT IS THE ORDER CODE THAT DECIDES, NOT THE FAMILY OR EVEN THE PART.
    // SIMCom sells A76XX parts with and without a receiver and distinguishes
    // them only by the suffix - "-F..." carries GNSS, "-L..." does not. An
    // A7670G-LABE therefore answers ERROR to every GNSS command including the
    // TEST forms, while an A7670G-F... of the same family, same driver and
    // same firmware line answers them happily.
    //
    // No table could get this right, and a table keyed on the part number
    // would get it confidently wrong. Asking the module is the only method
    // that works, which is why this is latched from its answer.
    //
    // Only an outright refusal latches. A timeout does not, because a module
    // that was merely busy deserves another chance.
    if (result.status() == GsmStatus::Error ||
        result.status() == GsmStatus::NotSupportedByModule) {
        self->_refused = true;
        self->_powered = false;
        self->_fix = GsmFix();
        return;
    }

    if (result.isOk()) {
        self->_powered = self->_wantPowered;
        if (!self->_powered) {
            // Powered down: the last fix is now history, and saying so is the
            // point of carrying quality at all.
            self->_fix.quality = GsmQuality::Unknown;
            self->_fix.hasPosition = false;
        }
    }
}

GsmResult GsmGnss::refresh() {
    if (_modem == nullptr || _modem->driver() == nullptr) {
        return GsmStatus::NotInitialised;
    }
    if (!_powered) { return GsmStatus::NotReady; }

    GsmCmdBuf command;
    _modem->driver()->gnssInfoCommand(command);
    if (command.isEmpty() || command.overflowed()) {
        return GsmStatus::NotSupportedByModule;
    }

    GsmCommand request;
    request.text = command.text();
    request.timeoutMs = 10000;
    request.onReply = &GsmGnss::onReply;
    request.context = this;
    request.supersedeKey = kKeyFix;
    return _modem->engine().submit(request);
}

void GsmGnss::onReply(void* context, const char* line, size_t length) {
    GsmGnss* self = (GsmGnss*)context;
    if (self->_modem->driver() == nullptr) { return; }

    GsmFix parsed;
    if (!self->_modem->driver()->parseGnssInfo(line, length, parsed)) { return; }

    if (parsed.hasPosition) {
        self->_fix = parsed;
        return;
    }

    // NO FIX IS AN ANSWER, and it must not leave the previous position
    // standing as though it were current. The coordinates are kept - they are
    // still the last known position, which is useful - but the quality says
    // they are no longer being confirmed.
    self->_fix.quality = GsmQuality::Bad;
    self->_fix.hasPosition = false;
}

gsm_time_t GsmGnss::ageMillis() const {
    if (_fix.stampMillis == 0) { return 0xFFFFFFFFu; }   // never had one
    return gsm_elapsed(_fix.stampMillis, gsm_millis());
}
