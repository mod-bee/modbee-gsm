// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#include "GsmClock.h"
#include "GsmModem.h"
#include "GsmNoExcept.h"

namespace { const uint16_t kKeyClock = 0xF020; }

GsmResult GsmClock::refresh() {
    if (_modem == nullptr || _modem->driver() == nullptr) {
        return GsmStatus::NotInitialised;
    }
    if (_pending) { return GsmStatus::Queued; }

    GsmCmdBuf command;
    _modem->driver()->clockCommand(command);
    if (command.isEmpty() || command.overflowed()) {
        return GsmStatus::NotSupportedByModule;
    }

    GsmCommand request;
    request.text = command.text();
    request.timeoutMs = 10000;
    request.onReply = &GsmClock::onReply;
    request.onDone = &GsmClock::onDone;
    request.context = this;

    // Superseded rather than queued twice: asking for the time again before
    // the first answer arrives wants the newer answer, not both.
    request.supersedeKey = kKeyClock;

    const GsmResult accepted = _modem->engine().submit(request);
    if (accepted.status() == GsmStatus::Queued) { _pending = true; }
    return accepted;
}

gsm_time_t GsmClock::ageMillis() const {
    if (!_has || _stamp == 0) { return 0xFFFFFFFFu; }
    // gsm_elapsed(since, now). Written the other way round, this makes every
    // age "now minus the future": 49.7 days less the true
    // age, so a time read a minute ago is reported as seven weeks old.
    return gsm_elapsed(_stamp, gsm_millis());
}

void GsmClock::onReply(void* context, const char* line, size_t length) {
    GsmClock* self = (GsmClock*)context;
    if (self->_modem == nullptr || self->_modem->driver() == nullptr) { return; }

    GsmParse::GsmDateTime parsed;
    if (self->_modem->driver()->parseClock(line, length, parsed)) {
        self->_time = parsed;
        self->_has = true;
        self->_stamp = gsm_millis();
    }
}

void GsmClock::onDone(void* context, GsmResult result) {
    (void)result;
    ((GsmClock*)context)->_pending = false;
}
