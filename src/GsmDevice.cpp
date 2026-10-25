// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#include "GsmDevice.h"

#include <new>
#include <type_traits>

#include "GsmNoExcept.h"

namespace {

void copyBounded(char* destination, size_t capacity, const char* source) {
    size_t i = 0;
    if (source != nullptr) {
        for (; i + 1 < capacity && source[i] != '\0'; i++) { destination[i] = source[i]; }
    }
    destination[i] = '\0';
}

// Whether `source` fits a buffer of `capacity` with its terminator.
bool fits(const char* source, size_t capacity) {
    if (source == nullptr) { return true; }
    size_t length = 0;
    while (source[length] != '\0') {
        if (++length >= capacity) { return false; }
    }
    return true;
}

} // namespace

GsmDevice::GsmDevice() {
    // Both shipped drivers, always compiled and always registered.
    //
    // NOT selected with the preprocessor, which is the single decision that
    // makes a library like this usable on a host: one binary can drive
    // whichever module is actually plugged in, and a board that only ever
    // meets one pays for the other in an unreferenced class the linker drops.
    _registry.add(&_a76xx);
    _registry.add(&_sim707x);
}

bool GsmDevice::begin(GsmTransport* transport) {
    // The storage first, so a board that cannot hold what was asked for is
    // told so by name, before the link is touched.
    _beginError = nullptr;
    if (!_sockets.reserve()) {
        _beginError = "out of memory for the sockets";
        return false;
    }
    if (!_modem.engine().reserve()) {
        _beginError = "out of memory for the AT command queue";
        return false;
    }
    if (!_modem.begin(transport, &_registry, _power)) { return false; }

    _modem.setApn(_apn);
    _modem.onEvent(&GsmDevice::onModemEvent, this);
    _sockets.begin(&_modem);
    if (_sms != nullptr) { _sms->begin(&_modem); }
    _gnss.begin(&_modem);
    _clock.begin(&_modem);
    _modem.start();
    return true;
}

bool GsmDevice::apn(const char* name, const char* user, const char* password,
                    GsmAuth auth) {
    if (!fits(name, sizeof _apnName) || !fits(user, sizeof _apnUser) ||
        !fits(password, sizeof _apnPassword)) {
        return false;
    }
    // Copied, because a sketch may pass a temporary and the bearer is brought
    // up minutes later - and brought up again on every reconnect after that.
    copyBounded(_apnName, sizeof _apnName, name);
    copyBounded(_apnUser, sizeof _apnUser, user);
    copyBounded(_apnPassword, sizeof _apnPassword, password);

    _apn.name = _apnName;
    _apn.user = (_apnUser[0] != '\0') ? _apnUser : nullptr;
    _apn.password = (_apnPassword[0] != '\0') ? _apnPassword : nullptr;
    _apn.auth = auth;
    _modem.setApn(_apn);
    return true;
}

void GsmDevice::poll() {
    _modem.poll();

    // SMS INDICATIONS ARE KEPT ARMED FOR AS LONG AS SOMEBODY LISTENS.
    //
    // AT+CNMI is not saved by the module, and neither is PDU mode. onSms()
    // used to configure them once - or not at all, if it was called before
    // the module had been identified - and after the module restarted itself
    // nothing asked again: an application subscribed for messages simply
    // stopped receiving them, while every status it could read said ready.
    //
    // So whenever the modem is Ready and the SMS layer is not configured for
    // THIS life of the module, it is configured - see smsPoll() below. One
    // check per poll; the commands go out once per module epoch, and not at
    // all for a device that never subscribed. Through a pointer set when the
    // SMS layer is attached, so this always-linked path names no SMS code.
    if (_smsPoll != nullptr) { _smsPoll(_sms, _modem); }

    _sockets.poll();
}

void GsmDevice::onModemEvent(void* context, GsmUrcKind kind, const GsmUrcInfo& info) {
    // The socket layer needs to know about data, closures and bearer loss;
    // the SMS layer needs to know a message arrived. Routed through here
    // rather than wired directly so that an application may also subscribe
    // without displacing either.
    GsmDevice* self = (GsmDevice*)context;
    self->_sockets.handleEvent(kind, info);
    if (self->_smsEvent != nullptr) { self->_smsEvent(self->_sms, kind, info); }
}

namespace {
void smsEvent(GsmSms* sms, GsmUrcKind kind, const GsmUrcInfo& info) {
    sms->handleEvent(kind, info);
}

void smsPoll(GsmSms* sms, const GsmModem& modem) {
    if (sms->hasSubscriber() && modem.isReady() && !sms->isReady()) { sms->configure(); }
}
}  // namespace

GsmSms& GsmDevice::smsPart() {
    static_assert(std::is_trivially_destructible<GsmSms>::value,
                  "GsmDevice never destroys the SMS layer it attached");
    if (_sms == nullptr) {
        _sms = new (_smsStorage) GsmSms();
        _sms->begin(&_modem);
        _smsEvent = &smsEvent;
        _smsPoll = &smsPoll;
    }
    return *_sms;
}

GsmResult GsmDevice::sendSms(const char* number, const char* text,
                             GsmDoneFn done, void* context) {
    // Configured on first use. A device that never sends a message never
    // spends a command on setting one up.
    GsmSms& sms = smsPart();
    if (!sms.isReady()) { sms.configure(); }
    return sms.send(number, text, done, context);
}

GsmResult GsmDevice::sendAt(const char* command, uint32_t timeoutMs,
                            GsmReplyFn onReply, GsmDoneFn onDone, void* context) {
    GsmCommand request;
    request.text = command;
    request.timeoutMs = timeoutMs;
    request.onReply = onReply;
    request.onDone = onDone;
    request.context = context;
    return _modem.engine().submit(request);
}
