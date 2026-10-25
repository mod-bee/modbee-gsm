// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#include "GsmClient.h"
#include "GsmNoExcept.h"

#if defined(GSM_PLATFORM_ARDUINO) || defined(GSM_ARDUINO_SHIM)

void GsmClient::renderAddress(const IPAddress& ip, char* out, size_t capacity) {
    // Four octets and three dots, longest "255.255.255.255" - sixteen bytes
    // with the terminator, which is what the caller supplies.
    // Each octet needs room for up to three digits, its dot (all but the
    // last), and the terminator - checked per octet, so "255.255.255.255"
    // fits sixteen bytes exactly and the fourth octet is never dropped.
    if (capacity == 0) { return; }
    size_t position = 0;
    for (int octet = 0; octet < 4 && position + (octet < 3 ? 5u : 4u) <= capacity;
         octet++) {
        const uint8_t value = ip[octet];
        if (value >= 100) { out[position++] = (char)('0' + value / 100); }
        if (value >= 10)  { out[position++] = (char)('0' + (value / 10) % 10); }
        out[position++] = (char)('0' + value % 10);
        if (octet < 3) { out[position++] = '.'; }
    }
    out[position] = '\0';
}

int GsmClient::connect(const char* host, uint16_t port, int timeoutSeconds) {
    stop();
    _failure = GsmStatus::Ok;

    if (_sockets == nullptr) { return 0; }
    // WHAT THIS CLIENT WAS BUILT TO ASK FOR. Once hardcoded to Plain, which
    // made module TLS unreachable from the Arduino surface - the one surface
    // most people use.
    _socket = _sockets->open(host, port, GsmSocketKind::Tcp, _security);
    if (_socket == nullptr) {
        GsmModem* modem = _sockets->modem();
        _failure = (modem == nullptr || modem->driver() == nullptr)
            ? GsmResult(GsmStatus::NotReady)
            : GsmResult(GsmStatus::NoSocketAvailable);
        return 0;
    }

    // EVERY FAILURE BELOW HANDS THE SLOT BACK. open() reserved one, and only
    // end() releases it: a socket that failed is still in use until somebody
    // ends it. These paths used to drop the pointer instead, so each failed
    // connect() leaked a slot - and with GSM_SOCKET_SLOTS at 2, the second
    // connect attempted before the bearer was up left the client unable to
    // open anything again until the board restarted. An MQTT client retrying
    // on a backoff makes exactly those attempts.
    if (!_socket->begin()) { release(); return 0; }

    // THE ONE BLOCKING CALL, AND IT IS BOUNDED.
    //
    // Arduino's Client::connect() must return success or failure, so there is
    // nowhere to put "still trying". This pumps the library until the socket
    // resolves or the deadline passes - it never waits on the network without
    // making progress, and it never waits without an end.
    //
    // An application that cannot afford this uses GsmSocket::begin() directly,
    // which starts the open and returns immediately.
    GsmModem* modem = _sockets->modem();
    const gsm_time_t deadline =
        (gsm_time_t)(gsm_millis() + (uint32_t)timeoutSeconds * 1000u);

    while (gsm_isFuture(gsm_millis(), deadline)) {
        modem->poll();
        _sockets->poll();

        if (_socket->state() == GsmSocketState::Open) { return 1; }
        if (_socket->state() == GsmSocketState::Failed) { release(); return 0; }

        gsm_yield();
    }

    _socket->end();
    _socket = nullptr;
    _failure = GsmStatus::Timeout;
    return 0;
}

void GsmClient::release() {
    // The reason is kept, because end() clears it with the slot.
    _failure = _socket->lastError();
    _socket->end();
    _socket = nullptr;
}

size_t GsmClient::write(const uint8_t* buffer, size_t size) {
    if (_socket == nullptr) { return 0; }

    // Non-blocking, and honest about it. A short write is ordinary here for
    // the same reason it is on a socket whose kernel buffer is full, and
    // callers written against Arduino's Client already handle it.
    const size_t taken = _socket->write(buffer, size);

    // Service once so the bytes start moving rather than waiting for the
    // application's next poll. This does not wait for them to arrive.
    if (taken > 0 && _sockets != nullptr) {
        _sockets->modem()->poll();
        _sockets->poll();
    }
    return taken;
}

int GsmClient::available() {
    if (_socket == nullptr) { return _peeked >= 0 ? 1 : 0; }

    // Pumped here because a caller polling available() in a loop is the
    // commonest shape in Arduino networking code, and nothing else in that
    // loop would drive the modem.
    _sockets->modem()->poll();
    _sockets->poll();

    const size_t waiting = _socket->readable();
    return (int)(waiting + (_peeked >= 0 ? 1 : 0));
}

int GsmClient::read() {
    if (_peeked >= 0) {
        const int value = _peeked;
        _peeked = -1;
        return value;
    }
    if (_socket == nullptr) { return -1; }

    uint8_t byte = 0;
    return (_socket->read(&byte, 1) == 1) ? (int)byte : -1;
}

int GsmClient::read(uint8_t* buffer, size_t size) {
    if (buffer == nullptr || size == 0) { return 0; }

    size_t taken = 0;
    if (_peeked >= 0) {
        buffer[taken++] = (uint8_t)_peeked;
        _peeked = -1;
    }
    if (_socket != nullptr && taken < size) {
        taken += _socket->read(buffer + taken, size - taken);
    }
    return (int)taken;
}

int GsmClient::peek() {
    if (_peeked >= 0) { return _peeked; }
    const int value = read();
    if (value >= 0) { _peeked = value; }
    return value;
}

void GsmClient::flush() {
    if (_socket == nullptr || _sockets == nullptr) { return; }

    // Arduino's flush() means "get the outbound data onto the wire", and every
    // core implements it by waiting. Bounded here by the same reasoning as
    // connect(): a wait with no end is a hang, and a modem that has gone will
    // never drain.
    const gsm_time_t deadline = (gsm_time_t)(gsm_millis() + 10000);
    while (_socket->pendingOut() > 0 && gsm_isFuture(gsm_millis(), deadline)) {
        _sockets->modem()->poll();
        _sockets->poll();
        gsm_yield();
    }
}

void GsmClient::stop() {
    _peeked = -1;
    if (_socket != nullptr) {
        _socket->end();
        _socket = nullptr;
    }
}

uint8_t GsmClient::connected() {
    if (_socket == nullptr) { return _peeked >= 0 ? 1 : 0; }

    // A peer that closed while bytes were still buffered is STILL CONNECTED as
    // far as a caller is concerned, because those bytes are the answer it was
    // waiting for. Reporting the close first is how the last reply of every
    // exchange that ends in a hang-up gets thrown away - which is most HTTP
    // responses without a content length.
    if (_socket->readable() > 0 || _peeked >= 0) { return 1; }
    return _socket->isConnected() ? 1 : 0;
}

#endif // Arduino or shim
