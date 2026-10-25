// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#include "GsmUDP.h"
#include "GsmNoExcept.h"

#if defined(GSM_PLATFORM_ARDUINO) || defined(GSM_ARDUINO_SHIM)

namespace {

void copyBounded(char* destination, size_t capacity, const char* source) {
    size_t i = 0;
    if (source != nullptr) {
        for (; i + 1 < capacity && source[i] != '\0'; i++) { destination[i] = source[i]; }
    }
    destination[i] = '\0';
}

} // namespace

GsmUDP::~GsmUDP() {
    stop();
    gsm_freeLarge(_out);
}

uint8_t GsmUDP::begin(uint16_t port) {
    stop();
    _localPort = port;

    if (_sockets == nullptr) { return 0; }

    // Both datagram buffers, once, in one piece. No room is a refusal now,
    // never a smaller packet later.
    if (_out == nullptr) {
        if (_packetBytes > ((size_t)-1) / 2) { return 0; }
        _out = (uint8_t*)gsm_allocLarge(_packetBytes * 2);
        if (_out == nullptr) { return 0; }
        _in = _out + _packetBytes;
    }

    // A datagram socket is BOUND rather than connected: the local port is what
    // matters and the destination travels with each packet. One module's open
    // command reflects that exactly - its UDP form takes the local port and
    // leaves the server fields empty.
    _socket = _sockets->open("", port, GsmSocketKind::Udp, GsmSecurity::Plain);
    if (_socket == nullptr) { return 0; }
    return _socket->begin() ? 1 : 0;
}

void GsmUDP::stop() {
    if (_socket != nullptr) {
        _socket->end();
        _socket = nullptr;
    }
    _outCount = 0;
    _inCount = 0;
    _inRead = 0;
    _building = false;
}

int GsmUDP::beginPacket(const char* host, uint16_t port) {
    if (_socket == nullptr) { return 0; }
    copyBounded(_target, sizeof _target, host);
    _targetPort = port;
    _outCount = 0;
    _building = true;
    return 1;
}

int GsmUDP::beginPacket(IPAddress ip, uint16_t port) {
    char text[16];
    size_t position = 0;
    for (int octet = 0; octet < 4; octet++) {
        const uint8_t value = ip[octet];
        if (value >= 100) { text[position++] = (char)('0' + value / 100); }
        if (value >= 10)  { text[position++] = (char)('0' + (value / 10) % 10); }
        text[position++] = (char)('0' + value % 10);
        if (octet < 3) { text[position++] = '.'; }
    }
    text[position] = '\0';
    return beginPacket(text, port);
}

size_t GsmUDP::write(const uint8_t* buffer, size_t size) {
    if (!_building || buffer == nullptr) { return 0; }

    const size_t room = _packetBytes - _outCount;
    const size_t take = (size < room) ? size : room;
    for (size_t i = 0; i < take; i++) { _out[_outCount + i] = buffer[i]; }
    _outCount += take;
    return take;
}

int GsmUDP::endPacket() {
    if (!_building || _socket == nullptr) { return 0; }
    _building = false;

    // ONE DATAGRAM IN THE SOCKET AT A TIME. While the last one is still
    // waiting to go - the socket busy with a fetch, the queue backed up - this
    // one is refused, and endPacket() returns 0 as Arduino's UDP contract
    // allows. Accepting it used to append it to the first and retarget both:
    // one merged datagram, delivered to the second address only.
    if (_socket->pendingOut() != 0) {
        _outCount = 0;
        return 0;
    }

    // The destination is per-packet, so it is applied to the socket before the
    // send. A connected stream would not need this; a datagram socket does,
    // and it is why UDP is not a flag on the TCP path.
    if (!_socket->setDatagramTarget(_target, _targetPort)) {
        _outCount = 0;
        return 0;
    }

    // Sent WHOLE or not at all. A datagram that went out in two pieces is two
    // datagrams, and the far end would read them as such - so a payload that
    // does not fit one send is refused rather than split.
    const size_t written = _socket->write(_out, _outCount);
    const bool complete = (written == _outCount);
    _outCount = 0;

    if (_sockets != nullptr) {
        _sockets->modem()->poll();
        _sockets->poll();
    }
    return complete ? 1 : 0;
}

int GsmUDP::parsePacket() {
    if (_socket == nullptr) { return 0; }

    _sockets->modem()->poll();
    _sockets->poll();

    if (_inRead < _inCount) { return (int)(_inCount - _inRead); }

    _inCount = 0;
    _inRead = 0;

    const size_t waiting = _socket->readable();
    if (waiting == 0) { return 0; }

    const size_t take = (waiting < _packetBytes) ? waiting : _packetBytes;
    _inCount = _socket->read(_in, take);
    return (int)_inCount;
}

int GsmUDP::available() { return (int)(_inCount - _inRead); }

int GsmUDP::read() {
    if (_inRead >= _inCount) { return -1; }
    return _in[_inRead++];
}

int GsmUDP::read(uint8_t* buffer, size_t size) {
    if (buffer == nullptr) { return 0; }
    size_t taken = 0;
    while (taken < size && _inRead < _inCount) { buffer[taken++] = _in[_inRead++]; }
    return (int)taken;
}

int GsmUDP::peek() {
    if (_inRead >= _inCount) { return -1; }
    return _in[_inRead];
}

void GsmUDP::flush() {
    _inCount = 0;
    _inRead = 0;
}

#endif // Arduino or shim
