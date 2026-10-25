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
// Arduino's UDP, over a modem
// =============================================================================
//
// A SEPARATE CLASS, NOT A FLAG ON GsmClient, and that is a deliberate design
// decision rather than an accident of inheritance.
//
// Datagrams have boundaries, no ordering and no connection. Squeezing them
// through a stream interface loses the boundaries, and a 512-byte datagram
// arriving as reads of 300 and 212 is not UDP any more - it is a stream that
// happens to have been carried by datagrams, which is a different and much
// less useful thing. So beginPacket/endPacket frame a send, and parsePacket
// yields exactly one datagram at a time.
//
// Drop-in for WiFiUDP and EthernetUDP, so NTP clients, syslog senders and
// anything else written against Arduino's UDP works over cellular unchanged.

#include "GsmPlatform.h"
#include "GsmSocket.h"

#if defined(GSM_PLATFORM_ARDUINO) || defined(GSM_ARDUINO_SHIM)

#if defined(GSM_ARDUINO_SHIM)
    #include "GsmArduinoShim.h"
#else
    #include <Arduino.h>
    #include <Udp.h>
    #include <IPAddress.h>
#endif

// The largest datagram, each way, BY DEFAULT. The constructor takes another.
#ifndef GSM_UDP_PACKET_MAX
#define GSM_UDP_PACKET_MAX 512
#endif

class GsmUDP : public UDP {
public:
    // `packetBytes` is the largest datagram sent or received; 0 is
    // GSM_UDP_PACKET_MAX. Its two buffers are taken by begin(), which returns
    // 0 when there is no room for them rather than carrying on smaller.
    explicit GsmUDP(GsmSockets& sockets, size_t packetBytes = GSM_UDP_PACKET_MAX)
        : _sockets(&sockets), _packetBytes(packetBytes != 0 ? packetBytes : GSM_UDP_PACKET_MAX) {}
    ~GsmUDP();
    GsmUDP(const GsmUDP&) = delete;
    GsmUDP& operator=(const GsmUDP&) = delete;

    uint8_t begin(uint16_t port) override;
    void stop() override;

    int beginPacket(const char* host, uint16_t port) override;
    int beginPacket(IPAddress ip, uint16_t port) override;
    int endPacket() override;

    size_t write(uint8_t value) override { return write(&value, 1); }
    size_t write(const uint8_t* buffer, size_t size) override;

    int parsePacket() override;
    int available() override;
    int read() override;
    int read(uint8_t* buffer, size_t size) override;

    // ARDUINO'S UDP DECLARES BOTH SPELLINGS, and both are pure. Implementing
    // only the unsigned one left GsmUDP abstract on every real core - it
    // compiled inside this library, satisfied a shim that was missing the
    // declaration, and could not be instantiated by a single sketch.
    //
    // Same bytes either way; char is not signed on every target, so the cast
    // is to the buffer type this class actually stores.
    int read(char* buffer, size_t size) override {
        return read(reinterpret_cast<uint8_t*>(buffer), size);
    }
    int peek() override;
    void flush() override;

    IPAddress remoteIP() override { return _remoteIp; }
    uint16_t remotePort() override { return _remotePort; }

    GsmSocket* socket() const { return _socket; }

private:
    GsmSockets* _sockets = nullptr;
    GsmSocket*  _socket = nullptr;

    // One datagram at a time, assembled before it is sent. A datagram is sent
    // whole or not at all, so partial delivery is not a thing that can be
    // reported honestly - which is why this buffers rather than streaming.
    size_t   _packetBytes;
    uint8_t* _out = nullptr;          // _packetBytes each, taken by begin()
    size_t  _outCount = 0;
    bool    _building = false;

    uint8_t* _in = nullptr;
    size_t  _inCount = 0;
    size_t  _inRead = 0;

    char     _target[64] = {0};
    uint16_t _targetPort = 0;
    uint16_t _localPort = 0;

    IPAddress _remoteIp;
    uint16_t  _remotePort = 0;
};

#endif // Arduino or shim
