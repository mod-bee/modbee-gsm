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
// Arduino's Client, over a modem
// =============================================================================
//
// A FIRST-CLASS DELIVERABLE, NOT A COMPATIBILITY SHIM.
//
// This is the doorway to every library already written against Arduino's
// networking: ArduinoHttpClient, PubSubClient, WebSockets, Blynk, and whatever
// somebody writes next. They take a Client&, and this is one, so they work
// over cellular with no source change at all.
//
// That is also the settled answer to "should the library implement HTTP?" -
// no. TinyGSM has no HTTP module and no WebSocket module either; its own
// example is four lines of layering over its Client. One transport, and the
// whole ecosystem on top of it, is the right shape and it is this one.
//
// --- ONE CALL BLOCKS, AND IT IS NOT OURS ---
//
//   int connect(const char* host, uint16_t port);
//
// Arduino's interface requires a verdict. There is nowhere in that signature
// to put "still trying", so this call pumps poll() until the socket opens or
// its deadline passes. It is bounded, it is documented, and it is the ONLY
// blocking call in this library on the running path. The one other wait is
// end(), which spends a bounded time letting the module power down cleanly;
// nothing else anywhere waits, and this one exists because Arduino's
// signature leaves no room for anything else.
//
// Everything after connect() - available(), read(), write(), stop() - is
// non-blocking exactly as the rest of the library is. An application that
// cannot afford even that one call uses GsmSocket directly, where begin()
// starts the open and returns.

#include "GsmPlatform.h"
#include "GsmSocket.h"

#if defined(GSM_PLATFORM_ARDUINO) || defined(GSM_ARDUINO_SHIM)

#if defined(GSM_ARDUINO_SHIM)
    #include "GsmArduinoShim.h"
#else
    #include <Arduino.h>
    #include <Client.h>
    #include <IPAddress.h>
#endif

class GsmClient : public Client {
public:
    // The security argument is how TLS is reached from the Arduino side. It
    // defaults to Plain, so every existing sketch behaves exactly as before -
    // and GsmClientSecure below is the spelling most people will use, because
    // it is the one every WiFi and Ethernet stack already taught them.
    explicit GsmClient(GsmSockets& sockets,
                       GsmSecurity security = GsmSecurity::Plain)
        : _sockets(&sockets), _security(security) {}

    // --- Client ---

    int connect(const char* host, uint16_t port) override {
        return connect(host, port, kDefaultConnectSeconds);
    }

    int connect(IPAddress ip, uint16_t port) override {
        // Arduino's IPAddress has no portable textual form across cores, so it
        // is rendered here rather than relying on one.
        char text[16];
        renderAddress(ip, text, sizeof text);
        return connect(text, port, kDefaultConnectSeconds);
    }

    // The timed form, which the Client interface also defines on most cores.
    int connect(const char* host, uint16_t port, int timeoutSeconds);

    size_t write(uint8_t value) override { return write(&value, 1); }
    size_t write(const uint8_t* buffer, size_t size) override;

    int available() override;
    int read() override;
    int read(uint8_t* buffer, size_t size) override;
    int peek() override;
    void flush() override;
    void stop() override;
    uint8_t connected() override;
    operator bool() override { return connected() != 0; }

    // --- Beyond the interface ---

    // The real socket, for an application that wants the non-blocking form
    // without giving up the libraries that need a Client.
    GsmSocket* socket() const { return _socket; }

    // Why the socket failed, or why the last connect() did not open one.
    GsmResult lastError() const {
        return _socket != nullptr ? _socket->lastError() : _failure;
    }

    // What this client will ask for. A module that cannot do it refuses the
    // open rather than quietly connecting in clear - see GsmSockets::open().
    GsmSecurity security() const { return _security; }

private:
    static const int kDefaultConnectSeconds = 75;

    static void renderAddress(const IPAddress& ip, char* out, size_t capacity);

    // Ends a socket that failed to open, keeping its reason. See connect().
    void release();

    GsmSockets* _sockets = nullptr;
    GsmSocket*  _socket = nullptr;
    GsmSecurity _security = GsmSecurity::Plain;

    // Why the last connect() came back without a socket. A failed socket is
    // ended - its slot handed back - so its own lastError() is gone with it.
    GsmResult   _failure = GsmStatus::Ok;

    // One byte of lookahead, because peek() must not consume and the socket
    // buffer is a queue rather than an array.
    int _peeked = -1;
};

// =============================================================================
// THE SAME THING, ENCRYPTED BY THE MODULE
// =============================================================================
//
// Named after WiFiClientSecure because that is the name people already know,
// and because PubSubClient over port 8883 is the commonest thing anybody wants
// from a cellular library. Before this existed, GsmClient hardcoded
// GsmSecurity::Plain and module TLS was unreachable from the Arduino surface
// altogether - the library implemented it, documented it and gave no way to
// ask for it.
//
// THE HANDSHAKE HAPPENS IN THE MODULE, not here, so this costs no flash and no
// RAM on the controller. What it costs instead is control: the module decides
// the cipher suites, and the two families offer only two or three concurrent
// TLS sessions where they offer ten or thirteen plain sockets. A design that
// needs certificate pinning, or its own trust store, or more sessions than
// that, puts wolfSSL over a plain GsmSocket instead - which is why TLS here is
// off by default rather than the default.
//
// A module whose driver reports no TLS refuses the open by name, at the point
// of asking. It does not fall back to plain.
class GsmClientSecure : public GsmClient {
public:
    explicit GsmClientSecure(GsmSockets& sockets)
        : GsmClient(sockets, GsmSecurity::Module) {}
};

#endif // Arduino or shim
