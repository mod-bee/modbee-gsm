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
// ARDUINO'S NETWORKING BASE CLASSES, FOR A HOST BUILD
// =============================================================================
//
// Compiled ONLY when GSM_ARDUINO_SHIM is defined, which nothing but the test
// suite does. On a real Arduino the core's own headers are used and this file
// is not included at all.
//
// WHY IT EXISTS: GsmClient and GsmUDP are the library's most important public
// surface - they are what lets ArduinoHttpClient, PubSubClient and everything
// else work over cellular. Without this shim they could only be verified by
// flashing a board, which means they would be verified rarely and late.
//
// These declarations match Arduino's published API. They are not a
// reimplementation of anything: Print, Stream, Client and UDP are abstract
// interfaces, and the whole content of them is the signatures.

#include <stdint.h>
#include <stddef.h>

class Print {
public:
    virtual ~Print() = default;
    virtual size_t write(uint8_t value) = 0;
    virtual size_t write(const uint8_t* buffer, size_t size) {
        size_t written = 0;
        for (size_t i = 0; i < size; i++) {
            if (write(buffer[i]) != 1) { break; }
            written++;
        }
        return written;
    }
    size_t print(const char* text) {
        size_t length = 0;
        while (text[length] != '\0') { length++; }
        return write((const uint8_t*)text, length);
    }
};

class Stream : public Print {
public:
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;
    virtual void flush() {}
};

// Four octets. Arduino's own class carries more, but nothing in this library
// uses more, and a shim that grew features the real one has would drift.
class IPAddress {
public:
    IPAddress() : _octets{0, 0, 0, 0} {}
    IPAddress(uint8_t a, uint8_t b, uint8_t c, uint8_t d) : _octets{a, b, c, d} {}
    uint8_t operator[](int index) const { return _octets[index & 3]; }
    uint8_t& operator[](int index) { return _octets[index & 3]; }
private:
    uint8_t _octets[4];
};

class Client : public Stream {
public:
    virtual int connect(const char* host, uint16_t port) = 0;
    virtual int connect(IPAddress ip, uint16_t port) = 0;
    virtual size_t write(uint8_t value) override = 0;
    virtual size_t write(const uint8_t* buffer, size_t size) override = 0;
    virtual int available() override = 0;
    virtual int read() override = 0;
    virtual int read(uint8_t* buffer, size_t size) = 0;
    virtual int peek() override = 0;
    virtual void flush() override = 0;
    virtual void stop() = 0;
    virtual uint8_t connected() = 0;
    virtual operator bool() = 0;
};

class UDP : public Stream {
public:
    virtual uint8_t begin(uint16_t port) = 0;
    virtual void stop() = 0;
    virtual int beginPacket(const char* host, uint16_t port) = 0;
    virtual int beginPacket(IPAddress ip, uint16_t port) = 0;
    virtual int endPacket() = 0;
    virtual size_t write(uint8_t value) override = 0;
    virtual size_t write(const uint8_t* buffer, size_t size) override = 0;
    virtual int parsePacket() = 0;
    virtual int available() override = 0;
    virtual int read() override = 0;
    virtual int read(uint8_t* buffer, size_t size) = 0;

    // THE REAL API DECLARES THIS ONE TOO, and leaving it out of the shim is
    // how GsmUDP shipped abstract: it implemented the two above, the host
    // tests were satisfied, and no real core could instantiate it. A stand-in
    // that is easier to satisfy than the thing it stands in for tests nothing.
    virtual int read(char* buffer, size_t size) = 0;

    virtual int peek() override = 0;
    virtual void flush() override = 0;
    virtual IPAddress remoteIP() = 0;
    virtual uint16_t remotePort() = 0;
};
