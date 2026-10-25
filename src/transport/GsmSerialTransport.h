// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#pragma once

// A real serial port on a host: POSIX termios or Win32. Compiles to nothing on
// Arduino.
//
// Opens the device non-blocking and keeps it that way. That is the whole trick
// and it is worth saying why: a tty opened the ordinary way blocks in read()
// until at least one byte arrives, and a library whose entire purpose is not
// to block cannot have that in its lowest layer.
//
// This also serves a PSEUDO-TERMINAL: /dev/pts/N looks exactly like a modem
// from here, which is what lets a modem be stood in for without changing a
// line of the library.

#include "../GsmTransport.h"

#if defined(GSM_PLATFORM_HOST)

#include <string>

class GsmSerialTransport : public GsmTransport {
public:
    GsmSerialTransport() = default;
    ~GsmSerialTransport() override;

    GsmSerialTransport(const GsmSerialTransport&) = delete;
    GsmSerialTransport& operator=(const GsmSerialTransport&) = delete;

    // `device` is "/dev/ttyUSB0", "/dev/pts/3", "COM3" - whatever the platform
    // calls it. Nothing is opened until begin().
    //
    // 8N1 is the only framing offered, and that is not an omission: every AT
    // modem manual specifies 8N1, and unlike Modbus RTU there is no installed
    // base of anything else to be compatible with.
    void configure(const char* device, uint32_t bitsPerSecond);

    // Hardware flow control, off by default.
    //
    // Turn it on ONLY when RTS and CTS are actually wired through to the
    // module. Enabling it on a three-wire cable is worse than leaving it off:
    // the driver waits for a CTS that is floating or tied off, and the port
    // appears dead with no error anywhere.
    void setFlowControl(bool on) { _wantFlowControl = on; }

    bool begin() override;
    void end() override;

    size_t readable() override;
    size_t read(uint8_t* into, size_t length) override;
    size_t write(const uint8_t* from, size_t length) override;

    // False once the port has failed underneath us - a USB adapter unplugged,
    // the far end of a pseudo-terminal closed - as well as before begin().
    // read() and write() then carry nothing until begin() opens it again.
    bool isConnected() const override;

    // What the kernel will hold for us between two calls to read().
    //
    // Reported as the configured low-water mark rather than the true driver
    // buffer size, which is not portably knowable. It is deliberately a floor:
    // asking the modem for less than the link can hold is safe, asking for
    // more is the failure this method exists to prevent.
    size_t receiveCapacity() const override { return _receiveCapacity; }
    void setReceiveCapacity(size_t bytes) { _receiveCapacity = bytes; }

    bool hasFlowControl() const override { return _flowControlActive; }
    uint32_t bitsPerSecond() const override { return _bitsPerSecond; }
    int waitHandle() const override { return _handle; }
    const char* name() const override { return _device.c_str(); }

    // Why begin() returned false, in the operating system's own words.
    // A status code cannot say which of forty things went wrong at a tty.
    const std::string& lastError() const { return _lastError; }

private:
    std::string _device;
    std::string _lastError;
    uint32_t    _bitsPerSecond = 115200;
    size_t      _receiveCapacity = 4096;
    int         _handle = -1;
    bool        _wantFlowControl = false;
    bool        _flowControlActive = false;

    // The handle is still open but the device behind it has gone. Kept apart
    // from _handle so that end() still closes it.
    bool        _lost = false;
};

#endif // GSM_PLATFORM_HOST
