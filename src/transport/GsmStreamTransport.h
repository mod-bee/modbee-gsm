// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#pragma once

// Arduino. Header-only, and compiles to nothing anywhere else.
//
// A template over the stream type rather than a class per board, so
// HardwareSerial, SoftwareSerial and a USB CDC port all work with the same
// code. The template parameter is named StreamType rather than after an
// Arduino class because from this library's side it is a byte channel and
// nothing more.
//
// THIS IS WHERE ARDUINO'S BYTE-AT-A-TIME IDIOM LIVES AND IT GOES NO FURTHER.
// GsmTransport::read() is bulk; Stream::read() is one byte. The loop that
// bridges them is here, in the one file that is allowed to know what a Stream
// is, so that nothing above it pays a virtual call per byte.

#include "../GsmTransport.h"

#if defined(GSM_PLATFORM_ARDUINO) || defined(GSM_ARDUINO_SHIM)

#if defined(GSM_ARDUINO_SHIM)
    #include "../GsmArduinoShim.h"
#else
    #include <Arduino.h>
#endif

template <typename StreamType>
class GsmStreamTransport : public GsmTransport {
public:
    // The stream is the caller's and must outlive this transport: the sketch
    // owns the serial port, and this does not open or close it.
    //
    // `bitsPerSecond` is the rate the sketch passed to begin(). It cannot be
    // read back portably - HardwareSerial has no baudRate() on every core -
    // and it is the one property of the line that cannot be measured from the
    // bytes, so it is given rather than guessed. Zero means "do not know",
    // which only costs slightly more generous timeouts.
    //
    // `receiveCapacity` is the size of the core's receive buffer. It matters
    // more than it looks: see GsmTransport::receiveCapacity(). The default of
    // 256 is the Arduino default on most cores, and it is deliberately the
    // pessimistic assumption - a sketch that raised its buffer says so, and a
    // sketch that forgot gets safe behaviour rather than a corrupted stream.
    explicit GsmStreamTransport(StreamType& stream,
                                uint32_t bitsPerSecond = 0,
                                size_t receiveCapacity = 256)
        : _stream(stream),
          _bitsPerSecond(bitsPerSecond),
          _receiveCapacity(receiveCapacity) {}

    bool begin() override { return true; }
    void end() override {}

    size_t readable() override {
        const int available = _stream.available();
        return available > 0 ? (size_t)available : 0;
    }

    size_t read(uint8_t* into, size_t length) override {
        if (into == nullptr || length == 0) { return 0; }

        size_t taken = 0;
        while (taken < length) {
            const int byte = _stream.read();
            if (byte < 0) { break; }          // nothing more; never blocks
            into[taken++] = (uint8_t)byte;
        }
        return taken;
    }

    size_t write(const uint8_t* from, size_t length) override {
        if (from == nullptr || length == 0) { return 0; }

        // availableForWrite() is how this stays non-blocking, and it is not on
        // every core - a plain Stream does not have it, and Print's default
        // answers 0 whatever the buffer holds. Where it is real, write only
        // what the buffer will take; where it is not, the core's own write()
        // blocks once its buffer fills, and that is the core's behaviour
        // rather than something this library can fix.
        //
        // ZERO IS AMBIGUOUS, AND THE STREAM ITSELF SETTLES IT. Read as "this
        // core does not say", a 0 from a core that DOES implement it - a full
        // transmit buffer - sent the whole write to a core that then waited
        // for its buffer to drain, which is a stalled poll(). So the first
        // time the stream reports any room at all, it has shown that it knows
        // its own buffer, and from then on 0 means full: nothing is written
        // and 0 is returned, and the engine offers the same bytes again on
        // the next poll. A stream that has never reported room keeps the old
        // behaviour, because for it 0 means nothing.
        const size_t room = writableRoom();
        if (room != 0) { _reportsRoom = true; }
        if (room == 0 && _reportsRoom) { return 0; }
        const size_t wanted = (room != 0 && room < length) ? room : length;
        return _stream.write(from, wanted);
    }

    bool isConnected() const override { return true; }

    size_t receiveCapacity() const override { return _receiveCapacity; }
    uint32_t bitsPerSecond() const override { return _bitsPerSecond; }

    // A three-wire connection, which is what nearly every module is wired
    // with. A board that has RTS/CTS wired AND enabled can say so.
    bool hasFlowControl() const override { return _hasFlowControl; }
    void setHasFlowControl(bool on) { _hasFlowControl = on; }

    const char* name() const override { return "stream"; }

private:
    // Separated so the "core has no availableForWrite()" case is one place
    // rather than a #if in the middle of write().
    size_t writableRoom() {
        const int room = _stream.availableForWrite();
        return room > 0 ? (size_t)room : 0;
    }

    StreamType& _stream;
    uint32_t    _bitsPerSecond;
    size_t      _receiveCapacity;
    bool        _hasFlowControl = false;

    // availableForWrite() has answered non-zero at least once, so this core
    // implements it and its 0 means a full buffer. See write().
    bool        _reportsRoom = false;
};

#endif // GSM_PLATFORM_ARDUINO
