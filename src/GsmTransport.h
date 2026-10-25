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
// THE LINK TO THE MODEM
// =============================================================================
//
// One byte channel, pointing DOWNWARD at the modem. Nine methods, no platform
// types, and no knowledge of AT.
//
// THIS LIBRARY HAS A BYTE CHANNEL AT BOTH ENDS AND THEY POINT OPPOSITE WAYS.
// Only this one is called a transport:
//
//      application
//           |  GsmSocket / GsmClient / GsmUDP    UP: what we PRODUCE
//      [ modbee-gsm ]
//           |  GsmTransport                      DOWN: what we CONSUME
//       the modem
//
// Implementations are an Arduino Stream, a POSIX or Win32 tty, a
// pseudo-terminal, or a TCP socket. The core reads and writes bytes and never
// learns which.

#include "GsmPlatform.h"

class GsmTransport {
public:
    virtual ~GsmTransport() = default;

    // Open the medium. False if it cannot be used.
    virtual bool begin() = 0;

    // Close and release. Must tolerate repeat calls.
    virtual void end() = 0;

    // Bytes readable without blocking.
    virtual size_t readable() = 0;

    // Takes up to `length` bytes into `into` and returns how many it took,
    // which may be fewer than asked for and may be zero. NEVER BLOCKS.
    //
    // Bulk, like write(), and that symmetry is the point. Arduino's Stream
    // idiom of one byte per call belongs inside GsmStreamTransport, which is
    // where it came from; handing bytes back one at a time through a virtual
    // call would undo every bulk read underneath it, and this library reads in
    // 1500 byte lumps.
    virtual size_t read(uint8_t* into, size_t length) = 0;

    // Writes what it can and returns how much it took, which MAY BE LESS THAN
    // ASKED FOR and may be zero. Never blocks waiting for room.
    //
    // A partial write here is ordinary and the caller resumes from a cursor on
    // the next poll(). It is NOT the same thing as a partial write to a
    // SOCKET, which on some modules is fatal to that socket - see
    // GsmResult::isRetryable().
    virtual size_t write(const uint8_t* from, size_t length) = 0;

    // Whether the medium is usable now.
    virtual bool isConnected() const = 0;

    // How many bytes this channel can hold before it drops them.
    //
    // THIS IS THE NUMBER THAT DECIDES HOW MUCH WE DARE ASK THE MODEM FOR, and
    // it is on the interface rather than in a configuration file because only
    // the transport knows it.
    //
    // Most AT wiring is three wires. RTS/CTS exist on nearly every module but
    // are very often not brought out, and on a USB-CDC modem there is no
    // hardware handshake at all. So the general case is that THE MODEM CANNOT
    // BE TOLD TO WAIT: ask it for 1500 bytes and 1500 bytes arrive, as fast as
    // the line carries them, whether or not anybody is draining the other end.
    // At 115200 that is 130 ms of solid transmission into a buffer that on a
    // typical Arduino core holds 256 bytes.
    //
    // The failure is worse than it sounds. An overrun does not lose a reading;
    // it DESYNCHRONISES THE AT STREAM, because the bytes that went missing
    // were part of a length-delimited payload. Everything after them is read
    // as the wrong thing until something times out.
    //
    // This is not speculative. TinyGSM's own HttpClient example carries the
    // warning in its header: "HttpClient library does not empty to serial
    // buffer fast enough and the buffer overflow causes the HttpClient library
    // to stall." Same problem, most-used library in the field, no structural
    // answer. The answer here is structural: the socket layer never requests
    // more than this.
    //
    // Return 0 for "unbounded or unknown", which is correct for a loopback and
    // for a TCP transport where the kernel has its own buffering.
    virtual size_t receiveCapacity() const { return 0; }

    // Whether the medium can exert back pressure on the modem.
    //
    // True for a tty with RTS/CTS wired and enabled, and for a TCP transport
    // where the far end is a socket rather than a UART. When it is true the
    // receiveCapacity() cap above is lifted, because the link can simply stop
    // accepting and the modem will wait.
    virtual bool hasFlowControl() const { return false; }

    // Bits per second the link clocks at, or 0 where that is not a meaningful
    // question. A serial port knows it; a loopback does not.
    //
    // Used to size how long a given number of bytes must take, so that a
    // "modem is not answering" deadline is not shorter than the airtime of the
    // answer we are waiting for. At 9600 baud a 1500 byte payload is 1.6
    // seconds of pure transmission before the modem has thought about
    // anything, and a timeout that ignored the rate would abandon it.
    virtual uint32_t bitsPerSecond() const { return 0; }

    // Something a host can wait on, or -1 where the platform has no such idea.
    // A POSIX tty returns its descriptor; an Arduino Stream returns -1.
    //
    // NOT the descriptor escaping the abstraction: nothing may read or write
    // through it. It exists so a host application can select() across this and
    // its own sockets instead of spinning on poll().
    virtual int waitHandle() const { return -1; }

    // Where the far end is, in whatever words the medium has - "/dev/ttyUSB0",
    // "127.0.0.1:2222", empty for a loopback. Diagnostics only.
    virtual const char* name() const { return ""; }
};
