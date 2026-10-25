// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#include "GsmSerialTransport.h"

#if defined(GSM_PLATFORM_HOST)

#include <cerrno>
#include <cstring>

#if defined(_WIN32)
    #include <windows.h>
#else
    #include <fcntl.h>
    #include <poll.h>
    #include <termios.h>
    #include <unistd.h>
    #include <sys/ioctl.h>
#endif
#include "../GsmNoExcept.h"

namespace {

#if !defined(_WIN32)

// The rate constants are not the rates. termios wants B115200, not 115200,
// and there is no arithmetic that converts one to the other - only a table.
// A rate that is not here is refused rather than silently run at whatever the
// port happened to be set to, because a wrong baud rate presents as a modem
// that answers in garbage, which is one of the least obvious faults there is.
speed_t speedConstant(uint32_t bitsPerSecond, bool* known) {
    *known = true;
    switch (bitsPerSecond) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 230400:  return B230400;
        // Not every POSIX host defines the two fastest: macOS stops at
        // B230400. Where a constant is missing the rate is refused, like any
        // other rate this table does not know.
#if defined(B460800)
        case 460800:  return B460800;
#endif
#if defined(B921600)
        case 921600:  return B921600;
#endif
        default:      *known = false; return B115200;
    }
}

#endif // !_WIN32

} // namespace

GsmSerialTransport::~GsmSerialTransport() { end(); }

void GsmSerialTransport::configure(const char* device, uint32_t bitsPerSecond) {
    _device = (device != nullptr) ? device : "";
    if (bitsPerSecond != 0) { _bitsPerSecond = bitsPerSecond; }
}

#if !defined(_WIN32)

bool GsmSerialTransport::begin() {
    end();
    _lastError.clear();
    _lost = false;

    if (_device.empty()) {
        _lastError = "no device configured";
        return false;
    }

    // O_NONBLOCK on the OPEN as well as in the flags, because opening a tty
    // that is not asserting carrier will otherwise block in open() itself -
    // before there is any handle to set flags on. O_NOCTTY because a modem
    // must never become this process's controlling terminal: if it did, a
    // hangup on the line would deliver SIGHUP to the application.
    _handle = ::open(_device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_handle < 0) {
        _lastError = _device + ": " + std::strerror(errno);
        return false;
    }

    termios settings;
    std::memset(&settings, 0, sizeof(settings));
    if (::tcgetattr(_handle, &settings) != 0) {
        _lastError = std::string("tcgetattr: ") + std::strerror(errno);
        end();
        return false;
    }

    bool knownRate = false;
    const speed_t speed = speedConstant(_bitsPerSecond, &knownRate);
    if (!knownRate) {
        _lastError = "unsupported baud rate";
        end();
        return false;
    }
    ::cfsetispeed(&settings, speed);
    ::cfsetospeed(&settings, speed);

    // Raw mode, in full, because a tty's default behaviour is catastrophic for
    // binary data and every one of these flags has bitten somebody:
    //
    //   ICANON  would buffer until a newline - and an AT payload is not lines
    //   ECHO    would send every byte we write straight back at us
    //   ISIG    would turn a 0x03 inside a payload into SIGINT
    //   IXON    would turn 0x11/0x13 inside a payload into flow control
    //   ICRNL   would rewrite CR as LF, silently corrupting every AT reply
    //   OPOST   would rewrite bytes on the way out
    //
    // cfmakeraw does all of it, and doing it by hand is how one gets missed.
    ::cfmakeraw(&settings);

    settings.c_cflag |= (CLOCAL | CREAD);   // ignore modem control lines; enable receive
    settings.c_cflag &= ~CSTOPB;            // 1 stop bit
    settings.c_cflag &= ~PARENB;            // no parity
    settings.c_cflag &= ~CSIZE;
    settings.c_cflag |= CS8;                // 8 data bits - every AT manual says 8N1

#if defined(CRTSCTS)
    if (_wantFlowControl) {
        settings.c_cflag |= CRTSCTS;
    } else {
        settings.c_cflag &= ~CRTSCTS;
    }
    _flowControlActive = _wantFlowControl;
#else
    _flowControlActive = false;
#endif

    // Return immediately with whatever is there. The handle is non-blocking
    // anyway; setting these too means the behaviour does not depend on which
    // of the two mechanisms a given platform honours.
    settings.c_cc[VMIN]  = 0;
    settings.c_cc[VTIME] = 0;

    if (::tcsetattr(_handle, TCSANOW, &settings) != 0) {
        _lastError = std::string("tcsetattr: ") + std::strerror(errno);
        end();
        return false;
    }

    // Discard whatever was in flight before we opened. A modem that was mid
    // reply when the last process died leaves a partial line in the driver
    // buffer, and reading it as the first line of our own session desynchro-
    // nises the parser before it has issued a single command.
    ::tcflush(_handle, TCIOFLUSH);
    return true;
}

void GsmSerialTransport::end() {
    if (_handle >= 0) {
        ::close(_handle);
        _handle = -1;
    }
    _flowControlActive = false;
}

size_t GsmSerialTransport::readable() {
    if (_handle < 0) { return 0; }
    int pending = 0;
    if (::ioctl(_handle, FIONREAD, &pending) != 0) { return 0; }
    return pending > 0 ? (size_t)pending : 0;
}

namespace {

// "Nothing right now" - the ordinary answer on a non-blocking handle - as
// opposed to "this handle is finished".
bool transient(int error) {
    return error == EAGAIN || error == EWOULDBLOCK || error == EINTR;
}

} // namespace

size_t GsmSerialTransport::read(uint8_t* into, size_t length) {
    if (_handle < 0 || _lost || into == nullptr || length == 0) { return 0; }

    const ssize_t taken = ::read(_handle, into, length);
    if (taken > 0) { return (size_t)taken; }

    // EAGAIN means "nothing right now", which is the ordinary case on a
    // non-blocking handle and is not an error. EINTR means a signal arrived
    // mid-call; the caller polls again in a moment either way.
    //
    // ANYTHING ELSE MEANS THE PORT HAS GONE, and it used to be folded into the
    // same "nothing yet". An unplugged USB adapter answers EIO on every read
    // for ever; the library went on polling a dead handle and reporting a
    // modem that had merely gone quiet, which sends whoever is debugging it
    // after the modem, the SIM and the network before the cable.
    if (taken < 0) {
        if (!transient(errno)) { _lost = true; }
        return 0;
    }

    // Zero is ambiguous on a tty. With VMIN and VTIME both 0 it is how an
    // empty buffer answers; after a hangup it is also how end-of-file
    // answers, and only the hangup flag tells them apart. Asked without
    // waiting - a timeout of zero - so this costs a system call, never a wait.
    pollfd probe;
    probe.fd = _handle;
    probe.events = POLLIN;
    probe.revents = 0;
    if (::poll(&probe, 1, 0) > 0 &&
        (probe.revents & (POLLHUP | POLLERR | POLLNVAL)) != 0) {
        _lost = true;
    }
    return 0;
}

size_t GsmSerialTransport::write(const uint8_t* from, size_t length) {
    if (_handle < 0 || _lost || from == nullptr || length == 0) { return 0; }

    const ssize_t written = ::write(_handle, from, length);
    if (written > 0) { return (size_t)written; }

    // A partial write, or none at all because the driver buffer is full. The
    // caller holds a cursor and resumes on the next poll(). Nothing spins here.
    // A real error, as in read(), is a port that has gone.
    if (written < 0 && !transient(errno)) { _lost = true; }
    return 0;
}

bool GsmSerialTransport::isConnected() const { return _handle >= 0 && !_lost; }

#else // _WIN32

// Win32 is deliberately a separate implementation rather than a shim over the
// POSIX one: overlapped I/O and DCB have no useful common subset with termios,
// and a lowest-common-denominator wrapper would be non-blocking on one and not
// on the other - which is the one property this class exists to guarantee.

bool GsmSerialTransport::begin() {
    end();
    _lastError.clear();
    _lost = false;

    if (_device.empty()) {
        _lastError = "no device configured";
        return false;
    }

    // The \\.\ prefix is required for COM10 and above. Applying it always
    // costs nothing and removes a limit that would otherwise appear only on a
    // machine with a lot of serial ports.
    const std::string path = "\\\\.\\" + _device;

    const HANDLE handle = ::CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE,
                                        0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (handle == INVALID_HANDLE_VALUE) {
        _lastError = _device + ": open failed";
        return false;
    }

    DCB dcb;
    std::memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if (!::GetCommState(handle, &dcb)) {
        _lastError = "GetCommState failed";
        ::CloseHandle(handle);
        return false;
    }

    // EVERY FIELD THAT MATTERS IS SET, NOT INHERITED. GetCommState() returns
    // whatever the last program to use the port left behind, and Windows
    // keeps it across opens. A terminal program that turned on fNull leaves a
    // port that silently DELETES every 0x00 byte - which a TLS record or any
    // binary payload is full of - and one that left fAbortOnError on leaves a
    // port that stops dead at the first framing error until somebody calls
    // ClearCommError(). So the whole of 8N1, binary, no software flow
    // control and no byte substitution is written here, every time.
    dcb.BaudRate = _bitsPerSecond;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary  = TRUE;           // Windows supports nothing else
    dcb.fParity  = FALSE;
    dcb.fOutxCtsFlow    = _wantFlowControl ? TRUE : FALSE;
    dcb.fOutxDsrFlow    = FALSE;
    dcb.fDtrControl     = DTR_CONTROL_ENABLE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fRtsControl     = _wantFlowControl ? RTS_CONTROL_HANDSHAKE : RTS_CONTROL_ENABLE;
    dcb.fOutX = FALSE;   // never software flow control - see the POSIX note
    dcb.fInX  = FALSE;
    dcb.fTXContinueOnXoff = TRUE;
    dcb.fErrorChar   = FALSE;      // no substitution for bytes with errors
    dcb.fNull        = FALSE;      // NUL bytes are data, not padding
    dcb.fAbortOnError = FALSE;     // an error does not stop the port

    if (!::SetCommState(handle, &dcb)) {
        _lastError = "SetCommState failed";
        ::CloseHandle(handle);
        return false;
    }

    // READS: ReadIntervalTimeout = MAXDWORD with both read totals zero, which
    // is the documented way to say "return immediately with whatever is in
    // the buffer". Without it ReadFile blocks, and this class would stop being
    // non-blocking on Windows only - the kind of difference that is found
    // late.
    //
    // WRITES: a total timeout of one millisecond. Left at zero, which means
    // "no timeout", WriteFile waits until every byte has gone - and with flow
    // control on and CTS held off, that is for ever. One millisecond makes it
    // hand back whatever the driver took, a short count the engine already
    // resumes from on the next poll. (The system timer may round it up to a
    // tick of the scheduler; a wait that is bounded by a tick is still a wait
    // that ends, which is the promise.)
    COMMTIMEOUTS timeouts;
    std::memset(&timeouts, 0, sizeof(timeouts));
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 1;
    if (!::SetCommTimeouts(handle, &timeouts)) {
        // Refused rather than carried on with: a port whose timeouts could not
        // be set is a port that may block, and that is the one property this
        // class exists to rule out.
        _lastError = "SetCommTimeouts failed";
        ::CloseHandle(handle);
        return false;
    }

    ::PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR);

    _handle = (int)(intptr_t)handle;
    _flowControlActive = _wantFlowControl;
    return true;
}

void GsmSerialTransport::end() {
    if (_handle >= 0) {
        ::CloseHandle((HANDLE)(intptr_t)_handle);
        _handle = -1;
    }
    _flowControlActive = false;
}

size_t GsmSerialTransport::readable() {
    if (_handle < 0) { return 0; }
    COMSTAT status;
    DWORD errors = 0;
    if (!::ClearCommError((HANDLE)(intptr_t)_handle, &errors, &status)) { return 0; }
    return (size_t)status.cbInQue;
}

size_t GsmSerialTransport::read(uint8_t* into, size_t length) {
    if (_handle < 0 || _lost || into == nullptr || length == 0) { return 0; }
    DWORD taken = 0;
    if (!::ReadFile((HANDLE)(intptr_t)_handle, into, (DWORD)length, &taken, nullptr)) {
        // With the timeouts above, a read that merely found nothing SUCCEEDS
        // with a count of zero. A read that fails is a port that has gone - a
        // USB adapter pulled out answers every call with an error from then
        // on - and is reported through isConnected() rather than as silence.
        _lost = true;
        return 0;
    }
    return (size_t)taken;
}

size_t GsmSerialTransport::write(const uint8_t* from, size_t length) {
    if (_handle < 0 || _lost || from == nullptr || length == 0) { return 0; }
    DWORD written = 0;
    if (!::WriteFile((HANDLE)(intptr_t)_handle, from, (DWORD)length, &written, nullptr)) {
        // A write that ran into its timeout is the expected short write, and
        // some drivers report it as a failure while still counting what they
        // took. Those bytes HAVE gone, so the count is returned either way -
        // dropping it would send them a second time. Any other failure is a
        // port that has gone.
        const DWORD error = ::GetLastError();
        if (error != ERROR_TIMEOUT && error != ERROR_SEM_TIMEOUT) { _lost = true; }
    }
    // Never more than was asked, whatever the driver claims.
    return (written <= (DWORD)length) ? (size_t)written : length;
}

bool GsmSerialTransport::isConnected() const { return _handle >= 0 && !_lost; }

#endif // _WIN32

#endif // GSM_PLATFORM_HOST
