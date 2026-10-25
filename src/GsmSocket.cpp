// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#include "GsmSocket.h"

#include <new>
#include "GsmCmdBuf.h"
#include "GsmNoExcept.h"

const char* gsm_socketStateName(GsmSocketState state) {
    switch (state) {
        case GsmSocketState::Closed:  return "closed";
        case GsmSocketState::Opening: return "opening";
        case GsmSocketState::Open:    return "open";
        case GsmSocketState::Closing: return "closing";
        case GsmSocketState::Failed:  return "failed";
    }
    return "unknown";
}

// =============================================================================
// GsmSockets
// =============================================================================

GsmSockets::~GsmSockets() { release(); }

void GsmSockets::release() {
    for (uint8_t i = 0; i < _count; i++) { _sockets[i].~GsmSocket(); }
    gsm_freeLarge(_sockets);
    _sockets = nullptr;
    _count = 0;
    _haveTx = _haveRx = 0;
}

void GsmSockets::setCapacity(uint8_t sockets, size_t txBytes, size_t rxBytes) {
    _wantSlots = sockets != 0 ? sockets : (uint8_t)GSM_SOCKET_SLOTS;
    _wantTx = txBytes != 0 ? txBytes : (size_t)GSM_SOCKET_TX_BUFFER;
    _wantRx = rxBytes != 0 ? rxBytes : (size_t)GSM_SOCKET_RX_BUFFER;
}

bool GsmSockets::reserve() {
    if (_sockets != nullptr && _count == _wantSlots && _haveTx == _wantTx &&
        _haveRx == _wantRx) {
        return true;
    }
    // Resized only while nothing is open: a socket's buffers never move under it.
    if (_sockets != nullptr) {
        for (uint8_t i = 0; i < _count; i++) { if (_sockets[i]._inUse) { return true; } }
    }
    release();

    // One piece: the sockets, then each one's send and receive buffer.
    size_t perSocket = 0;
    size_t buffers = 0;
    size_t objects = 0;
    if (_wantTx > (size_t)-1 - _wantRx) { return false; }
    perSocket = _wantTx + _wantRx;
    if (!gsm_multiply(_wantSlots, perSocket, buffers)) { return false; }
    if (!gsm_multiply(_wantSlots, sizeof(GsmSocket), objects)) { return false; }
    const size_t align = alignof(GsmSocket);
    objects = (objects + align - 1) / align * align;
    if (objects > (size_t)-1 - buffers) { return false; }

    void* memory = gsm_allocLarge(objects + buffers);
    if (memory == nullptr) { return false; }

    _sockets = (GsmSocket*)memory;
    uint8_t* bytes = (uint8_t*)memory + objects;
    for (uint8_t i = 0; i < _wantSlots; i++) {
        GsmSocket* socket = new (&_sockets[i]) GsmSocket();
        socket->_tx = bytes;
        socket->_txSize = _wantTx;
        bytes += _wantTx;
        socket->_rx = bytes;
        socket->_rxSize = _wantRx;
        bytes += _wantRx;
    }
    _count = _wantSlots;
    _haveTx = _wantTx;
    _haveRx = _wantRx;
    return true;
}

bool GsmSockets::begin(GsmModem* modem) {
    _modem = modem;
    if (!reserve()) { return false; }
    for (uint8_t i = 0; i < _count; i++) {
        GsmSocket& socket = _sockets[i];
        // A SOCKET WITH A COMMAND STILL OUT IS NOT WIPED UNDER IT. The engine
        // still holds a pointer to it as that command's context, and a socket
        // reset here would be free for open() to hand out before the late
        // completion lands in it. It is released when that completion comes,
        // like any other socket ended mid-command - but with no close, because
        // whatever it held belonged to the session being replaced.
        if (socket._busy) {
            socket._releasing = true;
            socket._closeOnRelease = false;
            socket._linkOpen = false;
            socket._state = GsmSocketState::Closing;
            continue;
        }
        socket.reset();
    }
    return true;
}

uint8_t GsmSockets::available() const {
    uint8_t free = 0;
    for (uint8_t i = 0; i < _count; i++) { if (!_sockets[i]._inUse) { free++; } }
    return free;
}

uint8_t GsmSockets::openCount() const {
    uint8_t count = 0;
    for (uint8_t i = 0; i < _count; i++) {
        if (_sockets[i]._state == GsmSocketState::Open) { count++; }
    }
    return count;
}

GsmSocket* GsmSockets::byModuleId(uint8_t socketId, bool secure) {
    for (uint8_t i = 0; i < _count; i++) {
        GsmSocket& socket = _sockets[i];
        if (!socket._inUse || socket._socketId != socketId) { continue; }

        // Ended by the application, still waiting for its last command. An
        // event for that id is about a connection nobody holds any more.
        if (socket._releasing) { continue; }

        // A URC from a separate TLS command family identifies a TLS SESSION,
        // and session 0 is not plain socket 0. Only a secure socket can own it.
        // A shared-identifier module reports both through the same prefixes and
        // passes secure = false, so it still matches either.
        if (secure && socket._endpoint.security != GsmSecurity::Module) { continue; }
        return &socket;
    }
    return nullptr;
}

GsmSocket* GsmSockets::open(const char* host, uint16_t port,
                            GsmSocketKind kind, GsmSecurity security) {
    if (_modem == nullptr || _modem->driver() == nullptr) { return nullptr; }

    // --- REFUSED AT THE DOOR, not twenty seconds later by the module ---
    //
    // A UDP local bind genuinely has no host: openUdp() passes "" on purpose.
    // Everything else needs one, and a port of zero is not a port.
    if (kind != GsmSocketKind::Udp && (host == nullptr || host[0] == '\0')) {
        return nullptr;
    }
    if (port == 0) { return nullptr; }

    // A HOST THAT DOES NOT FIT IS THE DANGEROUS ONE. configure() copies into a
    // fixed buffer, and a copy that truncates does not fail - it produces a
    // DIFFERENT hostname, which can resolve. Connecting to the wrong machine
    // and reporting success is worse than any refusal, so the length is
    // checked here where there is still somewhere to say no.
    if (host != nullptr) {
        size_t length = 0;
        while (host[length] != '\0') {
            if (++length >= sizeof(GsmSocket::_hostCopy)) { return nullptr; }
        }
    }

    const GsmProfile& profile = _modem->driver()->profile();

    // The module's own limit, not ours. A build with four slots on a module
    // that declares two must refuse the third, and vice versa - which is why
    // the profile is asked rather than a constant assumed.
    const uint8_t moduleLimit = profile.maxSockets;

    for (uint8_t index = 0; index < _count; index++) {
        GsmSocket& socket = _sockets[index];
        if (socket._inUse) { continue; }
        if (index >= moduleLimit) { break; }

        // Module socket identifiers start where the module says they start;
        // some number from zero and some from one.
        socket._socketId = (uint8_t)(profile.firstSocketId + index);

        GsmEndpoint endpoint;
        endpoint.host = host;
        endpoint.port = port;
        endpoint.kind = kind;
        endpoint.security = security;

        // TLS the module cannot do is refused HERE, by name, rather than
        // attempted and failed obscurely twenty seconds later.
        if (security == GsmSecurity::Module && profile.tlsStyle == GsmTlsStyle::None) {
            return nullptr;
        }

        // AND THE MODULE'S TLS SESSION COUNT IS NOT ITS SOCKET COUNT.
        //
        // One family runs TLS over the same identifiers as plain sockets and
        // has as many; the other has a separate stack with only two, while
        // offering ten plain sockets. Asking for a third there must be
        // refused HERE, not twenty seconds later by an open that fails for a
        // reason that reads like a network fault.
        if (security == GsmSecurity::Module) {
            uint8_t secure = 0;
            for (uint8_t other = 0; other < _count; other++) {
                if (_sockets[other]._inUse &&
                    _sockets[other]._endpoint.security == GsmSecurity::Module) {
                    secure++;
                }
            }
            if (profile.maxTlsSessions != 0 && secure >= profile.maxTlsSessions) {
                return nullptr;
            }

            // AND ITS IDENTIFIER COMES FROM THE TLS SPACE, NOT THE SLOT.
            //
            // A separate TLS family numbers its sessions 0..maxTlsSessions-1,
            // independently of the plain sockets. Handing it the array slot
            // would send "open session 5" to a module that accepts 0 and 1 -
            // so a TLS socket worked only while it happened to land low.
            if (profile.tlsStyle == GsmTlsStyle::SeparateFamily) {
                uint8_t session = 0;
                for (; session < profile.maxTlsSessions; session++) {
                    bool taken = false;
                    for (uint8_t other = 0; other < _count; other++) {
                        if (_sockets[other]._inUse &&
                            _sockets[other]._endpoint.security == GsmSecurity::Module &&
                            _sockets[other]._socketId == session) {
                            taken = true;
                            break;
                        }
                    }
                    if (!taken) { break; }
                }
                if (session >= profile.maxTlsSessions) { return nullptr; }
                socket._socketId = session;
            }
        }
        if (kind == GsmSocketKind::Udp && !profile.hasUdp) { return nullptr; }

        socket.configure(this, endpoint);
        return &socket;
    }
    return nullptr;
}

void GsmSockets::close(GsmSocket* socket) {
    if (socket != nullptr) { socket->end(); }
}

void GsmSockets::poll() {
    for (uint8_t i = 0; i < _count; i++) {
        GsmSocket& socket = _sockets[i];
        if (!socket._inUse || socket._releasing) { continue; }

        // A SOCKET DOES NOT OUTLIVE ITS BEARER, whoever noticed the loss.
        //
        // The bearer's own URC fails every socket (handleEvent below), but the
        // modem also drops the bearer by itself - a recovery, a registration
        // loss, a module that restarted under us - and none of those sent an
        // event. The sockets stayed Open on a context that no longer existed:
        // writes queued, nothing arrived, and the application above was told
        // nothing until its own keepalive gave up minutes later.
        if ((socket._state == GsmSocketState::Open ||
             socket._state == GsmSocketState::Opening) && _modem != nullptr &&
            (!_modem->hasBearer() || _modem->bearerEpoch() != socket._bearerEpoch)) {
            socket.fail(GsmStatus::BearerLost);
        }
        socket.service();
    }
}

void GsmSockets::handleEvent(GsmUrcKind kind, const GsmUrcInfo& info) {
    switch (kind) {
        case GsmUrcKind::DataAvailable: {
            GsmSocket* socket = byModuleId(info.socketId, info.secure);
            if (socket != nullptr) {
                socket->_dataWaiting = true;
                socket->armUnreadCheck();
            }
            break;
        }

        case GsmUrcKind::SocketClosed: {
            GsmSocket* socket = byModuleId(info.socketId, info.secure);
            if (socket != nullptr) {
                // The peer went, but whatever it already sent is still the
                // answer to somebody's request. Discarding it here would lose
                // the last reply of every exchange that ends with the server
                // hanging up.
                //
                // AND IT IS NOT ENOUGH TO CHECK OUR OWN BUFFER. The bytes may
                // still be in the MODULE, announced and not yet fetched -
                // measured on an A7670G, where "+CCHEVENT: 0,RECV EVENT" and
                // "+CCH_PEER_CLOSED: 0" arrive in the SAME read. Concluding on
                // the close discards a reply that was sitting there for the
                // asking, and the socket layer never issues the fetch the
                // module just asked for.
                socket->_peerClosed = true;
                if (socket->_rxCount == 0 && !socket->_dataWaiting) {
                    socket->fail(GsmStatus::SocketClosed);
                }
            }
            break;
        }

        case GsmUrcKind::BearerDown:
            // EVERY SOCKET, not one. This is the distinction that stops an
            // application reopening sockets over a context that is gone.
            for (uint8_t i = 0; i < _count; i++) {
                if (_sockets[i]._inUse && !_sockets[i]._releasing) {
                    _sockets[i].fail(GsmStatus::BearerLost);
                }
            }
            break;

        default:
            break;
    }
}

// =============================================================================
// GsmSocket
// =============================================================================

void GsmSocket::reset() {
    _tlsStep = 0;
    _state = GsmSocketState::Closed;
    _lastError = GsmStatus::Ok;
    _inUse = false;
    _busy = false;
    _peerClosed = false;
    _dataWaiting = false;
    _linkOpen = false;
    _releasing = false;
    _closeOnRelease = false;
    _bearerEpoch = 0;
    _unreadCheckAt = 0;
    _txCount = 0;
    _txInFlight = 0;
    _rxHead = 0;
    _rxCount = 0;
    _expectedFetch = 0;
}

void GsmSocket::configure(GsmSockets* owner, GsmEndpoint endpoint) {
    reset();
    _owner = owner;
    _endpoint = endpoint;

    // The hostname is COPIED. GsmEndpoint carries a borrowed pointer, and a
    // socket outlives the call that opened it by minutes - on a reconnect, by
    // far longer. A caller passing a temporary would otherwise leave a
    // dangling pointer at the exact moment a reopen needs it.
    size_t i = 0;
    if (endpoint.host != nullptr) {
        for (; i + 1 < sizeof _hostCopy && endpoint.host[i] != '\0'; i++) {
            _hostCopy[i] = endpoint.host[i];
        }
    }
    _hostCopy[i] = '\0';
    _endpoint.host = _hostCopy;

    _inUse = true;
}

bool GsmSocket::setDatagramTarget(const char* host, uint16_t port) {
    if (_endpoint.kind != GsmSocketKind::Udp) { return false; }

    // The datagram already written goes where it was meant to. Rewriting the
    // target under it sent the first packet to the second packet's address.
    if (_txCount != 0) { return false; }

    size_t i = 0;
    if (host != nullptr) {
        for (; i + 1 < sizeof _hostCopy && host[i] != '\0'; i++) {
            _hostCopy[i] = host[i];
        }
    }
    _hostCopy[i] = '\0';
    _endpoint.host = _hostCopy;
    _endpoint.port = port;
    return true;
}

bool GsmSocket::begin() {
    if (_owner == nullptr || _owner->modem() == nullptr) { return false; }
    if (!_inUse || _releasing) { return false; }

    // ONCE. A second begin() used to reset the state and issue ANOTHER open
    // on the same socket - two AT+CIPOPENs for one identifier, and the module
    // answering both. The late conclusion then arrived after end() had closed
    // the socket and set it Open again, so the socket came back from the dead
    // and the slot was leaked. Nothing in the library did this; an application
    // retrying a slow connect does it immediately.
    if (_state == GsmSocketState::Opening || _state == GsmSocketState::Open) {
        return false;
    }

    // Refused rather than attempted when the bearer is not up. Opening a
    // socket over a context that does not exist fails after a long timeout
    // with an error nobody can act on; saying so now is more useful.
    if (!_owner->modem()->hasBearer()) {
        _lastError = GsmStatus::NoBearer;
        _state = GsmSocketState::Failed;
        return false;
    }

    _state = GsmSocketState::Opening;
    _lastError = GsmStatus::Ok;
    _peerClosed = false;
    _bearerEpoch = _owner->modem()->bearerEpoch();
    beginOpen();

    // An open refused before it was ever sent - a full queue, a host that
    // cannot be expressed - has already failed the socket, and says so here
    // as well as through state().
    return _state != GsmSocketState::Failed;
}

void GsmSocket::end() {
    if (!_inUse || _releasing) { return; }

    // A FAILED SOCKET MAY STILL HOLD A LINK AT THE MODULE. Only the far end's
    // close (_peerClosed) says the module has let it go; a socket this layer
    // failed - a stream with a hole in it, a bearer the modem gave up on
    // without a word from the module - has not been closed there, and leaving
    // it open makes the next open of the same id fail.
    const bool holdsLink = _state == GsmSocketState::Open ||
                           _state == GsmSocketState::Opening ||
                           (_linkOpen && !_peerClosed);

    // A COMMAND OF OURS IS STILL OUT. The slot waits for it - see _releasing -
    // and so does the close: an open still queued or still being answered
    // would otherwise be overtaken by its own close, and then succeed with
    // nothing left to close it. The transmit buffer is left exactly as it is,
    // because a send in flight is still reading it.
    if (_busy) {
        _releasing = true;
        _closeOnRelease = holdsLink;
        _state = GsmSocketState::Closing;
        _txCount = 0;
        _rxCount = 0;
        _dataWaiting = false;
        return;
    }

    if (holdsLink) { submitClose(); }
    reset();
}

void GsmSocket::submitClose() {
    GsmDriver* driver = _owner->modem()->driver();
    if (driver == nullptr) { return; }
    GsmCmdBuf command;
    driver->closeCommand(_socketId, _endpoint, command);
    if (command.isEmpty() || command.overflowed()) { return; }
    GsmCommand request;
    request.text = command.text();
    request.timeoutMs = 10000;
    _owner->modem()->engine().submit(request);
}

// The last command of a socket the application has already let go of has
// concluded. Its outcome is nobody's and is discarded.
//
// THE MODULE'S SIDE IS NOT. If the socket might hold a link there - it was
// opening or open when it was ended, or a late open said it succeeded - a close
// goes now, after that answer and not before it. Without this, an open that
// succeeded after GsmClient gave up on it at 75 s held its socket at the module
// until the module restarted.
void GsmSocket::finishRelease() {
    if (_closeOnRelease || (_linkOpen && !_peerClosed)) { submitClose(); }
    reset();
}

// =============================================================================
// OPEN
// =============================================================================

void GsmSocket::beginOpen() {
    GsmModem* modem = _owner->modem();
    GsmDriver* driver = modem->driver();
    if (driver == nullptr) { fail(GsmStatus::NotReady); return; }

    // TLS IS CONFIGURED BEFORE THE OPEN, ONE STEP PER POLL.
    //
    // Both families need setup first - a context and a flag on one, a whole
    // service start and session binding on the other - and none of it can be
    // folded into the open command. Walked here so the socket cannot reach
    // Open having skipped it: a socket asked for TLS that opens without it
    // would send in clear and say nothing.
    if (_endpoint.security == GsmSecurity::Module) {
        GsmCmdBuf setup;
        const GsmStep step = driver->tlsSetupStep(_tlsStep, _socketId,
                                                  _endpoint, setup);
        if (step.present) {
            if (setup.isEmpty() || setup.overflowed()) {
                fail(GsmStatus::InvalidArgument);
                return;
            }
            _tlsStep++;
            GsmCommand request;
            request.text = setup.text();
            request.timeoutMs = step.timeoutMs;
            request.onDone = &GsmSocket::onTlsSetupDone;
            request.context = this;

            // A SETUP STEP MAY ANSWER TWICE TOO - AT+CCHSTART does. Dropping
            // these here concluded it on its OK and left the real answer to
            // arrive as an orphan, with the socket opening meanwhile.
            request.deferredPrefix = step.deferredPrefix;
            request.deferredTimeoutMs = step.deferredTimeoutMs;
            const GsmResult accepted = modem->engine().submit(request);
            if (accepted.status() == GsmStatus::Queued) {
                _busy = true;
            } else {
                // Refused at the door - a full queue, most likely. Nothing is
                // out, so nothing will ever conclude this; left in Opening the
                // socket would wait for ever. It fails now, with the reason.
                fail(accepted);
            }
            return;
        }
    }

    GsmCmdBuf command;
    driver->openCommand(_socketId, _endpoint, modem->apn(), command);
    if (command.isEmpty() || command.overflowed()) {
        // A hostname too long to express, or one containing a character that
        // cannot appear inside an AT string parameter. Refused rather than
        // sent as something that means a different thing.
        fail(GsmStatus::InvalidArgument);
        return;
    }

    GsmCommand request;
    request.text = command.text();
    request.timeoutMs = driver->profile().socketOpenTimeoutMs;
    request.onDone = &GsmSocket::onOpenDone;
    request.onReply = &GsmSocket::onOpenReply;
    request.context = this;

    // THE TWO-STAGE CONCLUSION. One family answers "OK" and then, up to two
    // minutes later, the line that actually says whether the socket opened.
    // Concluding on the OK would report every socket as open the instant it
    // was asked for.
    //
    // The other family answers in one go, and says so by supplying no deferred
    // prefix - which is exactly the kind of difference the driver exists to
    // absorb.
    request.deferredPrefix = driver->openDeferredPrefix(_endpoint);
    request.deferredTimeoutMs = driver->profile().socketOpenTimeoutMs;

    // A REFUSED OPEN FAILS THE SOCKET. It used to leave it in Opening with
    // nothing out: no command to conclude, no deadline to pass, and a caller
    // watching state() for an answer that could never come.
    const GsmResult accepted = modem->engine().submit(request);
    if (accepted.status() == GsmStatus::Queued) {
        _busy = true;
    } else {
        fail(accepted);
    }
}

void GsmSocket::onTlsSetupDone(void* context, GsmResult result) {
    GsmSocket* self = (GsmSocket*)context;
    self->_busy = false;
    if (self->_releasing) { self->finishRelease(); return; }

    // A REFUSED TLS STEP FAILS THE SOCKET. Carrying on would open a plain
    // connection to a port expecting a handshake, and the failure would
    // surface much later as something that looks like a protocol fault.
    if (!result.isOk()) { self->fail(result); return; }

    // Next step, or the open itself - beginOpen() decides which.
    self->beginOpen();
}

void GsmSocket::onOpenReply(void* context, const char* line, size_t length) {
    GsmSocket* self = (GsmSocket*)context;
    GsmDriver* driver = self->_owner->modem()->driver();
    if (driver == nullptr) { return; }

    GsmOpenResult result;
    if (driver->parseOpenResult(line, length, result, self->_endpoint)) {
        if (result.socketId != self->_socketId) { return; }

        // AN ANSWER FOR A SOCKET THAT HAS STOPPED WAITING FOR ONE. Ended, or
        // failed underneath the open (the bearer went). It changes nothing
        // here - a Failed socket coming back to life as Open was a real
        // defect - except that the module now holds a link, which somebody
        // has to close.
        if (self->_releasing || self->_state != GsmSocketState::Opening) {
            if (result.opened) { self->_linkOpen = true; }
            return;
        }
        if (result.opened) {
            self->_state = GsmSocketState::Open;
            self->_linkOpen = true;
            self->armUnreadCheck();
        } else {
            // The module's own reason, in its own numbering, carried intact.
            // One family reports certificate faults here that no standard
            // error code can express - an expired certificate and a mismatched
            // common name are different commissioning mistakes.
            self->_lastError = result.result;
            self->_state = GsmSocketState::Failed;
        }
    }
}

void GsmSocket::onOpenDone(void* context, GsmResult result) {
    GsmSocket* self = (GsmSocket*)context;
    self->_busy = false;
    if (self->_releasing) { self->finishRelease(); return; }

    if (self->_state == GsmSocketState::Opening) {
        // The command concluded without a result line naming this socket.
        // Success without confirmation is not success.
        if (result.isOk()) {
            self->fail(GsmStatus::Error);
        } else {
            self->fail(result);
        }
    }
}

// =============================================================================
// SEND
// =============================================================================

size_t GsmSocket::write(const uint8_t* from, size_t length) {
    if (from == nullptr || length == 0) { return 0; }
    if (_state != GsmSocketState::Open) { return 0; }

    // A DATAGRAM IS ONE WRITE, WHOLE, AND ALONE. Appended to one still
    // waiting, two datagrams went as one - and to whichever destination was
    // set last. Accepted in part, the rest became a second datagram the far
    // end never asked for. So a datagram is taken entire, only into an empty
    // buffer, and only if a single send can carry it.
    if (_endpoint.kind == GsmSocketKind::Udp) {
        if (_txCount != 0) { return 0; }
        if (length > _txSize || length > datagramLimit()) { return 0; }
    }

    // Buffered, never sent from inside this call.
    //
    // AND THE BUFFERING IS NOT MERELY CONVENIENCE. A module manual states that
    // once a send command has been issued, "when the total size of the
    // inputted data reaches <len>, TA will report the following code.
    // Otherwise, the serial port will be blocked." A send issued before its
    // bytes are in hand can therefore wedge the AT channel outright - so the
    // whole chunk is buffered here first, and the command is only issued when
    // it can be completed.
    const size_t room = _txSize - _txCount;
    const size_t take = (length < room) ? length : room;
    for (size_t i = 0; i < take; i++) { _tx[_txCount + i] = from[i]; }
    _txCount += take;

    // A partial accept is ordinary and is reported honestly. The caller holds
    // a cursor and offers the rest on the next poll, exactly as it would to a
    // socket whose kernel buffer was full.
    return take;
}

// The largest datagram one send can carry whole. See beginSend().
size_t GsmSocket::datagramLimit() const {
    GsmDriver* driver = (_owner != nullptr && _owner->modem() != nullptr)
                            ? _owner->modem()->driver() : nullptr;
    size_t limit = 1400;
    if (driver != nullptr && driver->profile().maxSendChunk != 0 &&
        driver->profile().maxSendChunk < limit) {
        limit = driver->profile().maxSendChunk;
    }
    return limit;
}

void GsmSocket::beginSend() {
    GsmModem* modem = _owner->modem();
    GsmDriver* driver = modem->driver();
    if (driver == nullptr) { return; }

    const GsmProfile& profile = driver->profile();

    size_t chunk = _txCount;

    // The module's hard limit, from its manual. Exceeding it is refused by the
    // module in ways that differ between firmware; never reaching it is free.
    const uint16_t limit = (_endpoint.security == GsmSecurity::Module &&
                            profile.maxTlsSendChunk != 0)
                         ? profile.maxTlsSendChunk : profile.maxSendChunk;
    if (chunk > limit) { chunk = limit; }

    // A datagram is not a stream: it is sent whole or not at all, so a
    // datagram larger than one send is an error rather than something to split.
    // One manual additionally warns that payloads above 1400 bytes may not
    // reach the far end depending on the carrier.
    if (_endpoint.kind == GsmSocketKind::Udp && chunk > 1400) { chunk = 1400; }

    GsmCmdBuf command;
    driver->sendCommand(_socketId, chunk, _endpoint, command);
    if (command.isEmpty() || command.overflowed()) { return; }

    GsmCommand request;
    request.text = command.text();
    request.payload = _tx;           // BORROWED; _busy keeps it still
    request.payloadLength = chunk;
    request.timeoutMs = profile.socketSendTimeoutMs;
    request.onDone = &GsmSocket::onSendDone;
    request.onReply = &GsmSocket::onSendReply;
    request.context = this;

    // THE SECOND ANSWER, where the module gives one.
    //
    // On one family the send confirms with OK and then, separately, with the
    // byte count it actually took - and that second line is the only place the
    // short-send rule can be enforced. Without declaring it here the command
    // concluded on the OK and the confirmation was routed away as an unhandled
    // URC, so parseSendResult() never saw a line on real hardware however
    // correct it was.
    request.deferredPrefix = driver->sendDeferredPrefix(_endpoint);
    request.deferredTimeoutMs = profile.socketSendTimeoutMs;

    if (modem->engine().submit(request).status() == GsmStatus::Queued) {
        _busy = true;
        _txInFlight = chunk;
    }
}

void GsmSocket::onSendReply(void* context, const char* line, size_t length) {
    GsmSocket* self = (GsmSocket*)context;
    if (self->_releasing) { return; }
    GsmDriver* driver = self->_owner->modem()->driver();
    if (driver == nullptr) { return; }

    GsmResult outcome = GsmStatus::Ok;
    if (driver->parseSendResult(line, length, self->_txInFlight, outcome, self->_endpoint)) {
        if (!outcome.isOk()) {
            // THE RULE THAT LOOKS TRANSIENT AND IS NOT.
            //
            // One family's manual: "If the <cnfSendLength> is not equal to the
            // <reqSendLength>, the socket then cannot be used further." So a
            // short send is terminal - the driver concludes SocketClosed, and
            // retrying the send would write into something already gone. The
            // socket must be reopened; the send must not be repeated on it.
            self->_lastError = outcome;
            self->_state = GsmSocketState::Failed;
        }
    }
}

void GsmSocket::onSendDone(void* context, GsmResult result) {
    GsmSocket* self = (GsmSocket*)context;
    self->_busy = false;
    if (self->_releasing) { self->finishRelease(); return; }

    if (self->_state == GsmSocketState::Failed) {
        self->_txCount = 0;
        self->_txInFlight = 0;
        return;
    }

    if (result.isOk()) {
        // Consume what went. Anything beyond it stays queued for the next
        // send, which is how a write larger than one chunk is carried.
        const size_t sent = self->_txInFlight;
        for (size_t i = sent; i < self->_txCount; i++) {
            self->_tx[i - sent] = self->_tx[i];
        }
        self->_txCount -= sent;
        self->_bytesSent += (uint32_t)sent;
    } else {
        self->fail(result);
        self->_txCount = 0;
    }
    self->_txInFlight = 0;
}

// =============================================================================
// RECEIVE
// =============================================================================

void GsmSocket::beginFetch() {
    GsmModem* modem = _owner->modem();
    GsmDriver* driver = modem->driver();
    if (driver == nullptr) { return; }

    const GsmProfile& profile = driver->profile();

    size_t room = _rxSize - _rxCount;
    if (room == 0) { return; }

    // NEVER ASK FOR MORE THAN THE LINK CAN CARRY WITHOUT LOSING IT.
    //
    // Most AT wiring is three wires, so the module cannot be told to wait. Ask
    // for 1500 bytes and 1500 bytes arrive, as fast as the line carries them,
    // whether or not anybody is draining the other end. An overrun does not
    // lose a reading - it DESYNCHRONISES THE AT STREAM, because the missing
    // bytes were part of a length-delimited payload.
    //
    // So the transport declares what it can hold and the request is capped to
    // it. A link that has real flow control says so and the cap is lifted,
    // because there the module genuinely can be made to wait.
    const GsmTransport* link = modem->engine().transport();
    if (link != nullptr && !link->hasFlowControl()) {
        const size_t capacity = link->receiveCapacity();
        if (capacity != 0) {
            // AND THE PAYLOAD IS NOT ALL THAT ARRIVES.
            //
            // It comes wrapped, and the wrapper lands in the same buffer:
            //
            //   A76XX plain  "+CIPRXGET: 2,0,1500,1500" + OK      34 bytes
            //   A76XX TLS    OK + "+CCHRECV: DATA,0,2048"
            //                   + "+CCHRECV: 0,0"                 48 bytes
            //   SIM707x      "+CARECV: 1460," + OK                22 bytes
            //
            // So capping the PAYLOAD to the buffer size still overruns it by
            // the framing: a truthful 256-byte declaration would ask for 256
            // and be sent about 285.
            //
            // AND A URC CAN LAND IN THE MIDDLE OF IT. The module does not wait
            // for a quiet moment to report that registration changed; a
            // "+CEREG: 2,5,"F2FE","0005668F",7" is 35 bytes that arrive
            // wherever they arrive. Reserving only the framing leaves nothing
            // for that, and the overrun it causes is the same one.
            //
            // 48 worst framing + 35 for one URC = 83, rounded up. It costs
            // nothing on a core that can resize its buffer - a 2048-byte
            // buffer is capped by the 1500-byte socket ring long before this
            // matters - and on a 256-byte core it trades some throughput for
            // a burst that certainly fits.
            const size_t framing = 96;
            const size_t usable = (capacity > framing) ? (capacity - framing) : 0;
            if (usable == 0) { return; }
            if (room > usable) { room = usable; }
        }
    }

    if (room > profile.maxRecvChunk) { room = profile.maxRecvChunk; }
    if (room == 0) { return; }

    GsmCmdBuf command;
    driver->fetchCommand(_socketId, room, _endpoint, command);
    if (command.isEmpty() || command.overflowed()) { return; }

    GsmCommand request;
    request.text = command.text();
    request.timeoutMs = profile.socketFetchTimeoutMs;
    request.onDone = &GsmSocket::onFetchDone;
    request.onReply = &GsmSocket::onFetchReply;
    request.context = this;

    // A module whose payload begins part-way through its header line says so,
    // and the engine arms the parser when the command actually starts. By the
    // time a header has been read as a whole line it is already too late: the
    // bytes after the comma have been scanned as text, and any line ending
    // inside them has cut the reply short.
    request.inlinePayloadPrefix = driver->inlineFetchPrefix(_endpoint);

    // A FETCH THAT CONCLUDES AFTER ITS OK, not on it. Empty for the ordinary
    // shape; see GsmDriver::fetchDeferredPrefix().
    GsmCmdBuf deferred;
    driver->fetchDeferredPrefix(_socketId, _endpoint, deferred);
    if (deferred.length() > 0) { request.deferredPrefix = deferred.text(); }

    if (modem->engine().submit(request).status() == GsmStatus::Queued) {
        _busy = true;
        _expectedFetch = 0;
        _fetchRequested = room;
        _fetchReceived = 0;
        _fetchRemaining = 0;
        _fetchHadRemaining = false;
    }
}

void GsmSocket::onFetchReply(void* context, const char* line, size_t length) {
    GsmSocket* self = (GsmSocket*)context;
    GsmDriver* driver = self->_owner->modem()->driver();
    if (driver == nullptr) { return; }

    // A header, or the payload itself. The engine delivers payload chunks
    // through this same handler as raw bytes, so the header is recognised
    // first and everything after it is data.
    GsmFetchInfo info;
    if (self->_expectedFetch == 0 &&
        driver->parseFetchHeader(line, length, info, self->_endpoint)) {
        // MORE THAN WAS ASKED FOR IS NOT AN ANSWER TO THIS QUESTION. A length
        // past the request is a corrupt header - a byte lost on a line with
        // no flow control turns "1460" into anything - and arming the parser
        // with it swallowed every reply behind it as payload until the fetch
        // timed out: "4000000000" took the link for eight seconds. Unarmed,
        // whatever follows is read as lines and the fetch still concludes on
        // its OK. The stream itself cannot be trusted past this point, so the
        // socket ends, as it does for any hole in the stream.
        if (info.length > self->_fetchRequested) {
            if (!self->_releasing) { self->fail(GsmStatus::Error); }
            return;
        }
        self->_expectedFetch = info.length;
        self->_fetchReceived = info.length;
        self->_fetchRemaining = info.remaining;
        self->_fetchHadRemaining = info.hasRemaining;

        // On a module with an inline payload the parser has ALREADY switched
        // itself - it did so at the comma, which is the only place it could.
        // Telling it again here would restart the count and swallow the OK.
        if (driver->inlineFetchPrefix(self->_endpoint) != nullptr) { return; }

        // TELLING THE PARSER HOW MANY BYTES FOLLOW is the whole reason a
        // socket payload survives intact. Without it the parser would keep
        // scanning for line endings, and a payload containing CR or LF - which
        // a TLS record always does - would be cut into pieces and the pieces
        // read as replies.
        if (info.length > 0) {
            self->_owner->modem()->engine().parser().expectPayload(info.length);
        }
        return;
    }

    if (self->_expectedFetch > 0) { self->takePayload((const uint8_t*)line, length); }
}

void GsmSocket::takePayload(const uint8_t* data, size_t length) {
    // The last fetch of a socket already ended: bytes nobody will read, from a
    // connection nobody holds. Counted, they would turn up in the totals of
    // whatever connection this slot carries next.
    if (_releasing) {
        if (_expectedFetch >= length) { _expectedFetch -= length; }
        else { _expectedFetch = 0; }
        return;
    }

    size_t stored = 0;
    for (size_t i = 0; i < length && _rxCount < _rxSize; i++) {
        const size_t tail = (_rxHead + _rxCount) % _rxSize;
        _rx[tail] = data[i];
        _rxCount++;
        stored++;
    }

    // COUNT WHAT WAS KEPT, NOT WHAT ARRIVED.
    //
    // This used to add the whole length whether or not the ring had room, so a
    // socket that was quietly dropping data reported having received it. That
    // is the worst shape a counter can take: the one number an operator would
    // check to find out whether bytes were being lost said they were not.
    //
    // The fetch is sized against free room before it is asked for, so a
    // shortfall here means the modem sent more than it was asked for - which
    // is worth counting separately rather than hiding in the received total.
    _bytesReceived += (uint32_t)stored;
    if (stored < length) { _bytesDropped += (uint32_t)(length - stored); }

    if (_expectedFetch >= length) { _expectedFetch -= length; }
    else { _expectedFetch = 0; }
}

void GsmSocket::onFetchDone(void* context, GsmResult result) {
    GsmSocket* self = (GsmSocket*)context;
    self->_busy = false;
    if (self->_releasing) { self->finishRelease(); return; }
    self->_expectedFetch = 0;
    self->armUnreadCheck();

    if (!result.isOk()) {
        // An empty-buffer refusal is the module saying "nothing there", which
        // is an answer rather than a fault - the socket is fine and there is
        // simply no more to collect.
        self->_dataWaiting = false;
        if (result.status() == GsmStatus::SocketClosed ||
            result.status() == GsmStatus::BearerLost) {
            self->fail(result);
            return;
        }

        // BUT NOT ONE THAT FAILED AFTER ITS HEADER. That reply was not empty:
        // the module handed over _fetchReceived bytes when it sent the header,
        // and they are gone from it whether or not they all arrived. A reply
        // cut short - bytes lost on a serial link with no flow control, so the
        // parser waits for a payload that never completes - concludes here as
        // a timeout. Carrying on delivers the stream with a hole in it: under
        // TLS a record that never completes, so a socket that looks open and
        // never delivers again; without TLS, wrong data. A byte stream with a
        // hole is not a byte stream, so the socket ends and the application
        // reconnects.
        if (self->_fetchReceived > 0) { self->fail(result); }
        return;
    }

    // A SUCCESSFUL FETCH IS NOT NECESSARILY THE END OF IT, and getting this
    // wrong goes one of two bad ways.
    //
    // The notification is EDGE-TRIGGERED: the module says "data waiting" once,
    // however much there is. Clear the flag on any successful fetch and a
    // reply larger than one fetch is stranded until the next arrival - for a
    // request-response protocol, the second half of an answer is never read.
    // Never clear it and the socket fetches the same bytes forever.
    //
    // So the flag is cleared when the module has said there is nothing left.
    // One family reports that exactly, as a remaining count in its header; the
    // other has no such field, and there a SHORT READ means the same thing -
    // it gave less than was asked for, so its buffer is empty.
    if (self->_fetchHadRemaining) {
        self->_dataWaiting = (self->_fetchRemaining > 0);
    } else {
        self->_dataWaiting = (self->_fetchReceived >= self->_fetchRequested);
    }
}

// =============================================================================
// READ / SERVICE
// =============================================================================

// Nothing, once ended: bytes a released socket's last fetch brings in are
// nobody's, and are thrown away with the slot.
size_t GsmSocket::readable() { return _releasing ? 0 : _rxCount; }

size_t GsmSocket::read(uint8_t* into, size_t length) {
    if (into == nullptr || length == 0 || _releasing) { return 0; }

    size_t taken = 0;
    while (taken < length && _rxCount > 0) {
        into[taken++] = _rx[_rxHead];
        _rxHead = (_rxHead + 1) % _rxSize;
        _rxCount--;
    }

    // A peer that closed while bytes were still buffered is reported only once
    // those bytes have been read - and only once the MODULE has no more to
    // give either. See peerClosed().
    if (_rxCount == 0 && !_dataWaiting && _peerClosed &&
        _state == GsmSocketState::Open) {
        fail(GsmStatus::SocketClosed);
    }
    return taken;
}

void GsmSocket::service() {
    if (_busy) { return; }      // one operation at a time per socket

    if (_state == GsmSocketState::Open) {
        // Sending first. An application that writes and then waits for a reply
        // deadlocks if the reply is fetched ahead of the request going out.
        if (_txCount > 0) { beginSend(); return; }
        if (_dataWaiting && _rxCount < _rxSize) { beginFetch(); return; }

        // Drained, and the peer had already gone. Concluding here rather than
        // in read() means an application that never reads still sees the close.
        if (_peerClosed && _rxCount == 0) { fail(GsmStatus::SocketClosed); return; }

        // Nothing announced and nothing to send: ask whether the module is
        // holding bytes whose notice never reached us.
        if (!_dataWaiting) { checkUnread(); }
    }
}

void GsmSocket::armUnreadCheck() {
    GsmDriver* driver = (_owner != nullptr && _owner->modem() != nullptr)
                            ? _owner->modem()->driver() : nullptr;
    const uint32_t every = (driver != nullptr) ? driver->profile().unreadCheckMs : 0;
    _unreadCheckAt = (gsm_time_t)(gsm_millis() + every);
}

void GsmSocket::checkUnread() {
    GsmModem* modem = _owner->modem();
    GsmDriver* driver = modem->driver();
    if (driver == nullptr || driver->profile().unreadCheckMs == 0) { return; }
    if (gsm_isFuture(gsm_millis(), _unreadCheckAt)) { return; }

    // Not again for a whole interval, whether or not this one can be asked.
    armUnreadCheck();

    GsmCmdBuf command;
    driver->unreadQueryCommand(_socketId, _endpoint, command);
    if (command.isEmpty() || command.overflowed()) { return; }

    GsmCommand request;
    request.text = command.text();
    request.timeoutMs = driver->profile().socketFetchTimeoutMs;
    request.onDone = &GsmSocket::onUnreadDone;
    request.onReply = &GsmSocket::onUnreadReply;
    request.context = this;

    // One operation at a time, like a send or a fetch: the answer must not
    // race a fetch that is taking the very bytes it counts.
    if (modem->engine().submit(request).status() == GsmStatus::Queued) {
        _busy = true;
    }
}

void GsmSocket::onUnreadReply(void* context, const char* line, size_t length) {
    GsmSocket* self = (GsmSocket*)context;
    GsmDriver* driver = self->_owner->modem()->driver();
    if (driver == nullptr) { return; }

    size_t held = 0;
    if (driver->parseUnread(line, length, self->_socketId, self->_endpoint, held) &&
        held > 0) {
        self->_dataWaiting = true;
    }
}

void GsmSocket::onUnreadDone(void* context, GsmResult result) {
    (void)result;       // a refused question changes nothing; it is asked again
    GsmSocket* self = (GsmSocket*)context;
    self->_busy = false;
    if (self->_releasing) { self->finishRelease(); return; }
    self->armUnreadCheck();
}

void GsmSocket::fail(GsmResult reason) {
    _lastError = reason;
    _state = GsmSocketState::Failed;
    _txCount = 0;
    _txInFlight = 0;
}
