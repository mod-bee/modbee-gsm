// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#pragma once

// A byte pipe - the library's product. Seven methods, and nothing in them
// knows or cares what is carried over it.
//
//   begin()      starts the open and RETURNS; state() says how it is going
//   readable()   what has arrived
//   read()       takes it, never blocks
//   write()      takes what it can and says how much; partial is ordinary
//   isConnected()
//   end()
//
// IT CARRIES OPAQUE BYTES AND NEVER INSPECTS THEM, which is what lets wolfSSL
// run above it rather than inside. No hex mode, no escaping, exact counts both
// ways, NUL passes through - a TLS record contains CR, LF and NUL early.
//
// Three conditions that are NOT the same, told apart by lastError():
//
//   SocketClosed  this socket went; others may be fine  -> reopen this one
//   BearerLost    the context went; all sockets dead    -> bearer, then all
//   Unresponsive  the modem stopped answering           -> recovery ladder

#include "GsmPlatform.h"
#include "GsmResult.h"
#include "GsmTypes.h"
#include "GsmModem.h"

// The DEFAULT socket count and per-socket buffers; GsmSockets::setCapacity()
// (GsmDevice::setCapacity(), GSM_MODEM's SOCKETS and SOCKET_BUFFER) changes
// them before the modem is opened. Deliberately modest: a profile may declare
// thirteen sockets, and thirteen times two 1500-byte buffers is 39 kB that few
// boards have spare. The setting is what the application can afford; the
// profile is what the module could manage, and the smaller of the two wins.
#ifndef GSM_SOCKET_TX_BUFFER
#define GSM_SOCKET_TX_BUFFER 1500
#endif

#ifndef GSM_SOCKET_RX_BUFFER
#define GSM_SOCKET_RX_BUFFER 1500
#endif

#ifndef GSM_SOCKET_SLOTS
#define GSM_SOCKET_SLOTS 2
#endif

enum class GsmSocketState : uint8_t {
    Closed = 0,
    Opening,     // begin() was called; the modem is working on it
    Open,
    Closing,
    Failed,      // it will not open, or it died; lastError() says why
};

const char* gsm_socketStateName(GsmSocketState state);

class GsmSockets;

class GsmSocket {
public:
    // --- The byte-pipe interface ---

    // Starts the open and RETURNS. Never waits for the network.
    //
    // This one call is the whole difference from every blocking GSM library:
    // there is nowhere in a `bool connect()` signature to put "still trying",
    // so a library shaped that way stalls its caller for as long as a socket
    // open takes - which one manual specifies as up to 120 seconds.
    bool begin();

    // Lets the socket go. The pointer must not be used afterwards.
    //
    // When a command of this socket's is still out with the module - an open
    // that can take two minutes, a fetch, a send - the SLOT is held until that
    // command concludes, and only then handed back. See _releasing.
    void end();

    size_t readable();
    size_t read(uint8_t* into, size_t length);
    size_t write(const uint8_t* from, size_t length);
    bool isConnected() const { return _state == GsmSocketState::Open; }

    // The far end sent a close and will send no more. NOT the same question as
    // isConnected(): bytes it already sent may still be buffered here, and
    // discarding them because the peer went would lose the last reply of every
    // request-response exchange.
    // True only once there is genuinely nothing left: the peer has gone, our
    // buffer is empty, AND the module is not still holding an announced read.
    bool peerClosed() const {
        return _peerClosed && _rxCount == 0 && !_dataWaiting;
    }

    // --- State ---

    // Where the NEXT datagram goes.
    //
    // A datagram socket is bound rather than connected: the local port is what
    // it was opened with, and the destination travels with each packet. So the
    // target is set per send, not once at open.
    //
    // Worth its own method because the alternative - building an endpoint and
    // hoping the send picks it up - is exactly the bug this replaced: the
    // address was computed on every packet and silently discarded, so every
    // datagram went wherever the socket had been opened. A compiler warning
    // about an unused variable is what caught it.
    //
    // Refused - false - on a stream socket, which has one peer for its
    // lifetime.
    //
    // AND FALSE WHILE A DATAGRAM IS STILL WAITING TO GO. Its destination was fixed
    // when it was written; changing it now would send that datagram to the
    // address meant for the next one.
    bool setDatagramTarget(const char* host, uint16_t port);

    GsmSocketState state() const { return _state; }
    GsmResult lastError() const { return _lastError; }
    uint8_t id() const { return _socketId; }
    const GsmEndpoint& endpoint() const { return _endpoint; }

    // Bytes still waiting to go out. Non-zero means write() has accepted data
    // the modem has not taken yet.
    size_t pendingOut() const { return _txCount; }

    // --- Counters ---
    uint32_t bytesSent() const { return _bytesSent; }
    uint32_t bytesReceived() const { return _bytesReceived; }

    // Payload bytes the modem delivered that did not fit the receive ring.
    // Non-zero means the application is not reading fast enough, or a fetch
    // came back larger than it was asked for. Either way it is data loss and
    // it is reported rather than absorbed.
    uint32_t bytesDropped() const { return _bytesDropped; }

private:
    friend class GsmSockets;

    void configure(GsmSockets* owner, GsmEndpoint endpoint);
    void service();
    void reset();

    void beginOpen();
    void beginSend();
    void beginFetch();
    void checkUnread();
    void armUnreadCheck();
    void fail(GsmResult reason);
    void submitClose();
    void finishRelease();
    size_t datagramLimit() const;

    static void onOpenDone(void* context, GsmResult result);
    static void onTlsSetupDone(void* context, GsmResult result);
    static void onOpenReply(void* context, const char* line, size_t length);
    static void onSendDone(void* context, GsmResult result);
    static void onSendReply(void* context, const char* line, size_t length);
    static void onFetchDone(void* context, GsmResult result);
    static void onFetchReply(void* context, const char* line, size_t length);
    static void onUnreadDone(void* context, GsmResult result);
    static void onUnreadReply(void* context, const char* line, size_t length);

    void takePayload(const uint8_t* data, size_t length);

    GsmSockets* _owner = nullptr;
    GsmEndpoint _endpoint;

    // Owned, because GsmEndpoint::host is a borrowed pointer and a caller's
    // hostname may be a temporary that is gone by the time the send runs.
    char _hostCopy[64] = {0};

    GsmSocketState _state = GsmSocketState::Closed;
    GsmResult _lastError = GsmStatus::Ok;
    uint8_t _socketId = 0;

    // How far through the driver's TLS setup this socket has got.
    uint8_t _tlsStep = 0;
    bool _inUse = false;
    bool _peerClosed = false;

    // An operation is outstanding with the engine. One at a time per socket:
    // a fetch issued while a send is in flight would interleave two payloads
    // on a channel that carries exactly one.
    bool _busy = false;

    // The module says data is waiting. Cleared when a fetch comes back empty,
    // because the notification is edge-triggered and a fetch that asked for
    // less than was there must not lose the rest.
    bool _dataWaiting = false;

    // The module accepted the open, so it holds a link for this socket until
    // it is closed - by the far end (_peerClosed), or by us in end(). A socket
    // WE fail - a broken stream, a bearer the modem gave up on - still owns
    // that link at the module, and a reopen of the same id would be refused.
    bool _linkOpen = false;

    // end() was called while a command of ours was still out with the engine.
    //
    // THE SLOT IS HELD UNTIL THAT COMMAND CONCLUDES. Releasing it at once was a
    // real defect, three ways over: open() handed the same slot straight back,
    // and the old command's completion then wrote into the NEW socket - a
    // fetch's bytes delivered to a different connection, an old open's result
    // reporting the new socket Open. And when setCapacity() rebuilt the
    // sockets in the meantime, the completion wrote into freed memory.
    //
    // Holding the slot is also simply true: the module is still using that
    // socket id until it answers, and an open of the same id would be refused
    // there. When the answer comes it is recognised as nobody's, discarded,
    // and - if the module opened something - followed by a close. See
    // finishRelease().
    bool _releasing = false;

    // Whether the module may hold a link for this socket when the release
    // completes. Decided at end(); a late successful open sets _linkOpen too.
    bool _closeOnRelease = false;

    // The bearer this socket was opened on - GsmBearer::epoch(). Any other, or
    // none, and the socket is dead whether or not an event said so.
    uint32_t _bearerEpoch = 0;

    // When to ask the module whether it is holding bytes nobody announced -
    // see GsmProfile::unreadCheckMs.
    gsm_time_t _unreadCheckAt = 0;

    // Transmit. THE BUFFER IS OURS AND THE ENGINE BORROWS IT - see
    // GsmCommand::payload. It may not be touched while a send is outstanding,
    // which _busy guarantees.
    uint8_t* _tx = nullptr;       // _txSize bytes, GsmSockets' storage
    size_t  _txSize = 0;
    size_t  _txCount = 0;
    size_t  _txInFlight = 0;

    uint8_t* _rx = nullptr;       // a ring of _rxSize bytes
    size_t  _rxSize = 0;
    size_t  _rxHead = 0;
    size_t  _rxCount = 0;

    size_t _expectedFetch = 0;

    // What the last fetch asked for and what it got, and what the module said
    // was left. Together they decide whether there is more to collect - see
    // onFetchDone() for why a successful fetch is not the end of the matter.
    size_t _fetchRequested = 0;
    size_t _fetchReceived = 0;
    size_t _fetchRemaining = 0;
    bool   _fetchHadRemaining = false;

    uint32_t _bytesSent = 0;
    uint32_t _bytesReceived = 0;
    uint32_t _bytesDropped = 0;
};

// Owns the slots and routes the modem's socket events to them.
class GsmSockets {
public:
    GsmSockets() = default;
    ~GsmSockets();
    GsmSockets(const GsmSockets&) = delete;
    GsmSockets& operator=(const GsmSockets&) = delete;

    // How many sockets this modem can hold open at once, and each one's send
    // and receive buffers in bytes. 0 keeps the default (GSM_SOCKET_SLOTS,
    // GSM_SOCKET_TX_BUFFER, GSM_SOCKET_RX_BUFFER). Takes effect at the next
    // reserve() - begin() makes one - while no socket is in use.
    void setCapacity(uint8_t sockets, size_t txBytes = 0, size_t rxBytes = 0);

    // Takes the storage for every socket now, in one piece. False when there
    // is no room; begin() then fails too, so the owner can say why instead of
    // every open() later coming back empty.
    bool reserve();

    bool begin(GsmModem* modem);
    void poll();

    // A socket, or null when every slot is in use.
    //
    // Refused by NAME rather than by returning something that silently never
    // connects: an application that asks for a fifth socket on a build with
    // four slots is told at the moment it asks, not by a timeout later.
    GsmSocket* open(const char* host, uint16_t port,
                    GsmSocketKind kind = GsmSocketKind::Tcp,
                    GsmSecurity security = GsmSecurity::Plain);

    void close(GsmSocket* socket);

    // What the module declares, and what this build can actually hold.
    uint8_t slots() const { return _count != 0 ? _count : _wantSlots; }
    size_t txBufferSize() const { return _wantTx; }
    size_t rxBufferSize() const { return _wantRx; }
    uint8_t available() const;
    uint8_t openCount() const;

    // Called by the modem when a socket-related URC arrives.
    void handleEvent(GsmUrcKind kind, const GsmUrcInfo& info);

    GsmModem* modem() const { return _modem; }

private:
    friend class GsmSocket;

    GsmSocket* byModuleId(uint8_t socketId, bool secure);

    void release();

    GsmModem* _modem = nullptr;
    GsmSocket* _sockets = nullptr;     // _count of them, then their buffers
    uint8_t _count = 0;
    size_t  _haveTx = 0;               // the buffers _sockets was built with
    size_t  _haveRx = 0;
    uint8_t _wantSlots = GSM_SOCKET_SLOTS;
    size_t  _wantTx = GSM_SOCKET_TX_BUFFER;
    size_t  _wantRx = GSM_SOCKET_RX_BUFFER;
};
