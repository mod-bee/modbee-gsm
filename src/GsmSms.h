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
// SMS
// =============================================================================
//
// Send and receive, in PDU mode, non-blocking like everything else.
//
// A thin object over the modem and the codec: it holds no protocol knowledge
// of its own. What an SMS looks like on the wire is GsmSmsPdu's business; when
// to ask the modem anything is the engine's.
//
// --- WHY AN ARRIVING MESSAGE IS NOT PUSHED AT YOU ---
//
// The module is configured to announce a new message by INDEX rather than to
// deliver it unsolicited. That is a deliberate choice made in the bring-up
// sequence: the alternative pushes a whole multi-line message into the middle
// of whatever command happens to be in flight, and the engine would have to
// unpick two interleaved replies. An index is one short line, and the message
// is fetched when convenient.

#include "GsmPlatform.h"
#include "GsmModem.h"
#include "GsmSmsPdu.h"

// Called when a message has been fetched and decoded.
typedef void (*GsmSmsHandler)(void* context, const GsmSmsMessage& message);

class GsmSms {
public:
    void begin(GsmModem* modem);

    // Runs the module's SMS configuration - PDU mode, and index notification.
    // Queued; isReady() says when it took.
    void configure();

    // Configured, IN THIS LIFE OF THE MODULE. The module keeps neither PDU
    // mode nor message indications across a restart, so a configuration done
    // before one is not a configuration now - see GsmModem::moduleEpoch().
    bool isReady() const {
        return _configured && _modem != nullptr &&
               _configuredEpoch == _modem->moduleEpoch();
    }

    // Whether anybody is listening for arriving messages - and so whether the
    // module has to be kept configured to announce them.
    bool hasSubscriber() const { return _handler != nullptr; }

    // Told about each message as it is decoded.
    void onMessage(GsmSmsHandler handler, void* context) {
        _handler = handler;
        _context = context;
    }

    // Queues a message. Returns Queued on acceptance, or the reason it was
    // refused - text that cannot be represented in the default alphabet, a
    // number that is not one, or more than one message's worth of text.
    //
    // REFUSED RATHER THAN TRUNCATED. Half an SMS that looks plausible is worse
    // than none, because nobody finds out.
    GsmResult send(const char* number, const char* text, GsmDoneFn done = nullptr,
                   void* context = nullptr);

    // Fetches the message at an index the module announced.
    GsmResult fetch(uint16_t index);

    // Deletes one. Worth doing: the SIM's message store is small, and a full
    // store makes the module refuse every new message with an error that does
    // not obviously mean "housekeeping".
    GsmResult remove(uint16_t index);

    // Called by the modem when an SMS URC arrives. Fetches automatically.
    void handleEvent(GsmUrcKind kind, const GsmUrcInfo& info);

    uint32_t received() const { return _received; }
    uint32_t sent() const { return _sent; }

private:
    static void onFetchReply(void* context, const char* line, size_t length);
    static void onFetchDone(void* context, GsmResult result);
    static void onConfigureDone(void* context, GsmResult result);

    void advanceConfigure();

    GsmModem* _modem = nullptr;
    GsmSmsHandler _handler = nullptr;
    void* _context = nullptr;

    bool    _configured = false;
    uint8_t _configureIndex = 0;
    bool    _configuring = false;

    // The module epoch a configuration was started in, and the one the last
    // completed configuration belongs to.
    uint32_t _configuringEpoch = 0;
    uint32_t _configuredEpoch = 0;

    // The PDU being assembled from a read reply. Held here rather than on the
    // stack because it arrives across two lines - a header, then the hex.
    uint8_t _pdu[GSM_SMS_PDU_MAX];
    size_t  _pduLength = 0;
    bool    _expectingPdu = false;

    // The payload for a send. OWNED HERE AND BORROWED BY THE ENGINE, so it
    // must not be reused while a send is outstanding - which _sending guards.
    char   _outgoing[GSM_SMS_PDU_MAX * 2 + 2];
    size_t _outgoingLength = 0;
    bool   _sending = false;

    // The caller's completion handler, held rather than handed to the engine.
    //
    // It used to go straight into the request, which replaced the handler that
    // clears _sending - so the flag latched on the first send and every later
    // one was refused with QueueFull for the life of the device. SMS worked
    // exactly once per boot, and the hardware test sends exactly one message
    // per run by design, so nothing ever saw the second.
    GsmDoneFn _sendDone = nullptr;
    void*     _sendContext = nullptr;

    static void onSendDone(void* context, GsmResult result);

    uint32_t _received = 0;
    uint32_t _sent = 0;
};
