// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#include "GsmSms.h"
#include "GsmCmdBuf.h"
#include "GsmNoExcept.h"

void GsmSms::begin(GsmModem* modem) {
    _modem = modem;
    _configured = false;
    _configuring = false;
    _configureIndex = 0;
}

void GsmSms::configure() {
    if (_modem == nullptr || _modem->driver() == nullptr) { return; }
    if (_configuring || isReady()) { return; }
    _configuring = true;
    _configureIndex = 0;
    _configuringEpoch = _modem->moduleEpoch();
    advanceConfigure();
}

void GsmSms::advanceConfigure() {
    GsmCmdBuf command;
    const GsmStep step = _modem->driver()->smsInitStep(_configureIndex, command);
    if (!step.present) {
        // Credited to the epoch it STARTED in. A module that restarted part
        // way through took the earlier steps with it, so a sequence that
        // straddles a restart finishes not-ready and is run again.
        _configured = true;
        _configuredEpoch = _configuringEpoch;
        _configuring = false;
        return;
    }

    GsmCommand request;
    request.text = command.text();
    request.timeoutMs = step.timeoutMs;
    request.onDone = &GsmSms::onConfigureDone;
    request.context = this;
    if (_modem->engine().submit(request).status() != GsmStatus::Queued) {
        _configuring = false;    // retried on the next configure() call
    }
}

void GsmSms::onConfigureDone(void* context, GsmResult result) {
    GsmSms* self = (GsmSms*)context;

    // CANCELLED IS NOT REFUSED. A refusal is the module answering, and a
    // module that does not offer one of these settings still sends and
    // receives with the rest - so a refused step does not stop the sequence.
    // Cancelled means the module never answered at all: it restarted under
    // us, or the link was ended. Carrying on from there used to submit the
    // rest into the same sweep and then mark the whole thing configured, so
    // a module that had just rebooted into text mode with no message
    // indications was reported ready for SMS. The sequence is dropped
    // instead, unconfigured, and the next configure() starts it again.
    if (result.status() == GsmStatus::Cancelled) {
        self->_configuring = false;
        self->_configureIndex = 0;
        return;
    }

    self->_configureIndex++;
    self->advanceConfigure();
}

GsmResult GsmSms::send(const char* number, const char* text, GsmDoneFn done,
                       void* context) {
    if (_modem == nullptr || _modem->driver() == nullptr) {
        return GsmStatus::NotInitialised;
    }

    // Asked of the profile rather than attempted and failed. A module without
    // PDU messaging cannot carry this however well the PDU is built.
    if (!_modem->driver()->profile().hasSmsPdu) {
        return GsmStatus::NotSupportedByModule;
    }
    if (_sending) { return GsmStatus::QueueFull; }
    if (number == nullptr || text == nullptr) { return GsmStatus::InvalidArgument; }

    uint8_t pdu[GSM_SMS_PDU_MAX];
    size_t tpduOctets = 0;
    const size_t length = GsmSmsPdu::encodeSubmit(number, text, pdu, sizeof pdu,
                                                  tpduOctets);
    if (length == 0) {
        // The codec refused: a character outside the default alphabet, a
        // number that is not one, or more than one message's worth of text.
        return GsmStatus::InvalidArgument;
    }

    // The module is handed HEX TEXT, not octets - which is why the outgoing
    // buffer is twice the PDU size plus a terminator.
    _outgoingLength = GsmSmsPdu::octetsToHex(pdu, length, _outgoing,
                                             sizeof _outgoing);
    if (_outgoingLength == 0) { return GsmStatus::TooLarge; }

    // CTRL-Z, AND WITHOUT IT NOTHING IS EVER SENT.
    //
    // [A76XX §9.2.13 AT+CMGS] in PDU mode:
    //
    //     AT+CMGS=<length>
    //     PDU is entered
    //     <CTRL-Z/ESC>
    //
    // The terminator is what submits the message; ESC (0x1B) would cancel it.
    // Send the PDU without either and the module simply waits at its prompt
    // for more, forever.
    //
    // THE FAILURE MODE IS SILENT AND COMPLETE. The library reported the send
    // as accepted, the PDU encoded correctly, the module answered "OK" to
    // AT+CMGF=0 and gave its ">" prompt on cue - and the operator's dashboard
    // showed no SMS activity at all, because the message was never submitted.
    // Nothing in this library could have noticed: every step it took
    // succeeded.
    //
    // The buffer is sized PDU_MAX * 2 + 2, so the hex plus this plus a
    // terminator always fits.
    if (_outgoingLength + 1 >= sizeof _outgoing) { return GsmStatus::TooLarge; }
    _outgoing[_outgoingLength++] = 0x1A;
    _outgoing[_outgoingLength] = '\0';

    GsmCmdBuf command;
    // THE LENGTH DECLARED IS NOT THE LENGTH SENT. It excludes the service
    // centre address octet, while the hex that follows includes it.
    _modem->driver()->smsSendCommand(number, tpduOctets, command);
    // An EMPTY command means the driver has no such command for this
    // module - a different answer from "your arguments were wrong", and it
    // sends whoever reads it somewhere different.
    if (command.isEmpty()) { return GsmStatus::NotSupportedByModule; }
    if (command.overflowed()) { return GsmStatus::TooLarge; }

    GsmCommand request;
    request.text = command.text();
    request.payload = (const uint8_t*)_outgoing;
    request.payloadLength = _outgoingLength;
    request.timeoutMs = 60000;          // a send can wait on the network

    // ALWAYS OUR OWN HANDLER, with the caller's kept beside it. The buffer the
    // engine is borrowing belongs to this object, so this object has to be the
    // one told when the engine is finished with it.
    _sendDone = done;
    _sendContext = context;
    request.onDone = &GsmSms::onSendDone;
    request.context = this;

    const GsmResult accepted = _modem->engine().submit(request);
    if (accepted.status() == GsmStatus::Queued) {
        _sending = true;
        _sent++;
    } else {
        _sendDone = nullptr;
        _sendContext = nullptr;
    }
    return accepted;
}

void GsmSms::onSendDone(void* context, GsmResult result) {
    GsmSms* self = (GsmSms*)context;

    // Released FIRST, so a handler that sends the next message from inside
    // this one is not refused by a flag that is about to be cleared anyway.
    self->_sending = false;

    GsmDoneFn done = self->_sendDone;
    void* callerContext = self->_sendContext;
    self->_sendDone = nullptr;
    self->_sendContext = nullptr;

    if (done != nullptr) { done(callerContext, result); }
}

GsmResult GsmSms::fetch(uint16_t index) {
    if (_modem == nullptr || _modem->driver() == nullptr) {
        return GsmStatus::NotInitialised;
    }

    GsmCmdBuf command;
    _modem->driver()->smsReadCommand(index, command);
    // An EMPTY command means the driver has no such command for this
    // module - a different answer from "your arguments were wrong", and it
    // sends whoever reads it somewhere different.
    if (command.isEmpty()) { return GsmStatus::NotSupportedByModule; }
    if (command.overflowed()) { return GsmStatus::TooLarge; }

    _pduLength = 0;
    _expectingPdu = false;

    GsmCommand request;
    request.text = command.text();
    request.timeoutMs = 10000;
    request.onReply = &GsmSms::onFetchReply;
    request.onDone = &GsmSms::onFetchDone;
    request.context = this;
    return _modem->engine().submit(request);
}

GsmResult GsmSms::remove(uint16_t index) {
    if (_modem == nullptr || _modem->driver() == nullptr) {
        return GsmStatus::NotInitialised;
    }
    GsmCmdBuf command;
    _modem->driver()->smsDeleteCommand(index, command);
    // An EMPTY command means the driver has no such command for this
    // module - a different answer from "your arguments were wrong", and it
    // sends whoever reads it somewhere different.
    if (command.isEmpty()) { return GsmStatus::NotSupportedByModule; }
    if (command.overflowed()) { return GsmStatus::TooLarge; }

    GsmCommand request;
    request.text = command.text();
    request.timeoutMs = 10000;
    return _modem->engine().submit(request);
}

void GsmSms::onFetchReply(void* context, const char* line, size_t length) {
    GsmSms* self = (GsmSms*)context;

    // A read answers with a header line and then the PDU as hexadecimal text.
    // The header is recognised by its prefix; anything after it that is all
    // hexadecimal is the PDU.
    if (length >= 5 && line[0] == '+') {
        self->_expectingPdu = true;
        return;
    }

    if (!self->_expectingPdu || length < 2) { return; }

    const size_t octets = GsmSmsPdu::hexToOctets(line, length, self->_pdu,
                                                 sizeof self->_pdu);
    if (octets > 0) {
        self->_pduLength = octets;
        self->_expectingPdu = false;
    }
}

void GsmSms::onFetchDone(void* context, GsmResult result) {
    GsmSms* self = (GsmSms*)context;
    self->_expectingPdu = false;

    if (!result.isOk() || self->_pduLength == 0) { return; }

    GsmSmsMessage message;
    if (!GsmSmsPdu::decodeDeliver(self->_pdu, self->_pduLength, message)) { return; }

    self->_received++;
    if (self->_handler != nullptr) { self->_handler(self->_context, message); }
}

void GsmSms::handleEvent(GsmUrcKind kind, const GsmUrcInfo& info) {
    if (kind != GsmUrcKind::SmsReceived) { return; }

    // The module announced an index. Fetching it now is right: the store is
    // small, and a message left there until somebody asks is a message lost
    // when the store fills.
    fetch((uint16_t)info.value);
}
