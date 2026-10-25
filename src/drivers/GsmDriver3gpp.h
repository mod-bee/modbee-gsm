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
// THE PART EVERY MODEM SHARES
// =============================================================================
//
// A CONCRETE base class, not an interface. It implements everything defined by
// 3GPP TS 27.007, TS 27.005 and ITU-T V.25ter - which is roughly seventy per
// cent of what this library ever sends, and is identical on every AT modem
// ever made: echo, error verbosity, identification, SIM state, signal,
// registration, operator selection, context definition, clock, SMS.
//
// A NEW DRIVER SUBCLASSES THIS AND OVERRIDES THE IP STACK AND THE LOCATION
// COMMANDS. That is the quarter that is genuinely vendor dialect, and it is
// why a driver here is a few hundred lines rather than the several hundred
// per module that a copy-everything design needs.
//
// Everything in this file cites the standard that defines it. Nothing is
// written from memory.

#include "../GsmDriver.h"

class GsmDriver3gpp : public GsmDriver {
public:
    // --- Bring-up, the universal prelude ---
    //
    // Four steps, in this order, and the order is not arbitrary.
    GsmStep initStep(uint8_t index, GsmCmdBuf& out) const override;

    // --- Identity ---
    void identificationCommand(GsmCmdBuf& out) const override;
    void revisionCommand(GsmCmdBuf& out) const override;
    void imeiCommand(GsmCmdBuf& out) const override;
    void imsiCommand(GsmCmdBuf& out) const override;

    // --- Network ---
    void signalCommand(GsmCmdBuf& out) const override;
    void registrationCommand(uint8_t index, GsmCmdBuf& out) const override;
    void operatorCommand(GsmCmdBuf& out) const override;
    void simStatusCommand(GsmCmdBuf& out) const override;
    void simUnlockCommand(const char* pin, GsmCmdBuf& out) const override;
    void radioReadbackCommand(uint8_t index, GsmCmdBuf& out) const override;
    bool parseRadioConfig(const char* line, size_t length,
                          GsmRadioConfig& out) const override;

    // --- Bearer: the context definition half, which IS standard ---
    //
    // Activating it is not - one module opens a socket service, another
    // activates an application context with its own command - so subclasses
    // continue the sequence from where this leaves off.
    GsmStep bearerUpStep(uint8_t index, const GsmApn& apn, GsmCmdBuf& out) const override;

    // --- Time ---
    void clockCommand(GsmCmdBuf& out) const override;
    bool parseClock(const char* line, size_t length,
                    GsmParse::GsmDateTime& out) const override;

    // --- SMS ---
    GsmStep smsInitStep(uint8_t index, GsmCmdBuf& out) const override;
    void smsSendCommand(const char* number, size_t textLength,
                        GsmCmdBuf& out) const override;
    void smsReadCommand(uint16_t index, GsmCmdBuf& out) const override;
    void smsDeleteCommand(uint16_t index, GsmCmdBuf& out) const override;

    // --- Shared parsing helpers, usable by any subclass ---

    // [3GPP TS 27.007 7.2] +CREG / +CGREG / +CEREG all carry <stat> in the
    // same place with the same meanings, so one parser serves all three.
    // `hasMode` says whether the reply leads with the <n> setting, which a
    // read command does and an unsolicited report does not.
    static bool parseRegistration(const char* line, size_t length,
                                  const char* prefix, bool hasMode,
                                  GsmRegistration& state, GsmTechnology& technology);

    // [3GPP TS 27.007 8.5] +CSQ: <rssi>,<ber>
    static bool parseSignalQuality(const char* line, size_t length, GsmSignal& out);

    // [3GPP TS 27.007 7.3] +COPS: <mode>,<format>,<oper>[,<AcT>]
    static bool parseOperator(const char* line, size_t length, GsmField& name);

    // [3GPP TS 27.007 8.3] +CPIN: <code>
    static bool parseSimStatus(const char* line, size_t length, bool& ready,
                               bool& needsPin, bool& needsPuk);

    // [3GPP TS 27.007 6.4] Maps the <AcT> value onto our vocabulary.
    static GsmTechnology technologyFromAccessTechnology(uint32_t value);

    // Whether a registration line is the answer to OUR QUERY rather than an
    // unsolicited report.
    //
    // THE SAME PREFIX SERVES BOTH, and telling them apart is not optional:
    //
    //   read reply   +CEREG: <n>,<stat>[,<tac>,<ci>[,<AcT>]]
    //   unsolicited  +CEREG: <stat>[,<tac>,<ci>[,<AcT>]]
    //
    // A URC handler that claimed the read form would swallow the answer to the
    // registration query, and the bring-up machine would wait for a reply that
    // had already been eaten - registering forever on a network that had
    // accepted it. Found exactly that way.
    //
    // The discrimination is structural and does not depend on knowing what was
    // asked: in the read form the SECOND field is a bare number (<stat>); in
    // the unsolicited form it is a QUOTED tracking-area code. One field means
    // unsolicited either way.
    static bool isRegistrationReadReply(const char* line, size_t length,
                                        const char* prefix);

protected:
    // How many steps initStep() owns, so a subclass continuing the sequence
    // knows where its own begin.
    // Attention, echo off, error verbosity, character set, the two
    // registration URC enables, PSM off and eDRX off.
    static const uint8_t kInitSteps = 8;

    // Likewise for the standard half of bringing a bearer up.
    static const uint8_t kBearerSteps = 1;
};
