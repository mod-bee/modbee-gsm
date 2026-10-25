// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#pragma once

// The A76XX dialect: A7670, A7672, A7678 and relatives. LTE Cat-1 with 2G
// fallback.
//
// Written from the A76XX Series AT Command Manual V1.12 (684 pages), its
// TCP/IP AT Command Manual V1.00, its TCP/IP Application Note V1.02, and the
// A7670 Series Hardware Design V1.00 for the power timings. Every command
// below cites the section that defines it.
//
// --- A NOTE ON WHICH MANUAL ---
//
// There is a SIM767XX Series manual that looks like it covers these parts and
// does not: its scope is the QCX216/SIM7670 platform. Built against it this
// driver would report FOUR sockets where this family has TEN, and a different
// reference-signal conversion. The mistake is easy to make and expensive, so
// it is written down here rather than only in a commit message.

#include "GsmDriver3gpp.h"

class GsmDriverA76XX : public GsmDriver3gpp {
public:
    GsmDriverA76XX();

    const GsmProfile& profile() const override { return _profile; }
    bool claims(const char* identification, const char* revision) const override;

    GsmStep initStep(uint8_t index, GsmCmdBuf& out) const override;

    void iccidCommand(GsmCmdBuf& out) const override;
    bool parseIccid(const char* line, size_t length, GsmField& out) const override;

    void systemInfoCommand(GsmCmdBuf& out) const override;
    bool parseSystemInfo(const char* line, size_t length, GsmSignal& signal,
                         GsmTechnology& technology) const override;

    GsmStep bearerUpStep(uint8_t index, const GsmApn& apn, GsmCmdBuf& out) const override;
    GsmStep bearerDownStep(uint8_t index, const GsmApn& apn, GsmCmdBuf& out) const override;

    GsmStep radioPreferenceStep(uint8_t index, GsmRadioPreference wanted,
                                GsmCmdBuf& out) const override;
    GsmStep bandConfigStep(uint8_t index, const GsmBandList& catM,
                           const GsmBandList& nbIot, GsmCmdBuf& out) const override;
    void addressCommand(const GsmApn& apn, GsmCmdBuf& out) const override;
    bool parseAddress(const char* line, size_t length, GsmField& out) const override;

    GsmStep tlsSetupStep(uint8_t index, uint8_t socketId,
                         const GsmEndpoint& endpoint, GsmCmdBuf& out) const override;
    void openCommand(uint8_t socketId, const GsmEndpoint& endpoint,
                     const GsmApn& apn, GsmCmdBuf& out) const override;
    const char* openDeferredPrefix(const GsmEndpoint& endpoint) const override;
    void sendCommand(uint8_t socketId, size_t length, const GsmEndpoint& endpoint,
                     GsmCmdBuf& out) const override;
    void fetchCommand(uint8_t socketId, size_t maxBytes,
                      const GsmEndpoint& endpoint, GsmCmdBuf& out) const override;
    void fetchDeferredPrefix(uint8_t socketId, const GsmEndpoint& endpoint,
                             GsmCmdBuf& out) const override;
    void unreadQueryCommand(uint8_t socketId, const GsmEndpoint& endpoint,
                            GsmCmdBuf& out) const override;
    bool parseUnread(const char* line, size_t length, uint8_t socketId,
                     const GsmEndpoint& endpoint, size_t& out) const override;
    void closeCommand(uint8_t socketId, const GsmEndpoint& endpoint,
                      GsmCmdBuf& out) const override;

    bool parseOpenResult(const char* line, size_t length, GsmOpenResult& out,
                         const GsmEndpoint& endpoint) const override;
    bool parseFetchHeader(const char* line, size_t length, GsmFetchInfo& out,
                          const GsmEndpoint& endpoint) const override;
    // [A76XX §15.2.4 / §19.2.14] Both stacks conclude a send on a second
    // line, and they are DIFFERENT lines - so which one to wait for depends
    // on whether this socket is TLS.
    const char* sendDeferredPrefix(const GsmEndpoint& endpoint) const override {
        return endpoint.security == GsmSecurity::Module ? "+CCHSEND:" : "+CIPSEND:";
    }
    bool parseSendResult(const char* line, size_t length, size_t requested,
                         GsmResult& out, const GsmEndpoint& endpoint) const override;

    GsmUrcKind classifyUrc(const char* line, size_t length, GsmUrcInfo& out) const override;
    const char* urcPrefix(uint8_t index) const override;
    bool isFinalResult(const char* line, size_t length, GsmResult& out) const override;

    void powerDownCommand(GsmCmdBuf& out) const override;
    void resetCommand(GsmCmdBuf& out) const override;

    void gnssPowerCommand(bool on, GsmCmdBuf& out) const override;
    void gnssInfoCommand(GsmCmdBuf& out) const override;
    bool parseGnssInfo(const char* line, size_t length, GsmFix& out) const override;

private:
    GsmProfile _profile;
};
