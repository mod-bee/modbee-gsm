// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#pragma once

// The SIM707x dialect: SIM7070, SIM7080, SIM7090 and relatives. LTE Cat-M1 and
// NB-IoT with GPRS fallback.
//
// Written from the SIM7070_SIM7080_SIM7090 Series AT Command Manual V1.05
// (384 pages), with V1.03 consulted where the two differ - and they do.
//
// THIS DRIVER IS THE PROOF THAT THE ABSTRACTION IS REAL. It shares a vendor
// with the A76XX driver and almost nothing else in the IP stack: a different
// bearer command, a different socket open, a different receive, a different
// identity query with a different reply shape, a different location command
// with a different field order and a different coordinate format. Everything
// it does NOT repeat - the queue, retry, backoff, reconnection, the bring-up
// sequence, the one-conclusion contract - is inherited.

#include "GsmDriver3gpp.h"

class GsmDriverSIM707x : public GsmDriver3gpp {
public:
    GsmDriverSIM707x();

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
    void radioReadbackCommand(uint8_t index, GsmCmdBuf& out) const override;
    bool parseRadioConfig(const char* line, size_t length,
                          GsmRadioConfig& out) const override;
    void addressCommand(const GsmApn& apn, GsmCmdBuf& out) const override;
    bool parseAddress(const char* line, size_t length, GsmField& out) const override;

    GsmStep tlsSetupStep(uint8_t index, uint8_t socketId,
                         const GsmEndpoint& endpoint, GsmCmdBuf& out) const override;
    void openCommand(uint8_t socketId, const GsmEndpoint& endpoint,
                     const GsmApn& apn, GsmCmdBuf& out) const override;
    void sendCommand(uint8_t socketId, size_t length, const GsmEndpoint& endpoint,
                     GsmCmdBuf& out) const override;
    void fetchCommand(uint8_t socketId, size_t maxBytes,
                      const GsmEndpoint& endpoint, GsmCmdBuf& out) const override;
    void closeCommand(uint8_t socketId, const GsmEndpoint& endpoint,
                      GsmCmdBuf& out) const override;

    bool parseOpenResult(const char* line, size_t length, GsmOpenResult& out,
                         const GsmEndpoint& endpoint) const override;
    bool parseFetchHeader(const char* line, size_t length, GsmFetchInfo& out,
                          const GsmEndpoint& endpoint) const override;
    const char* inlineFetchPrefix(const GsmEndpoint& endpoint) const override;
    GsmStep bandConfigStep(uint8_t index, const GsmBandList& catM,
                           const GsmBandList& nbIot,
                           GsmCmdBuf& out) const override;
    GsmStep radioPreferenceStep(uint8_t index, GsmRadioPreference wanted,
                                GsmCmdBuf& out) const override;
    bool parseSendResult(const char* line, size_t length, size_t requested,
                         GsmResult& out, const GsmEndpoint& endpoint) const override;

    GsmUrcKind classifyUrc(const char* line, size_t length, GsmUrcInfo& out) const override;
    const char* urcPrefix(uint8_t index) const override;

    void powerDownCommand(GsmCmdBuf& out) const override;
    void resetCommand(GsmCmdBuf& out) const override;

    void gnssPowerCommand(bool on, GsmCmdBuf& out) const override;
    void gnssInfoCommand(GsmCmdBuf& out) const override;
    bool parseGnssInfo(const char* line, size_t length, GsmFix& out) const override;

private:
    GsmProfile _profile;
};
