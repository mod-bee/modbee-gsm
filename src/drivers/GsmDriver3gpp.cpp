// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#include "GsmDriver3gpp.h"
#include "../GsmNoExcept.h"

// =============================================================================
// BRING-UP
// =============================================================================

GsmStep GsmDriver3gpp::initStep(uint8_t index, GsmCmdBuf& out) const {
    out.clear();
    GsmStep step;

    switch (index) {
        case 0:
            // ITU-T V.25ter - a bare attention command.
            //
            // FIRST, AND ITS ONLY JOB IS TO PROVE THE MODULE IS THERE. One of
            // the modules this library drives documents no boot banner at all,
            // and the other suppresses its banner when auto-bauding is active,
            // so polling this until it answers is the ONLY portable way to
            // know the AT port is alive. A library that waited for a banner
            // would hang on both.
            out.add("AT");
            step.present = true;
            step.timeoutMs = 1000;
            return step;

        case 1:
            // ITU-T V.25ter - echo off.
            //
            // Before anything whose reply matters. With echo on, every command
            // comes back before its answer, and a parser that has not been
            // told to expect that reads its own words as the module's.
            out.add("ATE0");
            step.present = true;
            step.timeoutMs = 1000;
            return step;

        case 2:
            // 3GPP TS 27.007 9.1 - report mobile equipment error.
            //
            // SET BEFORE ANYTHING THAT CAN FAIL. Without it every failure is a
            // bare ERROR with no reason; with it, a SIM fault arrives as
            // "+CME ERROR: 13" and that is the difference between a diagnosis
            // and a guess.
            //
            // MODE 1, NUMERIC, NOT MODE 2. Mode 2 reports the same errors as
            // English prose, which varies between firmware revisions - so a
            // parser matching on it starts failing after an update that
            // changed nothing else. A number has one representation.
            out.add("AT+CMEE=1");
            step.present = true;
            step.timeoutMs = 1000;
            return step;

        case 3:
            // 3GPP TS 27.007 5.5 - character set.
            //
            // IRA, so that everything this library reads back is plain ASCII.
            // A module left in a UCS2 character set returns every string as
            // hexadecimal, which parses as nonsense rather than as an error.
            //
            // Optional: a module that does not offer the setting is not
            // broken, it simply has one character set.
            out.add("AT+CSCS=\"IRA\"");
            step.present = true;
            step.timeoutMs = 1000;
            step.ignoreFailure = true;
            return step;
        case 4:
            // 3GPP TS 27.007 7.2/7.10 - registration status URCs, unsolicited.
            //
            // THE LIBRARY ALREADY LISTENS FOR THESE. Both drivers register
            // "+CEREG:" and "+CGREG:" with the URC router - and until now
            // nothing ever asked the module to send them, so the module
            // answered only when polled and every handler sat idle.
            //
            // With them on, a registration that is lost at three in the
            // morning announces itself instead of waiting to be noticed on
            // the next poll.
            // MODE 2, NOT 1, BECAUSE IT SAYS MORE FOR THE SAME COST.
            //
            // 3GPP TS 27.007 7.10: mode 1 reports only <stat>, while mode 2
            // adds <tac>,<ci>,<AcT> - the tracking area, the cell, and the
            // ACCESS TECHNOLOGY. On a module that can try Cat-M1 and NB-IoT
            // the last of those is the difference between "denied" and
            // "denied on NB-IoT, which this SIM is not provisioned for".
            //
            // Mode 3 would add the reject cause itself, and is not offered
            // here: a real SIM7070G answers AT+CEREG=? with "(0-2,4)". So 2
            // is the most this module will give, and asking for 3 would be
            // refused - which is why the supported set is worth reading
            // rather than assuming.
            out.add("AT+CEREG=2");
            step.present = true;
            step.timeoutMs = 9000;

            // Ignored on purpose: a module with no LTE radio refuses this and
            // is perfectly healthy. The polling path remains either way.
            step.ignoreFailure = true;
            return step;

        case 5:
            // 3GPP TS 27.007 10.1.19 - GPRS/PS registration URCs, for the
            // same reason. Refused by modules with no packet radio.
            out.add("AT+CGREG=1");
            step.present = true;
            step.timeoutMs = 9000;
            step.ignoreFailure = true;
            return step;

        case 6:
            // 3GPP TS 27.007 8.27 - POWER SAVING MODE, TURNED OFF.
            //
            // PSM lets a module sleep so deeply that the network stops trying
            // to reach it, and on an LPWA part it is exactly the feature you
            // buy the part for - on a battery tracker that wakes hourly.
            //
            // It is the wrong default for a CONTROLLER, which has mains power
            // and has to be reachable between scans. Worse, a module that
            // sleeps looks identical to one that lost coverage: the operator
            // sees "attached" and then "detached" and the device sees
            // registration come and go, with nothing anywhere saying why.
            //
            // So it is turned off explicitly rather than left to whatever the
            // module shipped with, which the manuals say varies by project.
            // An application that wants PSM can set it afterwards; one that
            // does not should never have to discover it was on.
            // 3GPP TS 27.007 8.27 - AT+CPSMS=0 disables power saving mode.
            out.add("AT+CPSMS=0");
            step.present = true;
            step.timeoutMs = 9000;
            step.ignoreFailure = true;
            return step;

        case 7:
            // 3GPP TS 27.007 8.29 - eDRX, off, for the same reason. It
            // lengthens the interval at which the module listens for paging,
            // which is a smaller version of the same trade and the same
            // surprise.
            out.add("AT+CEDRXS=0");
            step.present = true;
            step.timeoutMs = 9000;
            step.ignoreFailure = true;
            return step;


        default:
            return step;    // present = false: the sequence ends
    }
}

// =============================================================================
// IDENTITY
// =============================================================================

void GsmDriver3gpp::identificationCommand(GsmCmdBuf& out) const {
    // ITU-T V.25ter - request identification information.
    out.clear();
    out.add("ATI");
}

void GsmDriver3gpp::revisionCommand(GsmCmdBuf& out) const {
    // 3GPP TS 27.007 5.3 - request revision identification.
    //
    // Earns its place because TWO PROFILE FACTS VARY BY FIRMWARE rather than
    // by part - a TLS session count, and whether socket closure is pushed as
    // a URC or must be polled. A driver chosen on the part alone is wrong on
    // half the stock in the field.
    out.clear();
    out.add("AT+CGMR");
}

void GsmDriver::attachQueryCommand(GsmCmdBuf& out) const {
    out.clear();
    // 3GPP TS 27.007 10.1.9 - AT+CGATT? reads the attach state.
    out.add("AT+CGATT?");
}

GsmStep GsmDriver::radioCycleStep(uint8_t index, GsmCmdBuf& out) const {
    out.clear();
    GsmStep step;

    if (index == 0) {
        // 3GPP TS 27.007 8.2 - AT+CFUN=0, minimum functionality (radio off).
        out.add("AT+CFUN=0");
    } else if (index == 1) {
        // 3GPP TS 27.007 8.2 - AT+CFUN=1, full functionality (radio on).
        out.add("AT+CFUN=1");
    } else {
        return step;
    }

    step.present = true;

    // Generous: turning a radio off means detaching from whatever it was on,
    // and turning it back on means the stack reinitialising.
    step.timeoutMs = 20000;

    // Ignored. A module that refuses to cycle its radio has simply not
    // applied the new configuration, which is a slower start rather than a
    // broken one.
    step.ignoreFailure = true;
    return step;
}

void GsmDriver::defaultContextCommand(const GsmApn& apn, GsmCmdBuf& out) const {
    out.clear();

    // 3GPP TS 27.007 10.1.1 - AT+CGDCONT=<cid>,<PDP_type>,<APN>
    //
    // The context identifier is one-based in the standard while this library
    // numbers its bearers from zero, so the conversion happens here.
    out.add("AT+CGDCONT=");
    out.add((uint32_t)(apn.contextId + 1));
    out.add(",\"IP\",");
    out.addQuoted(apn.name != nullptr ? apn.name : "");
}

void GsmDriver::attachCommand(GsmCmdBuf& out) const {
    out.clear();
    // 3GPP TS 27.007 10.1.9 - AT+CGATT=1, attach to the packet service.
    out.add("AT+CGATT=1");
}

void GsmDriver::forbiddenNetworksReadCommand(GsmCmdBuf& out) const {
    out.clear();
    // 3GPP TS 27.007 8.18 - AT+CRSM=<command>,<fileid>,<P1>,<P2>,<P3>
    //   176   READ BINARY
    //   28539 0x6F7B, EF_FPLMN - the forbidden network list
    //   12    bytes, which is four networks of three octets each
    out.add("AT+CRSM=176,28539,0,0,12");
}

void GsmDriver::forbiddenNetworksClearCommand(GsmCmdBuf& out) const {
    out.clear();
    // 3GPP TS 27.007 8.18 - 214 is UPDATE BINARY. All-Fs is the empty list.
    //
    // This WRITES TO THE SIM, and that is the point: the list lives on the
    // card and no amount of restarting the module will shift it.
    out.add("AT+CRSM=214,28539,0,0,12,\"FFFFFFFFFFFFFFFFFFFFFFFF\"");
}

bool GsmDriver::parseForbiddenNetworks(const char* line, size_t length,
                                       bool& anyForbidden) const {
    // 3GPP TS 27.007 8.18 - "+CRSM: <sw1>,<sw2>[,<response>]"
    //
    //   +CRSM: 144,0,"FFFFFFFFFFFFFFFFFFFFFFFF"    empty
    //   +CRSM: 144,0,"13F05A..."                   at least one forbidden
    //
    // sw1=144 (0x90) with sw2=0 is the card's "OK". Anything else is not an
    // answer about networks at all.
    GsmField rest, field;
    if (!GsmParse::afterPrefix(line, length, "+CRSM", rest)) { return false; }

    uint32_t sw1 = 0;
    if (!GsmParse::field(rest.data, rest.length, 0, field) ||
        !GsmParse::toUnsigned(field, sw1) || sw1 != 144) { return false; }

    if (!GsmParse::field(rest.data, rest.length, 2, field)) { return false; }

    // Any hex digit that is not 'F' means a network is recorded. Case is not
    // guaranteed, so both are accepted.
    anyForbidden = false;
    for (size_t i = 0; i < field.length; i++) {
        const char c = field.data[i];
        if (c != 'F' && c != 'f') { anyForbidden = true; break; }
    }
    return true;
}

void GsmDriver::operatorScanCommand(GsmCmdBuf& out) const {
    // 3GPP TS 27.007 7.3 - the test form asks what is on air.
    out.clear();
    out.add("AT+COPS=?");
}

bool GsmDriver::parseOperatorScan(const char* line, size_t length,
                                  GsmOperatorList& out) const {
    // 3GPP TS 27.007 7.3 - the reply is a run of tuples:
    //
    //   +COPS: (<stat>,<long>,<short>,<numeric>,<AcT>),(...),,(0-4),(0,1,2)
    //
    // <stat> 0 unknown, 1 available, 2 current, 3 FORBIDDEN. Measured shape:
    //   (1,"Name","Nm","53001",7),(1,"Other","Ot","53005",9)
    //
    // Only <stat> 1 and 2 are collected - a forbidden network is one the SIM
    // has already been refused by, and trying it again wastes a whole
    // registration window each pass.
    //
    // The numeric code is the LAST QUOTED FIELD in each tuple, which is what
    // this matches on: names may themselves contain commas and parentheses,
    // so counting separators is not safe.
    GsmField rest;
    if (!GsmParse::afterPrefix(line, length, "+COPS", rest)) { return false; }

    bool found = false;
    size_t i = 0;
    while (i < rest.length) {
        if (rest.data[i] != '(') { i++; continue; }

        const size_t open = ++i;
        while (i < rest.length && rest.data[i] != ')') { i++; }
        if (i >= rest.length) { break; }
        const size_t close = i;

        // The trailing "(0-4),(0,1,2)" groups carry no quotes at all.
        size_t lastOpen = 0, lastClose = 0;
        bool inQuote = false;
        for (size_t j = open; j < close; j++) {
            if (rest.data[j] != '"') { continue; }
            if (!inQuote) { lastOpen = j + 1; inQuote = true; }
            else { lastClose = j; inQuote = false; }
        }
        if (lastClose <= lastOpen) { continue; }

        // <stat> is the first character of the tuple.
        const char stat = rest.data[open];
        if (stat != '1' && stat != '2') { continue; }

        if (out.add(rest.data + lastOpen, lastClose - lastOpen)) { found = true; }
    }
    return found;
}

void GsmDriver::operatorSelectCommand(const char* numericCode,
                                      GsmCmdBuf& out) const {
    out.clear();

    if (numericCode == nullptr || numericCode[0] == '\0') {
        // 3GPP TS 27.007 7.3 - AT+COPS=0, automatic selection.
        out.add("AT+COPS=0");
        return;
    }

    // 3GPP TS 27.007 7.3 - AT+COPS=<mode>,<format>,<oper>
    //
    //   <mode>   1 manual. NOT 4 ("manual, then automatic"): on a roaming SIM
    //            the fallback wanders onto networks it is not entitled to.
    //   <format> 2 numeric - names vary by firmware and language.
    out.add("AT+COPS=1,2,");
    out.addQuoted(numericCode);
}

void GsmDriver3gpp::imeiCommand(GsmCmdBuf& out) const {
    // 3GPP TS 27.007 5.4 - request product serial number.
    out.clear();
    out.add("AT+CGSN");
}

void GsmDriver3gpp::imsiCommand(GsmCmdBuf& out) const {
    // 3GPP TS 27.007 5.6 - request international mobile subscriber identity.
    out.clear();
    out.add("AT+CIMI");
}

// =============================================================================
// NETWORK
// =============================================================================

void GsmDriver3gpp::signalCommand(GsmCmdBuf& out) const {
    // 3GPP TS 27.007 8.5 - signal quality.
    out.clear();
    out.add("AT+CSQ");
}

void GsmDriver3gpp::registrationCommand(uint8_t index, GsmCmdBuf& out) const {
    // 3GPP TS 27.007 7.2 / 10.1.19 / 10.1.20 - registration status.
    //
    // ASKED IN PREFERENCE ORDER, NOT PICKED ONCE. The packet-domain registers
    // come first because packet service is what this library exists to carry,
    // but a module can be attached and report it on a different one - so the
    // caller walks this list rather than believing the first answer.
    out.clear();

    uint8_t slot = 0;

    // [3GPP TS 27.007 10.1.20] EPS network registration status.
    if (profile().hasEpsRegistration) {
        if (index == slot) { out.add("AT+CEREG?"); return; }
        slot++;
    }

    // [3GPP TS 27.007 10.1.19] GPRS network registration status.
    if (profile().hasPacketRegistration) {
        if (index == slot) { out.add("AT+CGREG?"); return; }
        slot++;
    }

    // [3GPP TS 27.007 7.2] Network registration. Every module has this one.
    if (index == slot) { out.add("AT+CREG?"); return; }

    // Past the end: the caller wraps to zero.
}

void GsmDriver3gpp::operatorCommand(GsmCmdBuf& out) const {
    // 3GPP TS 27.007 7.3 - PLMN selection.
    out.clear();
    out.add("AT+COPS?");
}

bool GsmDriver3gpp::parseRadioConfig(const char* line, size_t length,
                                     GsmRadioConfig& out) const {
    GsmField rest, field;
    uint32_t value = 0;

    // [SIM707x §5.2.16] / [A76XX §4.2.7] "+CNMP: <mode>"
    if (GsmParse::afterPrefix(line, length, "+CNMP", rest)) {
        if (GsmParse::field(rest.data, rest.length, 0, field) &&
            GsmParse::toUnsignedInRange(field, 0, 255, value)) {
            out.networkMode = (int16_t)value;
        }
        return true;
    }
    return false;
}

void GsmDriver3gpp::radioReadbackCommand(uint8_t index, GsmCmdBuf& out) const {
    // The queries both families share. A driver whose module has more adds
    // them after these - see GsmDriverSIM707x.
    out.clear();
    switch (index) {
        // [SIM707x §5.2.16] / [A76XX §4.2.7] AT+CNMP? - preferred mode.
        case 0: out.add("AT+CNMP?"); return;

        // 3GPP TS 27.007 7.3 - AT+COPS? tells us which operator, and in
        // which selection mode, the module believes it is on.
        case 1: out.add("AT+COPS?"); return;

        default: return;
    }
}

void GsmDriver3gpp::simUnlockCommand(const char* pin, GsmCmdBuf& out) const {
    // 3GPP TS 27.007 8.3 - AT+CPIN=<pin>[,<newpin>]
    //
    // The write form of the same command whose read form asks what the SIM
    // wants. Quoted because a PIN with a leading zero is not a number.
    out.clear();
    if (pin == nullptr || pin[0] == '\0') { return; }
    out.add("AT+CPIN=");
    out.addQuoted(pin);
}

void GsmDriver3gpp::simStatusCommand(GsmCmdBuf& out) const {
    // 3GPP TS 27.007 8.3 - enter PIN.
    //
    // The read form asks what the SIM wants rather than supplying anything.
    // Worth knowing during bring-up: a module answers "+CME ERROR: 14" - SIM
    // busy - for some seconds after power-on, and that is NORMAL rather than a
    // broken SIM. The bring-up machine retries rather than failing.
    out.clear();
    out.add("AT+CPIN?");
}

// =============================================================================
// BEARER - the standard half
// =============================================================================

GsmStep GsmDriver3gpp::bearerUpStep(uint8_t index, const GsmApn& apn,
                                    GsmCmdBuf& out) const {
    out.clear();
    GsmStep step;

    if (index == 0) {
        // 3GPP TS 27.007 10.1.1 - define PDP context.
        //
        // AT+CGDCONT=<cid>,<PDP_type>,<APN>
        //
        // The context identifier is one-based in the standard, while this
        // library numbers its bearers from zero the way the rest of its
        // indices run - so the conversion happens HERE, once, rather than at
        // every call site where it would eventually be forgotten.
        out.add("AT+CGDCONT=");
        out.add((uint32_t)(apn.contextId + 1));
        out.add(",\"IP\",");
        out.addQuoted(apn.name != nullptr ? apn.name : "");

        step.present = true;
        step.timeoutMs = profile().defaultCommandTimeoutMs;
        return step;
    }

    return step;
}

// =============================================================================
// TIME
// =============================================================================

void GsmDriver3gpp::clockCommand(GsmCmdBuf& out) const {
    // 3GPP TS 27.007 8.15 - clock.
    out.clear();
    out.add("AT+CCLK?");
}

bool GsmDriver3gpp::parseClock(const char* line, size_t length,
                               GsmParse::GsmDateTime& out) const {
    // +CCLK: "yy/MM/dd,hh:mm:ss+zz"
    GsmField rest;
    if (!GsmParse::afterPrefix(line, length, "+CCLK", rest)) { return false; }
    return GsmParse::toDateTime(rest, out);
}

// =============================================================================
// SMS
// =============================================================================

GsmStep GsmDriver3gpp::smsInitStep(uint8_t index, GsmCmdBuf& out) const {
    out.clear();
    GsmStep step;

    switch (index) {
        case 0:
            // 3GPP TS 27.005 3.2.3 - message format.
            //
            // PDU mode, not text mode. Text mode cannot carry anything outside
            // the basic alphabet, cannot do concatenation reliably, and its
            // reply formatting varies between modules - so the simpler-looking
            // option is the one that breaks in the field.
            out.add("AT+CMGF=0");
            step.present = true;
            step.timeoutMs = profile().defaultCommandTimeoutMs;
            return step;

        case 1:
            // 3GPP TS 27.005 3.4.1 - new message indications.
            //
            // AT+CNMI=<mode>,<mt>,<bm>,<ds>,<bfr> = 2,1,0,0,0
            //
            // mt=1 means "tell me the INDEX of a new message", not "send me
            // the message". That matters: mt=2 pushes the whole message
            // unsolicited, in the middle of whatever command is in flight, as
            // a multi-line payload the engine would have to unpick. An index
            // is one short line, and the message is fetched when convenient.
            out.add("AT+CNMI=2,1,0,0,0");
            step.present = true;
            step.timeoutMs = profile().defaultCommandTimeoutMs;
            return step;

        default:
            return step;
    }
}

void GsmDriver3gpp::smsSendCommand(const char* number, size_t textLength,
                                   GsmCmdBuf& out) const {
    // 3GPP TS 27.005 3.5.1 - send message. In PDU mode the parameter is the
    // length of the PDU in octets EXCLUDING the service centre address, and
    // the PDU itself follows the prompt.
    (void)number;
    out.clear();
    out.add("AT+CMGS=");
    out.add((uint32_t)textLength);
}

void GsmDriver3gpp::smsReadCommand(uint16_t index, GsmCmdBuf& out) const {
    // 3GPP TS 27.005 3.4.3 - read message.
    out.clear();
    out.add("AT+CMGR=");
    out.add((uint32_t)index);
}

void GsmDriver3gpp::smsDeleteCommand(uint16_t index, GsmCmdBuf& out) const {
    // 3GPP TS 27.005 3.5.4 - delete message.
    out.clear();
    out.add("AT+CMGD=");
    out.add((uint32_t)index);
}

// =============================================================================
// SHARED PARSING
// =============================================================================

GsmTechnology GsmDriver3gpp::technologyFromAccessTechnology(uint32_t value) {
    // 3GPP TS 27.007 7.2 - the <AcT> field.
    switch (value) {
        case 0:  return GsmTechnology::Gsm;
        case 1:  return GsmTechnology::Gsm;          // GSM Compact
        case 2:  return GsmTechnology::Utran;
        case 3:  return GsmTechnology::GsmEgprs;
        case 4:  return GsmTechnology::UtranHs;      // HSDPA
        case 5:  return GsmTechnology::UtranHs;      // HSUPA
        case 6:  return GsmTechnology::UtranHs;      // both
        case 7:  return GsmTechnology::Eutran;
        case 8:  return GsmTechnology::Eutran;       // EC-GSM-IoT
        case 9:  return GsmTechnology::EutranNbS1;   // NB-IoT
        default: return GsmTechnology::Unknown;
    }
}

bool GsmDriver3gpp::isRegistrationReadReply(const char* line, size_t length,
                                            const char* prefix) {
    GsmField rest;
    if (!GsmParse::afterPrefix(line, length, prefix, rest)) { return false; }

    // Walk to just past the first separating comma, honouring quotes so that a
    // comma inside one does not count.
    size_t i = 0;
    bool inQuotes = false;
    for (; i < rest.length; i++) {
        if (rest.data[i] == '"') { inQuotes = !inQuotes; continue; }
        if (rest.data[i] == ',' && !inQuotes) { i++; break; }
    }
    if (i >= rest.length) { return false; }     // one field: unsolicited

    while (i < rest.length && (rest.data[i] == ' ' || rest.data[i] == '\t')) { i++; }
    if (i >= rest.length) { return false; }

    // A quote here means the second field is the tracking-area code, so the
    // first was <stat> and this is an unsolicited report.
    return rest.data[i] >= '0' && rest.data[i] <= '9';
}

bool GsmDriver3gpp::parseRegistration(const char* line, size_t length,
                                      const char* prefix, bool hasMode,
                                      GsmRegistration& state,
                                      GsmTechnology& technology) {
    // 3GPP TS 27.007 7.2 - +CREG: [<n>,]<stat>[,<lac>,<ci>[,<AcT>]]
    //
    // The <n> field is present in a READ reply and absent in an unsolicited
    // one, which is the trap: parsing both with the same field index reads the
    // mode as the state and reports a registered device as unregistered.
    GsmField rest;
    if (!GsmParse::afterPrefix(line, length, prefix, rest)) { return false; }

    const size_t statIndex = hasMode ? 1 : 0;
    GsmField field;
    uint32_t value = 0;

    if (!GsmParse::field(rest.data, rest.length, statIndex, field)) { return false; }
    if (!GsmParse::toUnsignedInRange(field, 0, 10, value)) { return false; }
    state = (GsmRegistration)value;

    // The access technology, where the module reported it. Its index shifts
    // with <n> for the same reason.
    const size_t actIndex = hasMode ? 4 : 3;
    if (GsmParse::field(rest.data, rest.length, actIndex, field) && !field.isEmpty()) {
        uint32_t act = 0;
        if (GsmParse::toUnsigned(field, act)) {
            technology = technologyFromAccessTechnology(act);
        }
    }
    return true;
}

bool GsmDriver3gpp::parseSignalQuality(const char* line, size_t length, GsmSignal& out) {
    // 3GPP TS 27.007 8.5 - +CSQ: <rssi>,<ber>
    GsmField rest, field;
    if (!GsmParse::afterPrefix(line, length, "+CSQ", rest)) { return false; }
    if (!GsmParse::field(rest.data, rest.length, 0, field)) { return false; }

    uint32_t rssi = 0;
    if (!GsmParse::toUnsigned(field, rssi)) { return false; }

    int dbm = 0;
    if (GsmParse::rssiToDbm(rssi, dbm)) {
        out.rssiDbm = dbm;
        out.hasRssi = true;
    } else {
        // 99 means "not known or not detectable". NOT a weak signal - a
        // reading of zero here would report a dead radio as the bottom of the
        // scale, and every alarm built on it fires for the wrong reason.
        out.hasRssi = false;
    }
    return true;
}

bool GsmParseOperatorHelper(const char* line, size_t length, GsmField& name);

bool GsmDriver3gpp::parseOperator(const char* line, size_t length, GsmField& name) {
    // 3GPP TS 27.007 7.3 - +COPS: <mode>[,<format>,<oper>[,<AcT>]]
    //
    // The operator field is absent entirely when the module is not registered,
    // which is a true answer rather than a parse failure - so a missing field
    // returns false and the caller keeps whatever it had.
    GsmField rest;
    if (!GsmParse::afterPrefix(line, length, "+COPS", rest)) { return false; }
    if (!GsmParse::field(rest.data, rest.length, 2, name)) { return false; }
    return !name.isEmpty();
}

bool GsmDriver3gpp::parseSimStatus(const char* line, size_t length, bool& ready,
                                   bool& needsPin, bool& needsPuk) {
    // 3GPP TS 27.007 8.3 - +CPIN: <code>
    GsmField rest;
    if (!GsmParse::afterPrefix(line, length, "+CPIN", rest)) { return false; }

    const GsmField code = GsmParse::trim(rest);
    ready = code.equals("READY");
    needsPin = code.equals("SIM PIN") || code.equals("SIM PIN2");
    needsPuk = code.equals("SIM PUK") || code.equals("SIM PUK2");
    return true;
}
