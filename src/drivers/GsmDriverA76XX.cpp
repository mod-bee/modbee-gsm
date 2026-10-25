// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#include "GsmDriverA76XX.h"
#include "../GsmNoExcept.h"

namespace {

// [A76XX §15.3.2] The <err> table carried by +CIPOPEN and +NETOPEN.
GsmResult socketErr(uint32_t code) {
    if (code == 0) { return GsmStatus::Ok; }
    return GsmResult::fromModule(GsmErrorSpace::SocketErr, (uint16_t)code);
}

bool startsWith(const char* line, size_t length, const char* prefix) {
    for (size_t i = 0; prefix[i] != '\0'; i++) {
        if (i >= length || line[i] != prefix[i]) { return false; }
    }
    return true;
}

// A76XX reports latitude as ddmm.mmmmmm with the hemisphere in its own field,
// which is NMEA's convention rather than a signed decimal degree. Converting
// it as if it were degrees puts a device a few hundred kilometres away, in a
// plausible-looking place.
bool nmeaCoordinate(const GsmField& value, const GsmField& hemisphere, double& out) {
    if (value.isEmpty()) { return false; }

    double raw = 0.0;
    double fraction = 0.0;
    double scale = 0.1;
    bool afterPoint = false;
    for (size_t i = 0; i < value.length; i++) {
        const char c = value.data[i];
        if (c == '.') { afterPoint = true; continue; }
        if (c < '0' || c > '9') { return false; }
        if (afterPoint) { fraction += (c - '0') * scale; scale *= 0.1; }
        else { raw = raw * 10.0 + (c - '0'); }
    }

    const double degrees = (double)((long)raw / 100);
    const double minutes = (raw - degrees * 100.0) + fraction;
    out = degrees + minutes / 60.0;

    if (!hemisphere.isEmpty() &&
        (hemisphere.data[0] == 'S' || hemisphere.data[0] == 'W')) {
        out = -out;
    }
    return true;
}

bool toDouble(const GsmField& in, double& out) {
    if (in.isEmpty()) { return false; }
    double value = 0.0, fraction = 0.0, scale = 0.1;
    bool afterPoint = false, negative = false;
    size_t i = 0;
    if (in.data[0] == '-') { negative = true; i = 1; }
    for (; i < in.length; i++) {
        const char c = in.data[i];
        if (c == '.') { afterPoint = true; continue; }
        if (c < '0' || c > '9') { return false; }
        if (afterPoint) { fraction += (c - '0') * scale; scale *= 0.1; }
        else { value = value * 10.0 + (c - '0'); }
    }
    out = negative ? -(value + fraction) : (value + fraction);
    return true;
}

// A socket or session id that fits the uint8_t every layer above carries.
//
// NARROWED UNCHECKED, "+IPCLOSE: 256,1" became a close of socket 0 - a corrupt
// or misread line ending a healthy connection that had nothing to do with it.
// Neither family numbers past a handful, so anything that does not fit is not
// a real id, and the line is declined rather than wrapped.
bool fitsSocketId(uint32_t id) { return id <= 0xFFu; }

} // namespace

// =============================================================================
// PROFILE - every number cited
// =============================================================================

GsmDriverA76XX::GsmDriverA76XX() {
    _profile.family = "A76XX";

    // [A76XX §15.2.3] "+CIPOPEN: (0-9),("TCP","UDP","UDP SERVER")" - TEN
    // sockets. The wrong manual for this part says four, which is the single
    // most expensive difference between the two documents.
    _profile.maxSockets = 10;
    _profile.firstSocketId = 0;

    // [A76XX §19.2.14] "<session_id> ... It's from 0 to 1" - TLS is a separate
    // family with only TWO sessions, against ten plain sockets. The manual
    // adds that a later firmware baseline supports three, which is why the
    // driver is selected on the revision as well as the part.
    // [A76XX §19.2.12] The test form answers "+CCHOPEN: (0,1),..." - TWO
    // sessions, 0 and 1. Hardware answers AT+CCHSEND=? with (0,1) as well,
    // so an earlier note that this module offered three was wrong.
    _profile.maxTlsSessions = 2;

    // [A76XX §15.2.4] "+CIPSEND: (0-9),(1-1500)". The manual separately warns
    // that UDP payloads above 1400 may not reach the far end depending on the
    // carrier, which the socket layer respects for datagrams.
    _profile.maxSendChunk = 1500;

    // [A76XX §15.2.5] "+CIPRXGET: (0-5),(0-9),(1-1500)" - mode 2 reads ASCII
    // up to 1500. Mode 3 reads hex up to 750, which this library never uses:
    // hex doubles the bytes on a link that has no flow control.
    _profile.maxRecvChunk = 1500;

    // [A76XX §19.2.14] TLS sends up to 2048 - a DIFFERENT limit from plain
    // TCP's 1500, on the same module.
    _profile.maxTlsSendChunk = 2048;

    // [A76XX §1.5.2] "The Command line buffer can accept a maximum of 3071
    // characters".
    _profile.maxAtLineLength = 3071;

    _profile.dataAccessMode = GsmDataAccessMode::BufferAccess;
    _profile.tlsStyle = GsmTlsStyle::SeparateFamily;

    // [A76XX §15.2.4] "If the <cnfSendLength> is not equal to the
    // <reqSendLength>, the socket then cannot be used further."
    _profile.sendAccounting = GsmSendAccounting::ConfirmedLength;

    // [A76XX §15.4] "+IPCLOSE: <client_index>,<close_reason>" - pushed.
    _profile.closeNotice = GsmCloseNotice::Urc;

    // [A76XX V1.12] NO BOOT BANNER IS DOCUMENTED - zero occurrences of RDY,
    // PB DONE or SMS DONE in 684 pages. Boot completion is detected by polling
    // a bare attention command until it answers.
    _profile.bootBanner = GsmBootBanner::None;

    _profile.hasUdp = true;
    _profile.hasTcpServer = true;          // [A76XX §15.2.13] AT+SERVERSTART
    // [A76XX §24] The FAMILY documents a full GNSS command set - but whether
    // a given module carries the receiver is decided by its ORDER CODE, not
    // by its family or even its part number. SIMCom's suffix says which:
    // "-F..." has GNSS, "-L..." does not, so an A7670G-LABE has none while an
    // A7670G-F... does.
    //
    // This flag therefore means only "the driver knows these commands".
    // Whether this particular module answers them is latched by GsmGnss from
    // what it actually says.
    _profile.hasGnss = true;
    _profile.hasPsm = true;                // [A76XX] AT+CPSMS is present
    _profile.hasEdrx = true;               // [A76XX §4.2.10] AT+CEDRXS
    _profile.hasSmsPdu = true;
    _profile.hasMultiContext = true;       // [A76XX §15.2.1] AT+NETOPEN=<cid>
    _profile.hasPacketRegistration = true; // AT+CGREG is present
    _profile.hasEpsRegistration = true;    // AT+CEREG is present

    // [A76XX §15.2.3, §15.2.4] "Max Response Time ... default: 120000ms",
    // settable 3000-120000 via AT+CIPTIMEOUT.
    _profile.socketOpenTimeoutMs = 120000;
    _profile.socketSendTimeoutMs = 120000;

    // [A76XX §15.2.5] "Max Response Time 8s".
    _profile.socketFetchTimeoutMs = 8000;

    _profile.bearerUpTimeoutMs = 120000;
    _profile.registrationTimeoutMs = 90000;
    _profile.operatorScanTimeoutMs = 180000;
    _profile.defaultCommandTimeoutMs = 9000;   // the manual's common figure

    // [A7670 Hardware Design §3.2, Table 9] Ton(uart) = 11.2 s typical from
    // power-on before the AT port answers at all. Anything sent before then is
    // lost, and a bring-up budgeted for five seconds would call a healthy
    // module dead on every cold start. 15 s leaves margin.
    // [A7670 HW Design §3.2.1, Table 9] Ton(uart) 11.2 s typical - the slowest
    // figure in either family, and the reason a shared constant had to be
    // generous enough to be wrong for the other one.
    _profile.powerOnToUartMs = 15000;

    // [A7670 Hardware Design Table 10] Toff(status) 1.9 s typical,
    // Toff-on 2 s minimum, Treset 1.5 s.
    _profile.powerOffSettleMs = 2500;
    _profile.powerOffToOnMs = 2000;
    _profile.resetSettleMs = 2000;

    // [A7670 Hardware Design V1.00 §3.2.3, Table 11] Treset, "the active low
    // level time impulse on RESET pin to reset module" = 1.5 s typical, and
    // the note under it: "The reset time is recommended to be 1.5s", with the
    // pin "only in case of emergency, such as the module is not responding" -
    // which is the recovery ladder's late rung and nowhere else.
    _profile.resetPulseMs = 1500;

    // --- The power key, as a pulse ---
    //
    // [A7670 Hardware Design V1.00 §3.2.1, Table 9] Ton, "the time of active
    // low level impulse of PWRKEY pin to power on module" = 50 ms TYPICAL.
    // The table gives no minimum and no maximum, so the typical figure is the
    // only one there is and is what is used.
    //
    // Note how far this is from the other family's 500 ms MINIMUM - a board
    // that hard-coded either number would fail silently on the other module,
    // which is the whole reason this lives in the driver.
    _profile.powerKeyOnPulseMs = 50;

    // [A7670 Hardware Design V1.00 §3.2.2, Table 10] Toff, "power off low
    // level pulse width" = 2.5 s MINIMUM.
    //
    // This is also the reason a held key is dangerous here rather than merely
    // useless: hold the key on a RUNNING A7670 for 2.5 s and it shuts down.
    _profile.powerKeyOffPulseMs = 2500;

    // No internal reset timer is documented for this family, so there is no
    // hard ceiling of the kind the SIM707x has. Left zero, which makes the
    // state machine fall back to the bootstrap guard rather than to no guard.
    _profile.powerKeyMaxHoldMs = 0;

    // §3.2.1 gives no settling requirement between the supply and the key for
    // this family - only the other one does. Zero means "not required".
    _profile.supplySettleMs = 0;

    // [A76XX §4.2.6] "<RSRP> Current reference signal received power in
    // (RSRP report value -140) dBm."
    //
    // NOT a tenth of a dBm, which is what the wrong manual for this part says.
    // That difference would be a tenfold error in every signal alarm.
    _profile.hasRsrpConversion = true;
    _profile.rsrpOffsetDbm = -140;
}

bool GsmDriverA76XX::claims(const char* identification, const char* revision) const {
    (void)revision;
    if (identification == nullptr) { return false; }

    // The identification text carries the family name. Matched as a substring
    // because modules pad it differently between revisions and some prepend a
    // manufacturer line.
    static const char* kNames[] = { "A7670", "A7672", "A7678", "A7608", "A76XX" };
    for (const char* name : kNames) {
        for (size_t i = 0; identification[i] != '\0'; i++) {
            size_t j = 0;
            while (name[j] != '\0' && identification[i + j] == name[j]) { j++; }
            if (name[j] == '\0') { return true; }
        }
    }
    return false;
}

// =============================================================================
// BRING-UP
// =============================================================================

GsmStep GsmDriverA76XX::initStep(uint8_t index, GsmCmdBuf& out) const {
    // The standard prelude first, then this module's own configuration.
    if (index < kInitSteps) { return GsmDriver3gpp::initStep(index, out); }

    out.clear();
    GsmStep step;

    switch (index - kInitSteps) {
        case 0:
            // AT+CIPCCFG IS DELIBERATELY NOT SENT. [A76XX §15.2.13]
            //
            // It configures socket parameters, and the only field this driver
            // ever wanted was <errMode>=0, which pins the <err_info> table to
            // numbers rather than English sentences. Every other field it set
            // was already the module's own default.
            //
            // BOTH WAYS OF ASKING FAILED ON REAL HARDWARE - an A7670G running
            // A110B06A7670M7:
            //
            //   AT+CIPCCFG=10,0,0,0,0,0,500   RESET THE MODULE. Every time,
            //       about 6.4 s later: no reply at all, then *ATREADY: 1 and
            //       a fresh boot. Every parameter was inside its documented
            //       range, so no amount of re-reading the manual would have
            //       found it.
            //
            //   AT+CIPCCFG=,,,0               ERROR, although §15.2.13 shows
            //       every field as optional.
            //
            // There is no form of this command that works here, and what it
            // bought was cosmetic. Leaving <errMode> at its default costs a
            // less tidy error string and keeps the module running, which is
            // not a close call.
            //
            // Kept as an empty step rather than deleted, so that the numbering
            // of the steps after it does not shift and so the reason survives
            // where the next person will look for it.
            step.present = false;
            return step;

        case 1:
            // [A76XX §15.2.5] AT+CIPRXGET=1 selects BUFFER ACCESS: the module
            // buffers received data and notifies, rather than pushing it at
            // the port as it arrives.
            //
            // THE DEFAULT IS 0, WHICH IS DIRECT PUSH AND HAS NO BACK PRESSURE
            // AT ALL. On a three-wire link that is not a dropped reading, it
            // is a desynchronised AT stream - the bytes that went missing were
            // part of a length-delimited payload, and everything after them is
            // read as the wrong thing until something times out.
            out.add("AT+CIPRXGET=1");
            step.present = true;

            // [A76XX §15.2] gives a 120000 ms default Max Response Time for
            // this chapter. This use only changes a mode and answers at once,
            // but the floor stops the step moving on mid-command - losing this
            // one leaves the module in Direct Push and the socket loses its
            // reply.
            step.timeoutMs = 10000;
            return step;

        default:
            return step;
    }
}

// =============================================================================
// IDENTITY
// =============================================================================

void GsmDriverA76XX::iccidCommand(GsmCmdBuf& out) const {
    // [A76XX §6.2.1] AT+CICCID - read ICCID from SIM card. Note the leading C:
    // the other family this library drives spells the same query AT+CCID and
    // answers it in a different shape.
    out.clear();
    out.add("AT+CICCID");
}

bool GsmDriverA76XX::parseIccid(const char* line, size_t length, GsmField& out) const {
    // [A76XX §6.2.1] "+ICCID: <ICCID>" - a prefixed line, where the SIM707x
    // family answers with the bare digits and no prefix at all.
    GsmField rest;
    if (!GsmParse::afterPrefix(line, length, "+ICCID", rest)) { return false; }
    out = GsmParse::unquote(GsmParse::trim(rest));
    return !out.isEmpty();
}

// =============================================================================
// SYSTEM INFORMATION
// =============================================================================

void GsmDriverA76XX::systemInfoCommand(GsmCmdBuf& out) const {
    // [A76XX §4.2.6] AT+CPSI? - inquiring UE system information.
    out.clear();
    out.add("AT+CPSI?");
}

bool GsmDriverA76XX::parseSystemInfo(const char* line, size_t length,
                                     GsmSignal& signal,
                                     GsmTechnology& technology) const {
    // [A76XX §4.2.6] THE REPLY HAS THREE DIFFERENT SHAPES and the field
    // positions do not line up between them:
    //
    //   GSM:   <mode>,<op>,<MCC>-<MNC>,<LAC>,<Cell ID>,<ARFCN>,<RxLev>,...
    //   WCDMA: <mode>,<op>,<MCC>-<MNC>,<LAC>,<Cell ID>,<Band>,<PSC>,...
    //   LTE:   <mode>,<op>,<MCC>-<MNC>,<TAC>,<SCellID>,<PCellID>,<Band>,
    //          <earfcn>,<dlbw>,<ulbw>,<RSRQ>,<RSRP>,<RSSI>,<RSSNR>
    //
    // So the mode MUST be read first and everything else indexed from what it
    // says. Reading field 11 unconditionally gets the reference signal quality
    // on LTE and an unrelated number on anything else.
    GsmField rest, mode;
    if (!GsmParse::afterPrefix(line, length, "+CPSI", rest)) { return false; }
    if (!GsmParse::field(rest.data, rest.length, 0, mode)) { return false; }

    if (mode.startsWith("LTE") || mode.startsWith("EUTRAN")) {
        technology = GsmTechnology::Eutran;

        GsmField field;
        int32_t value = 0;

        // RSRQ at index 10, RSRP at index 11.
        if (GsmParse::field(rest.data, rest.length, 10, field) &&
            GsmParse::toSigned(field, value)) {
            // [A76XX §4.2.6] "(RSRQ report value -40)/2 dBm".
            signal.rsrqDb = (int)((value - 40) / 2);
            signal.hasRsrq = true;
        }
        if (GsmParse::field(rest.data, rest.length, 11, field) &&
            GsmParse::toSigned(field, value)) {
            // [A76XX §4.2.6] "(RSRP report value -140) dBm".
            signal.rsrpDbm = (int)value + _profile.rsrpOffsetDbm;
            signal.hasRsrp = true;
        }
        return true;
    }

    if (mode.startsWith("WCDMA")) { technology = GsmTechnology::Utran; return true; }
    if (mode.startsWith("GSM")) { technology = GsmTechnology::Gsm; return true; }

    // "NO SERVICE" is a legitimate answer and not a parse failure.
    technology = GsmTechnology::Unknown;
    return true;
}

// =============================================================================
// BEARER
// =============================================================================

GsmStep GsmDriverA76XX::bearerUpStep(uint8_t index, const GsmApn& apn,
                                     GsmCmdBuf& out) const {
    // The context definition is standard; activating it is not.
    if (index < kBearerSteps) { return GsmDriver3gpp::bearerUpStep(index, apn, out); }

    out.clear();
    GsmStep step;

    if (index - kBearerSteps == 0) {
        // [A76XX §15.2.1] AT+NETOPEN - start socket service by activating the
        // PDP context. "You can activate the specified PDP context by <cid>."
        //
        // AND IT ANSWERS TWICE: "OK" first, then "+NETOPEN: <err>[,<cid>]"
        // carrying the real outcome. The core is told that through
        // GsmModem's deferred handling; concluding on the OK would report
        // every bearer as up the instant it was asked for.
        out.add("AT+NETOPEN");
        if (_profile.hasMultiContext && apn.contextId != 0) {
            out.add('=');
            out.add((uint32_t)(apn.contextId + 1));
        }
        step.present = true;
        step.timeoutMs = 10000;
        step.deferredPrefix = "+NETOPEN:";
        step.deferredTimeoutMs = _profile.bearerUpTimeoutMs;
        return step;
    }

    if (index - kBearerSteps == 1) {
        // [A76XX §15.2.5] AT+CIPRXGET=1 - Buffer Access. SET HERE, AFTER
        // NETOPEN, AND NOT ONLY DURING CONFIGURATION.
        //
        // This is the single most consequential setting the driver makes. In
        // its default mode 0 - Direct Push - the module shoves received data
        // at the AT port unsolicited as "RECV FROM:" and "+IPD<len>", which
        // this library deliberately does not read (there is no back pressure
        // in that mode: data arrives whether or not there is room for it).
        // Mode 1 makes the module BUFFER instead and merely announce
        // "+CIPRXGET: 1,<link>", which we then fetch at our own pace.
        //
        // WHY IT IS REPEATED HERE. On a real A7670G the same command answered
        // ERROR when sent about a second after boot and OK when sent about
        // three and a half seconds after boot - the IP stack is simply not
        // ready that early, and the manual says nothing about it. Configuring
        // runs early by design, so it sometimes loses this race.
        //
        // Losing it was expensive to diagnose because NOTHING LOOKED WRONG:
        // bring-up completed, the bearer came up, a socket opened, a request
        // went out and the far end replied - and the application received
        // zero bytes, because the reply was pushed in a mode nobody was
        // listening to.
        //
        // Here the bearer is up, so the stack is unambiguously ready. Setting
        // a mode that is already set costs one command and answers OK.
        //
        // [A76XX §15.2.5] AT+CIPRXGET=<mode>, mode 1 = Buffer Access.
        out.add("AT+CIPRXGET=1");
        step.present = true;
        step.timeoutMs = 10000;

        // NOT ignoreFailure. Everything above the socket layer depends on
        // this, so a module that refuses it is a module we cannot carry data
        // for - and saying so is far kinder than delivering silence.
        step.ignoreFailure = false;
        return step;
    }

    return step;
}

GsmStep GsmDriverA76XX::radioPreferenceStep(uint8_t index,
                                            GsmRadioPreference wanted,
                                            GsmCmdBuf& out) const {
    // THIS FAMILY HAD NO RADIO CONTROL AT ALL UNTIL NOW.
    //
    // Neither this method nor bandConfigStep was overridden, so the base
    // class's empty step was returned and gsm.radioPreference(LteOnly) on an
    // A7670G was accepted, did nothing, and reported nothing. That breaks the
    // rule that every accepted operation reaches exactly one conclusion, and
    // it is not a limitation of the part: a board vendor's own A76xx driver
    // drives AT+CNMP on exactly this silicon.
    out.clear();
    GsmStep step;
    if (index != 0) { return step; }

    // [A76XX §4.2.7] AT+CNMP=<mode>. MEASURED on an A7670G-LABE, firmware
    // A110B06A7670M7: "+CNMP: (2,13,38)" - THREE values. The manual lists 14
    // (WCDMA only) as well; this part does not offer it, which is consistent
    // with a -G order code that carries no WCDMA. Nothing below emits 14.
    //
    // Not the same set as the SIM707x family either, which has 51 (GSM and
    // LTE) - so 51 is refused here rather than sent and answered with ERROR.
    const char* mode = nullptr;
    switch (wanted) {
        case GsmRadioPreference::Automatic: mode = "2";  break;
        case GsmRadioPreference::GsmOnly:   mode = "13"; break;
        case GsmRadioPreference::LteOnly:   mode = "38"; break;

        // Cat-M and NB-IoT are E-UTRAN, so both are "LTE" here. There is no
        // AT+CMNB on this family to choose between them - it does not exist
        // in 684 pages - because an A7670 is Cat-1, not an LPWA part.
        case GsmRadioPreference::CatM:
        case GsmRadioPreference::NbIot:
        case GsmRadioPreference::CatMAndNbIot: mode = "38"; break;

        // [A76XX §4.2.7] 51 is a SIM707x value. Asking for it here answers
        // ERROR, so it is refused before it is sent.
        case GsmRadioPreference::GsmAndLte: return step;
        default: return step;
    }

    out.add("AT+CNMP=");
    out.add(mode);
    step.present = true;

    // [A76XX §4.2.7] The table says 9000 ms; the section's own NOTE says the
    // write takes up to 10 seconds. Where a manual contradicts itself, the
    // longer figure is the safe one - a timeout that fires while the module
    // is still working sends the next command on top of it.
    step.timeoutMs = 10000;

    // NOT ignoreFailure. Pointing the radio somewhere is the difference
    // between registering and searching for ever.
    step.ignoreFailure = false;
    return step;
}

GsmStep GsmDriverA76XX::bandConfigStep(uint8_t index, const GsmBandList& catM,
                                       const GsmBandList& nbIot,
                                       GsmCmdBuf& out) const {
    // [A76XX §4.2.8] AT+CNBP=<mode>[,<lte_mode>[,<lte_modeExt>[,<saveMode>]]]
    //
    // A COMPLETELY DIFFERENT SHAPE FROM THE OTHER FAMILY. SIM707x takes
    // AT+CBANDCFG="CAT-M",3,8,28 - a named technology and a list of band
    // numbers. This family has no AT+CBANDCFG at all (zero occurrences in the
    // whole manual) and instead takes AT+CNBP with a 64-BIT HEX BITMASK where
    // "the value is '1' << '<pos>', then or by bit" and <pos> is the band
    // number minus one.
    //
    // Which is why the band list crosses this interface as numbers and each
    // driver renders them the way its own module wants.
    out.clear();
    GsmStep step;
    if (index != 0) { return step; }
    if (!catM.isSet() && !nbIot.isSet()) { return step; }

    // Both lists mean the same thing here - this part has one LTE mode, not
    // two - so they are merged rather than one being silently ignored.
    uint64_t lte = 0;
    // isSet(), not count alone: a list with a count and no array is a
    // caller's mistake, and reading through its null pointer would crash.
    for (uint8_t i = 0; catM.isSet() && i < catM.count; i++) {
        const uint8_t band = catM.bands[i];
        if (band >= 1 && band <= 64) { lte |= (uint64_t)1 << (band - 1); }
    }
    for (uint8_t i = 0; nbIot.isSet() && i < nbIot.count; i++) {
        const uint8_t band = nbIot.bands[i];
        if (band >= 1 && band <= 64) { lte |= (uint64_t)1 << (band - 1); }
    }
    if (lte == 0) { return step; }

    // [A76XX §4.2.8] <mode> is the GSM/WCDMA mask. "Any" is
    // 0xFFFFFFFF7FFFFFFF, and it is passed through unchanged: this command
    // narrows LTE bands, and quietly disabling every 2G and 3G band as a side
    // effect of asking about LTE would be a surprise.
    out.add("AT+CNBP=0xFFFFFFFF7FFFFFFF,0x");

    char hex[17];
    for (int i = 15; i >= 0; i--) {
        const uint8_t nibble = (uint8_t)((lte >> (i * 4)) & 0xF);
        hex[15 - i] = (char)(nibble < 10 ? ('0' + nibble) : ('A' + nibble - 10));
    }
    hex[16] = '\0';
    out.add(hex);

    step.present = true;
    step.timeoutMs = 10000;     // [A76XX §4.2.8] 9000 ms documented
    step.ignoreFailure = false;
    return step;
}

GsmStep GsmDriverA76XX::bearerDownStep(uint8_t index, const GsmApn& apn,
                                       GsmCmdBuf& out) const {
    (void)apn;
    out.clear();
    GsmStep step;

    if (index == 0) {
        // [A76XX §15.2.2] AT+NETCLOSE - stop socket service.
        //
        // [A76XX §15.3.1] It fails with "Net close failed for socket opened"
        // when any socket is still up, so the socket layer closes its own
        // first. Tolerated here rather than treated as fatal: a bearer that is
        // already down answers an error too, and that is not a failure.
        //
        // AND IT ANSWERS TWICE, like AT+NETOPEN: "OK" first, then
        // "+NETCLOSE: <err>[,<cid>]" with the real outcome. Concluding on the
        // OK reports the bearer down while the module is still closing it,
        // and the next attempt then collides with the tail of this one.
        out.add("AT+NETCLOSE");
        step.present = true;
        step.timeoutMs = 20000;
        step.deferredPrefix = "+NETCLOSE:";
        step.deferredTimeoutMs = 120000;
        step.ignoreFailure = true;
        return step;
    }
    return step;
}

void GsmDriverA76XX::addressCommand(const GsmApn& apn, GsmCmdBuf& out) const {
    (void)apn;
    // [A76XX §15.2.7] AT+IPADDR - inquire socket PDP address.
    out.clear();
    out.add("AT+IPADDR");
}

bool GsmDriverA76XX::parseAddress(const char* line, size_t length, GsmField& out) const {
    // [A76XX §15.2.7] "+IPADDR: <ip address>"
    GsmField rest;
    if (!GsmParse::afterPrefix(line, length, "+IPADDR", rest)) { return false; }
    out = GsmParse::unquote(GsmParse::trim(rest));
    return !out.isEmpty();
}

// =============================================================================
// SOCKETS
// =============================================================================

GsmStep GsmDriverA76XX::tlsSetupStep(uint8_t index, uint8_t socketId,
                                     const GsmEndpoint& endpoint,
                                     GsmCmdBuf& out) const {
    // TLS ON THIS FAMILY IS A SEPARATE COMMAND STACK, not a flag. Open, send,
    // fetch and close are all different commands with different reply
    // prefixes - see openCommand() and the ones beside it - and only TWO
    // sessions exist: [A76XX §19.2.12] answers "+CCHOPEN: (0,1),...".
    out.clear();
    GsmStep step;
    if (endpoint.security != GsmSecurity::Module) { return step; }

    const uint32_t context = 0;      // [A76XX §19.2.10] the SSL context index

    switch (index) {
        case 0:
            // [A76XX §19.2.1] AT+CSSLCFG="sslversion",<ctx>,<version>
            //   3 = TLS 1.2
            out.add("AT+CSSLCFG=\"sslversion\",");
            out.add(context);
            out.add(",3");
            break;

        case 1:
            // [A76XX §19.2.1] Server Name Indication on. Without it a host
            // sharing an address is served the wrong certificate.
            out.add("AT+CSSLCFG=\"enableSNI\",");
            out.add(context);
            out.add(",1");
            break;

        case 2:
            // [A76XX §19.2.5] AT+CCHSET=<report_send_result>,<recv_mode>
            //
            // report_send_result 1 is what makes AT+CCHSEND answer a second
            // time with "+CCHSEND: <session>,<err>". Without it a send
            // concludes on its OK and a failed send looks like a good one.
            out.add("AT+CCHSET=1,1");
            break;

        case 3:
            // [A76XX §19.2.7] Start the SSL service. Harmless if it is
            // already running, which it is for the second TLS socket.
            out.add("AT+CCHSTART");
            step.present = true;
            step.timeoutMs = 120000;      // [A76XX §19] chapter default
            step.ignoreFailure = true;    // already started is not a failure

            // [A76XX §19.2.7] ANSWERS TWICE: "OK", then "+CCHSTART: 0". The
            // second line is the one that means the SSL service is up.
            // Concluding on the OK left it orphaned - and opened the socket
            // while the service was still starting.
            step.deferredPrefix = "+CCHSTART:";
            step.deferredTimeoutMs = 120000;
            return step;

        case 4:
            // [A76XX §19.2.10] AT+CCHSSLCFG=<session_id>,<ssl_ctx_index>
            out.add("AT+CCHSSLCFG=");
            out.add((uint32_t)socketId);
            out.add(',');
            out.add(context);
            break;

        default:
            return step;
    }

    step.present = true;
    step.timeoutMs = 120000;          // [A76XX §19] chapter default

    // NOT ignoreFailure. A socket asked for TLS that opens without it sends
    // in clear and says nothing.
    step.ignoreFailure = false;
    return step;
}

void GsmDriverA76XX::openCommand(uint8_t socketId, const GsmEndpoint& endpoint,
                                 const GsmApn& apn, GsmCmdBuf& out) const {
    (void)apn;
    out.clear();

    // [A76XX §15.2.3]
    //   TCP: AT+CIPOPEN=<link_num>,"TCP",<serverIP>,<serverPort>[,<localPort>]
    //   UDP: AT+CIPOPEN=<link_num>,"UDP",,,<localPort>
    //
    // The UDP form takes the LOCAL port and leaves the server fields empty -
    // a datagram socket is bound rather than connected, and the destination
    // travels with each send instead.
    // [A76XX §19.2.12] AT+CCHOPEN=<session_id>,<host>,<port>[,<client_type>]
    //   <client_type> 2 = SSL/TLS
    //
    // A DIFFERENT COMMAND, not a flag: TLS here is a parallel stack with its
    // own open, send, fetch and close. Only sessions 0 and 1 exist - the test
    // form answers "+CCHOPEN: (0,1),...".
    if (endpoint.security == GsmSecurity::Module) {
        out.add("AT+CCHOPEN=");
        out.add((uint32_t)socketId);
        out.add(',');
        out.addQuoted(endpoint.host);
        out.add(',');
        out.add((uint32_t)endpoint.port);
        out.add(",2");
        return;
    }

    // [A76XX §15.2.3] The plain stack:
    //   TCP: AT+CIPOPEN=<link_num>,"TCP",<serverIP>,<serverPort>
    //   UDP: AT+CIPOPEN=<link_num>,"UDP",,,<localPort>
    out.add("AT+CIPOPEN=");
    out.add((uint32_t)socketId);

    if (endpoint.kind == GsmSocketKind::Udp) {
        out.add(",\"UDP\",,,");
        out.add((uint32_t)endpoint.port);
        return;
    }

    out.add(",\"TCP\",");
    out.addQuoted(endpoint.host);
    out.add(',');
    out.add((uint32_t)endpoint.port);
}

const char* GsmDriverA76XX::openDeferredPrefix(const GsmEndpoint& endpoint) const {
    // [A76XX §15.2.3] "OK" then, separately, "+CIPOPEN: <link_num>,<err>" -
    // up to 120 seconds later. The OK means the command was accepted, not
    // that the socket opened.
    //
    // [A76XX §19.2.12] The TLS stack does the same on its own prefix.
    return endpoint.security == GsmSecurity::Module ? "+CCHOPEN:" : "+CIPOPEN:";
}

void GsmDriverA76XX::sendCommand(uint8_t socketId, size_t length,
                                 const GsmEndpoint& endpoint, GsmCmdBuf& out) const {
    out.clear();

    // [A76XX §15.2.4]
    //   TCP: AT+CIPSEND=<link_num>,<length>
    //   UDP: AT+CIPSEND=<link_num>,<length>,<serverIP>,<serverPort>
    //
    // The FIXED LENGTH form, never the changeable one. The changeable form
    // ends on a Ctrl-Z and requires every Ctrl-Z, ESC and ETX inside the
    // payload to be escaped - which makes a binary payload unrepresentable
    // and would corrupt the first TLS record that contained one.
    // [A76XX §19.2.14] AT+CCHSEND=<session_id>,<len>, limit 1-2048 - larger
    // than the 1500 plain TCP allows on the same module.
    if (endpoint.security == GsmSecurity::Module) {
        out.add("AT+CCHSEND=");
        out.add((uint32_t)socketId);
        out.add(',');
        out.add((uint32_t)length);
        return;
    }

    out.add("AT+CIPSEND=");
    out.add((uint32_t)socketId);
    out.add(',');
    out.add((uint32_t)length);

    if (endpoint.kind == GsmSocketKind::Udp) {
        out.add(',');
        out.addQuoted(endpoint.host);
        out.add(',');
        out.add((uint32_t)endpoint.port);
    }
}

void GsmDriverA76XX::fetchCommand(uint8_t socketId, size_t maxBytes,
                                  const GsmEndpoint& endpoint,
                                     GsmCmdBuf& out) const {
    out.clear();

    // [A76XX §15.2.5] AT+CIPRXGET=2,<link_num>[,<len>] - retrieve data in
    // ASCII form. Mode 2, never mode 3: hex doubles the bytes on a link that
    // has no flow control, for no benefit this library needs.
    // [A76XX §19.2.15] AT+CCHRECV=<session_id>[,<max_recv_len>]
    if (endpoint.security == GsmSecurity::Module) {
        out.add("AT+CCHRECV=");
        out.add((uint32_t)socketId);
        out.add(',');
        out.add((uint32_t)maxBytes);
        return;
    }

    out.add("AT+CIPRXGET=2,");
    out.add((uint32_t)socketId);
    out.add(',');
    out.add((uint32_t)maxBytes);
}

void GsmDriverA76XX::fetchDeferredPrefix(uint8_t socketId,
                                         const GsmEndpoint& endpoint,
                                         GsmCmdBuf& out) const {
    out.clear();

    // [A76XX §19.2.15] MEASURED on an A7670G: AT+CCHRECV answers
    //
    //     OK
    //     +CCHRECV: DATA,0,266
    //     <266 bytes>
    //     +CCHRECV: 0,0
    //
    // The OK is acceptance, not the answer. Concluding on it ended the fetch
    // before a single byte arrived, and the whole reply was then orphaned.
    //
    // The session id is in the prefix on purpose - "+CCHRECV: 0," cannot match
    // "+CCHRECV: DATA,...", so the data header passes through to the header
    // parser instead of being mistaken for the terminator.
    if (endpoint.security != GsmSecurity::Module) { return; }

    out.add("+CCHRECV: ");
    out.add((uint32_t)socketId);
    out.add(',');
}

void GsmDriverA76XX::unreadQueryCommand(uint8_t socketId, const GsmEndpoint& endpoint,
                                        GsmCmdBuf& out) const {
    out.clear();

    // [A76XX §19.2.15] AT+CCHRECV? - "+CCHRECV: LEN,<cache_len_0>,<cache_len_1>",
    // both sessions in one answer. Takes nothing from either.
    if (endpoint.security == GsmSecurity::Module) {
        out.add("AT+CCHRECV?");
        return;
    }

    // [A76XX §15.2.5] AT+CIPRXGET=4,<link_num> - "+CIPRXGET: 4,<link>,<rest_len>",
    // the bytes still buffered for that link. MEASURED on an A7670G with a
    // stalled socket: "+CIPRXGET: 4,0,5120", while no notice was ever going
    // to come for them.
    out.add("AT+CIPRXGET=4,");
    out.add((uint32_t)socketId);
}

bool GsmDriverA76XX::parseUnread(const char* line, size_t length, uint8_t socketId,
                                 const GsmEndpoint& endpoint, size_t& out) const {
    GsmField rest, field;
    uint32_t value = 0;

    if (endpoint.security == GsmSecurity::Module) {
        // "+CCHRECV: LEN,<cache_len_0>,<cache_len_1>". The data header
        // "+CCHRECV: DATA,..." and the terminator "+CCHRECV: 0,0" share the
        // prefix and are told apart by the first field.
        if (!GsmParse::afterPrefix(line, length, "+CCHRECV", rest)) { return false; }
        if (!GsmParse::field(rest.data, rest.length, 0, field) || !field.equals("LEN")) {
            return false;
        }
        if (socketId > 1) { return false; }       // the answer covers sessions 0 and 1
        if (!GsmParse::field(rest.data, rest.length, 1u + socketId, field) ||
            !GsmParse::toUnsigned(field, value)) {
            return false;
        }
        out = (size_t)value;
        return true;
    }

    // "+CIPRXGET: 4,<link_num>,<rest_len>"
    if (!GsmParse::afterPrefix(line, length, "+CIPRXGET", rest)) { return false; }
    uint32_t mode = 0, link = 0;
    if (!GsmParse::field(rest.data, rest.length, 0, field) ||
        !GsmParse::toUnsigned(field, mode) || mode != 4) {
        return false;
    }
    if (!GsmParse::field(rest.data, rest.length, 1, field) ||
        !GsmParse::toUnsigned(field, link) || link != socketId) {
        return false;
    }
    if (!GsmParse::field(rest.data, rest.length, 2, field) ||
        !GsmParse::toUnsigned(field, value)) {
        return false;
    }
    out = (size_t)value;
    return true;
}

void GsmDriverA76XX::closeCommand(uint8_t socketId, const GsmEndpoint& endpoint,
                                       GsmCmdBuf& out) const {
    // [A76XX §15.2.6] AT+CIPCLOSE=<link_num>
    out.clear();
    // [A76XX §19.2.13] AT+CCHCLOSE=<session_id>
    if (endpoint.security == GsmSecurity::Module) {
        out.add("AT+CCHCLOSE=");
        out.add((uint32_t)socketId);
        return;
    }

    out.add("AT+CIPCLOSE=");
    out.add((uint32_t)socketId);
}

bool GsmDriverA76XX::parseOpenResult(const char* line, size_t length,
                                     GsmOpenResult& out,
                                       const GsmEndpoint& endpoint) const {
    // [A76XX §15.2.3] "+CIPOPEN: <link_num>,<err>"
    GsmField rest, field;
    const char* prefix = (endpoint.security == GsmSecurity::Module)
                       ? "+CCHOPEN" : "+CIPOPEN";
    if (!GsmParse::afterPrefix(line, length, prefix, rest)) { return false; }

    uint32_t socketId = 0;
    if (!GsmParse::field(rest.data, rest.length, 0, field) ||
        !GsmParse::toUnsigned(field, socketId) || !fitsSocketId(socketId)) { return false; }
    out.socketId = (uint8_t)socketId;

    // The read form lists sockets with no error field, e.g. "+CIPOPEN: 2".
    // That is a status line rather than an open result, and treating its
    // absent error as success would report an unopened socket as open.
    uint32_t err = 0;
    if (!GsmParse::field(rest.data, rest.length, 1, field) ||
        !GsmParse::toUnsigned(field, err)) { return false; }

    out.result = socketErr(err);
    out.opened = (err == 0);
    return true;
}

bool GsmDriverA76XX::parseFetchHeader(const char* line, size_t length,
                                      GsmFetchInfo& out,
                                        const GsmEndpoint& endpoint) const {
    // [A76XX §15.2.5] "+CIPRXGET: <mode>,<link_num>,<read_len>,<rest_len>"
    // followed by exactly <read_len> raw bytes.
    GsmField rest, field;
    // [A76XX §19.2.15] The TLS stack announces data as
    //   +CCHRECV: DATA,<session_id>,<len>
    // - the literal word DATA first, then the session, then the length. Its
    // read form answers "+CCHRECV: LEN,..." instead, which is a different
    // thing and must not be taken as a header.
    if (endpoint.security == GsmSecurity::Module) {
        if (!GsmParse::afterPrefix(line, length, "+CCHRECV", rest)) { return false; }
        GsmField kind;
        if (!GsmParse::field(rest.data, rest.length, 0, kind)) { return false; }
        if (!kind.equals("DATA")) { return false; }

        uint32_t session = 0, bytes = 0;
        if (!GsmParse::field(rest.data, rest.length, 1, field) ||
            !GsmParse::toUnsigned(field, session)) { return false; }
        if (!GsmParse::field(rest.data, rest.length, 2, field) ||
            !GsmParse::toUnsigned(field, bytes)) { return false; }
        if (!fitsSocketId(session) || bytes > _profile.maxRecvChunk) { return false; }

        out.socketId = (uint8_t)session;
        out.length = (size_t)bytes;
        out.remaining = 0;
        out.hasRemaining = false;      // this stack does not report a remainder
        return true;
    }

    if (!GsmParse::afterPrefix(line, length, "+CIPRXGET", rest)) { return false; }

    uint32_t mode = 0;
    if (!GsmParse::field(rest.data, rest.length, 0, field) ||
        !GsmParse::toUnsigned(field, mode)) { return false; }

    // Mode 2 is a data-bearing reply; mode 1 is the notification URC and mode
    // 4 is a length query. Only mode 2 is followed by bytes, and telling them
    // apart is the whole reason this is a driver's job.
    if (mode != 2) { return false; }

    uint32_t socketId = 0, readLength = 0, restLength = 0;
    if (!GsmParse::field(rest.data, rest.length, 1, field) ||
        !GsmParse::toUnsigned(field, socketId)) { return false; }
    if (!GsmParse::field(rest.data, rest.length, 2, field) ||
        !GsmParse::toUnsigned(field, readLength)) { return false; }

    // A LENGTH THE MODULE COULD NEVER SEND IS NOT A HEADER. It is a corrupt
    // line - and taken at its word, "4000000000" armed the parser to swallow
    // every reply that followed as payload, the fetch's own OK included, until
    // the fetch timed out. The module never returns more than one read may
    // ask for, so anything past that is declined here, before it can arm
    // anything. The socket layer bounds it again by what it actually asked.
    if (!fitsSocketId(socketId) || readLength > _profile.maxRecvChunk) { return false; }

    out.socketId = (uint8_t)socketId;
    out.length = readLength;

    if (GsmParse::field(rest.data, rest.length, 3, field) &&
        GsmParse::toUnsigned(field, restLength)) {
        out.remaining = restLength;
        out.hasRemaining = true;
    }
    return true;
}

bool GsmDriverA76XX::parseSendResult(const char* line, size_t length,
                                     size_t requested, GsmResult& out,
                                       const GsmEndpoint& endpoint) const {
    // [A76XX §15.2.4] "+CIPSEND: <link_num>,<reqSendLength>,<cnfSendLength>"
    GsmField rest, field;
    // [A76XX §19.2.14] TLS answers "+CCHSEND: <session>,<err>" - an ERROR
    // CODE, not a confirmed length, so the two are parsed differently.
    if (endpoint.security == GsmSecurity::Module) {
        if (!GsmParse::afterPrefix(line, length, "+CCHSEND", rest)) { return false; }
        uint32_t err = 0;
        if (!GsmParse::field(rest.data, rest.length, 1, field) ||
            !GsmParse::toUnsigned(field, err)) { return false; }
        out = (err == 0) ? GsmResult(GsmStatus::Ok)
                         : GsmResult::fromModule(GsmErrorSpace::SocketErr, (uint16_t)err);
        return true;
    }
    if (!GsmParse::afterPrefix(line, length, "+CIPSEND", rest)) { return false; }

    int32_t confirmed = 0;
    if (!GsmParse::field(rest.data, rest.length, 2, field) ||
        !GsmParse::toSigned(field, confirmed)) { return false; }

    if (confirmed >= 0 && (size_t)confirmed == requested) {
        out = GsmStatus::Ok;
        return true;
    }

    // THE RULE THAT LOOKS LIKE A TRANSIENT FAILURE AND IS NOT.
    //
    // [A76XX §15.2.4] "If the <cnfSendLength> is not equal to the
    // <reqSendLength>, the socket then cannot be used further." -1 means the
    // connection is already gone; 0 means the send buffer or the far end's
    // window is full. In every case the socket is finished, so this concludes
    // SocketClosed and the socket layer reopens rather than retrying a send
    // into something that no longer exists.
    out = GsmStatus::SocketClosed;
    return true;
}

// =============================================================================
// UNSOLICITED
// =============================================================================

const char* GsmDriverA76XX::urcPrefix(uint8_t index) const {
    // [A76XX §15.4] plus the power and supply reports.
    static const char* kPrefixes[] = {
        "+CIPRXGET:",    // data waiting - AND a reply prefix; see classifyUrc
        "+IPCLOSE:",     // a socket closed
        "+CIPEVENT:",    // the bearer went
        "+CLIENT:",      // an inbound connection was accepted

        // [A76XX §19.4] THE TLS FAMILY REPORTS SEPARATELY, and none of the
        // plain prefixes above covers it - a TLS socket saw neither its data
        // nor its closure until these were listed.
        "+CCHEVENT:",        // data waiting on a TLS session
        "+CCH_PEER_CLOSED:", // the server closed a TLS session
        "+CCH_RECV_CLOSED:", // a receive error killed one
        "+CCH: CCH STOP",    // the whole TLS stack went with the network

        "+CEREG:",       // registration changed
        "+CGREG:",
        "+CMTI:",        // an SMS arrived
        "+CPIN:",        // SIM state changed
        "UNDER-VOLTAGE",
        "OVER-VOLTAGE",
        "NORMAL POWER DOWN",

        // NOT IN THE MANUAL, AND THE MANUAL IS NOT WRONG - just incomplete.
        // A76XX V1.12 documents no boot banner anywhere in 684 pages: no RDY,
        // no PB DONE, no SMS DONE. A real A7670G on firmware A110B06A7670M7
        // emits all of *ATREADY: 1, +CPIN: READY and SMS DONE, and does it
        // TWICE on a cold start - once, then again about seven seconds later.
        //
        // Listening for it is what lets the library notice that the module it
        // configured has been replaced by a freshly booted one.
        "*ATREADY:",     // the AT interface came up - a RESTART if we are past
                         // bring-up, an accelerator if we are not
        "SMS DONE",      // subsystem milestones; recognised so they are not
        "PB DONE",       // counted as unhandled, but acted on by nothing
        nullptr,
    };
    if (index >= (sizeof kPrefixes / sizeof kPrefixes[0])) { return nullptr; }
    return kPrefixes[index];
}

GsmUrcKind GsmDriverA76XX::classifyUrc(const char* line, size_t length,
                                       GsmUrcInfo& out) const {
    out = GsmUrcInfo();

    // THE MODULE HAS (RE)BOOTED. Captured, not documented - see urcPrefix().
    // Either line means the firmware has come up fresh, which is only good
    // news before bring-up and is a lost configuration after it.
    if (startsWith(line, length, "*ATREADY:")) {
        out.kind = GsmUrcKind::Ready;
        return out.kind;
    }

    // Boot milestones, NOT boot signals. They follow a restart, which makes
    // them easy to mistake for one - and treating them as one would have the
    // library reconfigure a healthy module every time its SMS storage or
    // phonebook finished loading.
    if (startsWith(line, length, "SMS DONE") ||
        startsWith(line, length, "PB DONE")) {
        out.kind = GsmUrcKind::Informational;
        return out.kind;
    }

    // [A76XX §15.2.5] THE AMBIGUOUS ONE, and the reason a handler is allowed
    // to decline.
    //
    // "+CIPRXGET: 1,<link_num>" is unsolicited - the module saying data is
    // waiting. "+CIPRXGET: 2,<link>,<len>,<rest>" is the ANSWER to our own
    // fetch, and is followed by the bytes themselves. They are told apart only
    // by the first field. Claiming the reply form as a URC would swallow every
    // socket read this library ever makes.
    if (startsWith(line, length, "+CIPRXGET:")) {
        GsmField rest, field;
        uint32_t mode = 0, socketId = 0;
        if (GsmParse::afterPrefix(line, length, "+CIPRXGET", rest) &&
            GsmParse::field(rest.data, rest.length, 0, field) &&
            GsmParse::toUnsigned(field, mode) && mode == 1 &&
            GsmParse::field(rest.data, rest.length, 1, field) &&
            GsmParse::toUnsigned(field, socketId) && fitsSocketId(socketId)) {
            out.kind = GsmUrcKind::DataAvailable;
            out.socketId = (uint8_t)socketId;
            return out.kind;
        }
        return GsmUrcKind::Unrecognised;    // declined; it falls through
    }

    // [A76XX §15.4] "+IPCLOSE: <client_index>,<close_reason>"
    //   0 closed by local, 1 closed by remote, 2 sending timeout or DTR off
    if (startsWith(line, length, "+IPCLOSE:")) {
        GsmField rest, field;
        uint32_t socketId = 0, reason = 0;
        if (GsmParse::afterPrefix(line, length, "+IPCLOSE", rest) &&
            GsmParse::field(rest.data, rest.length, 0, field) &&
            GsmParse::toUnsigned(field, socketId) && fitsSocketId(socketId)) {
            out.kind = GsmUrcKind::SocketClosed;
            out.socketId = (uint8_t)socketId;
            if (GsmParse::field(rest.data, rest.length, 1, field)) {
                GsmParse::toUnsigned(field, reason);
                out.value = reason;
            }
            return out.kind;
        }
        return GsmUrcKind::Unrecognised;
    }

    // [A76XX §19.4] THE TLS FAMILY'S OWN REPORTS.
    //
    // "+CCHEVENT: <session_id>,RECV EVENT" is the TLS counterpart of
    // "+CIPRXGET: 1,<link>", and unambiguous where that one is not: the CCH
    // family answers a fetch with "+CCHRECV: DATA,..." instead, so nothing
    // here can be mistaken for a reply.
    if (startsWith(line, length, "+CCHEVENT:")) {
        GsmField rest, field;
        uint32_t session = 0;
        if (GsmParse::afterPrefix(line, length, "+CCHEVENT", rest) &&
            GsmParse::field(rest.data, rest.length, 0, field) &&
            GsmParse::toUnsigned(field, session) && fitsSocketId(session)) {
            out.kind = GsmUrcKind::DataAvailable;
            out.socketId = (uint8_t)session;
            out.secure = true;      // a SESSION id, not a plain socket id
            return out.kind;
        }
        return GsmUrcKind::Unrecognised;
    }

    // "+CCH_PEER_CLOSED: <session_id>" and "+CCH_RECV_CLOSED: <session_id>,<err>".
    // Both end the session; the second carries a reason as well.
    if (startsWith(line, length, "+CCH_PEER_CLOSED:") ||
        startsWith(line, length, "+CCH_RECV_CLOSED:")) {
        const bool recvError = startsWith(line, length, "+CCH_RECV_CLOSED:");
        const char* prefix = recvError ? "+CCH_RECV_CLOSED" : "+CCH_PEER_CLOSED";
        GsmField rest, field;
        uint32_t session = 0, err = 0;
        if (GsmParse::afterPrefix(line, length, prefix, rest) &&
            GsmParse::field(rest.data, rest.length, 0, field) &&
            GsmParse::toUnsigned(field, session) && fitsSocketId(session)) {
            out.kind = GsmUrcKind::SocketClosed;
            out.socketId = (uint8_t)session;
            out.secure = true;
            if (recvError && GsmParse::field(rest.data, rest.length, 1, field) &&
                GsmParse::toUnsigned(field, err)) {
                out.value = err;
            }
            return out.kind;
        }
        return GsmUrcKind::Unrecognised;
    }

    // "+CCH: CCH STOP[,<cid>]" - the TLS stack stopped on a network error.
    // EVERY TLS SESSION IS GONE, and CCHSTART must run again before any
    // reopen, so this is the bearer's loss and not one session's.
    if (startsWith(line, length, "+CCH: CCH STOP")) {
        out.kind = GsmUrcKind::BearerDown;
        return out.kind;
    }

    // [A76XX §15.4] "+CIPEVENT: NETWORK CLOSED UNEXPECTEDLY[,<cid>]"
    //
    // EVERY SOCKET IS DEAD, not just one. Reporting this as a socket closure
    // would have the application reopen sockets over a bearer that is gone,
    // which never succeeds and never explains itself.
    if (startsWith(line, length, "+CIPEVENT:")) {
        out.kind = GsmUrcKind::BearerDown;
        return out.kind;
    }

    if (startsWith(line, length, "+CEREG:") || startsWith(line, length, "+CGREG:") ||
        startsWith(line, length, "+CREG:")) {
        // DECLINED WHEN IT IS THE ANSWER TO OUR OWN QUERY. The same prefix
        // carries both a read reply and an unsolicited report, and claiming
        // the reply here would swallow it - leaving the bring-up machine
        // waiting forever for a registration state it had already been sent.
        const char* prefix = startsWith(line, length, "+CEREG:") ? "+CEREG"
                           : startsWith(line, length, "+CGREG:") ? "+CGREG" : "+CREG";
        if (GsmDriver3gpp::isRegistrationReadReply(line, length, prefix)) {
            return GsmUrcKind::Unrecognised;
        }
        out.kind = GsmUrcKind::RegistrationChanged;
        return out.kind;
    }

    if (startsWith(line, length, "+CMTI:")) {
        GsmField rest, field;
        uint32_t index = 0;
        if (GsmParse::afterPrefix(line, length, "+CMTI", rest) &&
            GsmParse::field(rest.data, rest.length, 1, field) &&
            GsmParse::toUnsigned(field, index)) {
            out.value = index;
        }
        out.kind = GsmUrcKind::SmsReceived;
        return out.kind;
    }

    // [A7670 Hardware Design §3.2.2] The module powers itself off outside
    // -40..+85 C and warns from -30..+80, and reports supply faults the same
    // way. On a board running from a switching supply this is the earliest
    // warning of a marginal rail there is, and it costs no extra hardware.
    //
    // BOTH SPELLINGS ARE MATCHED. The manual prints "WARNNING" in one chapter
    // and "WARNING" in another; a table that picked one silently misses half
    // the events.
    if (startsWith(line, length, "UNDER-VOLTAGE") ||
        startsWith(line, length, "OVER-VOLTAGE")) {
        out.kind = GsmUrcKind::SupplyWarning;
        return out.kind;
    }

    if (startsWith(line, length, "NORMAL POWER DOWN")) {
        out.kind = GsmUrcKind::PowerDown;
        return out.kind;
    }

    if (startsWith(line, length, "+CPIN:")) {
        // Declined on purpose: this is far more often the answer to our own
        // SIM query than an unsolicited report, and claiming it would leave
        // that query with no reply.
        return GsmUrcKind::Unrecognised;
    }

    return GsmUrcKind::Unrecognised;
}

bool GsmDriverA76XX::isFinalResult(const char* line, size_t length,
                                   GsmResult& out) const {
    // [A76XX §15.3.1] "+IP ERROR: <err_info>" concludes a socket command and
    // is neither OK, ERROR, nor a +CME code - so a command that did not
    // recognise it would sit until its deadline instead of failing at once.
    //
    // With errMode pinned to 0 at bring-up the value is numeric; a module left
    // in the default string mode still concludes, just without a code.
    if (startsWith(line, length, "+IP ERROR:")) {
        GsmField rest;
        uint32_t code = 0;
        if (GsmParse::afterPrefix(line, length, "+IP ERROR", rest) &&
            GsmParse::toUnsigned(GsmParse::trim(rest), code)) {
            out = GsmResult::fromModule(GsmErrorSpace::SocketInfo, (uint16_t)code);
        } else {
            out = GsmStatus::Error;
        }
        return true;
    }
    return false;
}

// =============================================================================
// POWER
// =============================================================================

void GsmDriverA76XX::powerDownCommand(GsmCmdBuf& out) const {
    // [A7670 Hardware Design §3.2.2] "Power off Module by AT command
    // 'AT+CPOF'", and the same section warns that cutting the supply while
    // the module runs "may cause damage to flash". So this command runs FIRST
    // and the application's power hook only afterwards.
    out.clear();
    out.add("AT+CPOF");
}

void GsmDriverA76XX::resetCommand(GsmCmdBuf& out) const {
    // [A76XX §3.2.4] AT+CRESET. [A7670 Hardware Design §3.2.3] "it is
    // recommended to use the reset pin only in case of emergency, such as the
    // module is not responding" - which is exactly where the recovery ladder
    // puts it, and never as a first resort.
    out.clear();
    out.add("AT+CRESET");
}

// =============================================================================
// LOCATION
// =============================================================================

void GsmDriverA76XX::gnssPowerCommand(bool on, GsmCmdBuf& out) const {
    // [A76XX §24.2.1] AT+CGNSSPWR=<0|1>. Note the double S: the SIM707x family
    // spells the same thing AT+CGNSPWR.
    out.clear();
    out.add(on ? "AT+CGNSSPWR=1" : "AT+CGNSSPWR=0");
}

void GsmDriverA76XX::gnssInfoCommand(GsmCmdBuf& out) const {
    // [A76XX §24.2.12] AT+CGNSSINFO. "This command is valid after the URC
    // reports +CGNSSPWR: READY!" - so an empty reply shortly after power-on is
    // the receiver still starting rather than a fault.
    out.clear();
    out.add("AT+CGNSSINFO");
}

bool GsmDriverA76XX::parseGnssInfo(const char* line, size_t length, GsmFix& out) const {
    // [A76XX §24.2.12]
    // +CGNSSINFO: <mode>,<GPS-SVs>,<BEIDOU-SVs>,<GLONASS-SVs>,<GALILEO-SVs>,
    //             <lat>,<N/S>,<log>,<E/W>,<date>,<UTC-time>,<alt>,<speed>,
    //             <course>,<PDOP>,<HDOP>,<VDOP>
    //
    // SEVENTEEN FIELDS, and the coordinates are NMEA-style - ddmm.mmmmmm with
    // the hemisphere in a field of its own. The SIM707x family reports
    // EIGHTEEN fields in a different order with signed decimal degrees. There
    // is no common parser; this is why parsing belongs to the driver.
    GsmField rest, field, hemisphere;
    if (!GsmParse::afterPrefix(line, length, "+CGNSSINFO", rest)) { return false; }

    // With no fix the module answers with empty fields, which is an honest
    // answer rather than a parse failure.
    if (!GsmParse::field(rest.data, rest.length, 5, field) || field.isEmpty()) {
        out.hasPosition = false;
        out.quality = GsmQuality::Bad;
        return true;
    }

    GsmParse::field(rest.data, rest.length, 6, hemisphere);
    if (!nmeaCoordinate(field, hemisphere, out.latitude)) { return false; }

    if (!GsmParse::field(rest.data, rest.length, 7, field)) { return false; }
    GsmParse::field(rest.data, rest.length, 8, hemisphere);
    if (!nmeaCoordinate(field, hemisphere, out.longitude)) { return false; }

    out.hasPosition = true;

    if (GsmParse::field(rest.data, rest.length, 11, field) &&
        toDouble(field, out.altitudeMetres)) { out.hasAltitude = true; }
    if (GsmParse::field(rest.data, rest.length, 12, field) &&
        toDouble(field, out.speedKnots)) { out.hasSpeed = true; }
    if (GsmParse::field(rest.data, rest.length, 13, field)) {
        toDouble(field, out.courseDegrees);
    }
    if (GsmParse::field(rest.data, rest.length, 15, field)) {
        toDouble(field, out.hdop);
    }

    // Satellites are reported per constellation; the total is what an
    // application means by "how many".
    uint32_t total = 0, count = 0;
    for (size_t i = 1; i <= 4; i++) {
        if (GsmParse::field(rest.data, rest.length, i, field) &&
            GsmParse::toUnsigned(field, count)) { total += count; }
    }
    out.satellitesInView = (uint8_t)(total > 255 ? 255 : total);

    out.quality = GsmQuality::Good;
    out.stampMillis = gsm_millis();
    return true;
}
