// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#include "GsmDriverSIM707x.h"
#include "../GsmNoExcept.h"

namespace {

bool startsWith(const char* line, size_t length, const char* prefix) {
    for (size_t i = 0; prefix[i] != '\0'; i++) {
        if (i >= length || line[i] != prefix[i]) { return false; }
    }
    return true;
}

bool toDouble(const GsmField& in, double& out) {
    if (in.isEmpty()) { return false; }
    double value = 0.0, fraction = 0.0, scale = 0.1;
    bool afterPoint = false, negative = false;
    size_t i = 0;
    if (in.data[0] == '-') { negative = true; i = 1; }
    else if (in.data[0] == '+') { i = 1; }
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

// A socket id this module can actually have.
//
// NARROWED UNCHECKED, "+CASTATE: 256,0" became a closure of socket 0 - a
// corrupt or misread line ending a healthy connection that had nothing to do
// with it. Mirrors the A76XX driver's guard, and is stricter for the same
// reason: [SIM707x §12.2.3] "+CAOPEN: (0-12),..." numbers thirteen sockets
// from zero, so anything outside the profile's range is not one of ours, and
// the line is declined rather than wrapped onto a socket that is.
bool fitsSocketId(const GsmProfile& profile, uint32_t id) {
    return id >= profile.firstSocketId &&
           id < (uint32_t)profile.firstSocketId + profile.maxSockets;
}

} // namespace

// =============================================================================
// PROFILE
// =============================================================================

GsmDriverSIM707x::GsmDriverSIM707x() {
    _profile.family = "SIM707x";

    // [SIM707x §12.2.3] "+CAOPEN: (0-12),(0-4),..." - THIRTEEN sockets, where
    // the A76XX family has ten. Not a variation on a theme; a different
    // number the core must ask for rather than assume.
    _profile.maxSockets = 13;
    _profile.firstSocketId = 0;

    // [SIM707x §12.2.2] TLS is a FLAG on the ordinary socket via AT+CASSLCFG,
    // not a parallel command family - so every socket can be secure, against
    // the A76XX family's two dedicated sessions.
    _profile.tlsStyle = GsmTlsStyle::FlagOnOpen;
    _profile.maxTlsSessions = 13;

    // 1460, NOT 1459 - AND THE MANUALS DISAGREE WITH EACH OTHER.
    //
    //   [SIM707x V1.03 §12.2.5] "+CASEND: (0-12),(1-1459),(100-10000)"
    //   [SIM707x V1.05 §12.2.5] "+CASEND: (0-12),(1-1460),(100-10000)"
    //
    // Both are SIMCom's own published manuals, and an earlier version of this
    // driver cited the older one. A real SIM7070G on firmware
    // 1951B16SIM7070 answers AT+CASEND=? with (1-1460) and AT+CARECV=? with
    // (1-1460), which settles it.
    //
    // One byte, and it would have shown up as an occasional refused send at
    // exactly the full chunk size - the kind of fault that looks like a
    // network problem.
    _profile.maxSendChunk = 1460;
    _profile.maxRecvChunk = 1460;
    _profile.maxTlsSendChunk = 1460;       // the same path, so the same limit

    // [SIM707x §1.4.4] "The Command line buffer can accept a maximum of 559
    // characters ... or 39 AT commands."
    _profile.maxAtLineLength = 559;

    _profile.dataAccessMode = GsmDataAccessMode::BufferAccess;

    // [SIM707x §12.2.5, §12.2.7] Free send-buffer space can be queried before
    // sending and unacknowledged bytes after - a better shape than discovering
    // an overrun afterwards, and the reason this is an enumeration rather than
    // one hard-coded behaviour.
    _profile.sendAccounting = GsmSendAccounting::LeftSizeAndAck;

    // [SIM707x §12.2.8] V1.03 documents AT+CASTATE? as a READ COMMAND only.
    // V1.04 added an unsolicited form: "If the remote connection is
    // disconnected: +CASTATE: <cid>,<state>".
    //
    // SO THIS VARIES BY FIRMWARE, NOT BY PART. The driver registers the URC
    // either way - an older firmware simply never sends it - and the socket
    // layer also polls, so neither firmware is left unable to notice a closed
    // socket.
    _profile.closeNotice = GsmCloseNotice::Urc;

    // [SIM707x §1.4] "Only enter AT Command through serial port after ...
    // Unsolicited Result Code 'RDY' is received", but also "If auto-bauding is
    // enabled, the Unsolicited Result Codes 'RDY' and so on are not indicated".
    //
    // So the banner exists but CANNOT BE RELIED ON. It is declared as an
    // accelerator; bring-up still polls a bare attention command, which works
    // in both cases.
    _profile.bootBanner = GsmBootBanner::Ready;

    _profile.hasUdp = true;
    _profile.hasTcpServer = true;          // [SIM707x §12.2.4] AT+CASERVER
    _profile.hasGnss = true;               // [SIM707x §8]
    _profile.hasPsm = true;                // [SIM707x §5.2.39] AT+CPSMS
    _profile.hasEdrx = true;
    _profile.hasSmsPdu = true;
    _profile.hasMultiContext = true;       // [SIM707x §7.2.1] <pdpidx> 0..3
    _profile.hasPacketRegistration = true;
    _profile.hasEpsRegistration = true;

    _profile.bearerUpTimeoutMs = 60000;
    _profile.socketOpenTimeoutMs = 60000;
    _profile.socketSendTimeoutMs = 30000;
    _profile.socketFetchTimeoutMs = 10000;
    _profile.operatorScanTimeoutMs = 180000;
    _profile.defaultCommandTimeoutMs = 5000;

    // [SIM707x §12.2.5] "<inputtime> Millisecond, should input data during
    // this period or you can't input data when timeout. ... The default
    // inputtime is 5000ms."
    //
    // A DEADLINE ON OUR OWN PAYLOAD DELIVERY, which the A76XX family does not
    // impose. Set explicitly at the generous end rather than left to default,
    // because a slow poll on a busy controller is not a reason to lose a send.
    _profile.sendInputTimeMs = 10000;

    // [SIM7070G-HP HW V1.02 3.2.1 T12] Ton(uart) = 3 s typical, 4 s max. 15 s is
    // kept anyway - it is a deadline, not a delay; Syncing spends it probing.
    
    // REGISTRATION ON AN LPWA MODULE IS NOT A 90-SECOND OPERATION. The inherited
    // default suits Cat-1: one wideband radio, a fast scan. This part searches up
    // to 18 Cat-M and 17 NB-IoT bands, each needing a long dwell on a narrowband
    // carrier. A cold search runs to many minutes.
    //
    // At 90 s the state gave up mid-scan and recovery restarted the radio, which
    // restarts the scan - so it never finished one and sat at "+CEREG: 1,2"
    // looking like a coverage fault. Naming an operator worked because that skips
    // the search, which is the tell.
    //
    // Ten minutes. An application that knows its bands cuts this to seconds with
    // bandConfigStep.
    _profile.registrationTimeoutMs = 600000;

    // [SIM7070G-HP HW V1.02 §3.2.1, Table 12] Ton(uart) 3 s typical, 4 s
    // maximum. A DEADLINE, not a delay - Syncing spends it probing - so this
    // is the documented maximum with margin, not a round number that suits
    // both families. The A76XX figure is nearly three times this.
    _profile.powerOnToUartMs = 10000;

    // [SIM7070G-HP §3.2.2, Table 13] Toff(uart)/Toff(status) 5 s typical,
    // Toff-on 2 s minimum.
    _profile.powerOffSettleMs = 5000;
    _profile.powerOffToOnMs = 2000;
    _profile.resetSettleMs = 3000;

    // NO RESET PIN. [SIM7070G-HP HW V1.02 §2.2, Table 7] lists PWRKEY as the
    // only System Control pin, and §3.2.1 gives the reset that exists instead:
    // "The PWRKEY pin has its own reset function ... the module will be reset
    // after 12 seconds." Zero, so a board's reset line - wired for some other
    // module - is never pulsed at this one.
    _profile.resetPulseMs = 0;

    // --- The power key, as a pulse ---
    //
    // [SIM7070G-HP Hardware Design V1.02 §3.2.1, Table 12] Ton, "the time of
    // active low level impulse of PWRKEY pin to power on module" = 0.5 s
    // MINIMUM, 12 s maximum.
    //
    // 1000 ms: comfortably clear of the 500 ms minimum, and far short of both
    // the 12 s maximum and the internal reset timer below. The other family
    // needs 50 ms for the same job - a twentyfold difference, and the reason
    // the application is never asked to time this.
    _profile.powerKeyOnPulseMs = 1000;

    // [SIM7070G-HP §3.2.2, Table 13] Toff, "the active low level time pulse on
    // PWRKEY pin to power off module" = 1.2 s MINIMUM. 1500 leaves margin
    // without approaching the reset timer.
    _profile.powerKeyOffPulseMs = 1500;

    // [SIM7070G-HP §3.2.1] "The PWRKEY pin has its own reset function. The
    // reset time is determined by the internal timer (default is 12 seconds).
    // After the PWRKEY is pulled low, the module will be reset after 12
    // seconds. Therefore, it is not recommended to connect PWRKEY to ground
    // all the time in external circuit design." Table 12's NOTE 1 repeats it:
    // "PWRKEY cannot be shorted all the way to ground."
    //
    // So a stuck key here is not inert - it is a module that resets every 12
    // seconds forever, which on a bench looks exactly like a dead one.
    _profile.powerKeyMaxHoldMs = 12000;

    // [SIM7070G-HP §3.2.1, Table 12] Ton(wait) = 1 s MINIMUM, "the time from
    // VBAT power-on stabilization to PWRKEY pin pull-down validity". NOTE 2
    // says the same: "ensure that the VBAT voltage rises and stabilizes before
    // pulling down the PWRKEY pin to start up." Pull the key earlier and
    // nothing happens at all.
    _profile.supplySettleMs = 1000;

    // [SIM707x §5.2.x] The system information reply carries RSRP, but THE
    // MANUAL GIVES NO CONVERSION for it - unlike the A76XX manual, which
    // states "(RSRP report value -140) dBm" explicitly.
    //
    // So no conversion is claimed. Reporting a number derived from a formula
    // this manual does not contain would be a guess dressed as a measurement,
    // and the whole point of this library is that it does not do that. The
    // raw value is still carried; a capture from real hardware settles it.
    _profile.hasRsrpConversion = false;
}

bool GsmDriverSIM707x::claims(const char* identification, const char* revision) const {
    (void)revision;
    if (identification == nullptr) { return false; }

    // [SIM707x §1.1] "including SIM7080G, SIM7070G, SIM7070E, SIM7070G-NG and
    // SIM7090G".
    static const char* kNames[] = { "SIM7070", "SIM7080", "SIM7090", "A7070" };
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

GsmStep GsmDriverSIM707x::initStep(uint8_t index, GsmCmdBuf& out) const {
    if (index < kInitSteps) { return GsmDriver3gpp::initStep(index, out); }

    out.clear();
    GsmStep step;

    if (index - kInitSteps == 0) {
        // [SIM707x §12.2.10] AT+CACFG - configure transparent transmission.
        //
        // Explicitly OFF. Transparent mode turns the socket into a raw pipe
        // and takes the AT channel away with it: one socket, no multiplexing,
        // and no way to ask the module anything while data is flowing.
        //
        // Optional because older firmware does not have the command, and a
        // module without it is not in transparent mode anyway.
        out.add("AT+CACFG=\"TRANSWAITTM\",0");
        step.present = true;
        step.timeoutMs = _profile.defaultCommandTimeoutMs;
        step.ignoreFailure = true;
        return step;
    }

    return step;
}

// =============================================================================
// IDENTITY
// =============================================================================

void GsmDriverSIM707x::iccidCommand(GsmCmdBuf& out) const {
    // [SIM707x §5.2.8] AT+CCID - show ICCID. NOT AT+CICCID, which is what the
    // A76XX family calls the same query.
    out.clear();
    out.add("AT+CCID");
}

bool GsmDriverSIM707x::parseIccid(const char* line, size_t length, GsmField& out) const {
    // [SIM707x §5.2.8] The reply is "Ccid data [ex. 898600810906F8048812]" -
    // THE BARE DIGITS, WITH NO PREFIX AT ALL, where the A76XX family answers
    // with a prefixed "+ICCID:" line.
    //
    // Two modules, one piece of information, two commands and two reply
    // shapes. This is what a driver is for.
    if (line == nullptr || length < 10) { return false; }

    for (size_t i = 0; i < length; i++) {
        const char c = line[i];
        const bool isIccidCharacter = (c >= '0' && c <= '9') ||
                                      (c >= 'A' && c <= 'F') ||
                                      (c >= 'a' && c <= 'f');
        if (!isIccidCharacter) { return false; }
    }

    out.data = line;
    out.length = length;
    return true;
}

// =============================================================================
// SYSTEM INFORMATION
// =============================================================================

void GsmDriverSIM707x::systemInfoCommand(GsmCmdBuf& out) const {
    // [SIM707x §5.2.x] AT+CPSI? - inquiring UE system information.
    out.clear();
    out.add("AT+CPSI?");
}

bool GsmDriverSIM707x::parseSystemInfo(const char* line, size_t length,
                                       GsmSignal& signal,
                                       GsmTechnology& technology) const {
    // [SIM707x] The LTE form carries "...,<Band>,<earfcn>,<dlbw>,<ulbw>,
    // <RSRQ>,<RSRP>,<RSSI>,<RSSNR>", and the manual notes RSRP is "Available
    // for CAT-M or NB-IOT".
    //
    // NO CONVERSION IS APPLIED, because this manual states none. The A76XX
    // manual says "(report - 140) dBm" in as many words; applying that formula
    // here would be borrowing a fact from a different document about different
    // silicon, which is exactly the mistake that cost this library a rewrite
    // once already.
    GsmField rest, mode;
    if (!GsmParse::afterPrefix(line, length, "+CPSI", rest)) { return false; }
    if (!GsmParse::field(rest.data, rest.length, 0, mode)) { return false; }

    // THE TECHNOLOGY IS THE SECOND WORD WHEN THERE ARE TWO. This firmware
    // answers "LTE CAT-M1" and "LTE NB-IOT" - the same "LTE" in front of both
    // - so testing only the start called every NB-IoT cell Cat-M. NB-IoT is
    // whatever names NB anywhere in the field; any other LTE form is Cat-M.
    if (mode.startsWith("LTE") || mode.startsWith("NB") || mode.startsWith("CAT")) {
        bool narrowband = false;
        for (size_t i = 0; i + 1 < mode.length; i++) {
            if (mode.data[i] == 'N' && mode.data[i + 1] == 'B') { narrowband = true; break; }
        }
        technology = narrowband ? GsmTechnology::EutranNbS1 : GsmTechnology::EutranCatM;
        return true;
    }
    if (mode.startsWith("GSM") || mode.startsWith("EDGE")) {
        technology = GsmTechnology::Gsm;
        return true;
    }
    (void)signal;
    technology = GsmTechnology::Unknown;
    return true;
}

// =============================================================================
// BEARER
// =============================================================================

GsmStep GsmDriverSIM707x::bandConfigStep(uint8_t index, const GsmBandList& catM,
                                         const GsmBandList& nbIot,
                                         GsmCmdBuf& out) const {
    out.clear();
    GsmStep step;

    // THE LISTS THAT ARE SET, NUMBERED FROM ZERO. Indexing them by position
    // - 0 Cat-M, 1 NB-IoT - meant an application that narrowed only NB-IoT
    // got nothing at all: step 0 had no Cat-M list, answered "not present",
    // and a step that is not present ENDS THE PHASE, so step 1 was never
    // asked for. Counting only the lists that exist keeps the numbering
    // stable for a retry and leaves no hole in the middle.
    const bool haveCatM = catM.isSet();
    const bool haveNbIot = nbIot.isSet();
    bool isCatM = false;
    if (index == 0 && haveCatM) {
        isCatM = true;
    } else if ((index == 0 && !haveCatM && haveNbIot) ||
               (index == 1 && haveCatM && haveNbIot)) {
        isCatM = false;
    } else {
        return step;
    }
    const GsmBandList& wanted = isCatM ? catM : nbIot;

    // [SIM707x §5.2.26] AT+CBANDCFG="CAT-M"|"NB-IOT",<band>[,<band>...]
    //
    // The module's own answer to AT+CBANDCFG? on a fresh part lists every
    // band it supports, which is thirty-five scans across the two
    // technologies. Naming the ones a site actually uses is the difference
    // between a search measured in minutes and one measured in seconds.
    out.add("AT+CBANDCFG=");
    out.addQuoted(isCatM ? "CAT-M" : "NB-IOT");
    for (uint8_t i = 0; i < wanted.count; i++) {
        out.add(',');
        out.add((uint32_t)wanted.bands[i]);
    }

    step.present = true;
    step.timeoutMs = 10000;

    // Ignored: a band this module does not support is a configuration
    // mistake worth surviving, not a reason to refuse to come up at all.
    step.ignoreFailure = true;
    return step;
}

GsmStep GsmDriverSIM707x::radioPreferenceStep(uint8_t index,
                                              GsmRadioPreference wanted,
                                              GsmCmdBuf& out) const {
    out.clear();
    GsmStep step;

    // AUTOMATIC IS ASSERTED, NOT ASSUMED. Sending nothing inherits whatever
    // the factory, a previous firmware or a diagnostic left in NVRAM - these
    // settings are AUTO_SAVE - and the library cannot then say what it is.
    //
    // Automatic therefore means "every technology this module has", stated:
    // CNMP=2 and CMNB=3. That is the opposite of pinning.

    if (index == 0) {
        // [SIM707x §5.2.16] AT+CNMP=<mode>
        //   2 Automatic, 13 GSM only, 38 LTE only, 51 GSM and LTE only
        //
        // NB-IoT and Cat-M are both E-UTRAN, so both live under "LTE" here -
        // which technology of the two is then chosen by AT+CMNB below.
        const char* mode = nullptr;
        switch (wanted) {
            // [SIM707x §5.2.16] The test form answers
            //   ((2-Automatic),(13-GSM Only),(38-LTE Only),(51-GSM And LTE Only))
            // on a real SIM7070G, so these four are the whole set.
            case GsmRadioPreference::Automatic: mode = "2"; break;
            case GsmRadioPreference::GsmOnly:   mode = "13"; break;
            case GsmRadioPreference::LteOnly:
            case GsmRadioPreference::CatM:
            case GsmRadioPreference::NbIot:
            case GsmRadioPreference::CatMAndNbIot: mode = "38"; break;
            case GsmRadioPreference::GsmAndLte: mode = "51"; break;
            default: return step;
        }
        // [SIM707x §5.2.16] AT+CNMP=<mode>
        out.add("AT+CNMP=");
        out.add(mode);
        step.present = true;
        step.timeoutMs = 10000;

        // NOT ignoreFailure. Pointing the radio somewhere is the difference
        // between registering and searching forever, and a module that
        // refuses is one whose behaviour we can no longer predict.
        step.ignoreFailure = false;
        return step;
    }

    if (index == 1) {
        // [SIM707x §5.2.17] AT+CMNB=<mode>  1 Cat-M, 2 NB-IoT, 3 both.
        //
        // Only meaningful once CNMP has allowed LTE at all, which is why it
        // runs second.
        const char* mode = nullptr;
        switch (wanted) {
            case GsmRadioPreference::CatM:         mode = "1"; break;
            case GsmRadioPreference::NbIot:        mode = "2"; break;

            // Automatic asks for BOTH, because "everything this module can
            // reach" is what automatic has to mean on a command whose only
            // values are one, the other, or both.
            case GsmRadioPreference::Automatic:
            case GsmRadioPreference::CatMAndNbIot: mode = "3"; break;
            default: return step;    // GSM/LTE choices do not touch this
        }
        // [SIM707x §5.2.17] AT+CMNB=<mode>
        out.add("AT+CMNB=");
        out.add(mode);
        step.present = true;
        step.timeoutMs = 10000;
        step.ignoreFailure = false;
        return step;
    }

    return step;
}

bool GsmDriverSIM707x::parseRadioConfig(const char* line, size_t length,
                                        GsmRadioConfig& out) const {
    GsmField rest, field;
    uint32_t value = 0;

    // [SIM707x §5.2.17] "+CMNB: <mode>" - 1 Cat-M, 2 NB-IoT, 3 both.
    if (GsmParse::afterPrefix(line, length, "+CMNB", rest)) {
        if (GsmParse::field(rest.data, rest.length, 0, field) &&
            GsmParse::toUnsignedInRange(field, 0, 255, value)) {
            out.preferredMode = (int16_t)value;
        }
        return true;
    }

    // [SIM707x §5.2.6] "+CNSMOD: <n>,<stat>" - the SECOND field is the one
    // that says which system we are actually on. The manual's own examples
    // misprint the prefix as "+CNSMODE:" while its syntax table says
    // "+CNSMOD:"; hardware sends the latter.
    if (GsmParse::afterPrefix(line, length, "+CNSMOD", rest)) {
        if (GsmParse::field(rest.data, rest.length, 1, field) &&
            GsmParse::toUnsignedInRange(field, 0, 255, value)) {
            out.systemMode = (int16_t)value;
        }
        return true;
    }

    // [SIM707x §5.2.26] "+CBANDCFG: "CAT-M",28,3" then a second line for
    // "NB-IOT". Kept as the module's own words - see GsmRadioConfig - and the
    // two lines are appended so both technologies are visible at once.
    //
    // A TECHNOLOGY ALREADY SHOWN MEANS A NEW ANSWER. The read-back is asked
    // on every bring-up and nothing cleared the text between them, so each
    // pass appended both lines again and the third ran off the end of the
    // buffer - a diagnostic page showing the bands twice and then a fragment.
    // The driver keeps no state to know where one answer ends, but the
    // answer itself says: the same technology turning up twice is the next
    // read-back starting.
    if (GsmParse::afterPrefix(line, length, "+CBANDCFG", rest)) {
        GsmField label;
        if (GsmParse::field(rest.data, rest.length, 0, label) && !label.isEmpty()) {
            for (size_t at = 0; at < sizeof out.bands && out.bands[at] != '\0'; at++) {
                size_t k = 0;
                while (k < label.length && at + k < sizeof out.bands &&
                       out.bands[at + k] == label.data[k]) { k++; }
                if (k == label.length) { out.bands[0] = '\0'; break; }
            }
        }
        size_t used = 0;
        while (used < sizeof out.bands - 1 && out.bands[used] != '\0') { used++; }
        if (used != 0 && used < sizeof out.bands - 2) { out.bands[used++] = ' '; }
        for (size_t i = 0; i < rest.length && used < sizeof out.bands - 1; i++) {
            const char c = rest.data[i];
            if (c == '\r' || c == '\n') { continue; }
            out.bands[used++] = c;
        }
        out.bands[used] = '\0';
        return true;
    }

    return GsmDriver3gpp::parseRadioConfig(line, length, out);
}

void GsmDriverSIM707x::radioReadbackCommand(uint8_t index, GsmCmdBuf& out) const {
    // The shared two first, then the three this family has and A76XX does not.
    if (index < 2) { GsmDriver3gpp::radioReadbackCommand(index, out); return; }

    out.clear();
    switch (index - 2) {
        // [SIM707x §5.2.17] AT+CMNB? - Cat-M, NB-IoT, or both.
        case 0: out.add("AT+CMNB?"); return;

        // [SIM707x §5.2.26] AT+CBANDCFG? - answers TWO lines, one per
        // technology: "CAT-M",<band>[,<band>...] then "NB-IOT",...
        case 1: out.add("AT+CBANDCFG?"); return;

        // [SIM707x §5.2.6] AT+CNSMOD? - which system the module is actually
        // on right now. 7 is LTE M1, 9 is LTE NB, 0 is no service.
        case 2: out.add("AT+CNSMOD?"); return;

        default: return;
    }
}

GsmStep GsmDriverSIM707x::bearerUpStep(uint8_t index, const GsmApn& apn,
                                       GsmCmdBuf& out) const {
    // Sequence per [SIM707x TCPUDP(S) App Note V1.03 §5.1], matching both
    // TinyGSM forks. Three points it turns on:
    //
    //   - TEAR DOWN FIRST (CNACT=0,0 then CGATT=0), so a module part-way
    //     through an attach is stopped rather than re-asked.
    //   - ASK AT+CGNAPN. [§5.2.20] the network delivers an APN on a successful
    //     attach; TinyGSM ships it saying only that it makes the connection
    //     more consistent.
    //   - DO NOT use AT+CGACT. [§6.2.3] "used to test PDPs with network
    //     simulators. Successful activation of PDP on real network is not
    //     guaranteed." CNACT is the production path.
    out.clear();
    GsmStep step;

    switch (index) {
        case 0:
            // [SIM707x §7.2.1] Deactivate whatever is there. Optional: on a
            // cold module there is nothing to deactivate and it answers ERROR.
            out.add("AT+CNACT=");
            out.add((uint32_t)apn.contextId);
            out.add(",0");
            step.present = true;
            step.timeoutMs = 20000;
            step.ignoreFailure = true;
            return step;

        case 1:
            // [3GPP TS 27.007 10.1.9] Detach. Same reasoning.
            out.add("AT+CGATT=0");
            step.present = true;
            step.timeoutMs = 75000;      // [SIM707x §6.2.1] Max Response 75 s
            step.ignoreFailure = true;
            return step;

        case 2:
            // [3GPP TS 27.007 10.1.1] The APN, before the attach that carries
            // it. The context id is one-based in the standard while this
            // library numbers bearers from zero.
            out.add("AT+CGDCONT=");
            out.add((uint32_t)(apn.contextId + 1));
            out.add(",\"IP\",");
            out.addQuoted(apn.name != nullptr ? apn.name : "");
            step.present = true;
            step.timeoutMs = 10000;
            return step;

        case 3:
            // [3GPP TS 27.007 10.1.9] Attach. THE ACTIVE COMMAND, not a query.
            // [SIM707x §6.2.1] Max Response Time 75 seconds - the longest in
            // this family's bring-up, and eight times the A76XX figure for the
            // same command.
            out.add("AT+CGATT=1");
            step.present = true;
            step.timeoutMs = 75000;
            return step;

        case 4:
            // [SIM707x §5.2.20] The APN the network actually delivered.
            // Execution form only; no read, no write. Optional because it is
            // only valid once attached, and we may not be.
            out.add("AT+CGNAPN");
            step.present = true;
            step.timeoutMs = 10000;
            step.ignoreFailure = true;
            return step;

        case 5: {
            // [SIM707x §7.2.2] AT+CNCFG=<pdpidx>,<ip_type>,[<APN>,[<user>,
            // <password>,[<auth>]]]  -  <ip_type> 1 = IPv4;
            // <auth> 0 none, 1 PAP, 2 CHAP, 3 either.
            //
            // A SEPARATE CONFIGURATION FROM THE STANDARD CONTEXT, and the
            // index is off by one from AT+CGDCONT above. TinyGSM hit the same
            // thing: "for reasons not understood by me, use PDP context
            // identifier of 0 for what we defined as 1 above".
            out.add("AT+CNCFG=");
            out.add((uint32_t)apn.contextId);
            out.add(",1,");
            out.addQuoted(apn.name != nullptr ? apn.name : "");

            if (apn.user != nullptr && apn.user[0] != '\0') {
                out.add(',');
                out.addQuoted(apn.user);
                out.add(',');
                out.addQuoted(apn.password != nullptr ? apn.password : "");
                out.add(',');
                switch (apn.auth) {
                    case GsmAuth::None:      out.add((uint32_t)0); break;
                    case GsmAuth::Pap:       out.add((uint32_t)1); break;
                    case GsmAuth::Chap:      out.add((uint32_t)2); break;
                    case GsmAuth::PapOrChap:
                    case GsmAuth::Auto:      out.add((uint32_t)3); break;
                }
            }
            step.present = true;
            step.timeoutMs = _profile.defaultCommandTimeoutMs;
            return step;
        }

        case 6:
            // [SIM707x §7.2.1] AT+CNACT=<pdpidx>,1 - Active. The note records
            // that "+APP PDP: <pdpidx>,ACTIVE will be reported if the app
            // network actived", which is the URC the core watches for.
            //
            // FOUR RETRIES. Both TinyGSM forks wrap this in
            // `while (!res && ntries < 5)`, each attempt waiting 60 s. Two
            // independently maintained forks arrived at the same number.
            //
            // Action 2 is "Auto Active" - the module retries by itself. NOT
            // used: this library owns its own reconnection policy, and two
            // retry loops fighting each other is worse than either alone.
            out.add("AT+CNACT=");
            out.add((uint32_t)apn.contextId);
            out.add(",1");
            step.present = true;
            step.timeoutMs = _profile.bearerUpTimeoutMs;
            step.retries = 4;
            return step;

        case 7:
            // [SIM707x §19.2.1] Which context DNS resolves through. EnviroDIY
            // sets this and the servers below; upstream TinyGSM sets neither
            // and its users hit name resolution failures.
            out.add("AT+CDNSPDPID=");
            out.add((uint32_t)apn.contextId);
            step.present = true;
            step.timeoutMs = 10000;
            step.ignoreFailure = true;
            return step;

        case 8:
            // [SIM707x §19.2.2] AT+CDNSCFG=<Primary DNS>,<Secondary DNS>
            //
            // ONLY IF THE APPLICATION NAMED ONE. Left alone, the module uses
            // the resolver the network handed it with the bearer, which on a
            // private APN is the only one that can see the private names.
            // See GsmApn::dnsPrimary for why this is not a default.
            if (apn.dnsPrimary == nullptr || apn.dnsPrimary[0] == '\0') {
                return step;
            }
            out.add("AT+CDNSCFG=");
            out.addQuoted(apn.dnsPrimary);
            if (apn.dnsSecondary != nullptr && apn.dnsSecondary[0] != '\0') {
                out.add(',');
                out.addQuoted(apn.dnsSecondary);
            }
            step.present = true;
            step.timeoutMs = 10000;
            step.ignoreFailure = true;
            return step;

        default:
            return step;
    }
}

GsmStep GsmDriverSIM707x::bearerDownStep(uint8_t index, const GsmApn& apn,
                                         GsmCmdBuf& out) const {
    out.clear();
    GsmStep step;

    if (index == 0) {
        // [SIM707x §7.2.1] AT+CNACT=<pdpidx>,0 - deactivate.
        out.add("AT+CNACT=");
        out.add((uint32_t)apn.contextId);
        out.add(",0");
        step.present = true;
        step.timeoutMs = 20000;
        step.ignoreFailure = true;
        return step;
    }
    return step;
}

void GsmDriverSIM707x::addressCommand(const GsmApn& apn, GsmCmdBuf& out) const {
    (void)apn;
    // [SIM707x §7.2.1] The read form: "+CNACT: <pdpidx>,<statusx>,<addressx>".
    out.clear();
    out.add("AT+CNACT?");
}

bool GsmDriverSIM707x::parseAddress(const char* line, size_t length, GsmField& out) const {
    // [SIM707x §7.2.1] "+CNACT: 0,1,\"10.0.4.7\"" - the address is the third
    // field, and the module lists EVERY context, so an inactive one answers
    // "0.0.0.0" rather than being absent.
    GsmField rest, field;
    if (!GsmParse::afterPrefix(line, length, "+CNACT", rest)) { return false; }

    uint32_t status = 0;
    if (!GsmParse::field(rest.data, rest.length, 1, field) ||
        !GsmParse::toUnsigned(field, status)) { return false; }
    if (status == 0) { return false; }      // deactivated: it has no address

    if (!GsmParse::field(rest.data, rest.length, 2, field)) { return false; }
    out = GsmParse::unquote(field);
    return !out.isEmpty() && !out.equals("0.0.0.0");
}

// =============================================================================
// SOCKETS
// =============================================================================

GsmStep GsmDriverSIM707x::tlsSetupStep(uint8_t index, uint8_t socketId,
                                       const GsmEndpoint& endpoint,
                                       GsmCmdBuf& out) const {
    // TLS ON THIS FAMILY IS A FLAG ON AN ORDINARY SOCKET. The context is
    // configured first, then AT+CAOPEN runs unchanged - unlike the A76XX,
    // which has a whole parallel command family.
    out.clear();
    GsmStep step;
    if (endpoint.security != GsmSecurity::Module) { return step; }

    // [SIM707x §11.2.1] <ctxindex> is 0-5. One context is enough: every
    // socket in this library wants the same thing, and the module keeps the
    // per-socket binding on AT+CASSLCFG instead.
    const uint32_t context = 0;

    switch (index) {
        case 0:
            // [SIM707x §11.2.1] AT+CSSLCFG="SSLVERSION",<ctxindex>,<sslversion>
            //   3 = QAPI_NET_SSL_PROTOCOL_TLS_1_2
            //
            // LOWER CASE, although the manual prints the key in capitals. The
            // module rejects "SSLVERSION"; TinyGSM carries the same note.
            out.add("AT+CSSLCFG=\"sslversion\",");
            out.add(context);
            out.add(",3");
            break;

        case 1:
            // [SIM707x §11.2.1] AT+CSSLCFG="SNI",<ctxindex>,<servername>
            //
            // Server Name Indication. Without it a host sharing an address
            // with others is served the wrong certificate and the handshake
            // fails for a reason that looks like a bad CA.
            if (endpoint.host == nullptr || endpoint.host[0] == '\0') { return step; }
            out.add("AT+CSSLCFG=\"sni\",");
            out.add(context);
            out.add(',');
            out.addQuoted(endpoint.host);
            break;

        case 2:
            // [SIM707x §12.2.2] AT+CASSLCFG=<cid>,"SSL",<sslFlag> - 1 turns
            // TLS on for this socket. The key here IS capitalised; only the
            // CSSLCFG keys above are not.
            out.add("AT+CASSLCFG=");
            out.add((uint32_t)socketId);
            out.add(",\"SSL\",1");
            break;

        default:
            return step;
    }

    step.present = true;
    step.timeoutMs = _profile.defaultCommandTimeoutMs;

    // NOT ignoreFailure. A socket that was asked for TLS and opens without it
    // is worse than one that refuses to open: the bytes go out in clear and
    // nothing says so.
    step.ignoreFailure = false;
    return step;
}

void GsmDriverSIM707x::openCommand(uint8_t socketId, const GsmEndpoint& endpoint,
                                   const GsmApn& apn, GsmCmdBuf& out) const {
    out.clear();

    // [SIM707x §12.2.3] AT+CAOPEN=<cid>,<pdp_index>,<conn_type>,<server>,
    // <port>[,<recv_mode>]
    //
    // <conn_type> IS A QUOTED STRING - "TCP", "UDP", "NONIP" - in V1.05.
    // V1.03 documented it as a NUMBER, 0 for TCP and 1 for UDP, and V1.04's
    // change list records "AT+CAOPEN Extend range of <conn_type>". The string
    // form is what the current manual specifies and what the A76XX family also
    // uses, so it is what is sent; the older form is noted here because a
    // module on ancient firmware would refuse this and the capture would
    // otherwise be baffling.
    out.add("AT+CAOPEN=");
    out.add((uint32_t)socketId);
    out.add(',');
    out.add((uint32_t)apn.contextId);
    out.add(',');
    out.add(endpoint.kind == GsmSocketKind::Udp ? "\"UDP\"," : "\"TCP\",");
    out.addQuoted(endpoint.host);
    out.add(',');
    out.add((uint32_t)endpoint.port);
}

void GsmDriverSIM707x::sendCommand(uint8_t socketId, size_t length,
                                   const GsmEndpoint& endpoint, GsmCmdBuf& out) const {
    (void)endpoint;
    out.clear();

    // [SIM707x §12.2.5] AT+CASEND=<cid>,<datalen>[,<inputtime>]
    //
    // The input time is passed EXPLICITLY rather than left to its 5000 ms
    // default. It is a deadline on OUR delivery of the payload, and a
    // controller running a slow scan is not a reason to lose a send.
    out.add("AT+CASEND=");
    out.add((uint32_t)socketId);
    out.add(',');
    out.add((uint32_t)length);
    if (_profile.sendInputTimeMs != 0) {
        out.add(',');
        out.add(_profile.sendInputTimeMs);
    }
}

void GsmDriverSIM707x::fetchCommand(uint8_t socketId, size_t maxBytes,
                                    const GsmEndpoint& endpoint,
                                    GsmCmdBuf& out) const {
    // UNUSED, AND THAT IS THE POINT: TLS on this family is a flag set on the
    // same socket, so only the setup differs. Fetch, close and the reply
    // prefixes are identical for a plain socket and a secure one.
    (void)endpoint;
    // [SIM707x §12.2.6] AT+CARECV=<cid>,<readlen>
    out.clear();
    out.add("AT+CARECV=");
    out.add((uint32_t)socketId);
    out.add(',');
    out.add((uint32_t)maxBytes);
}

void GsmDriverSIM707x::closeCommand(uint8_t socketId, const GsmEndpoint& endpoint,
                                    GsmCmdBuf& out) const {
    // UNUSED, AND THAT IS THE POINT: TLS on this family is a flag set on the
    // same socket, so only the setup differs. Fetch, close and the reply
    // prefixes are identical for a plain socket and a secure one.
    (void)endpoint;
    // [SIM707x §12.2.9] AT+CACLOSE=<cid>
    out.clear();
    out.add("AT+CACLOSE=");
    out.add((uint32_t)socketId);
}

bool GsmDriverSIM707x::parseOpenResult(const char* line, size_t length,
                                       GsmOpenResult& out,
                                       const GsmEndpoint& endpoint) const {
    // UNUSED, AND THAT IS THE POINT: TLS on this family is a flag set on the
    // same socket, so only the setup differs. Fetch, close and the reply
    // prefixes are identical for a plain socket and a secure one.
    (void)endpoint;
    // [SIM707x §12.2.3] "+CAOPEN: <cid>,<result>" - and unlike the A76XX
    // family this arrives BEFORE the OK, so it is an ordinary reply rather
    // than a deferred conclusion. That is why openDeferredPrefix() is not
    // overridden here.
    GsmField rest, field;
    if (!GsmParse::afterPrefix(line, length, "+CAOPEN", rest)) { return false; }

    uint32_t socketId = 0, result = 0;
    if (!GsmParse::field(rest.data, rest.length, 0, field) ||
        !GsmParse::toUnsigned(field, socketId) ||
        !fitsSocketId(_profile, socketId)) { return false; }
    if (!GsmParse::field(rest.data, rest.length, 1, field) ||
        !GsmParse::toUnsigned(field, result)) { return false; }

    out.socketId = (uint8_t)socketId;
    out.opened = (result == 0);

    // [SIM707x §12.2.3] A THIRD ERROR SPACE, 0..27, distinct from both of the
    // A76XX family's. It carries things no +CME code can express - 24 is a
    // certificate whose time has expired, 25 a common name that does not
    // match, 26 both - and those are commissioning mistakes somebody fixes in
    // a minute if they are told which one it was.
    out.result = (result == 0)
        ? GsmResult(GsmStatus::Ok)
        : GsmResult::fromModule(GsmErrorSpace::ConnResult, (uint16_t)result);
    return true;
}

bool GsmDriverSIM707x::parseFetchHeader(const char* line, size_t length,
                                        GsmFetchInfo& out,
                                        const GsmEndpoint& endpoint) const {
    // UNUSED, AND THAT IS THE POINT: TLS on this family is a flag set on the
    // same socket, so only the setup differs. Fetch, close and the reply
    // prefixes are identical for a plain socket and a secure one.
    (void)endpoint;
    // [SIM707x §12.2.6] "+CARECV: <recvlen>," followed by the bytes.
    //
    // NOTE THE SHAPE: one length and a trailing comma, where the A76XX family
    // sends four comma-separated fields. There is no socket id and no
    // remaining count, so the socket layer keeps asking until a fetch returns
    // nothing rather than being told how much is left.
    GsmField rest, field;
    if (!GsmParse::afterPrefix(line, length, "+CARECV", rest)) { return false; }
    if (!GsmParse::field(rest.data, rest.length, 0, field)) { return false; }

    uint32_t received = 0;
    if (!GsmParse::toUnsigned(field, received)) { return false; }

    out.length = received;
    out.hasRemaining = false;
    return true;
}

const char* GsmDriverSIM707x::inlineFetchPrefix(const GsmEndpoint& endpoint) const {
    // UNUSED, AND THAT IS THE POINT: TLS on this family is a flag set on the
    // same socket, so only the setup differs. Fetch, close and the reply
    // prefixes are identical for a plain socket and a secure one.
    (void)endpoint;
    // [SIM707x §12.2.6] "+CARECV: [<remote IP>,<remote port>,]<recvlen>,……
    // //output data" - THE PAYLOAD IS ON THE SAME LINE, after the comma that
    // follows the length.
    //
    // The optional address fields appear only when AT+CASRIP=1, which this
    // library deliberately never enables: leaving it off keeps the length in a
    // fixed position, and the remote address of a connected socket is already
    // known to whoever opened it.
    return "+CARECV: ";
}

bool GsmDriverSIM707x::parseSendResult(const char* line, size_t length,
                                       size_t requested, GsmResult& out,
                                       const GsmEndpoint& endpoint) const {
    // UNUSED, AND THAT IS THE POINT: TLS on this family is a flag set on the
    // same socket, so only the setup differs. Fetch, close and the reply
    // prefixes are identical for a plain socket and a secure one.
    (void)endpoint;
    (void)line; (void)length; (void)requested; (void)out;

    // [SIM707x §12.2.5] There is no confirmation line after the payload: the
    // module answers "+CASEND: <cid>,<datalen>" BEFORE the prompt, and a plain
    // OK afterwards.
    //
    // So a send concludes on the OK, and there is nothing here to match. The
    // A76XX family's fatal short-send rule has no counterpart; this family
    // prevents the overrun instead, by letting the free buffer size be queried
    // before the send is issued.
    return false;
}

// =============================================================================
// UNSOLICITED
// =============================================================================

const char* GsmDriverSIM707x::urcPrefix(uint8_t index) const {
    // [SIM707x §22.3] plus §12.2.3's note, which the summary table omits.
    static const char* kPrefixes[] = {
        "+CADATAIND:",   // [§12.2.3 note] data waiting on a socket
        "+CASTATE:",     // [§12.2.8] a socket closed - V1.04 firmware onward
        "+APP PDP:",     // [§7.2.1 note] the application bearer came or went
        "+PDP: DEACT",   // [§22.3] the network dropped the context
        "+CEREG:",
        "+CGREG:",
        "+CMTI:",
        "UNDER-VOLTAGE",
        "OVER-VOLTAGE",
        "NORMAL POWER DOWN",
        "RDY",
        nullptr,
    };
    if (index >= (sizeof kPrefixes / sizeof kPrefixes[0])) { return nullptr; }
    return kPrefixes[index];
}

GsmUrcKind GsmDriverSIM707x::classifyUrc(const char* line, size_t length,
                                         GsmUrcInfo& out) const {
    out = GsmUrcInfo();

    // [SIM707x §12.2.3] "After open a connection successfully, if module
    // receives data, it will report '+CADATAIND: <cid>'".
    //
    // CLEANLY SEPARATE FROM THE FETCH REPLY, unlike the A76XX family where one
    // prefix serves both and has to be told apart by a field value. Worth
    // noting because it is why classifyUrc is allowed to decline: neither
    // shape could be assumed.
    if (startsWith(line, length, "+CADATAIND:")) {
        GsmField rest, field;
        uint32_t socketId = 0;
        if (GsmParse::afterPrefix(line, length, "+CADATAIND", rest) &&
            GsmParse::field(rest.data, rest.length, 0, field) &&
            GsmParse::toUnsigned(field, socketId) && fitsSocketId(_profile, socketId)) {
            out.kind = GsmUrcKind::DataAvailable;
            out.socketId = (uint8_t)socketId;
            return out.kind;
        }
        return GsmUrcKind::Unrecognised;
    }

    // [SIM707x §12.2.8] "+CASTATE: <cid>,<state>" with 0 closed, 1 connected,
    // 2 listening. Both a read reply and - from V1.04 - a URC, so the state
    // value decides: only a closure is unsolicited news.
    if (startsWith(line, length, "+CASTATE:")) {
        GsmField rest, field;
        uint32_t socketId = 0, state = 0;
        if (GsmParse::afterPrefix(line, length, "+CASTATE", rest) &&
            GsmParse::field(rest.data, rest.length, 0, field) &&
            GsmParse::toUnsigned(field, socketId) && fitsSocketId(_profile, socketId) &&
            GsmParse::field(rest.data, rest.length, 1, field) &&
            GsmParse::toUnsigned(field, state) && state == 0) {
            out.kind = GsmUrcKind::SocketClosed;
            out.socketId = (uint8_t)socketId;
            return out.kind;
        }
        return GsmUrcKind::Unrecognised;    // a status line; let it fall through
    }

    // [SIM707x §7.2.1] "+APP PDP: <pdpidx>,ACTIVE" / ",DEACTIVE"
    if (startsWith(line, length, "+APP PDP:")) {
        GsmField rest, field;
        if (GsmParse::afterPrefix(line, length, "+APP PDP", rest) &&
            GsmParse::field(rest.data, rest.length, 1, field)) {
            out.kind = field.startsWith("ACTIVE") ? GsmUrcKind::BearerUp
                                                  : GsmUrcKind::BearerDown;
            return out.kind;
        }
        out.kind = GsmUrcKind::BearerDown;
        return out.kind;
    }

    // [SIM707x §22.3] "+PDP: DEACT - GPRS is disconnected by network".
    if (startsWith(line, length, "+PDP: DEACT")) {
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

    // [SIM707x §22.3] Under and over voltage, both spellings, and the power
    // down the module announces before it goes.
    if (startsWith(line, length, "UNDER-VOLTAGE") ||
        startsWith(line, length, "OVER-VOLTAGE")) {
        out.kind = GsmUrcKind::SupplyWarning;
        return out.kind;
    }
    if (startsWith(line, length, "NORMAL POWER DOWN")) {
        out.kind = GsmUrcKind::PowerDown;
        return out.kind;
    }

    // [SIM707x §22.3] "RDY - Power on procedure is completed". An accelerator
    // only; bring-up does not depend on it, because the same table says it is
    // absent when auto-bauding is active.
    if (startsWith(line, length, "RDY")) {
        out.kind = GsmUrcKind::Ready;
        return out.kind;
    }

    return GsmUrcKind::Unrecognised;
}

// =============================================================================
// POWER
// =============================================================================

void GsmDriverSIM707x::powerDownCommand(GsmCmdBuf& out) const {
    // [SIM707x §22.3] "NORMAL POWER DOWN - Module is powered down by the
    // PWRKEY pin or AT command 'AT+CPOWD=1'". The A76XX family spells the same
    // operation AT+CPOF.
    out.clear();
    out.add("AT+CPOWD=1");
}

void GsmDriverSIM707x::resetCommand(GsmCmdBuf& out) const {
    // [SIM707x §5.2.46] AT+CREBOOT.
    out.clear();
    out.add("AT+CREBOOT");
}

// =============================================================================
// LOCATION
// =============================================================================

void GsmDriverSIM707x::gnssPowerCommand(bool on, GsmCmdBuf& out) const {
    // [SIM707x §8.2.1] AT+CGNSPWR=<0|1>. One S, where the A76XX family uses
    // two - AT+CGNSSPWR. A single character apart, and neither module accepts
    // the other's spelling.
    out.clear();
    out.add(on ? "AT+CGNSPWR=1" : "AT+CGNSPWR=0");
}

void GsmDriverSIM707x::gnssInfoCommand(GsmCmdBuf& out) const {
    // [SIM707x §8.2.3] AT+CGNSINF.
    out.clear();
    out.add("AT+CGNSINF");
}

bool GsmDriverSIM707x::parseGnssInfo(const char* line, size_t length, GsmFix& out) const {
    // [SIM707x §8.2.3]
    // +CGNSINF: <GNSS run status>,<Fix status>,<UTC date & Time>,<Latitude>,
    //           <Longitude>,<MSL Altitude>,<Speed Over Ground>,
    //           <Course Over Ground>,<Fix Mode>,<Reserved1>,<HDOP>,<PDOP>,
    //           <VDOP>,<Reserved2>,<GNSS Satellites in View>,<Reserved3>,
    //           <HPA>,<VPA>
    //
    // EIGHTEEN FIELDS IN A DIFFERENT ORDER from the A76XX family's seventeen,
    // and - the part that would silently produce a position a few hundred
    // kilometres out - THE COORDINATES ARE SIGNED DECIMAL DEGREES here, where
    // that family reports NMEA ddmm.mmmmmm with the hemisphere in its own
    // field. Same information, no common parser.
    GsmField rest, field;
    if (!GsmParse::afterPrefix(line, length, "+CGNSINF", rest)) { return false; }

    uint32_t fixStatus = 0;
    if (!GsmParse::field(rest.data, rest.length, 1, field) ||
        !GsmParse::toUnsigned(field, fixStatus)) { return false; }

    if (fixStatus == 0) {
        // No fix. An honest answer, not a parse failure - and reported as Bad
        // rather than left as the last known position, which is how a tracked
        // asset ends up never appearing to move.
        out.hasPosition = false;
        out.quality = GsmQuality::Bad;
        return true;
    }

    if (!GsmParse::field(rest.data, rest.length, 3, field) ||
        !toDouble(field, out.latitude)) { return false; }
    if (!GsmParse::field(rest.data, rest.length, 4, field) ||
        !toDouble(field, out.longitude)) { return false; }
    out.hasPosition = true;

    if (GsmParse::field(rest.data, rest.length, 5, field) &&
        toDouble(field, out.altitudeMetres)) { out.hasAltitude = true; }
    if (GsmParse::field(rest.data, rest.length, 6, field) &&
        toDouble(field, out.speedKnots)) { out.hasSpeed = true; }
    if (GsmParse::field(rest.data, rest.length, 7, field)) {
        toDouble(field, out.courseDegrees);
    }
    if (GsmParse::field(rest.data, rest.length, 10, field)) {
        toDouble(field, out.hdop);
    }

    uint32_t satellites = 0;
    if (GsmParse::field(rest.data, rest.length, 14, field) &&
        GsmParse::toUnsigned(field, satellites)) {
        out.satellitesInView = (uint8_t)(satellites > 255 ? 255 : satellites);
    }

    out.quality = GsmQuality::Good;
    out.stampMillis = gsm_millis();
    return true;
}
