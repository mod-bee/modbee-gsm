// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#pragma once

// The vocabulary shared between the core, the drivers and the application.
// No behaviour, no I/O - just the words.

#include "GsmPlatform.h"
#include "GsmParse.h"
#include "GsmResult.h"

// --- Network ---

// [3GPP TS 27.007 7.2] The <stat> field of +CREG / +CGREG / +CEREG.
//
// The values are the standard's, so a driver never translates them. Worth
// knowing: 1 and 5 both mean "usable" - a device roaming is registered - and
// code that only accepts 1 refuses to work abroad.
enum class GsmRegistration : uint8_t {
    NotRegisteredNotSearching = 0,
    Home = 1,
    Searching = 2,
    Denied = 3,
    Unknown = 4,
    Roaming = 5,
    SmsOnlyHome = 6,
    SmsOnlyRoaming = 7,
    EmergencyOnly = 8,
    CsfbNotPreferredHome = 9,
    CsfbNotPreferredRoaming = 10,
};

// What the radio is actually set to, as the module reports it - not what was
// asked for. These settings are AUTO_SAVE, so a value left by a previous
// session outlives every power cycle and has to be readable.
//
// Everything is Unknown until asked. A field never read must not look like a
// field that reads zero.
// Operators a scan found, as numeric MCC+MNC.
//
// Bounded and small: a scan returns what is on air at one spot, which is a
// handful of networks, and this has to sit on a microcontroller.
//
// Only the NUMERIC code is kept. Long and short names vary by firmware and
// language; "53001" does not.
struct GsmOperatorList {
    static const uint8_t kMax = 6;

    char    codes[kMax][8] = {{0}};
    uint8_t count = 0;

    void clear() { count = 0; codes[0][0] = '\0'; }

    bool add(const char* code, size_t length) {
        if (count >= kMax || code == nullptr || length == 0 || length >= 8) {
            return false;
        }
        for (uint8_t i = 0; i < count; i++) {          // scans repeat entries
            size_t j = 0;
            while (j < length && codes[i][j] == code[j]) { j++; }
            if (j == length && codes[i][j] == '\0') { return false; }
        }
        for (size_t j = 0; j < length; j++) { codes[count][j] = code[j]; }
        codes[count][length] = '\0';
        count++;
        return true;
    }
};

struct GsmRadioConfig {
    // [SIM707x §5.2.16] / [A76XX §4.2.7] AT+CNMP: 2 automatic, 13 GSM only,
    // 14 WCDMA only (A76XX), 38 LTE only, 51 GSM and LTE (SIM707x).
    int16_t networkMode = -1;

    // [SIM707x §5.2.17] AT+CMNB: 1 Cat-M, 2 NB-IoT, 3 both. Absent on A76XX,
    // which has no such command - it is a Cat-1 part, not an LPWA one.
    int16_t preferredMode = -1;

    // [SIM707x §5.2.6] AT+CNSMOD: 0 none, 1 GSM, 3 EGPRS, 7 LTE M1, 9 LTE NB.
    // The direct answer to "which technology am I actually on".
    int16_t systemMode = -1;

    // [SIM707x §5.2.26] The band list the module answered with, verbatim, in
    // its own words. Kept as text rather than parsed into numbers because the
    // two families disagree about the shape - one lists bands, the other
    // reports a 64-bit mask - and an operator reading a diagnostic page wants
    // what the module said, not our interpretation of it.
    char bands[64] = { 0 };

    bool hasNetworkMode() const { return networkMode >= 0; }
    bool hasPreferredMode() const { return preferredMode >= 0; }
    bool hasSystemMode() const { return systemMode >= 0; }
    bool hasBands() const { return bands[0] != '\0'; }
};

inline bool gsm_isRegistered(GsmRegistration state) {
    return state == GsmRegistration::Home || state == GsmRegistration::Roaming;
}

// [3GPP TS 27.007 7.2] The <AcT> field. Only the values these modules can
// report are named; anything else is carried as Unknown rather than refused.
enum class GsmTechnology : uint8_t {
    Unknown = 0,
    Gsm,          // 2G
    GsmEgprs,     // 2G with EDGE
    Utran,        // 3G
    UtranHs,      // 3G with HSPA
    Eutran,       // LTE
    EutranNbS1,   // NB-IoT
    EutranCatM,   // LTE Cat-M1
};

const char* gsm_technologyName(GsmTechnology technology);

// How much to believe a reading. The same four values modbee-mqtt and
// modbee-modbus use, so an application reading several of them reads one
// vocabulary.
//
// Unknown and Bad are deliberately different: one means it was never read at
// all - a commissioning problem - and the other means it WAS read and has
// since gone stale. An alarm list that conflates them sends somebody to the
// wrong end of a site.
enum class GsmQuality : uint8_t { Unknown = 0, Bad, Uncertain, Good };

// What a signal reading says when nobody has taken one.
//
// NOT zero. Zero dBm is a real and outstanding signal level, so reporting an
// unread field as zero is how an alarm built on it never fires.
#ifndef GSM_UNKNOWN_DBM
#define GSM_UNKNOWN_DBM (-32768)
#endif

struct GsmSignal {
    // From the universal signal query. -113..-51 dBm, or unset.
    int  rssiDbm = 0;
    bool hasRssi = false;

    // Reference signal power and quality, where the module reports them and
    // the driver knows how to convert. On a narrowband technology these are
    // the only meaningful numbers - the 2G-era indicator bottoms out long
    // before the radio does.
    int  rsrpDbm = 0;
    bool hasRsrp = false;
    int  rsrqDb = 0;
    bool hasRsrq = false;

    GsmQuality quality = GsmQuality::Unknown;
};

// --- Bearer ---

enum class GsmAuth : uint8_t { Auto = 0, None, Pap, Chap, PapOrChap };

// The longest APN, user name and password GsmDevice::apn() holds, in
// characters. 3GPP allows a 100-octet APN [TS 23.003 9.1]. No standard bounds
// the user name and password; 127 each is what the modules take [SIM7070 AT
// 7.2.2, "+CNCFG: ...,150,127,127"].
#ifndef GSM_APN_MAX
#define GSM_APN_MAX 100
#endif
#ifndef GSM_APN_USER_MAX
#define GSM_APN_USER_MAX 127
#endif
#ifndef GSM_APN_PASSWORD_MAX
#define GSM_APN_PASSWORD_MAX 127
#endif

struct GsmApn {
    const char* name = nullptr;
    const char* user = nullptr;
    const char* password = nullptr;
    GsmAuth auth = GsmAuth::Auto;
    uint8_t contextId = 0;       // which PDP context, where the module has several

    // Name servers, if the deployment wants its own. Null means say nothing,
    // and the module uses whatever the network handed it with the bearer -
    // which on a private APN is the only resolver that can see private names.
    //
    // Whose resolver the lookups go to is the application's decision.
    const char* dnsPrimary = nullptr;
    const char* dnsSecondary = nullptr;
};

// --- Sockets ---

enum class GsmSocketKind : uint8_t { Tcp = 0, Udp };

enum class GsmSecurity : uint8_t {
    // The default, and the library does no crypto. A socket is a byte pipe;
    // whether those bytes are a TLS record is the business of whatever sits
    // above it - which on this stack is usually wolfSSL inside the MQTT layer.
    Plain = 0,

    // The module's own TLS engine. Opt-in, for boards where a software engine
    // plus a certificate chain will not fit.
    Module,
};

struct GsmEndpoint {
    const char* host = nullptr;
    uint16_t port = 0;
    GsmSocketKind kind = GsmSocketKind::Tcp;
    GsmSecurity security = GsmSecurity::Plain;
};

// What a driver made of a socket-open reply.
struct GsmOpenResult {
    uint8_t  socketId = 0;
    bool     opened = false;
    GsmResult result = GsmStatus::Ok;
};

// What a driver made of a receive-header line.
struct GsmFetchInfo {
    uint8_t socketId = 0;
    size_t  length = 0;        // bytes that follow, to be read as a payload
    size_t  remaining = 0;     // still buffered in the module afterwards
    bool    hasRemaining = false;
};

// What a URC turned out to be. The core acts on the kind; only the driver
// knows which text means which.
// Which radio to look for. Ignored by single-radio parts.
//
// An LPWA module pointed at a technology that is not on air searches for ever
// with no error - indistinguishable from poor coverage. The default is
// Automatic; narrowing it is the application's call.
enum class GsmRadioPreference : uint8_t {
    Automatic = 0,   // whatever the module defaults to; change nothing
    GsmOnly,
    LteOnly,
    GsmAndLte,
    CatM,            // LPWA modules only
    NbIot,           // LPWA modules only
    CatMAndNbIot,    // LPWA modules only
};
// Which bands to search, by 3GPP band number. Empty leaves the module's own
// configuration alone.
//
// Naming the bands in use turns a multi-minute cold search into a short one.
// The list is a deployment fact, so it comes from the application.
struct GsmBandList {
    // Band numbers as the 3GPP names them - 1, 3, 20, 28 and so on.
    const uint8_t* bands = nullptr;
    uint8_t        count = 0;

    bool isSet() const { return bands != nullptr && count > 0; }
};

enum class GsmUrcKind : uint8_t {
    Unrecognised = 0,
    DataAvailable,     // a socket has bytes waiting
    SocketClosed,      // one socket went
    BearerDown,        // the data context went; every socket with it
    BearerUp,
    RegistrationChanged,
    SmsReceived,
    PowerDown,         // the module is shutting down, by command or by fault
    SupplyWarning,     // under or over voltage, or temperature
    Ready,             // a boot banner, where the module emits one

    // RECOGNISED, AND DELIBERATELY NOTHING TO DO.
    //
    // Modules announce subsystem milestones - phonebook loaded, SMS storage
    // ready - that are worth KNOWING are expected, so they are not counted as
    // unhandled, but which no state machine should act on.
    //
    // The distinction is not pedantry. "SMS DONE" follows a boot, so it is
    // tempting to read it as one; doing that makes the library tear down a
    // perfectly good session and configure the module again every time its
    // SMS storage finishes initialising.
    Informational,
};

struct GsmUrcInfo {
    GsmUrcKind kind = GsmUrcKind::Unrecognised;
    uint8_t socketId = 0;
    uint32_t value = 0;        // a code or index, meaning depends on the kind

    // WHICH IDENTIFIER SPACE socketId belongs to. A module whose TLS is a
    // separate command family numbers its sessions from zero too, so session 0
    // and plain socket 0 can both exist and mean different sockets. Without
    // this, one socket's data would be delivered to the other.
    bool secure = false;
};

// --- Location ---

// A fix, WITH ITS QUALITY AND ITS AGE.
//
// A receiver that lost the sky five minutes ago keeps reporting its last
// position perfectly happily. Handing an application a bare latitude and
// longitude lets that become a tracked asset that never moved, so the quality
// and the age travel with the numbers rather than beside them.
struct GsmFix {
    double latitude = 0.0;
    double longitude = 0.0;
    double altitudeMetres = 0.0;
    double speedKnots = 0.0;
    double courseDegrees = 0.0;

    uint8_t satellitesInView = 0;
    double  hdop = 0.0;

    bool hasPosition = false;
    bool hasAltitude = false;
    bool hasSpeed = false;

    GsmParse::GsmDateTime utc;
    bool hasTime = false;

    GsmQuality quality = GsmQuality::Unknown;
    gsm_time_t stampMillis = 0;      // when this library last accepted it
};

// --- Identity ---

struct GsmIdentity {
    char manufacturer[32] = {0};
    char model[32] = {0};
    char revision[48] = {0};
    char imei[20] = {0};
    char iccid[24] = {0};
    char imsi[20] = {0};
};
