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
// GsmDevice - THE SIMPLE WAY IN
// =============================================================================
//
// Everything this library can do is reachable through GsmModem, GsmSockets and
// a driver. That is the full form, and nothing here replaces it.
//
// This is the other form: ONE OBJECT, sensible defaults, the fiddly parts done.
//
//     GsmDevice gsm;
//
//     void setup() {
//         Serial1.setRxBufferSize(2048);     // see receive flow control below
//         Serial1.begin(115200);
//         gsm.apn("internet");
//         gsm.begin(&Serial1);               // returns immediately
//     }
//
//     void loop() {
//         gsm.poll();                        // the only call that does work
//     }
//
// WHAT IT DOES FOR YOU:
//   - registers both shipped drivers and picks the right one from what the
//     module says about itself, so no module is named anywhere in your sketch
//   - runs the whole bring-up sequence, with the module's own timings
//   - reconnects by itself, with a jittered backoff
//   - hands you Arduino's own Client and UDP over cellular
//
// WHAT IT DOES NOT DO. It does not block, and it does not hide anything:
// modem(), sockets() and engine() return the real objects, and everything you
// can reach from there works exactly as it does in the long form. Start here,
// and drop down when you need to; nothing you wrote has to be rewritten.
//
// AND IT OWNS NO HARDWARE. Power and reset are hooks your application supplies
// - see GsmPowerControl.h. Supply none and the module is assumed already on,
// which is right for a USB modem and for a board that wires it live.

#include "GsmPlatform.h"

#include <type_traits>

#include "GsmModem.h"
#include "GsmSocket.h"
#include "GsmSms.h"
#include "GsmGnss.h"
#include "GsmClock.h"
#include "GsmDriverRegistry.h"
#include "drivers/GsmDriverA76XX.h"
#include "drivers/GsmDriverSIM707x.h"

#if defined(GSM_PLATFORM_ARDUINO) || defined(GSM_ARDUINO_SHIM)
#include "transport/GsmStreamTransport.h"
#endif

class GsmDevice {
public:
    GsmDevice();

    // --- Opening ---

    // A transport you built. The host form, and the one a test uses.
    //
    // False when the link will not open, or when the storage setCapacity()
    // asks for cannot be found - beginError() then says which.
    bool begin(GsmTransport* transport);

    // What this modem holds, set BEFORE begin(): sockets open at once, each
    // socket's send and receive buffer in bytes, and AT commands that may wait.
    // 0 keeps a default - 2 sockets, 1500 bytes each way, 12 commands. All of
    // it is taken once, in begin(), and never grows on the poll path.
    void setCapacity(uint8_t sockets, size_t socketTxBytes, size_t socketRxBytes,
                     size_t commands) {
        _sockets.setCapacity(sockets, socketTxBytes, socketRxBytes);
        _modem.engine().setCapacity(commands);
    }

    // Why begin() failed when the reason was this object's own memory -
    // "out of memory for the sockets" or "... for the AT command queue".
    // Null when begin() did not fail for that.
    const char* beginError() const { return _beginError; }

#if defined(GSM_PLATFORM_ARDUINO) || defined(GSM_ARDUINO_SHIM)
    // The Arduino convenience form: hand it the serial port.
    //
    // `receiveCapacity` is the size of the core's receive buffer, and it
    // matters more than it looks - see GsmTransport::receiveCapacity(). The
    // default is the pessimistic one, so a sketch that forgot to enlarge its
    // buffer still behaves rather than corrupting the AT stream under load.
    // CONSTRAINED SO IT CAN NEVER MATCH A TRANSPORT, and that is the whole
    // fix rather than a refinement of it.
    //
    // Unconstrained, this template is an EXACT match for any pointer - a
    // GsmTransport*, a GsmLoopbackTransport*, or the GsmStreamTransport it
    // builds itself - while the non-template overload needs a derived-to-base
    // conversion. An exact match beats a conversion, so the template won every
    // time and called itself, wrapping a transport in a transport until the
    // compiler gave up at depth 900.
    //
    // An explicit cast on the inner call fixes the recursion and NOT the rest:
    // a caller passing any other transport still bound to the template instead
    // of the overload meant for it. Excluding the whole hierarchy from
    // deduction is what makes the overload set say what was meant.
    template <typename StreamType,
              typename = typename std::enable_if<
                  !std::is_base_of<GsmTransport, StreamType>::value>::type>
    bool begin(StreamType* stream, uint32_t bitsPerSecond = 115200,
               size_t receiveCapacity = 256) {
        static GsmStreamTransport<StreamType> transport(*stream, bitsPerSecond,
                                                        receiveCapacity);

        // Cast as well as constrained. The constraint alone is enough; saying
        // it twice costs nothing and makes the intent legible at the call.
        return begin(static_cast<GsmTransport*>(&transport));
    }
#endif

    void end() { _modem.end(); }

    // --- Configuration, before or after begin() ---

    // False, and nothing changed, when any of them is longer than the
    // library holds (GSM_APN_MAX 100, GSM_APN_USER_MAX and
    // GSM_APN_PASSWORD_MAX 127): a cut APN is a different APN.
    bool apn(const char* name, const char* user = nullptr,
             const char* password = nullptr, GsmAuth auth = GsmAuth::Auto);

    void pin(const char* pin) { _modem.setPin(pin); }

    // Your board's power wiring, if it has any. Optional; see the header note.
    void powerControl(GsmPowerControl* hooks) { _power = hooks; }

    // A driver for a module this library does not ship. Registered before the
    // built-in ones, so a specific driver wins over a general one.
    bool addDriver(GsmDriver* driver) { return _registry.add(driver); }

    // --- Driving ---

    void poll();
    void start() { _modem.start(); }
    void stop() { _modem.stop(); }

    // --- State ---

    bool isReady() const { return _modem.isReady() && _modem.hasBearer(); }
    // Which radio to search for, on a module that has a choice.
    //
    // Leave it alone and the module keeps its own default, which is the right
    // behaviour on a Cat-1 part with one radio. On an LPWA part it is worth
    // setting deliberately: a module searching for a technology that is not
    // on air never registers and never says why.
    void radioPreference(GsmRadioPreference wanted) {
        _modem.radioPreference(wanted);
    }

    // Pin the module to one operator by numeric MCC+MNC, or nullptr for
    // automatic. Worth setting on a roaming SIM: a module that attaches to a
    // network it is not entitled to sits there rather than moving on.
    void operatorCode(const char* numericCode) {
        _modem.operatorCode(numericCode);
    }

    // Narrow the band search to the ones a site actually uses.
    //
    // The most effective thing available against a slow cold start on an LPWA
    // module: a fresh part searches every band it supports, which is
    // thirty-five scans across two technologies, each needing a long dwell.
    // Naming one or two turns minutes into seconds.
    void bands(const GsmBandList& catM, const GsmBandList& nbIot) {
        _modem.bands(catM, nbIot);
    }

    // Forget every network that has ever refused this SIM.
    //
    // Rarely needed by hand - bring-up does it by itself when registration
    // will not happen - but exposed because it is the single most useful
    // thing to try on a device that worked on the bench and will not connect
    // in the field.
    GsmResult clearForbiddenNetworks() { return _modem.clearForbiddenNetworks(); }

    GsmState state() const { return _modem.state(); }
    const char* stateName() const { return gsm_stateName(_modem.state()); }

    bool isRegistered() const { return _modem.isRegistered(); }
    bool hasBearer() const { return _modem.hasBearer(); }

    const char* module() const {
        return _modem.driver() != nullptr ? _modem.driver()->profile().family : "";
    }
    const GsmIdentity& identity() const { return _modem.identity(); }
    const GsmSignal& signal() const { return _modem.signal(); }

    // Signal quality bands, radio read-back and everything else an operator
    // looks at when a link is unhappy. None of it steers the state machine.
    GsmDiagnostics& diagnostics() { return _modem.diagnostics(); }

    // Registration, operator selection and the escalation ladder used when
    // automatic selection never settles.
    GsmNetwork& network() { return _modem.network(); }

    // The data context: which APN, whether it is up, what address it got.
    // Registered and having a bearer are DIFFERENT things.
    const GsmApn& bearer() const { return _modem.apn(); }

    // WHAT THE RADIO IS ACTUALLY SET TO, in the module's own words.
    //
    // Which mode, which LPWA technology, which bands, and which system it is
    // on right now. Read once per bring-up, after the library has finished
    // configuring it - so this is the module's answer, not our request.
    //

    const GsmRadioConfig& radioConfig() const { return _modem.radioConfig(); }
    GsmRegistration registration() const { return _modem.registration(); }
    GsmTechnology technology() const { return _modem.technology(); }
    const char* operatorName() const { return _modem.operatorName(); }
    const char* ipAddress() const { return _modem.ipAddress(); }

    GsmResult lastError() const { return _modem.lastError(); }

    // --- Named scalars ------------------------------------------------
    //
    // The same facts as identity(), signal() and counters(), one value at a
    // time. Structs suit C++; a function-block pin needs a scalar, and PLC
    // function-block wrappers are built from this surface.
    //
    // A reading that has never been taken returns GSM_UNKNOWN_DBM rather than
    // zero - zero is a real and excellent signal level, and reporting "not
    // asked yet" as excellent is how an alarm never fires.
    const char* imei() const { return _modem.identity().imei; }
    const char* iccid() const { return _modem.identity().iccid; }
    const char* imsi() const { return _modem.identity().imsi; }
    const char* firmware() const { return _modem.identity().revision; }

    int rssiDbm() const {
        return _modem.signal().hasRssi ? _modem.signal().rssiDbm : GSM_UNKNOWN_DBM;
    }
    int rsrpDbm() const {
        return _modem.signal().hasRsrp ? _modem.signal().rsrpDbm : GSM_UNKNOWN_DBM;
    }
    int rsrqDb() const {
        return _modem.signal().hasRsrq ? _modem.signal().rsrqDb : GSM_UNKNOWN_DBM;
    }
    GsmQuality signalQuality() const { return _modem.signal().quality; }

    // The module's own words for the last failure, or the library's where the
    // failure was ours. Never a bare number.
    const char* lastErrorText() const { return _modem.lastError().describe(); }

    const GsmModem::Counters& counters() const { return _modem.counters(); }

    // --- SMS, beyond sending ------------------------------------------
    //
    // The SMS layer is ATTACHED by the first call that uses it - sms(),
    // onSms(), sendSms(), readSms(), deleteSms() - so a program that never
    // sends or reads a message links none of the SMS code. Until then an
    // incoming message is left on the SIM, unread.
    GsmResult readSms(uint16_t index) { return smsPart().fetch(index); }
    GsmResult deleteSms(uint16_t index) { return smsPart().remove(index); }
    uint32_t smsReceived() const { return _sms ? _sms->received() : 0; }
    uint32_t smsSent() const { return _sms ? _sms->sent() : 0; }

    // Refreshes signal, registration and system information. Superseded, so
    // calling it every scan costs one query rather than a backlog.
    void refreshStatus() { _modem.refreshStatus(); }

    // --- What poll() costs, and the knob that bounds it --------------------
    //
    // The library promises BOUNDED WORK per poll(), not a wall-clock figure.
    // A microsecond target cannot be a contract across five architectures -
    // the same work is 1153 us on an ESP32-S3 at 240 MHz and several times
    // that on a 48 MHz part - and it goes red when poll() is legitimately busy
    // moving a large socket read.
    //
    // So: at most maxBytesPerPoll() bytes off the link per call, at most one
    // command submitted, and no loop bounded by anything the modem chose.
    // Measure it once on your board and budget for it; if that is still too
    // much for your scan cycle, turn the budget down and call poll() more
    // often. Same throughput, smaller bites.
    void   setMaxBytesPerPoll(size_t bytes) { _modem.engine().setMaxBytesPerPoll(bytes); }
    size_t maxBytesPerPoll() { return _modem.engine().maxBytesPerPoll(); }

    // How often this application is actually calling poll(), smoothed. Every
    // timeout in the library is floored by it, so a caller polling slower than
    // a timeout does not manufacture timeouts out of its own scheduling.
    gsm_time_t serviceIntervalUs() const { return _modem.serviceIntervalUs(); }
    gsm_time_t serviceIntervalMs() const { return _modem.serviceIntervalMs(); }

    // --- Sockets ---

    GsmSocket* openTcp(const char* host, uint16_t port,
                       GsmSecurity security = GsmSecurity::Plain) {
        return _sockets.open(host, port, GsmSocketKind::Tcp, security);
    }
    GsmSocket* openUdp(uint16_t localPort) {
        return _sockets.open("", localPort, GsmSocketKind::Udp, GsmSecurity::Plain);
    }
    void close(GsmSocket* socket) { _sockets.close(socket); }

    uint8_t socketsOpen() const { return _sockets.openCount(); }
    uint8_t socketsAvailable() const { return _sockets.available(); }

    // --- SMS ---
    //
    // Configured on demand rather than during bring-up: a device that never
    // sends a message should not spend its first seconds setting up for one.

    GsmResult sendSms(const char* number, const char* text,
                      GsmDoneFn done = nullptr, void* context = nullptr);

    void onSms(GsmSmsHandler handler, void* context) {
        smsPart().onMessage(handler, context);
        smsPart().configure();
    }

    // --- Location ---

    // --- Network time -------------------------------------------------
    //
    // The network sets the module's clock at registration, so this needs no
    // NTP, no socket and no bearer. Asked for, never polled: refreshTime()
    // queues the query, networkTime() reports the last answer.
    GsmResult refreshTime() { return _clock.refresh(); }
    bool hasNetworkTime() const { return _clock.hasTime(); }
    const GsmParse::GsmDateTime& networkTime() const { return _clock.time(); }
    gsm_time_t networkTimeAgeMillis() const { return _clock.ageMillis(); }
    GsmClock& clock() { return _clock; }

    GsmResult gnss(bool on) { return _gnss.power(on); }
    GsmResult refreshFix() { return _gnss.refresh(); }
    bool hasFix() const { return _gnss.hasFix(); }

    // Whether THIS module has a receiver, learned from what it answered
    // rather than assumed from its family. False on a part whose datasheet
    // offers network location instead of satellites.
    bool hasGnssReceiver() const { return _gnss.isSupported(); }
    const GsmFix& fix() const { return _gnss.fix(); }

    // How long ago the fix was last confirmed. The number an alarm should be
    // built on - a receiver that lost the sky keeps reporting its last
    // position perfectly happily.
    gsm_time_t fixAgeMillis() const { return _gnss.ageMillis(); }

    // --- Nothing is a wall ---

    GsmModem& modem() { return _modem; }
    GsmSockets& sockets() { return _sockets; }
    GsmSms& sms() { return smsPart(); }
    GsmGnss& gnssLayer() { return _gnss; }
    GsmAtEngine& engine() { return _modem.engine(); }
    GsmDriver* driver() const { return _modem.driver(); }

    // The profile: what THIS module can actually do, rather than what this
    // library assumes. Null before one has been identified.
    const GsmProfile* profile() const {
        return _modem.driver() != nullptr ? &_modem.driver()->profile() : nullptr;
    }

    // --- The escape hatch, supported and documented ---
    //
    // There is always one more vendor command. A library that forced a fork to
    // send one it did not anticipate would be forked, and the fork is what
    // would run in the field. Queued, timed and concluded like anything else.
    GsmResult sendAt(const char* command, uint32_t timeoutMs,
                     GsmReplyFn onReply, GsmDoneFn onDone, void* context);

private:
    static void onModemEvent(void* context, GsmUrcKind kind, const GsmUrcInfo& info);

    GsmDriverRegistry _registry;
    GsmDriverA76XX    _a76xx;
    GsmDriverSIM707x  _sim707x;

    GsmModem   _modem;
    GsmSockets _sockets;
    // The SMS layer's room, constructed by smsPart() on first use; events reach
    // it through _smsEvent, so the always-linked event path names no SMS code.
    // _smsPoll is the same arrangement for poll(): keeping the module
    // configured for SMS after it restarts, without linking SMS into a
    // program that never uses it.
    alignas(GsmSms) unsigned char _smsStorage[sizeof(GsmSms)];
    GsmSms*    _sms = nullptr;
    void (*_smsEvent)(GsmSms* sms, GsmUrcKind kind, const GsmUrcInfo& info) = nullptr;
    void (*_smsPoll)(GsmSms* sms, const GsmModem& modem) = nullptr;
    GsmSms& smsPart();
    GsmGnss    _gnss;
    GsmClock   _clock;

    GsmPowerControl* _power = nullptr;
    GsmApn _apn;
    const char* _beginError = nullptr;

    // GSM_APN_MAX, GSM_APN_USER_MAX, GSM_APN_PASSWORD_MAX (GsmTypes.h).
    char _apnName[GSM_APN_MAX + 1] = {0};
    char _apnUser[GSM_APN_USER_MAX + 1] = {0};
    char _apnPassword[GSM_APN_PASSWORD_MAX + 1] = {0};
};
