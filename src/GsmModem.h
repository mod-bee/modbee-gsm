// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#pragma once

// Bring-up, as one state machine owned by the core and identical for every
// module. The driver says what each step SENDS; this decides when it runs,
// what happens when it fails, and how long it may take.
//
//   Off -> PoweringUp -> Syncing -> Identifying -> Configuring -> SimWait
//       -> Registering -> BearerUp -> Ready
//
// EVERY STATE HAS A DEADLINE AND EVERY DEADLINE HAS AN EXIT. A state without
// one is a hang, and it happens in a basement rather than on a bench.
//
// READY IS RE-ENTRANT. Losing registration drops back to Registering without
// tearing the module down; reconnection is the normal path run again.
//
// THE MODULE IS NOT ASSUMED TO ANNOUNCE ITSELF. One family documents no boot
// banner; the other documents one and says it is suppressed under
// auto-bauding. Syncing polls a bare AT until it answers.
//
// POWER IS A SEQUENCE, NOT A PIN. See GsmPowerControl.h.
#include "GsmAtEngine.h"
#include "GsmUrcRouter.h"
#include "GsmDriver.h"
#include "GsmDriverRegistry.h"
#include "GsmDiagnostics.h"
#include "GsmBearer.h"
#include "GsmNetwork.h"
#include "GsmPowerControl.h"

enum class GsmState : uint8_t {
    Off = 0,
    PoweringUp,     // the application's hook has run; waiting for the AT port
    Syncing,        // polling a bare command until the module answers
    Identifying,    // asking what it is, so a driver can be chosen
    Configuring,    // the driver's own bring-up steps
    SimWait,        // waiting for the SIM to become usable
    ReadingIdentity,// IMEI and ICCID, once and only once
    NetworkSetup,   // APN, operator selection and attach - all need the SIM
    Registering,    // waiting for the network to accept us
    BearerUp,       // bringing the data context up
    Ready,          // sockets may be opened
    Recovering,     // the ladder: resync, restart, reset, power cycle
    Failed,         // exhausted; retried on a backoff
};

const char* gsm_stateName(GsmState state);

// Told when something happens that an application may want to act on.
typedef void (*GsmEventFn)(void* context, GsmUrcKind kind, const GsmUrcInfo& info);

class GsmModem {
public:
    GsmModem();

    // --- Setup ---

    // The transport, the registry and the power hooks are all the caller's and
    // must outlive this object. Power hooks are optional: with none, the
    // module is assumed to be already on, which is correct for a USB modem and
    // for a board that hard-wires the module live.
    bool begin(GsmTransport* transport, GsmDriverRegistry* registry,
               GsmPowerControl* power = nullptr);

    // Concludes everything, runs the orderly power-down sequence, releases the
    // link. Safe to call twice.
    void end();

    void setApn(const GsmApn& apn) { _bearer.apn(apn); }
    const GsmApn& apn() const { return _bearer.apn(); }

    // Setting a PIN - any PIN, even the same text again - clears the memory of
    // a rejected one, so a corrected PIN is offered on the next bring-up.
    void setPin(const char* pin) { _pin = pin; _pinRejected = false; }

    // The SIM refused the PIN it was given. It is never offered again until
    // setPin() is called: three wrong entries lock the card behind a PUK,
    // and a PUK needs somebody on site.
    bool pinRejected() const { return _pinRejected; }

    // Told about registration changes, bearer loss, socket closure, SMS
    // arrival, supply warnings. The socket layer subscribes to this; so may an
    // application.
    void onEvent(GsmEventFn handler, void* context) {
        _eventHandler = handler;
        _eventContext = context;
    }

    // --- Driving ---

    void poll();

    // Starts the sequence. Returns immediately; isReady() says when it
    // arrived. Calling it while already up is harmless.
    void start();

    // Stands the module down without destroying it.
    void stop();

    // --- State ---

    // Which radio to search for. Takes effect at the next bring-up, because
    // it is applied during configuration - changing it on a registered module
    // would mean tearing the registration down to act on it.
    void radioPreference(GsmRadioPreference wanted) { _network.preference(wanted); }

    // Pin the module to one operator by its numeric MCC+MNC, or nullptr to
    // let it choose. Applied during configuration, so it takes effect at the
    // next bring-up.
    //
    // The string is BORROWED and must outlive the device, like the APN.
    void operatorCode(const char* numericCode) { _network.operatorCode(numericCode); }

    // Narrow the band search. Both lists are BORROWED and must outlive the
    // device. Leave them unset to keep the module's own configuration, which
    // on a fresh part means every band it can do.
    void bands(const GsmBandList& catM, const GsmBandList& nbIot) {
        _network.bands(catM, nbIot);
    }

    // Forget every network that has ever refused this SIM.
    //
    // Queued like any other command. See GsmDriver's notes on EF_FPLMN: the
    // list is on the CARD, so nothing short of this clears it, and a device
    // that filled it while its SIM was inactive will otherwise never connect
    // again however long it is left.
    GsmResult clearForbiddenNetworks();

    // Whether the last check found any. Unknown until one has been done.
    GsmTristate hasForbiddenNetworks() const { return _network.hasForbiddenNetworks(); }
    const char* operatorCode() const { return _network.operatorCode(); }
    GsmRadioPreference radioPreference() const { return _network.preference(); }

    GsmState state() const { return _state; }
    bool isReady() const { return _state == GsmState::Ready; }
    bool isRegistered() const { return _network.isRegistered(); }
    bool hasBearer() const { return _bearer.isUp(); }
    // Which bearer is up - see GsmBearer::markUp(). Changes with every new one.
    uint32_t bearerEpoch() const { return _bearer.epoch(); }

    // Which life of the module this is. Moves on with every bring-up, every
    // recovery and every restart the module announces by itself - each of
    // which may have thrown away settings that a layer above put there. A
    // layer that configured the module remembers the epoch it did so in, and
    // treats a different one as "not configured".
    uint32_t moduleEpoch() const { return _moduleEpoch; }

    GsmDriver* driver() const { return _driver; }
    GsmAtEngine& engine() { return _engine; }
    GsmUrcRouter& urcRouter() { return _router; }

    const GsmIdentity& identity() const { return _identity; }
    const GsmSignal& signal() const { return _diagnostics.signal(); }

    // Signal, quality bands and radio read-back. None of it decides anything
    // here - it is what a diagnostic page shows.
    GsmDiagnostics& diagnostics() { return _diagnostics; }

    // Registration, operator selection and the escalation ladder.
    GsmNetwork& network() { return _network; }
    const GsmDiagnostics& diagnostics() const { return _diagnostics; }

    // What the radio reported about itself. See GsmRadioConfig.
    const GsmRadioConfig& radioConfig() const { return _diagnostics.radio(); }
    GsmRegistration registration() const { return _network.registration(); }
    GsmTechnology technology() const { return _network.technology(); }
    const char* operatorName() const { return _operatorName; }
    const char* ipAddress() const { return _bearer.address(); }

    GsmResult lastError() const { return _lastError; }

    // Counters an operator can act on: how many times we gave up and
    // restarted, and how often the bearer went.
    struct Counters {
        uint32_t bringUps = 0;
        uint32_t recoveries = 0;
        uint32_t bearerLosses = 0;
        uint32_t registrationLosses = 0;
        uint32_t supplyWarnings = 0;

        // How many times the MODULE restarted on its own, announced by its
        // boot banner while the library was already past bring-up. Not the
        // same as a recovery, which is the library restarting the module
        // deliberately - this one is the module deciding by itself, and a
        // count that climbs is a supply or a firmware problem rather than a
        // coverage one.
        uint32_t moduleRestarts = 0;

        // Status queries (signal, address, operator) the module refused.
        // They change nothing, so they are counted rather than escalated -
        // but a rising count means the readings on a diagnostic page are
        // older than they look.
        uint32_t statusRefusals = 0;

        // Bring-up steps that failed and were given another attempt. A rising
        // count is a module that needs coaxing rather than one that is broken.
        uint32_t stepRetries = 0;

        // Times the network explicitly refused to register us. Kept apart
        // from registrationLosses, which counts a registration we HAD and
        // then lost - a different fault with a different owner.
        uint32_t registrationDenials = 0;

        // Times the library fell back to scanning and trying each operator.
        uint32_t operatorScans = 0;

        // How many times the SIM's forbidden-network list was emptied because
        // registration was not happening. Climbing on every bring-up means
        // something keeps refusing this SIM.
        uint32_t forbiddenClears = 0;
    };
    const Counters& counters() const { return _counters; }

    // The interval the application actually calls poll() at, smoothed.
    //
    // THIS IS THE FLOOR UNDER EVERY TIMEOUT IN THE LIBRARY. A timeout shorter
    // than the gap between two looks at the modem cannot be enforced however
    // fast the module answers - an application polling every 100 ms while a
    // step asks for 50 ms would see phantom timeouts for ever, and no amount
    // of correct modem behaviour would help.
    //
    // Taken from modbee-modbus (ModbusLink::serviceIntervalUs) and
    // modbee-beebus (BeeBusProtocol), both of which have had it from the
    // start. This library did not, which is a house-style deviation and was
    // a real defect: a PLC task calling poll() once per scan would have
    // produced timeouts that had nothing to do with the modem.
    gsm_time_t serviceIntervalUs() const { return _serviceUs; }
    gsm_time_t serviceIntervalMs() const { return (_serviceUs + 999) / 1000; }

    // Refreshes signal, registration and system information. Superseded if one
    // is already queued, so calling it every scan costs one query rather than
    // a backlog.
    void refreshStatus();

    // How long a state is given, in milliseconds, with this driver and this
    // configuration. Zero for Off and Ready, which wait for nothing.
    //
    // ADDED UP FROM THE STEPS, NOT CHOSEN. A state's budget used to be a round
    // number written next to it, and the steps inside it grew without it: the
    // SIM707x bearer state had 60 s for an AT+CNACT that is itself allowed
    // 60 s, five times over. The state then timed out with a command still on
    // the wire. Public so that a test can hold every driver to it.
    uint32_t stateBudgetMs(GsmState state) const;

private:
    void enter(GsmState state, uint32_t timeoutMs);

    // Arms the current state's deadline this far from now - zero meaning
    // "due now", which is why "no deadline" is a flag and not a value.
    void setDeadline(uint32_t fromNowMs);

    // Starts a bring-up from Off or Failed. start() is the application's
    // door to it; the Failed state's retry uses the same one.
    void beginBringUp();

    // Forgets everything a bring-up learned about the module, the SIM and
    // the network. See the definition for when that is, and what it cost
    // when it was not.
    void resetBringUpMemory();

    // One register's word on registration, from a reply or an unsolicited
    // report. Decides whether that reaches registration, or loses it.
    void noteRegistration(uint8_t which, GsmRegistration state,
                          GsmTechnology technology);

    uint32_t configureBudgetMs() const;
    uint32_t bearerUpBudgetMs() const;
    void stepMachine();
    void advanceSequence();
    void beginRecovery();

    // A step the driver marked REQUIRED did not succeed. The sequence it
    // belongs to cannot continue, because every later step assumed this one
    // worked - so the whole bring-up goes back to the recovery ladder rather
    // than carrying on and failing later for a reason that makes no sense.
    //
    // This exists because the first version did not have it: the failure
    // branch of onStepDone called advanceSequence(), exactly as the success
    // branch did, under a comment describing behaviour it did not implement.
    // Every step in Configuring, NetworkSetup and BearerUp was therefore
    // optional, including AT+CGDCONT, the one step deliberately marked
    // required. Nothing in the drivers' careful required/optional marking
    // reached the modem.
    void failSequence(GsmResult reason);

    // The module's own power-on-to-UART figure once known; a family-safe
    // constant before that.
    uint32_t syncTimeoutMs() const;

    void submitStep(const GsmCmdBuf& command, uint32_t timeoutMs, bool ignoreFailure,
                    const char* deferredPrefix = nullptr,
                    uint32_t deferredTimeoutMs = 0,
                    uint8_t retries = 0);

    static void onStepDone(void* context, GsmResult result);
    static void onStatusDone(void* context, GsmResult result);
    static void onReplyLine(void* context, const char* line, size_t length);
    static void onStepReply(void* context, const char* line, size_t length);
    static bool onUrc(void* context, const char* line, size_t length);
    static bool onFinalResult(void* context, const char* line, size_t length,
                              GsmResult& out);

    void handleReply(const char* line, size_t length);
    void handleUrcInfo(const GsmUrcInfo& info);

    GsmTransport*      _transport = nullptr;
    GsmDriverRegistry* _registry = nullptr;
    GsmDriver*         _driver = nullptr;
    GsmPowerControl*   _power = nullptr;
    GsmNoPowerControl  _noPower;

    GsmAtEngine  _engine;
    GsmUrcRouter _router;

    // --- The power-key pulse, run across poll() calls ---
    //
    // A pulse is "assert, wait, release", and the wait is up to a second - so
    // it cannot happen inside one call without blocking. These carry it across
    // polls. See GsmPowerControl for why the library times this and the board
    // does not.
    enum class PowerPhase : uint8_t {
        Idle = 0,
        SupplySettling,   // supply applied; waiting before the key may be used
        KeyAsserted,      // key held; waiting out the pulse width
        Done,             // key released; the boot wait belongs to Syncing
    };
    PowerPhase _powerPhase = PowerPhase::Idle;
    gsm_time_t _powerPhaseUntil = 0;
    gsm_time_t _powerKeyHeldSince = 0;
    bool       _powerKeyHeld = false;
    bool       _hasPowerKey = false;   // the board answered setPowerKey()

    // Whether the power key has been pulsed for THIS bring-up. The library
    // probes before it pulses - see start() - so this is what stops it
    // probing forever on a module that really is off.
    bool       _hasPulsedPowerKey = false;

    // Restarts seen during THIS bring-up, so a configuration step that resets
    // the module is reported rather than retried forever.
    uint32_t   _restartsThisBringUp = 0;

    // Pulse widths for the CURRENT step. Taken from the profile once the
    // module is identified, and from GsmBootstrap before that - because the
    // first pulse necessarily happens before anything is known.
    uint32_t powerOnPulseMs() const;
    uint32_t supplySettleMs() const;

    // Drives the phases above. Returns true while a pulse is still in
    // progress, so the caller knows to wait rather than to proceed.
    bool servicePowerKey();
    void beginPowerKeyPulse();

    // --- The reset-line pulse, the same way ---
    //
    // Asserted by the recovery ladder for the profile's resetPulseMs and
    // released by serviceResetLine() on the poll that finds it due - never
    // waited out inside one call. Never held together with the power key:
    // [A7670 HW 3.2] "It is forbidden to pull down both RESET key and PWRKEY
    // to power on the module at the same time."
    bool       _resetHeld = false;
    gsm_time_t _resetUntil = 0;
    void serviceResetLine();
    void releaseResetLine();

    uint8_t    _operatorAttempts = 0;
    gsm_time_t _registeringSince = 0;

    // How many of Configuring's steps belonged to the driver's own init, so
    // the radio steps that follow can be numbered from zero.
    // Configuration runs as five sequences back to back, each numbered from
    // its own base. Explicit rather than arithmetic - see the Configuring
    // state for what the arithmetic version cost.
    enum class ConfigurePhase : uint8_t {
        DriverInit = 0,
        Radio,
        Bands,
        RadioCycle,

        // What the module says its radio is NOW set to, asked after we have
        // finished setting it. Reported, never acted on: the value of this is
        // that a diagnostic page can show it, and that "the bands were
        // narrowed to the wrong operator's by a previous session" becomes
        // something you can see rather than something you deduce.
        RadioReadback,
        Done,
    };
    ConfigurePhase _configurePhase = ConfigurePhase::DriverInit;
    uint8_t    _phaseBase = 0;

    // Which step of NetworkSetup is next. Separate from _sequenceIndex
    // because that is reset on entering the state.
    uint8_t    _networkStep = 0;
    bool       _radioChanged = false;

    // Whether Configuring has run. SimWait is entered before it and again
    // after, if the radio was cycled.
    bool       _configureDone = false;

    // Counts registration polls so signal can be sampled among them. See
    // the Registering state: a long search with no signal at all is a
    // different fault from a long search with plenty.
    uint32_t   _registerPolls = 0;

    // Which registration register the next poll asks. Rotates.

    // Explicit <stat> 3 refusals seen this bring-up. A denial is a decision
    // and is concluded rather than polled through - see Registering.

    // The operator-scan escalation: what a scan found, how far through the
    // list we are, and whether the scan has been run at all this bring-up.

    // Next moment a network-paced question may be asked again. See
    // kNetworkPollIntervalMs.
    gsm_time_t _networkPollAt = 0;

    gsm_time_t _serviceUs = 0;
    gsm_time_t _lastPollUs = 0;

    GsmState   _state = GsmState::Off;
    gsm_time_t _deadline = 0;

    // Whether _deadline means anything. A FLAG, NOT A SENTINEL VALUE: zero
    // used to mean "no deadline", and once in every 2^32 milliseconds a state
    // entered at the wrong moment computed a deadline of exactly zero, had
    // none, and waited for ever.
    bool       _hasDeadline = false;
    uint8_t    _sequenceIndex = 0;
    bool       _stepOutstanding = false;

    // WHICH STATE A STEP BELONGS TO. _stateGeneration moves on every time a
    // state is entered; _stepGeneration is what it was when the step now in
    // flight was sent. A conclusion whose generation is not the current one
    // belongs to a state that has already been left, and is dropped.
    //
    // Without this a state change abandoned nothing but its own bookkeeping.
    // The step was still in the engine, the new state queued its own behind
    // it, and the OLD answer, arriving first, was taken as the new one's: a
    // late AT+CNACT OK became the answer to Syncing's AT, Identifying ran one
    // step out of line, and the module was reported unsupported.
    uint16_t   _stateGeneration = 0;
    uint16_t   _stepGeneration = 0;

    uint32_t   _moduleEpoch = 0;

    // Whether the step now in flight may fail without stopping its sequence.
    // Captured at submit time because the completion callback is static and
    // the GsmStep that carried the flag is long gone by then.
    bool       _stepOptional = false;

    // Retries left for the step at _retryIndex. The step machine rebuilds the
    // same command from the driver while _sequenceIndex is unchanged, so "do
    // not advance" IS the retry - there is nothing to re-send by hand.
    uint8_t    _stepRetriesLeft = 0;
    uint8_t    _retryIndex = 0xFF;
    bool       _wantUp = false;

    // Where the recovery ladder has got to. Each rung is tried once before the
    // next, and the whole ladder restarts on a backoff rather than spinning.
    uint8_t    _recoveryRung = 0;
    gsm_time_t _retryAt = 0;
    uint32_t   _backoffMs = 1000;

    // Commands that timed out with not one byte from the module since the
    // first of them, and the engine's totals they were counted against. Ready
    // has no deadline, so this is how it notices a module that went quiet.
    uint8_t    _silentTimeouts = 0;
    uint32_t   _seenTimeouts = 0;
    uint32_t   _seenBytesIn = 0;

    const char* _pin = nullptr;

    // Whether the SIM has asked for a PIN, and whether we have already given
    // it one. Offered at most once per bring-up - see SimWait.
    bool        _needsPin = false;
    bool        _pinOffered = false;

    // ACROSS bring-ups, not per bring-up. Offering once per bring-up was not
    // enough: a wrong PIN failed SimWait, the recovery ladder started a new
    // bring-up, and that offered the same wrong PIN again - three bring-ups,
    // a few minutes, and the SIM was PUK-locked on a site nobody can reach.
    bool        _pinRejected = false;

    GsmIdentity _identity;
    GsmDiagnostics _diagnostics;
    GsmBearer      _bearer;
    GsmNetwork     _network;
    char _operatorName[32] = {0};      // long enough for an IPv6 literal

    bool _simReady = false;

    GsmResult _lastError = GsmStatus::Ok;

    GsmEventFn _eventHandler = nullptr;
    void*      _eventContext = nullptr;

    Counters _counters;
};
