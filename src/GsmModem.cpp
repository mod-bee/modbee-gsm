// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#include "GsmModem.h"
#include "GsmCmdBuf.h"
#include "drivers/GsmDriver3gpp.h"
#include "drivers/GsmBootstrap.h"
#include "GsmNoExcept.h"

namespace {

// Supersede keys for the status queries, so polling every scan costs one
// query rather than a backlog. Distinct values in this file's own namespace -
// the key space belongs to whoever submits, not to the engine.
const uint16_t kKeySignal = 0xF001;
const uint16_t kKeyRegistration = 0xF002;
const uint16_t kKeySystemInfo = 0xF003;

// How long the sync state keeps probing before giving up.
//
// It has to cover the WORST documented cold-start time, because the probe
// begins the moment power is applied rather than after a fixed wait. One
// family's hardware design gives 11.2 seconds typical from power-on until the
// UART answers; thirty leaves room for a cold module on a cold day, and a
// module that is already awake answers on the first probe and costs none of it.
const uint32_t kSyncTimeoutMs = 30000;

// How long to ASK before pressing anything.
//
// The window has to clear the boot time of a module that is already starting
// on its own, or the probe gives up while it is still coming up and presses
// the key mid-boot - which is the failure this exists to avoid.
//
//   [A7670 Hardware Design V1.00 §3.2.1, Table 9]  Ton(uart) 11.2 s typical
//   [SIM7070G-HP Hardware Design V1.02 §3.2.1, Table 12]  Ton(uart) 3-4 s
//
// 14 s covers the slower of the two with margin. A module that is genuinely
// off costs that much before its key is pressed, once, at startup - against a
// reset loop on every board whose key capacitor starts it automatically.
const uint32_t kPowerProbeMs = 14000;

// How many self-inflicted module restarts a single bring-up tolerates before
// it is called a fault. Three is enough to ride out a genuinely flaky boot and
// few enough that a command which resets the module every time is reported in
// seconds rather than spun on forever.
const uint32_t kMaxRestartsPerBringUp = 3;

// How often to ASK a question whose answer changes on the network's schedule,
// not on ours.
//
// Registration and SIM readiness are both like this: they take seconds to
// minutes, and asking faster does not make them arrive sooner. Without a
// limit these states resubmit as fast as poll() is called, which on a
// controller doing nothing else is very fast indeed.
//
// Measured on a SIM7070G that was refusing to register: 41,486 AT+CEREG?
// commands in 190 seconds - about 218 every second, to do the work of one.
// The module answered every one of them in 3 ms, so nothing looked wrong, but
// the AT channel was saturated for the whole of a registration attempt and
// every one of those commands was a wasted transmit, a wasted parse and a
// wasted conclusion.
//
// One second is far finer than the thing being measured.
const uint32_t kNetworkPollIntervalMs = 1000;

// How long to let the radio look around before naming an operator.
//
// Long enough that the module has seen something - a manual selection issued
// into silence just answers "no network service" - and short enough that a
// site which needs the preferred network is not left on the wrong one.
const uint32_t kOperatorSelectAfterMs = 8000;

// How many times to ask for the named operator before giving up on it.
// Manual selection is refused outright when that operator is not visible at
// the moment of asking, and during a cold LPWA search it often is not.
const uint8_t kOperatorSelectAttempts = 4;

// How many explicit refusals to take before believing them.
//
// One can arrive while the module is still working through networks, and the
// forbidden list on the SIM has not been cleared at that point. Three of them,
// with that clear already done, is the network meaning it.
const uint8_t kDenialsBeforeGivingUp = 3;

// The floor for any command whose manual gives a "Max Response Time".
//
// [A76XX V1.12] states one for every command in the set, and for a large
// number of the ones this library sends it is 9000 ms - AT+CPSI, AT+CICCID,
// AT+IPADDR, AT+CPOF, AT+CRESET, AT+CGNSSPWR, AT+CGNSSINFO and AT+CIPCCFG
// among them. The TCP/IP chapter goes to 120000 ms for the commands that
// touch the network.
//
// A TIMEOUT SHORTER THAN THAT IS NOT A CONSERVATIVE CHOICE, IT IS A BUG, and
// its symptom is nothing like a timeout. Abandoning a step does not abandon
// the command - the module is still working on it - so the next command goes
// out on top of one still in flight, which both manuals forbid (A76XX §1.5.2
// item 5). What comes back after that belongs to neither.
//
// Measured on an A7670G: AT+CIPCCFG at a 5000 ms step went unanswered, the
// step moved on, the module stopped responding and re-announced itself with
// *ATREADY: 1. The command lost in that window was AT+CIPRXGET=1, so the
// module stayed in its default Direct Push mode - and the symptom, hours of
// bench time later, was a socket that opened, sent, was closed politely by
// the far end and delivered zero bytes.
const uint32_t kMaxResponseMs = 10000;

// --- STATE BUDGETS, ADDED UP RATHER THAN CHOSEN ---
//
// A state's deadline used to be a round number written beside it, and the
// steps inside grew while the number did not. Identifying had 20 s for three
// questions allowed 10 s each; ReadingIdentity 15 s for three more; the
// SIM707x bearer state 60 s for an AT+CNACT allowed 60 s on each of five
// attempts. Each of those timed out WITH A COMMAND STILL ON THE WIRE, and the
// recovery that followed restarted work that was progressing normally.
//
// So every budget below is a sum of the steps it contains, and the ones that
// cannot be summed are asserted against them at compile time.

// Allowed on top of each step's own timeout. A step's clock starts when its
// command reaches the wire, not when the state queued it - it may wait behind
// a status query or a socket send - and the next step is not sent until a
// poll() after the last one concluded. A second covers both on anything that
// polls at all.
const uint32_t kStepSlackMs = 1000;

constexpr uint32_t stepWorstMs(uint32_t timeoutMs) { return timeoutMs + kStepSlackMs; }

// [3GPP TS 27.007 7.3] "Max Response Time  Write command: 120 seconds" for
// AT+COPS, with margin. Every selection - named, automatic, or a rung of the
// Registering ladder - is given this.
const uint32_t kOperatorSelectMs = 130000;

// AT+CGATT=1 in NetworkSetup. [SIM707x §6.2.1] allows 75 s; the rest is margin.
const uint32_t kAttachMs = 90000;

// SimWait's two steps. [SIM707x §3.2.11] AT+CPIN Max Response 5 s; the unlock
// writes the card and is given double.
const uint32_t kSimStatusStepMs = 5000;
const uint32_t kSimUnlockStepMs = 10000;

// Identifying: ATI, AT+CGMM, AT+CGMR. ReadingIdentity: AT+CGSN, the ICCID,
// AT+CIMI. Each at the module's Max Response Time.
const uint32_t kIdentifyingBudgetMs = 3 * stepWorstMs(kMaxResponseMs);
const uint32_t kReadingIdentityBudgetMs = 3 * stepWorstMs(kMaxResponseMs);

// NetworkSetup: read EF_FPLMN, the APN, the operator selection, the attach.
const uint32_t kNetworkSetupBudgetMs =
    stepWorstMs(kMaxResponseMs) + stepWorstMs(kMaxResponseMs) +
    stepWorstMs(kOperatorSelectMs) + stepWorstMs(kAttachMs);

// SimWait is POLLED, not sequenced: it asks until the SIM is ready or this
// runs out. It must still hold one unlock and one status question back to
// back, with the pacing between them, or a slow card that would have answered
// is called broken.
const uint32_t kSimWaitBudgetMs = 20000;
static_assert(kSimWaitBudgetMs >= stepWorstMs(kSimUnlockStepMs) +
                                  stepWorstMs(kSimStatusStepMs) + kNetworkPollIntervalMs,
              "SimWait cannot hold its own unlock and status steps");

const uint32_t kRecoveringBudgetMs = 30000;

// READY HAS NO DEADLINE, SO IT NEEDS ANOTHER WAY TO NOTICE A MODULE THAT HAS
// STOPPED ANSWERING. This many commands timing out in a row, with not one
// byte from the module in between, is that: a firmware hang, a supply that
// went and left it off, a power cycle it did not start itself after. A module
// that still talks - one command it never answers, a deferred result that
// does not come - sends bytes, and is left alone.
const uint8_t kSilentTimeoutsBeforeRecovery = 3;

// One step of a driver's sequence at its worst: its own timeout, the deferred
// answer if it has one, and every retry it is allowed.
uint32_t stepWorstMs(const GsmStep& step) {
    return (step.timeoutMs + step.deferredTimeoutMs + kStepSlackMs) *
           (uint32_t)(step.retries + 1u);
}

// No driver sequence comes near this; it bounds the walks below, which run
// inside poll() and so may not loop on anything a driver chose.
const uint8_t kMaxSequenceSteps = 64;

void copyBounded(char* destination, size_t capacity, const char* source, size_t length) {
    size_t i = 0;
    for (; i + 1 < capacity && i < length; i++) { destination[i] = source[i]; }
    destination[i] = '\0';
}

bool startsWith(const char* line, size_t length, const char* prefix) {
    for (size_t i = 0; prefix[i] != '\0'; i++) {
        if (i >= length || line[i] != prefix[i]) { return false; }
    }
    return true;
}

} // namespace

const char* gsm_stateName(GsmState state) {
    switch (state) {
        case GsmState::Off:          return "off";
        case GsmState::PoweringUp:   return "powering up";
        case GsmState::Syncing:      return "syncing";
        case GsmState::Identifying:  return "identifying";
        case GsmState::NetworkSetup: return "network setup";
        case GsmState::ReadingIdentity: return "reading identity";
        case GsmState::Configuring:  return "configuring";
        case GsmState::SimWait:      return "waiting for SIM";
        case GsmState::Registering:  return "registering";
        case GsmState::BearerUp:     return "bringing up bearer";
        case GsmState::Ready:        return "ready";
        case GsmState::Recovering:   return "recovering";
        case GsmState::Failed:       return "failed";
    }
    return "unknown";
}

GsmModem::GsmModem() {
    _engine.setUrcRouter(&_router);
}

bool GsmModem::begin(GsmTransport* transport, GsmDriverRegistry* registry,
                     GsmPowerControl* power) {
    end();

    _transport = transport;
    _registry = registry;
    _power = (power != nullptr) ? power : &_noPower;

    if (_transport == nullptr || _registry == nullptr) { return false; }
    if (!_engine.begin(_transport)) { return false; }

    _engine.setFinalResultHook(&GsmModem::onFinalResult, this);
    _state = GsmState::Off;
    _stepOutstanding = false;

    // The engine's totals start again with its begin().
    _silentTimeouts = 0;
    _seenTimeouts = _engine.counters().timeouts;
    _seenBytesIn = _engine.counters().bytesReceived;

    // THE ONLY PLACE THE BACKOFF STARTS AGAIN, apart from reaching Ready. An
    // explicit begin() is the application deciding to try afresh; a retry
    // from Failed is the library deciding, and that is exactly when the wait
    // has to keep growing.
    _backoffMs = 1000;
    return true;
}

void GsmModem::end() {
    if (_transport == nullptr) { return; }

    // Off FIRST, before anything below gives the engine a chance to conclude
    // what it holds. A step finishing during the teardown waits - or swept up
    // by _engine.end() - then belongs to a state already left, and is dropped
    // rather than acted on: a refused step here used to start the recovery
    // ladder on a modem that was being switched off.
    const bool wasRunning = (_state != GsmState::Off);
    enter(GsmState::Off, 0);

    // The orderly sequence, not a pin drop. See GsmPowerControl.h: a module's
    // hardware design document warns that cutting the supply while it runs can
    // corrupt its flash, so the command goes first and the application's hook
    // only afterwards.
    //
    // THE WAITS BELOW ARE THE ONLY ONES IN THE LIBRARY, and each is bounded by
    // a deadline. They still hand the processor back on every turn: end() can
    // spend several seconds here, and on an ESP32 that is long enough for the
    // task watchdog to decide the loop task has hung and reset the board -
    // in the middle of the orderly shutdown that exists to protect the
    // module's flash.
    if (_driver != nullptr && wasRunning) {
        // THE BEARER COMES DOWN BEFORE THE MODULE DOES. Leaving a context
        // active on a module that is about to lose power leaves the network
        // holding a session it will not see closed, and the next attach may be
        // refused until it ages out.
        //
        // Bounded and best-effort: the module may already be gone.
        if (_bearer.isUp()) {
            for (uint8_t index = 0; index < 8; index++) {
                GsmCmdBuf teardown;
                const GsmStep step = _driver->bearerDownStep(index, _bearer.apn(), teardown);
                if (!step.present || teardown.isEmpty() || teardown.overflowed()) {
                    break;
                }
                GsmCommand request;
                request.text = teardown.text();
                request.timeoutMs = step.timeoutMs;
                _engine.submit(request);

                const gsm_time_t until = (gsm_time_t)(gsm_millis() + 500);
                while (gsm_isFuture(gsm_millis(), until) && _engine.isBusy()) {
                    _engine.poll();
                    gsm_yield();
                }
            }
            _bearer.markDown();
        }

        GsmCmdBuf command;
        _driver->powerDownCommand(command);
        if (!command.isEmpty() && !command.overflowed()) {
            GsmCommand powerDown;
            powerDown.text = command.text();
            // [A76XX §3.2.5 AT+CPOF] Max Response Time 9000 ms.
            powerDown.timeoutMs = kMaxResponseMs;
            _engine.submit(powerDown);

            // Given a bounded chance to reach the modem. This is a shutdown
            // path, so it may spend a little time - but it is bounded by a
            // deadline rather than by hope, because a module that has already
            // gone will never answer.
            // [A7670 HW 3.2.2] Toff(status) and [SIM7070G-HP HW 3.2.2]
            // differ, so the module's own figure decides - bounded, because a
            // module that has already gone will never answer.
            const uint32_t settle = _driver->profile().powerOffSettleMs;
            const gsm_time_t until =
                (gsm_time_t)(gsm_millis() + (settle != 0 ? settle : 3000));
            while (gsm_isFuture(gsm_millis(), until) && _engine.isBusy()) {
                _engine.poll();
                gsm_yield();
            }
        }
    }

    // A power-off pulse still running from the recovery ladder is ended
    // here, on time or not: nothing will poll this object again to end it,
    // and a key left held is a reset loop on one family. A start pulse not
    // yet pressed is simply not pressed.
    if (_powerKeyHeld && _power != nullptr) { _power->setPowerKey(false); }
    _powerKeyHeld = false;
    _powerPhase = PowerPhase::Idle;
    releaseResetLine();

    if (_power != nullptr) { _power->powerOff(); }

    _engine.end();
    _router.clear();
    _transport = nullptr;
    _driver = nullptr;
    _state = GsmState::Off;
    _stepOutstanding = false;
    _bearer.markDown();
    _wantUp = false;
}

void GsmModem::start() {
    _wantUp = true;
    if (_state == GsmState::Off || _state == GsmState::Failed) {
        beginBringUp();
    }
}

// --- What a bring-up must not inherit ---
//
// EVERYTHING THE LAST ATTEMPT LEARNED IS FORGOTTEN HERE, and it is called from
// the three places where the module stops being the one we learned about: a
// new bring-up, the recovery ladder, and the module announcing that it
// restarted by itself.
//
// Each piece of this used to survive one of those, and each did damage:
//
//   - REGISTRATION was remembered across a module restart, so a module that
//     rebooted and came back unregistered skipped straight past Registering
//     into bringing a bearer up on no network.
//   - "CONFIGURED" and "NETWORK SET UP" were remembered too, so the restarted
//     module - echo back on, socket mode back to its default, no APN - was
//     never configured again. That is the very failure the restart handler
//     says it exists to prevent.
//   - THE LADDER was climbed once per process. A bring-up that ended in a
//     refusal left the forbidden-list clear and the scan marked done, and
//     every later bring-up gave up on its first registration window.
//   - A CONFIGURATION PHASE cut short by a failed required step was resumed
//     after recovery rather than restarted, and the failed step was skipped.
//
// Each call starts a new MODULE EPOCH: whatever a layer above configured on
// the module before this - SMS indications, say - is not there any more.
//
// Identity is kept: the module did not change, and the SIM is the same card.
void GsmModem::resetBringUpMemory() {
    _configurePhase = ConfigurePhase::DriverInit;
    _phaseBase = 0;
    _networkStep = 0;
    _configureDone = false;
    _radioChanged = false;
    _simReady = false;
    _needsPin = false;
    _pinOffered = false;
    _operatorAttempts = 0;
    _registeringSince = 0;
    _registerPolls = 0;
    // "Due now", not zero: zero is only due while the clock is in the
    // lower half of its range, and after 24.8 days of uptime gsm_isFuture
    // would put it 24.8 days ahead - SimWait and Registering would never
    // ask, and every bring-up would time out until the clock wrapped.
    _networkPollAt = gsm_millis();
    _network.reset();
    _moduleEpoch++;
}

void GsmModem::beginBringUp() {
    {   // the body start() used to have, kept at its indentation
        _counters.bringUps++;
        _recoveryRung = 0;

        // NOT THE BACKOFF. It used to be reset to one second here, and the
        // Failed state retries by coming back here - so the backoff doubled
        // from one second to two on every failure, for ever, and a hundred
        // sites that lost one tower together retried in lockstep every few
        // seconds. It now resets only on reaching Ready, or on begin().

        // TALK FIRST. PULSE ONLY IF NOBODY ANSWERS.
        //
        // A power key is a toggle, so pressing it is only safe when the module
        // is off - and the library cannot know that it is. Several boards make
        // the module start on its own the moment its supply appears (both
        // modules' hardware designs note that a capacitor above 10 nF on the
        // key does exactly that, and real hats fit 100 nF), so an unconditional
        // pulse presses the button on a module that is already booting.
        //
        // Measured, not theorised. On an A7670G that sequence gave:
        //
        //     [2523] > AT           x6, one per second, all unanswered
        //     [8280] < *ATREADY: 1                 the module finishes booting
        //     [8586] > AT+CIPCCFG=...              no reply for 4999 ms
        //     [15036] < *ATREADY: 1                IT BOOTED AGAIN
        //
        // The pulse landed mid-boot, the module restarted, and configuration
        // was lost in the gap - which showed up much later as a socket that
        // opened, sent, and received nothing, because AT+CIPRXGET=1 was one of
        // the commands that went into the dark.
        //
        // So: raise the supply, then SPEND A FEW SECONDS ASKING. Only if the
        // module stays silent is it actually off and worth a press. Costs a
        // few seconds on a genuinely dead module; saves a reset loop on every
        // board that self-starts.
        _hasPulsedPowerKey = false;
        _restartsThisBringUp = 0;
        resetBringUpMemory();

        // Release the key before anything else, and learn from the answer
        // whether this board has one at all. A board that came up with its
        // line already asserted - a warm reset that left a GPIO high - would
        // otherwise be holding the button down, which one family turns into a
        // reset every twelve seconds.
        // A reset pulse from the ladder is ended first: the key may be about
        // to be pressed, and the two must never be down together.
        releaseResetLine();
        _hasPowerKey = _power->setPowerKey(false);
        _powerKeyHeld = false;
        _powerPhase = PowerPhase::Idle;

        // The supply, where the board switches one. Independent of the key:
        // a board may have a rail and no key, a key and no rail, or both.
        _power->setSupply(true);

        if (!_hasPowerKey) {
            // NO KEY. Then power is not a toggle on this board - it is a
            // switch, a relay or a USB port - and the whole-operation hook is
            // both safe to call immediately and the only way up. Marked as
            // already tried, so the probe timeout goes to recovery rather
            // than waiting for a press that can never happen.
            _power->powerOn();
            _hasPulsedPowerKey = true;
        }

        enter(GsmState::Syncing, kPowerProbeMs);
    }
}

// --- The power key, timed here because its width is a module fact ---

uint32_t GsmModem::powerOnPulseMs() const {
    // The profile once the module has identified itself; the bootstrap figure
    // before that, because the FIRST pulse necessarily happens before anything
    // is known about what is on the other end of the wire.
    if (_driver != nullptr && _driver->profile().powerKeyOnPulseMs > 0) {
        return _driver->profile().powerKeyOnPulseMs;
    }
    return GsmBootstrap::kBootstrapOnPulseMs;
}

uint32_t GsmModem::supplySettleMs() const {
    if (_driver != nullptr && _driver->profile().supplySettleMs > 0) {
        return _driver->profile().supplySettleMs;
    }
    return GsmBootstrap::kBootstrapSupplySettleMs;
}

void GsmModem::beginPowerKeyPulse() {
    _powerKeyHeld = false;
    _hasPowerKey = false;

    // Make sure the key is RELEASED before anything else. A board that came up
    // with its line already asserted - a warm reset that left a GPIO high, or
    // an application that set it itself - would otherwise be holding the key
    // down, which on at least one family means the module resets on an
    // internal timer for as long as it is held. Releasing first costs nothing
    // and turns that failure into a non-event.
    if (!_power->setPowerKey(false)) {
        // No key on this board. The whole-operation hooks are the fallback.
        _powerPhase = PowerPhase::Idle;
        return;
    }
    _hasPowerKey = true;

    // Supply first, where the board switches one. Order matters: one family
    // requires the supply to be stable for a second before the key may be
    // pulled at all, and pulling it early simply does nothing.
    _power->setSupply(true);

    _powerPhase = PowerPhase::SupplySettling;
    _powerPhaseUntil = (gsm_time_t)(gsm_millis() + supplySettleMs());
}

bool GsmModem::servicePowerKey() {
    switch (_powerPhase) {
        case PowerPhase::Idle:
        case PowerPhase::Done:
            return false;

        case PowerPhase::SupplySettling:
            if (gsm_isFuture(gsm_millis(), _powerPhaseUntil)) { return true; }
            // Settled. Press the key and start the pulse.
            _power->setPowerKey(true);
            _powerKeyHeld = true;
            _powerKeyHeldSince = gsm_millis();
            _powerPhase = PowerPhase::KeyAsserted;
            _powerPhaseUntil = (gsm_time_t)(gsm_millis() + powerOnPulseMs());
            return true;

        case PowerPhase::KeyAsserted: {
            // THE HAZARD GUARD. One family documents an internal timer that
            // RESETS the module if the key is held too long, and says in as
            // many words that the key must not be tied down. If anything ever
            // leaves this phase running - a clock that jumped, a deadline that
            // was not reset - release the key rather than sit on it. A stuck
            // line there is not an inert bug; it is a reset loop that looks
            // exactly like a dead module.
            const uint32_t held = gsm_elapsed(_powerKeyHeldSince, gsm_millis());
            const uint32_t limit =
                (_driver != nullptr && _driver->profile().powerKeyMaxHoldMs > 0)
                    ? _driver->profile().powerKeyMaxHoldMs
                    : GsmBootstrap::kBootstrapMaxHoldMs;

            if (gsm_isFuture(gsm_millis(), _powerPhaseUntil) && held < limit) {
                return true;
            }

            _power->setPowerKey(false);
            _powerKeyHeld = false;
            _powerPhase = PowerPhase::Done;

            // The boot wait is NOT held here. The module takes seconds to
            // bring its AT port up, and that wait belongs to Syncing, which
            // spends it probing rather than sleeping.
            return false;
        }
    }
    return false;
}

void GsmModem::serviceResetLine() {
    if (_resetHeld && !gsm_isFuture(gsm_millis(), _resetUntil)) { releaseResetLine(); }
}

void GsmModem::releaseResetLine() {
    if (_resetHeld && _power != nullptr) { _power->setReset(false); }
    _resetHeld = false;
}

GsmResult GsmModem::clearForbiddenNetworks() {
    if (_driver == nullptr) { return GsmStatus::NotInitialised; }
    GsmCmdBuf command;
    _driver->forbiddenNetworksClearCommand(command);
    if (command.isEmpty()) { return GsmStatus::NotSupportedByModule; }

    GsmCommand request;
    request.text = command.text();
    request.timeoutMs = kMaxResponseMs;
    // The list is being cleared, so what was on it is no longer known.
    return _engine.submit(request);
}

void GsmModem::stop() {
    _wantUp = false;
    _bearer.markDown();

    // A start pulse still waiting for its supply to settle is not pressed:
    // the key is now serviced in every state, Off included, and pressing it
    // after being told to stop would start the module we were stopping. A
    // pulse already being held is left to finish, and released on time.
    if (_powerPhase == PowerPhase::SupplySettling) { _powerPhase = PowerPhase::Idle; }
    enter(GsmState::Off, 0);
}

// How long to wait for the AT port after power, in MILLISECONDS.
//
// Before a driver exists this has to be a constant that suits every supported
// family. Afterwards the module's own hardware design figure is the right one,
// and they differ by a factor of four - [A7670 HW 3.2.1] Ton(uart) 11.2 s
// against [SIM7070G-HP HW 3.2.1 T12] 3 s typical, 4 s maximum.
uint32_t GsmModem::syncTimeoutMs() const {
    if (_driver == nullptr) { return kSyncTimeoutMs; }
    const uint32_t declared = _driver->profile().powerOnToUartMs;
    return declared != 0 ? declared : kSyncTimeoutMs;
}

void GsmModem::enter(GsmState state, uint32_t timeoutMs) {
    _state = state;
    _stateGeneration++;
    _sequenceIndex = 0;

    // _stepOutstanding IS NOT CLEARED. A step on the wire does not stop being
    // on the wire because the state that sent it has been left - the module
    // is still working on it, and its answer is still coming. Clearing the
    // flag let the new state send its first step behind it and then take the
    // old answer as its own. The flag now stays up until that step concludes,
    // the new state waits for it, and the generation above marks the
    // conclusion as a straggler to be dropped.
    _stepOptional = false;
    _stepRetriesLeft = 0;
    _retryIndex = 0xFF;

    // Zero is "this state waits for nothing" - Off and Ready - and it is the
    // ARGUMENT that says so, never the stored deadline. See _hasDeadline.
    _hasDeadline = (timeoutMs != 0);
    _deadline = (gsm_time_t)(gsm_millis() + timeoutMs);
}

void GsmModem::setDeadline(uint32_t fromNowMs) {
    _hasDeadline = true;
    _deadline = (gsm_time_t)(gsm_millis() + fromNowMs);
}

// =============================================================================
// BUDGETS
// =============================================================================

uint32_t GsmModem::configureBudgetMs() const {
    if (_driver == nullptr) { return stepWorstMs(kMaxResponseMs); }

    // Walked exactly as Configuring walks it: the driver's own init, the
    // radio technology, the bands, a radio cycle and the read-back. The cycle
    // is counted whether or not it will run - a budget may be generous, it
    // may not be short.
    GsmCmdBuf command;
    uint32_t total = 0;
    for (uint8_t i = 0; i < kMaxSequenceSteps; i++) {
        const GsmStep step = _driver->initStep(i, command);
        if (!step.present) { break; }
        total += stepWorstMs(step);
    }
    for (uint8_t i = 0; i < kMaxSequenceSteps; i++) {
        const GsmStep step = _driver->radioPreferenceStep(i, _network.preference(), command);
        if (!step.present) { break; }
        total += stepWorstMs(step);
    }
    for (uint8_t i = 0; i < kMaxSequenceSteps; i++) {
        const GsmStep step = _driver->bandConfigStep(i, _network.catMBands(),
                                                     _network.nbIotBands(), command);
        if (!step.present) { break; }
        total += stepWorstMs(step);
    }
    for (uint8_t i = 0; i < kMaxSequenceSteps; i++) {
        const GsmStep step = _driver->radioCycleStep(i, command);
        if (!step.present) { break; }
        total += stepWorstMs(step);
    }
    for (uint8_t i = 0; i < kMaxSequenceSteps; i++) {
        _driver->radioReadbackCommand(i, command);
        if (command.isEmpty()) { break; }
        total += stepWorstMs(kMaxResponseMs);
    }
    return total;
}

uint32_t GsmModem::bearerUpBudgetMs() const {
    if (_driver == nullptr) { return stepWorstMs(kMaxResponseMs); }

    // The module's own figure for bringing a bearer up is a FLOOR, not the
    // budget. It used to be both - the deadline for the whole state and the
    // timeout of one step inside it - and the state could not then hold the
    // step it was named after plus anything else.
    GsmCmdBuf command;
    uint32_t total = 0;
    for (uint8_t i = 0; i < kMaxSequenceSteps; i++) {
        const GsmStep step = _driver->bearerUpStep(i, _bearer.apn(), command);
        if (!step.present) { break; }
        total += stepWorstMs(step);
    }
    const uint32_t floor = _driver->profile().bearerUpTimeoutMs;
    return total > floor ? total : floor;
}

uint32_t GsmModem::stateBudgetMs(GsmState state) const {
    switch (state) {
        case GsmState::Syncing:         return syncTimeoutMs();
        case GsmState::Identifying:     return kIdentifyingBudgetMs;
        case GsmState::SimWait:         return kSimWaitBudgetMs;
        case GsmState::Configuring:     return configureBudgetMs();
        case GsmState::ReadingIdentity: return kReadingIdentityBudgetMs;
        case GsmState::NetworkSetup:    return kNetworkSetupBudgetMs;
        case GsmState::BearerUp:        return bearerUpBudgetMs();
        case GsmState::Recovering:      return kRecoveringBudgetMs;

        // The search itself, on the module's own figure. The ladder's rungs
        // re-arm it, each at least as long as the step the rung sends.
        case GsmState::Registering:
            return (_driver != nullptr) ? _driver->profile().registrationTimeoutMs
                                        : stepWorstMs(kMaxResponseMs);

        // Set where they are entered: PoweringUp from the pulse it is timing,
        // Failed from the backoff or the reason. Off and Ready wait for
        // nothing.
        case GsmState::PoweringUp:
        case GsmState::Failed:
        case GsmState::Off:
        case GsmState::Ready:
            return 0;
    }
    return 0;
}

// =============================================================================
// POLL
// =============================================================================

void GsmModem::poll() {
    // How often is the application actually looking at us? Smoothed the same
    // way ModbusLink does it - three parts history to one part now - and a gap
    // of more than a second is treated as a pause rather than a rate, because
    // an application that stopped calling poll() has not slowed down, it has
    // been somewhere else.
    {
        const gsm_time_t nowUs = gsm_micros();
        if (_lastPollUs != 0) {
            const gsm_time_t delta = (gsm_time_t)(nowUs - _lastPollUs);
            if (delta < 1000000u) {
                _serviceUs = (_serviceUs == 0) ? delta : (_serviceUs * 3 + delta) / 4;
            }
        }
        _lastPollUs = nowUs;
        _engine.setServiceIntervalUs(_serviceUs);
    }

    if (_transport == nullptr) { return; }

    _engine.poll();

    // THE PORT HAS GONE. A transport that can tell - a POSIX tty returning
    // EIO or hanging up, a Windows handle whose reads fail - says so through
    // isConnected(), and nothing used to listen: the modem stayed Ready, with
    // a bearer, over a handle that could carry nothing, and every command
    // timed out one at a time with no reason given.
    //
    // So the session ends here, concluded rather than left to rot: Failed,
    // not Recovering, because every rung of the ladder is a command or a
    // power cycle sent to a module we can no longer reach. Failed retries on
    // the backoff - see its deadline below, which reopens the port before it
    // tries anything else. Entered BEFORE the engine is emptied, so the step
    // that is concluded is recognisably a straggler.
    if (_state != GsmState::Off && _state != GsmState::Failed &&
        !_transport->isConnected()) {
        _lastError = GsmStatus::Unresponsive;
        if (_bearer.markDown()) { _counters.bearerLosses++; }
        enter(GsmState::Failed, _backoffMs + (gsm_millis() & 0x3FF));
        _engine.abandonAll(GsmStatus::Unresponsive);
        return;
    }

    // THE MODULE HAS STOPPED ANSWERING, OVER A PORT THAT IS STILL THERE. A
    // UART cannot tell, so the check above never fires for one - and Ready,
    // which has no deadline, stayed Ready over a silent module for ever: every
    // command timed out one by one, every socket was refused, and the ladder
    // the "modem gone" case is documented to run never ran: on a site, a link
    // down until somebody cut the power.
    //
    // So: timeouts in a row with not one byte from the module are counted, and
    // a run of them in Ready is the module gone - the ladder, from the bottom.
    // The other states have their own deadlines, which lead to the ladder too.
    // Entered BEFORE the engine is emptied, so what it concludes is a straggler.
    {
        const GsmAtEngine::Counters& seen = _engine.counters();
        if (seen.bytesReceived != _seenBytesIn) {
            _seenBytesIn = seen.bytesReceived;
            _silentTimeouts = 0;
        } else if (seen.timeouts != _seenTimeouts && _silentTimeouts < 0xFF) {
            _silentTimeouts++;
        }
        _seenTimeouts = seen.timeouts;
        if (_state != GsmState::Ready) {
            _silentTimeouts = 0;
        } else if (_silentTimeouts >= kSilentTimeoutsBeforeRecovery) {
            _silentTimeouts = 0;
            _lastError = GsmStatus::Unresponsive;
            if (_bearer.markDown()) { _counters.bearerLosses++; }
            beginRecovery();
            _engine.abandonAll(GsmStatus::Unresponsive);
            return;
        }
    }

    // A state with a deadline that has passed moves on. EVERY state that can
    // wait has one; a state without an exit is a hang, and it happens in a
    // basement rather than on a bench.
    if (_hasDeadline && !gsm_isFuture(gsm_millis(), _deadline)) {
        switch (_state) {
            case GsmState::PoweringUp:
                // The settle is over; start talking to it. Whether it answers,
                // and how long it may take to start answering, is Syncing's
                // business.
                enter(GsmState::Syncing, syncTimeoutMs());
                break;

            case GsmState::Syncing:
                // Silence so far. If the key has not been tried yet, this is
                // the moment it earns its place: the module is not answering,
                // so it is very likely off, and pressing the button can no
                // longer interrupt a boot that is not happening.
                if (!_hasPulsedPowerKey) {
                    _hasPulsedPowerKey = true;
                    beginPowerKeyPulse();
                    if (_powerPhase != PowerPhase::Idle) {
                        enter(GsmState::PoweringUp,
                              supplySettleMs() + powerOnPulseMs() + 500);
                        break;
                    }
                }
                _lastError = GsmStatus::Timeout;
                beginRecovery();
                break;

            case GsmState::Identifying:
            case GsmState::Configuring:
            case GsmState::SimWait:
            case GsmState::BearerUp:
            // THE THREE BELOW WERE MISSING, AND THAT WAS A HANG.
            //
            // They fell through to `default:`, which cleared the deadline
            // instead of acting on it - so the budget they were entered with
            // was silently discarded and the state had no exit but a URC.
            // NetworkSetup is the one that matters: it contains an AT+COPS
            // worth up to 60 s on one family and an AT+CGATT worth up to 75 s
            // on the other, and if either never concluded the library simply
            // stopped, in a basement, with no way to say why.
            case GsmState::ReadingIdentity:
            case GsmState::NetworkSetup:
            case GsmState::Recovering:
                _lastError = GsmStatus::Timeout;
                beginRecovery();
                break;

            case GsmState::Registering:
                // THE LADDER WAITS FOR WHATEVER IS ON THE WIRE. Its rungs are
                // steps like any other, and sending one on top of a
                // registration poll still in flight would put two steps in
                // the engine at once, with one flag to track them. The
                // deadline stays spent, so the rung runs on the first poll
                // after that step concludes - a few seconds at most.
                if (_stepOutstanding) { break; }

                // BEFORE BLAMING THE RADIO, EMPTY THE SIM'S GRUDGE LIST.
                //
                // Every network that has ever refused this SIM is recorded in
                // EF_FPLMN on the card, and a module in automatic selection
                // will not try any of them again - so a SIM that was once
                // refused, for any reason, can be left unable to see the very
                // networks that would now accept it. See GsmDriver.
                //
                // It is the cheapest thing on the list and the only one that
                // survives a power cycle, so it goes first. Once per bring-up:
                // if clearing it did not help, it was not the problem.
                // THE LADDER, in order of cost. GsmNetwork decides which
                // rung is next; this turns it into commands.
                //
                // Every manual selection below is TEMPORARY and always
                // released back to automatic, so this is more connecting than
                // leaving it alone - not a pin to one network.
                {
                const uint32_t window = _driver->profile().registrationTimeoutMs;
                _registeringSince = gsm_millis();

                switch (_network.nextEscalation()) {
                    case GsmNetwork::Escalation::ClearForbidden: {
                        // Every network that ever refused this SIM is on
                        // EF_FPLMN, and automatic selection will not try any
                        // of them again - so one old refusal can hide the
                        // networks that would now accept it. Cheapest rung,
                        // and the only one that survives a power cycle.
                        _counters.forbiddenClears++;
                        GsmCmdBuf clear;
                        _driver->forbiddenNetworksClearCommand(clear);
                        submitStep(clear, kMaxResponseMs, true);
                        _operatorAttempts = 0;
                        setDeadline(window > stepWorstMs(kMaxResponseMs)
                                        ? window : stepWorstMs(kMaxResponseMs));
                        break;
                    }

                    case GsmNetwork::Escalation::Scan: {
                        _counters.operatorScans++;
                        _network.operators().clear();
                        GsmCmdBuf scan;
                        _driver->operatorScanCommand(scan);
                        const uint32_t sweep = _driver->profile().operatorScanTimeoutMs;
                        submitStep(scan, sweep, true);
                        setDeadline(sweep + 10000);
                        break;
                    }

                    case GsmNetwork::Escalation::TryOperator: {
                        const char* code = _network.takeNextOperator();
                        GsmCmdBuf select;
                        _driver->operatorSelectCommand(code, select);
                        submitStep(select, kOperatorSelectMs, true);
                        // At least the selection's own allowance. One family's
                        // whole registration window is 90 s and AT+COPS may
                        // take 120 s, so the window alone would expire with
                        // the selection still being decided.
                        setDeadline(window > stepWorstMs(kOperatorSelectMs)
                                        ? window : stepWorstMs(kOperatorSelectMs));
                        break;
                    }

                    case GsmNetwork::Escalation::GiveUp: {
                        // A REFUSAL AND A TIMEOUT END DIFFERENTLY. If anything
                        // ever said no, this is a refusal however the ladder
                        // ended; if nothing ever answered at all, it is not.
                        if (!_network.everDenied() && !_network.anyOperatorsFound()) {
                            // NOTHING ON AIR AND NOBODY SAID NO. That is a
                            // radio, antenna or coverage problem, not a
                            // network one, so it is a timeout and the
                            // recovery ladder is the right answer.
                            //
                            // This used to say "falls through to the timeout
                            // path" and then break out of the case ABOVE that
                            // path, which was therefore unreachable. With the
                            // deadline already spent, this empty rung ran
                            // again on every poll: measured, five and a half
                            // simulated hours in Registering and not one
                            // recovery.
                            //
                            // Not a registration loss either. That counter is
                            // for a registration we HAD; this one never was.
                            _lastError = GsmStatus::Timeout;
                            beginRecovery();
                            break;
                        }

                        // Released to automatic first: a module left pinned to
                        // a network that will not have it is worse off than
                        // one left searching.
                        if (_network.anyOperatorsFound()) {
                            GsmCmdBuf release;
                            _driver->operatorSelectCommand(nullptr, release);
                            submitStep(release, kOperatorSelectMs, true);
                        }
                        _lastError = GsmStatus::RegistrationDenied;
                        _counters.registrationDenials++;
                        enter(GsmState::Failed, 60000);
                        break;
                    }
                }
                }
                break;

            case GsmState::Failed:
                if (!_wantUp) { break; }

                // A PORT THAT WENT IS REOPENED FIRST, once per backoff, and
                // nothing is sent until it is back. The transport only clears
                // its own "lost" on begin(), so without this an unplugged and
                // replugged USB modem stayed unreachable until the application
                // restarted the library; with it reopened every poll instead,
                // a dead port would be hammered thousands of times a second.
                if (!_transport->isConnected()) {
                    _transport->end();
                    if (!_transport->begin() || !_transport->isConnected()) {
                        _backoffMs = (_backoffMs < 30000) ? _backoffMs * 2 : 60000;
                        enter(GsmState::Failed, _backoffMs + (gsm_millis() & 0x3FF));
                        break;
                    }
                }
                _state = GsmState::Off;
                start();
                break;

            case GsmState::Off:
            case GsmState::Ready:
                // The only two states that legitimately have no deadline.
                // Off is not trying to do anything; Ready is re-entrant and
                // leaves only when the network, the application or a URC says
                // so. Both are entered with a deadline of zero, so neither
                // reaches here - the cases exist so that adding a state
                // without a deadline is a compile warning rather than a hang.
                _hasDeadline = false;
                break;
        }
    }

    stepMachine();
    _engine.poll();
}

void GsmModem::stepMachine() {
    // THE POWER KEY IS SERVICED IN EVERY STATE, before anything can return.
    //
    // It used to be serviced only in PoweringUp. The recovery ladder presses
    // it too - the power-off pulse - and then moves to Recovering and Failed,
    // where nothing released it: measured, a 1 s pulse held for six seconds,
    // until the next bring-up happened to pass through PoweringUp. One family
    // resets itself after twelve. And never behind the outstanding-step check:
    // a pulse is timed in milliseconds, and a step can take two minutes.
    const bool pulsing = servicePowerKey();
    serviceResetLine();

    if (_stepOutstanding) { return; }      // one thing at a time, always

    switch (_state) {
        case GsmState::Off:
        case GsmState::Failed:
            return;

        case GsmState::PoweringUp: {
            // A power-key pulse, if this board has a key, must finish before
            // anything else happens - and it takes up to a second, so it is
            // spread across polls rather than waited out here.
            if (pulsing) { return; }

            // Otherwise nothing to wait for: either the board cannot switch
            // power, or a STATUS line says the module is up already.
            if (!_hasDeadline || _power->isPoweredOn() == GsmTristate::Yes) {
                enter(GsmState::Syncing, syncTimeoutMs());
            }
            return;
        }

        case GsmState::Syncing: {
            // A bare attention command, repeatedly, until one is answered.
            //
            // THIS IS THE ONLY PORTABLE WAY TO KNOW THE PORT IS ALIVE. One
            // family documents no boot banner; the other documents one and
            // says it is suppressed under auto-bauding. Polling works in both
            // cases and needs no banner at all.
            GsmCmdBuf command;
            GsmBootstrap::probeCommand(command);
            submitStep(command, 1000, true);
            return;
        }

        case GsmState::Identifying: {
            GsmCmdBuf command;
            if (_sequenceIndex == 0) {
                // Asked BEFORE a driver exists, so it cannot come from one.
                // See drivers/GsmBootstrap.h: the commands sent before the
                // module has been identified live with every other AT string,
                // so that the core never learns a dialect.
                GsmBootstrap::identifyCommand(command);
                submitStep(command, kMaxResponseMs, true);
            } else if (_sequenceIndex == 1) {
                // THE MODEL, ASKED PROPERLY. ATI is free-form and one real
                // module answers it with nothing but a build number, so the
                // standardised query is what actually identifies a module.
                GsmBootstrap::modelCommand(command);
                submitStep(command, kMaxResponseMs, true);
            } else if (_sequenceIndex == 2) {
                GsmBootstrap::revisionCommand(command);
                submitStep(command, kMaxResponseMs, true);
            } else {
                _driver = _registry->select(_identity.model, _identity.revision);
                if (_driver == nullptr) {
                    // NOTHING IS GUESSED. Driving an unrecognised module with
                    // somebody else's dialect fails in ways that look like a
                    // hardware fault, so it is reported as what it is.
                    _lastError = GsmStatus::NotSupportedByModule;
                    enter(GsmState::Failed, 30000);
                    return;
                }

                // The module's own AT line limit now bounds every command.
                _engine.setAtLineLimit(_driver->profile().maxAtLineLength);

                // The driver's URCs are registered now that we know which one
                // it is. Everything above this point spoke only standard
                // commands.
                // Removed first: identification runs again after every
                // restart and recovery, and the router does not refuse a
                // duplicate - so each one used to add the whole set again
                // until the table was full and a later subscriber was
                // silently turned away.
                _router.removeAll(this);
                for (uint8_t i = 0; ; i++) {
                    const char* prefix = _driver->urcPrefix(i);
                    if (prefix == nullptr) { break; }
                    _router.add(prefix, &GsmModem::onUrc, this);
                }
                // A STATE'S BUDGET MUST COVER ITS SLOWEST STEP.
                //
                // Configuring is no longer four quick settings. It now
                // contains an operator selection, whose write command the
                // manual allows 120 s, and a packet attach, which on an LPWA
                // module legitimately takes over a minute.
                //
                // With the old 30 s this state timed out WHILE A COMMAND WAS
                // STILL IN FLIGHT - measured as "syncing 30.0s" then
                // "recovering 15.0s" while AT+CGATT=1 was progressing
                // normally and answered 66 s in. Recovery then restarted the
                // radio and threw the whole attempt away.
                //
                // The budget is the sum of what the steps are allowed, not a
                // round number: 130 s for the selection, 90 s for the attach,
                // and the rest for everything quick.
                // THE SIM FIRST, THEN THE RADIO.
                //
                // Configuring sets the radio technology and the bands and
                // then cycles AT+CFUN to apply them. Every one of those wants
                // the SIM, and AT+CFUN=1 restarts SIM initialisation - so
                // running them before +CPIN: READY produces "+CME ERROR: 13"
                // (SIM failure) on commands that were correct.
                //
                // Every reference confirms the SIM first: DFRobot is
                // AT+CPIN? then AT+CNMP then AT+CMNB; a board vendor's
                // A76xx driver waits for SIM_READY before touching network
                // mode; TinyGSM ends init()
                // with getSimStatus() and re-runs all of init() after any CFUN
                // cycle.
                enter(GsmState::SimWait, stateBudgetMs(GsmState::SimWait));
            }
            return;
        }

        case GsmState::Configuring: {
            // A PHASE MACHINE, NOT INDEX ARITHMETIC.
            //
            // Configuration is five independent sequences run back to back -
            // the driver's own init, the radio technology, the bands, a radio
            // cycle to make those take effect, and the PDP context. Each has
            // its own step numbering starting at zero.
            //
            // Phases are EXPLICIT, each indexed from its own base, rather
            // than threaded together by subtracting running totals from
            // _sequenceIndex - one unassigned total there silently skips every
            // optional phase. A phase with nothing to do falls through in one
            // poll.
            GsmCmdBuf command;
            GsmStep step;
            const uint8_t phaseIndex = (uint8_t)(_sequenceIndex - _phaseBase);

            switch (_configurePhase) {
                case ConfigurePhase::DriverInit:
                    step = _driver->initStep(phaseIndex, command);
                    break;

                case ConfigurePhase::Radio:
                    step = _driver->radioPreferenceStep(phaseIndex,
                                                        _network.preference(), command);
                    if (step.present) { _radioChanged = true; }
                    break;

                case ConfigurePhase::Bands:
                    step = _driver->bandConfigStep(phaseIndex, _network.catMBands(),
                                                   _network.nbIotBands(), command);
                    if (step.present) { _radioChanged = true; }
                    break;

                case ConfigurePhase::RadioCycle:
                    // Only when something above actually changed. Cycling a
                    // radio that is already correct costs a detach and a
                    // fresh search for nothing.
                    if (_radioChanged) {
                        step = _driver->radioCycleStep(phaseIndex, command);
                    }
                    break;

                case ConfigurePhase::RadioReadback: {
                    GsmCmdBuf query;
                    _driver->radioReadbackCommand(phaseIndex, query);
                    if (query.isEmpty()) { break; }     // phase over
                    step.present = true;
                    step.timeoutMs = kMaxResponseMs;
                    step.ignoreFailure = true;          // reporting, never gating
                    command = query;
                    break;
                }

                case ConfigurePhase::Done:
                    _configureDone = true;
                    if (_radioChanged) {
                        // The radio was cycled, so the SIM is re-initialising
                        // and has to be asked again before anything trusts it.
                        _simReady = false;
                        enter(GsmState::SimWait, stateBudgetMs(GsmState::SimWait));
                    } else {
                        enter(GsmState::ReadingIdentity,
                              stateBudgetMs(GsmState::ReadingIdentity));
                    }
                    return;
            }

            if (!step.present) {
                // This phase is finished. The next one starts its own
                // numbering here, and nothing is submitted this poll.
                _configurePhase = (ConfigurePhase)((uint8_t)_configurePhase + 1);
                _phaseBase = _sequenceIndex;
                return;
            }

            submitStep(command, step.timeoutMs, step.ignoreFailure,
                       step.deferredPrefix, step.deferredTimeoutMs, step.retries);
            return;
        }

        case GsmState::SimWait: {
            if (_simReady) {
                // Entered twice: once before Configuring, and again after it
                // if the radio was cycled - because AT+CFUN=1 restarts SIM
                // initialisation and the answer from before it is stale.
                if (!_configureDone) {
                    enter(GsmState::Configuring, stateBudgetMs(GsmState::Configuring));
                } else {
                    enter(GsmState::ReadingIdentity,
                          stateBudgetMs(GsmState::ReadingIdentity));
                }
                return;
            }
            // Paced for the same reason as registration: a SIM that is busy
            // becomes ready on its own schedule, and "+CME ERROR: 14" arriving
            // two hundred times a second is no more informative than once.
            if (gsm_isFuture(gsm_millis(), _networkPollAt)) { return; }
            _networkPollAt = (gsm_time_t)(gsm_millis() + kNetworkPollIntervalMs);

            GsmCmdBuf command;

            // THE PIN, IF THE SIM HAS ASKED FOR ONE AND WE WERE GIVEN ONE.
            //
            // setPin() used to store a string that nothing ever read, so a
            // PIN-locked SIM could never be unlocked however carefully the
            // application supplied the code. The library sat in SimWait until
            // its deadline and then went round the recovery ladder, forever.
            //
            // ONCE ONLY, and only when the SIM itself said "SIM PIN". Three
            // wrong attempts lock the card behind a PUK, so this is the one
            // place in the library where retrying is worse than giving up.
            //
            // And never a PIN the SIM has already refused, in this bring-up
            // or an earlier one - see _pinRejected.
            if (_needsPin && !_pinOffered && !_pinRejected &&
                _pin != nullptr && _pin[0] != '\0') {
                _pinOffered = true;
                _driver->simUnlockCommand(_pin, command);
                submitStep(command, kSimUnlockStepMs, true);
                return;
            }

            _driver->simStatusCommand(command);

            // "+CME ERROR: 14" - SIM busy - is NORMAL for some seconds after
            // power-on and is not a broken SIM. So a failure here retries
            // rather than escalating, until the state's own deadline runs out.
            submitStep(command, kSimStatusStepMs, true);
            return;
        }

        case GsmState::ReadingIdentity: {
            // IMEI AND ICCID, ASKED ONCE.
            //
            // Both drivers have carried imeiCommand() and iccidCommand() from
            // the beginning and NOTHING EVER CALLED THEM - so imei() and
            // iccid() returned empty strings for the life of the device, which
            // is not a missing feature but a documented one that never worked.
            //
            // Found on a bench, not by reading: asking the module directly
            // printed both values happily, while driving the same module
            // through the library printed neither. Two views of the same
            // hardware disagreeing is what made it visible.
            //
            // HERE, rather than with the other identity work in Identifying,
            // because the ICCID is on the SIM and the SIM is not readable
            // until SimWait has finished with it. The IMEI could be asked
            // earlier; keeping the pair together is worth more than the
            // second it would save.
            //
            // Neither is fatal. A module that will not give its IMEI is odd
            // but perfectly able to carry traffic, so both steps ignore
            // failure and the state moves on regardless.
            GsmCmdBuf command;
            if (_sequenceIndex == 0) {
                _driver->imeiCommand(command);
                submitStep(command, kMaxResponseMs, true);
            } else if (_sequenceIndex == 1) {
                // [A76XX §6.2.1 AT+CICCID] Max Response Time 9000 ms.
                _driver->iccidCommand(command);
                submitStep(command, kMaxResponseMs, true);
            } else if (_sequenceIndex == 2) {
                // 3GPP TS 27.007 5.6 - AT+CIMI, the subscriber identity.
                //
                // WORTH THE ONE COMMAND. The IMSI says which HOME network the
                // SIM belongs to, which on a roaming IoT SIM is the only way
                // to tell a local card from a global one - and it is the
                // number an operator asks for first. This was written in the
                // driver from the start and nothing called it, so identity()
                // has never carried it.
                GsmCmdBuf command;
                _driver->imsiCommand(command);
                submitStep(command, kMaxResponseMs, true);
            } else {
                enter(GsmState::NetworkSetup, stateBudgetMs(GsmState::NetworkSetup));
            }
            return;
        }

        case GsmState::NetworkSetup: {
            // EVERYTHING HERE NEEDS A WORKING SIM, WHICH IS WHY IT IS HERE.
            //
            // These three used to run inside Configuring, before the SIM had
            // been checked. That was survivable until a radio cycle was added
            // ahead of them - AT+CFUN=0 then AT+CFUN=1 REINITIALISES THE SIM,
            // and for a second or two afterwards every SIM-dependent command
            // fails. Measured exactly that:
            //
            //   AT+COPS=0    +CME ERROR: 13      (SIM failure)
            //   AT+CGATT=1   ERROR
            //   AT+CPIN?     +CME ERROR: 13 ... then +CPIN: READY, SMS Ready
            //
            // Both network commands were refused by a SIM that was simply not
            // back yet, and the attach never happened. TinyGSM waits for
            // "SMS Ready" after its own restart for the same reason.
            //
            // Running after SimWait and ReadingIdentity means the SIM has
            // already answered +CPIN: READY and given up its ICCID, so it is
            // demonstrably there.
            GsmCmdBuf command;

            if (_networkStep == 0) {
                // 3GPP TS 27.007 8.18 - read EF_FPLMN, the list of networks
                // that have refused this SIM. A module in automatic selection
                // will not retry any of them, so a stale entry looks exactly
                // like no coverage. Read here so it can be reported; cleared
                // in Registering only when refusals actually happen.
                _networkStep++;
                GsmCmdBuf list;
                _driver->forbiddenNetworksReadCommand(list);
                submitStep(list, kMaxResponseMs, true);
                return;
            }

            if (_networkStep == 1) {
                _networkStep++;
                // THE APN, BEFORE THE ATTACH. On LTE the attach request
                // carries it - see GsmDriver::defaultContextCommand.
                if (_bearer.apn().name != nullptr && _bearer.apn().name[0] != '\0') {
                    _driver->defaultContextCommand(_bearer.apn(), command);
                    submitStep(command, kMaxResponseMs, false);
                    return;
                }
            }

            if (_networkStep == 2) {
                _networkStep++;
                // ALWAYS ASSERTED, even for automatic - the module remembers
                // this across power cycles and would otherwise inherit
                // whatever the last session left.
                const bool named = _network.operatorCode() != nullptr &&
                                   _network.operatorCode()[0] != '\0';
                _driver->operatorSelectCommand(named ? _network.operatorCode() : nullptr,
                                               command);
                submitStep(command, kOperatorSelectMs, true);
                return;
            }

            if (_networkStep == 3) {
                _networkStep++;
                // ASK IT TO ATTACH, rather than waiting to notice that it
                // has. On this family AT+CGATT=1 is what drives the module to
                // go and find a network; every working implementation sends
                // it and waits for the OK.
                _driver->attachCommand(command);
                submitStep(command, kAttachMs, true);
                return;
            }

            enter(GsmState::Registering, stateBudgetMs(GsmState::Registering));
            return;
        }

        case GsmState::Registering: {
            if (_network.isRegistered()) {
                enter(GsmState::BearerUp, stateBudgetMs(GsmState::BearerUp));
                return;
            }

            // A DENIAL IS A DECISION, NOT A DELAY.
            //
            // [3GPP TS 27.007 7.2] <stat> 3 is the network saying no; asking
            // again gets the same no. Reporting it as a Timeout instead sends
            // whoever reads it to check the antenna, the bands and the signal.
            //
            // Not on the first one, and not before EF_FPLMN has been cleared -
            // a denial can arrive while the module is still working through
            // networks, and the forbidden list may be the reason for it.
            // Counted in handleReply(), not here: this runs on every poll(),
            // so counting here reaches three denials in a millisecond. Work
            // that belongs to an ANSWER must not sit in a clock-driven path.
            // ENOUGH REFUSALS TO BELIEVE THEM. The same ladder the
            // registration window runs, brought forward: a network that says
            // no repeatedly is not going to be talked round by waiting.
            if (_network.denials() >= kDenialsBeforeGivingUp) {
                _network.clearDenials();
                setDeadline(0);                // run the ladder on the next poll
            }

            if (_registeringSince == 0) { _registeringSince = gsm_millis(); }

            // PACED. See kNetworkPollIntervalMs - the network decides when
            // this changes, and asking it 218 times a second does not help.
            if (gsm_isFuture(gsm_millis(), _networkPollAt)) { return; }
            _networkPollAt = (gsm_time_t)(gsm_millis() + kNetworkPollIntervalMs);

            GsmCmdBuf command;

            // THE OPERATOR IS CHOSEN HERE, NOT DURING CONFIGURATION.
            //
            // Naming an operator before the module has found any network at
            // all simply fails - a real SIM7070G answers "+CME ERROR: 30",
            // no network service, because at that instant it is true. The
            // module has to be looking before it can be told where to look.
            //
            // So this waits until registration is genuinely under way, gives
            // the radio a few seconds to see what is out there, and then asks
            // once. It is not repeated: AT+COPS mode 4 already falls back to
            // automatic on its own, and re-issuing it would restart a search
            // that is making progress.
            // RETRIED, BECAUSE MANUAL SELECTION CAN BE TOO EARLY.
            //
            // AT+COPS=1 refuses with "+CME ERROR: 30", no network service,
            // when the named operator is not visible at that instant - which
            // during a cold LPWA search it often is not. One attempt is
            // therefore not enough, and giving up after one would silently
            // leave the module on automatic, wandering between networks it is
            // not entitled to.
            //
            // A few attempts, spaced by the same network pacing as everything
            // else here, and then it stands aside.
            if (_operatorAttempts <= kOperatorSelectAttempts &&
                _network.operatorCode() != nullptr && _network.operatorCode()[0] != '\0' &&
                gsm_elapsed(_registeringSince, gsm_millis()) > kOperatorSelectAfterMs) {
                _operatorAttempts++;

                // THE LAST ATTEMPT HANDS THE CHOICE BACK.
                //
                // A manual selection that fails LEAVES THE MODULE IN MANUAL
                // MODE, and a module pinned to a network it cannot have does
                // not fall back to looking for another - it stops. Measured:
                // four refusals of AT+COPS=1 left it reporting "+CEREG: 2,0",
                // not registered and NOT SEARCHING, for the rest of the run.
                // That is a worse failure than the wandering it was meant to
                // cure, because nothing is happening at all.
                //
                // So once the attempts are spent, automatic selection is
                // restored explicitly. Being on the wrong network beats being
                // on none.
                if (_operatorAttempts > kOperatorSelectAttempts) {
                    _driver->operatorSelectCommand(nullptr, command);
                    submitStep(command, kOperatorSelectMs, true);
                    return;
                }

                _driver->operatorSelectCommand(_network.operatorCode(), command);
                if (!command.isEmpty()) {
                    // [3GPP TS 27.007 7.3] "Max Response Time  Write command:
                    // 120 seconds" - the module leaves whatever it is on and
                    // attaches to a named network.
                    submitStep(command, kOperatorSelectMs, true);
                    return;
                }
            }

            // SIGNAL, EVERY FEW POLLS, WHILE WE ARE STILL LOOKING.
            //
            // Without this, "searching" is the only thing an operator ever
            // sees, and it means two completely different things: a radio
            // that can hear networks and has not been accepted by one, or a
            // radio that can hear nothing at all because its antenna is not
            // connected.
            //
            // AT+CSQ answering 99 is the module saying "not known or not
            // detectable", and a run of those during a long search is the
            // difference between a coverage problem and a wiring one. It
            // costs one command in five and it is the question anybody
            // debugging a stuck registration asks first.
            // WHICH NETWORK IS REFUSING US, AND HOW STRONG IS IT.
            //
            // "Denied" on its own is nearly useless: a roaming SIM can see
            // several networks and be entitled to only one of them, and the
            // module will cheerfully cycle through the others being refused.
            // Knowing WHICH one it is talking to turns that from a mystery
            // into a question the operator can answer.
            //
            // Alternated with the signal sample so neither costs more than
            // one command in five.
            // IS IT ATTACHED? A question nobody was asking.
            //
            // Registration and attachment are different states and the module
            // reports them separately. Watching only CEREG meant a module
            // that was attached but not reporting EPS registration looked
            // like a failure, and one that was neither looked the same.
            if (_registerPolls % 10 == 3) {
                _driver->attachQueryCommand(command);
                if (!command.isEmpty()) {
                    ++_registerPolls;
                    submitStep(command, kMaxResponseMs, true);
                    return;
                }
            }

            if (_registerPolls % 10 == 5) {
                _driver->operatorCommand(command);
                if (!command.isEmpty()) {
                    ++_registerPolls;
                    submitStep(command, kMaxResponseMs, true);
                    return;
                }
            }

            if (++_registerPolls % 5 == 0) {
                _driver->signalCommand(command);
                if (!command.isEmpty()) {
                    submitStep(command, kMaxResponseMs, true);
                    return;
                }
            }

            // ROTATED, so a module attached on a register we are not
            // currently asking is still found. Wraps when the driver runs out.
            _driver->registrationCommand(_network.nextRegisterQuery(), command);
            if (command.isEmpty()) {
                _network.wrapRegisterQuery();
                _driver->registrationCommand(_network.nextRegisterQuery(), command);
            }
            submitStep(command, kMaxResponseMs, true);
            return;
        }

        case GsmState::BearerUp: {
            GsmCmdBuf command;
            const GsmStep step = _driver->bearerUpStep(_sequenceIndex, _bearer.apn(), command);
            if (!step.present) {
                _bearer.markUp();
                _backoffMs = 1000;          // a success resets the backoff
                enter(GsmState::Ready, 0);

                // ONE STATUS READ ON ARRIVAL, so that "ready" also means
                // "there is something to show".
                //
                // Periodic refreshing stays the application's decision - it
                // is AT traffic, and a controller that only wants a socket
                // should not pay for a signal reading every second. But the
                // FIRST reading is different: an application that has just
                // been told it is ready will ask, and finding hasRssi false
                // at that exact moment reads as a fault rather than as "no
                // one has asked the modem yet".
                //
                // Cost is two queries, once per bring-up. Found on a bench,
                // where a module that had just reported -89 dBm when asked
                // directly showed nothing at all through the library.
                refreshStatus();

                // What address the network gave us, and who gave it. Asked
                // ONCE on arrival rather than polled: both change only when
                // the bearer is re-established, and this is the moment that
                // just happened. An operator diagnosing a site wants them on
                // the page without having to ask for a refresh.
                {
                    GsmCmdBuf address;
                    _driver->addressCommand(_bearer.apn(), address);
                    if (!address.isEmpty() && !address.overflowed()) {
                        GsmCommand request;
                        request.text = address.text();
                        request.timeoutMs = 10000;
                        request.onReply = &GsmModem::onReplyLine;
                        request.onDone = &GsmModem::onStatusDone;
                        request.context = this;
                        _engine.submit(request);
                    }

                    GsmCmdBuf who;
                    _driver->operatorCommand(who);
                    if (!who.isEmpty() && !who.overflowed()) {
                        GsmCommand request;
                        request.text = who.text();
                        request.timeoutMs = 10000;
                        request.onReply = &GsmModem::onReplyLine;
                        request.onDone = &GsmModem::onStatusDone;
                        request.context = this;
                        _engine.submit(request);
                    }
                }
                return;
            }
            submitStep(command, step.timeoutMs, step.ignoreFailure,
                       step.deferredPrefix, step.deferredTimeoutMs, step.retries);
            return;
        }

        case GsmState::Ready:
            return;

        case GsmState::Recovering: {
            if (gsm_isFuture(gsm_millis(), _retryAt)) { return; }

            // THE LADDER. Each rung is tried once, in increasing severity, and
            // each is bounded. The cheapest thing that might work is tried
            // first, because a module that merely lost its network does not
            // need its flash power-cycled.
            _counters.recoveries++;
            switch (_recoveryRung) {
                case 0:
                    // Just try talking to it again. A transient fault - a
                    // burst of line noise, a missed reply - needs nothing more.
                    _recoveryRung++;
                    enter(GsmState::Syncing, 15000);
                    return;

                case 1: {
                    // A software restart of the radio, which is far cheaper
                    // than restarting the module and fixes most registration
                    // faults.
                    GsmCmdBuf command;
                    GsmBootstrap::radioRestartCommand(command);
                    _recoveryRung++;
                    submitStep(command, 15000, true);
                    setDeadline(20000);
                    return;
                }

                case 2: {
                    // The module's own reset command. Its hardware design
                    // document is explicit that reset "is recommended ... only
                    // in case of emergency, such as the module is not
                    // responding" - which is exactly here, and never earlier.
                    if (_driver != nullptr) {
                        GsmCmdBuf command;
                        _driver->resetCommand(command);
                        submitStep(command, 10000, true);
                    }
                    _recoveryRung++;

            // Treset differs between families, so the module's own settling
            // figure sets the window rather than one constant suiting neither.
            {
                const uint32_t settle = (_driver != nullptr)
                                      ? _driver->profile().resetSettleMs : 3000;
                setDeadline(15000 + settle);
            }
                    return;
                }

                case 3: {
                    // The application's hooks, in the order the hardware
                    // document requires: down, settle, up.
                    //
                    // The KEY first where the board has one, because that is
                    // the module's own documented way to be shut down and the
                    // hardware designs are explicit that pulling the supply on
                    // a running module risks its flash. A rail drop is what
                    // happens when there is no key to pull.
                    const GsmProfile* profile =
                        (_driver != nullptr) ? &_driver->profile() : nullptr;
                    const uint32_t resetPulse =
                        (profile != nullptr) ? profile->resetPulseMs : 0;
                    const uint32_t resetSettle =
                        (profile != nullptr) ? profile->resetSettleMs : 3000;
                    uint32_t wait = 1000 + ((profile != nullptr) ? profile->powerOffToOnMs : 2000);

                    if (_power->setPowerKey(true)) {
                        _powerKeyHeld = true;
                        _hasPowerKey = true;
                        _powerPhase = PowerPhase::KeyAsserted;
                        _powerKeyHeldSince = gsm_millis();
                        const uint32_t offPulse =
                            (profile != nullptr && profile->powerKeyOffPulseMs > 0)
                                ? profile->powerKeyOffPulseMs
                                : GsmBootstrap::kBootstrapOnPulseMs;
                        _powerPhaseUntil = (gsm_time_t)(gsm_millis() + offPulse);
                    } else if (resetPulse != 0 && _power->setReset(true)) {
                        // NO KEY, BUT A RESET LINE - the board's line-level
                        // hook, preferred over its whole-operation one for the
                        // same reason as the key: the width is the module's,
                        // and only the library knows which module it is.
                        // Asserted now, released by serviceResetLine() when
                        // the profile's Treset is up; then the module's own
                        // settling time before anything talks to it.
                        //
                        // Only where the profile names a width. Zero is a
                        // module with no RESET pin, and pulsing a line wired
                        // for something else is not a reset.
                        _resetHeld = true;
                        _resetUntil = (gsm_time_t)(gsm_millis() + resetPulse);
                        wait = resetPulse + resetSettle;
                    } else {
                        _power->powerOff();
                        _power->reset();
                    }
                    _recoveryRung++;
                    // Toff-on, or Treset and its settling: the module must be
                    // left alone this long before the next rung. The figures
                    // are the module's.
                    _retryAt = (gsm_time_t)(gsm_millis() + wait);
                    setDeadline(wait + 10000);
                    return;
                }

                default:
                    // Exhausted. Reported on a pin and retried with a backoff
                    // that spreads - a hundred sites that lose a tower together
                    // must not re-register in lockstep and bring it down again
                    // as it comes back.
                    //
                    // Doubled HERE, once per exhausted ladder, and capped -
                    // the old "below the cap, double it" went from 32 s to
                    // 64 s, past the cap it was checking.
                    _backoffMs = (_backoffMs < 30000) ? _backoffMs * 2 : 60000;
                    enter(GsmState::Failed, _backoffMs + (gsm_millis() & 0x3FF));
                    return;
            }
        }
    }
}

void GsmModem::beginRecovery() {
    _bearer.markDown();

    // THE WHOLE SEQUENCE AGAIN, FROM ITS FIRST STEP. Every rung of the ladder
    // either restarts something - the radio, the module, its power - or
    // resynchronises with a module whose state is no longer known, and what
    // comes back has to be configured, not trusted. See resetBringUpMemory().
    //
    // It used to keep its place in Configuring instead. The step count was
    // reset under a phase that was not, the phase's own numbering wrapped
    // past every step in it, and the required step that had just failed - the
    // reason recovery was running at all - was never sent again. "Required"
    // lasted exactly one failure.
    resetBringUpMemory();
    _retryAt = gsm_millis();
    enter(GsmState::Recovering, stateBudgetMs(GsmState::Recovering));
}

// =============================================================================
// STEPS
// =============================================================================

void GsmModem::submitStep(const GsmCmdBuf& command, uint32_t timeoutMs,
                          bool ignoreFailure, const char* deferredPrefix,
                          uint32_t deferredTimeoutMs, uint8_t retries) {
    // ONE STEP AT A TIME, ALWAYS. Every caller checks first; this is the
    // backstop, because a second step sent behind the first would share one
    // flag and one generation with it, and the first conclusion would be
    // taken for both.
    if (_stepOutstanding) { return; }

    _stepOptional = ignoreFailure;

    // The retry budget belongs to the STEP, not to the attempt - so it is set
    // once when this index is first reached and left alone while it is retried.
    if (_retryIndex != _sequenceIndex) {
        _retryIndex = _sequenceIndex;
        _stepRetriesLeft = retries;
    }

    if (command.isEmpty() || command.overflowed()) {
        // A command that would not fit is refused before anything reaches the
        // modem. A truncated AT command is a DIFFERENT command, not a shorter
        // one, and its answer would be matched against the wrong expectation.
        //
        // A REQUIRED step that cannot even be built has failed, and is treated
        // as one - the alternative is carrying on as though it had been sent.
        if (!ignoreFailure) {
            failSequence(GsmStatus::TooLarge);
            return;
        }
        advanceSequence();
        return;
    }

    GsmCommand request;
    request.text = command.text();
    request.timeoutMs = timeoutMs;
    request.onDone = &GsmModem::onStepDone;
    request.onReply = &GsmModem::onStepReply;
    request.context = this;
    request.deferredPrefix = deferredPrefix;
    request.deferredTimeoutMs = deferredTimeoutMs;

    if (_engine.submit(request).status() == GsmStatus::Queued) {
        _stepOutstanding = true;
        _stepGeneration = _stateGeneration;
    } else {
        // The queue refused. Not fatal: the state's own deadline still runs,
        // and the step is retried on the next poll.
        _stepOutstanding = false;
    }
}

void GsmModem::advanceSequence() {
    _sequenceIndex++;
    _stepOutstanding = false;
}

void GsmModem::onStepDone(void* context, GsmResult result) {
    GsmModem* self = (GsmModem*)context;
    self->_stepOutstanding = false;

    // A STRAGGLER. The state that sent this step has been left - by a
    // deadline, a URC, stop() or end() - and its answer means nothing to the
    // state that is current now. Dropped, and the current state, which has
    // been waiting for the wire to clear, sends its own step on the next poll.
    if (self->_stepGeneration != self->_stateGeneration) { return; }

    // STOPPED: a step that was on the wire when stop() came concludes later -
    // answered, refused or timed out - and belongs to a sequence nobody runs
    // any more. Without this a late failure fell through to failSequence()
    // and started the recovery ladder (resync, restart, power cycle) on a
    // modem that had been switched off. Failed is the same: it has its own
    // backoff, and a straggler from before must not restart anything.
    if (self->_state == GsmState::Off || self->_state == GsmState::Failed) { return; }

    const bool succeeded = result.isOk();
    if (!succeeded) { self->_lastError = result; }

    switch (self->_state) {
        case GsmState::Syncing:
            if (succeeded) {
                self->enter(GsmState::Identifying,
                            self->stateBudgetMs(GsmState::Identifying));
            }
            // Otherwise the state deadline governs: keep asking until it
            // answers or the wait runs out.
            return;

        case GsmState::SimWait:
        case GsmState::Registering:
            // Polled states. They advance on what the REPLY said, not on the
            // command succeeding, so nothing happens here.
            return;

        case GsmState::Recovering:
            // A rung completed; the next poll tries the next one.
            return;

        default:
            break;
    }

    // A STEP THAT IS ALLOWED ANOTHER GO GETS ONE.
    //
    // Not advancing the sequence is the retry: the step machine rebuilds the
    // same command from the driver on the next poll because _sequenceIndex has
    // not moved. Both TinyGSM forks do this for AT+CNACT, five attempts each.
    if (!succeeded && self->_stepRetriesLeft > 0) {
        self->_stepRetriesLeft--;
        self->_counters.stepRetries++;
        return;                         // _stepOutstanding already cleared
    }

    if (succeeded || self->_stepOptional) {
        // Either it worked, or the driver said this one is allowed to fail.
        // AT+CEREG=2 on a module that does not offer mode 2 is the shape of
        // the second case: worth asking for, not worth stopping over.
        self->advanceSequence();
        return;
    }

    // Required, and it failed. Everything after it in this sequence assumed it
    // worked, so running them would produce a second, more confusing failure.
    self->failSequence(result);
}

void GsmModem::failSequence(GsmResult reason) {
    _lastError = reason;
    _stepOutstanding = false;

    // Back to the ladder. Its first rung is only "try talking to it again",
    // which is the right weight for a single refused command; the rungs after
    // it escalate on their own if that does not help.
    beginRecovery();
}

void GsmModem::onStatusDone(void* context, GsmResult result) {
    // STATUS QUERIES CONCLUDE TOO.
    //
    // Signal, address and operator are asked for without anything depending on
    // the answer, so they used to be submitted with a reply handler and no
    // completion handler at all - outside the one-conclusion contract the rest
    // of the library keeps. A refused AT+CSQ simply vanished.
    //
    // They still do not change the state machine. But a failure is counted, so
    // "the signal reading is stale" has somewhere to show up other than a
    // field that never changes.
    if (!result.isOk()) { ((GsmModem*)context)->_counters.statusRefusals++; }
}

void GsmModem::onReplyLine(void* context, const char* line, size_t length) {
    ((GsmModem*)context)->handleReply(line, length);
}

void GsmModem::onStepReply(void* context, const char* line, size_t length) {
    GsmModem* self = (GsmModem*)context;

    // A straggler's reply lines are dropped with its conclusion. They would be
    // read by the CURRENT state's rules - which key on _sequenceIndex - so an
    // IMEI arriving after the state moved on could be filed as an ICCID, or
    // the old model as the new revision.
    if (self->_stepGeneration != self->_stateGeneration) { return; }
    self->handleReply(line, length);
}

void GsmModem::handleReply(const char* line, size_t length) {
    switch (_state) {
        case GsmState::Identifying:
            if (_sequenceIndex == 1) {
                // AT+CGMM, and it WINS over anything ATI offered.
                //
                // 3GPP TS 27.007 5.2 specifies this reply where ATI leaves it
                // free-form, so where the two disagree this is the one to
                // believe. Captured:
                //
                //   A7670G-LABE  ATI "R..."/"Model: A7670G-LABE"
                //                CGMM "A7670G-LABE"
                //   SIM7070G     ATI "R1951.05"        <- no model at all
                //                CGMM "SIMCOM_SIM7070"
                //
                // Some modules label this line too, so it goes through the
                // same rule as ATI rather than being trusted raw.
                GsmField value;
                const GsmParse::GsmIdentityLine kind =
                    GsmParse::identityLine(line, length, value);
                if (kind != GsmParse::GsmIdentityLine::OtherLabel &&
                    value.length >= 4) {
                    copyBounded(_identity.model, sizeof _identity.model,
                                value.data, value.length);
                }
                return;
            }
            if (_sequenceIndex == 0) {
                // ATI answers with one or more free-form lines and the MODEL
                // is what a driver recognises. Modules disagree about where it
                // goes - see GsmParse::identityLine() in GsmParse.h for the
                // two shapes and for the failure that made this explicit.
                GsmField value;
                const GsmParse::GsmIdentityLine kind =
                    GsmParse::identityLine(line, length, value);

                if (kind == GsmParse::GsmIdentityLine::Model && value.length > 0) {
                    // Labelled, and therefore authoritative: it overrides
                    // anything an earlier line offered.
                    copyBounded(_identity.model, sizeof _identity.model,
                                value.data, value.length);
                } else if (kind == GsmParse::GsmIdentityLine::Unlabelled &&
                           _identity.model[0] == '\0' && length >= 4) {
                    // The first substantial unlabelled line, which is what a
                    // module answering on one line gives.
                    copyBounded(_identity.model, sizeof _identity.model, line, length);
                }
            } else if (_sequenceIndex == 2) {
                // AT+CGMR comes back three different ways across two modules:
                //
                //   "+CGMR: A110B06A7670M7"      prefixed, A7670G
                //   "Revision:1951B16SIM7070"    LABELLED, SIM7070G
                //   "1951B04SIM7080"             bare
                //
                // All three are the same fact. Stripping whichever decoration
                // is present matters because the profile is keyed on firmware
                // as well as model, and "Revision:1951B16SIM7070" would match
                // nothing a driver looks for.
                GsmField rest;
                if (GsmParse::afterPrefix(line, length, "+CGMR", rest)) {
                    copyBounded(_identity.revision, sizeof _identity.revision,
                                rest.data, rest.length);
                    return;
                }
                if (GsmParse::afterPrefix(line, length, "Revision", rest) &&
                    length > 8 && line[8] == ':') {
                    copyBounded(_identity.revision, sizeof _identity.revision,
                                rest.data, rest.length);
                    return;
                }
                if (_identity.revision[0] == '\0' && length >= 4) {
                    copyBounded(_identity.revision, sizeof _identity.revision,
                                line, length);
                }
            }
            return;

        case GsmState::ReadingIdentity: {
            if (_sequenceIndex == 0) {
                // 3GPP TS 27.007 5.4 - AT+CGSN answers with the serial number
                // on a BARE line, no prefix at all:
                //
                //     490154203237518
                //
                // [Shape captured from an A7670G, firmware A110B06A7670M7;
                // the digits are an example, not that module's IMEI]
                //
                // With no prefix to match on, the shape is the only check
                // there is - so it has to be a real one. An IMEI is 15
                // decimal digits (14 plus a Luhn check digit); IMEISV is 16.
                // Accepting anything else here would quietly store an "OK",
                // an error string or a URC that happened to arrive first.
                if (length >= 14 && length <= 17 && _identity.imei[0] == '\0') {
                    bool allDigits = true;
                    for (size_t i = 0; i < length; i++) {
                        if (line[i] < '0' || line[i] > '9') { allDigits = false; break; }
                    }
                    if (allDigits) {
                        copyBounded(_identity.imei, sizeof _identity.imei,
                                    line, length);
                    }
                }
            } else {
                // The ICCID's shape differs between families - one prefixes
                // it, one does not - so the DRIVER reads it. That is the
                // whole reason parseIccid is a driver method.
                GsmField iccid;
                if (_sequenceIndex == 2) {
                    // 3GPP TS 27.007 5.6 - AT+CIMI answers with a BARE line,
                    // no prefix: "001010123456789". An IMSI is 14 or 15
                    // digits, so the shape is the only check available and it
                    // has to be a real one - otherwise an "OK" or a URC that
                    // arrived first would be stored as a subscriber identity.
                    if (length >= 14 && length <= 16 && _identity.imsi[0] == '\0') {
                        bool allDigits = true;
                        for (size_t k = 0; k < length; k++) {
                            if (line[k] < '0' || line[k] > '9') { allDigits = false; break; }
                        }
                        if (allDigits) {
                            copyBounded(_identity.imsi, sizeof _identity.imsi,
                                        line, length);
                        }
                    }
                } else if (_driver != nullptr && _driver->parseIccid(line, length, iccid)) {
                    copyBounded(_identity.iccid, sizeof _identity.iccid,
                                iccid.data, iccid.length);
                }
            }
            return;
        }

        case GsmState::SimWait: {
            bool ready = false, needsPin = false, needsPuk = false;
            if (GsmDriver3gpp::parseSimStatus(line, length, ready, needsPin, needsPuk)) {
                _simReady = ready;
                _needsPin = needsPin;

                // Asked for a PIN and we have none to give. Said plainly
                // rather than waited out: no amount of polling produces a PIN.
                if (needsPin && (_pin == nullptr || _pin[0] == '\0')) {
                    _lastError = GsmResult::fromModule(GsmErrorSpace::Cme, 11);
                    enter(GsmState::Failed, 60000);
                    return;
                }

                // Still asking for a PIN after it was given one: the SIM
                // refused it, and counted the attempt. Remembered across
                // bring-ups, so the recovery ladder cannot spend the other
                // two attempts on the same wrong PIN. [TS 27.007 Annex A]
                // CME 16 is "incorrect password".
                if (needsPin && (_pinOffered || _pinRejected)) {
                    _pinRejected = true;
                    _lastError = GsmResult::fromModule(GsmErrorSpace::Cme, 16);
                    enter(GsmState::Failed, 60000);
                    return;
                }
                if (needsPuk) {
                    // A PUK is not something a library may attempt. Three wrong
                    // PIN entries is a SIM that needs an operator, and guessing
                    // would burn the PUK retries too.
                    _lastError = GsmResult::fromModule(GsmErrorSpace::Cme, 12);
                    enter(GsmState::Failed, 60000);
                }
            }
            return;
        }

        default:
            break;
    }

    // Status replies, whatever state we are in.
    if (_driver == nullptr) { return; }

    // WHICHEVER REGISTER ANSWERED. We rotate through CEREG, CGREG and CREG,
    // so pinning the parse to one prefix would throw away two thirds of the
    // replies we asked for. TinyGSM does the same and says why: "check for
    // any of the three for simplicity".
    //
    // IN EVERY STATE, not only Registering. A status refresh in Ready asks
    // AT+CEREG? too, and its answer used to fall through here unread - so a
    // module that the network had dropped said "not registered" to a direct
    // question and the library carried on reporting Ready. What each answer
    // MEANS depends on the state; see noteRegistration().
    //
    // Only Home and Roaming count as registered; 6 and 7 ("SMS only") are a
    // state where the network has accepted the SIM and will carry no data,
    // and on SIM707x's CGREG a 6 is not even that but a leaked internal
    // sentinel (DSAT_REG_REGISTERED_MAX, manual §6.2.5).
    {
        static const char* const kPrefixes[] = { "+CEREG", "+CGREG", "+CREG" };
        static const uint8_t kRegisters[] = {
            GsmNetwork::kRegisterEps, GsmNetwork::kRegisterPacket,
            GsmNetwork::kRegisterCircuit,
        };
        for (size_t i = 0; i < 3; i++) {
            GsmRegistration state = _network.registration();
            GsmTechnology technology = _network.technology();
            if (GsmDriver3gpp::parseRegistration(line, length, kPrefixes[i],
                                                 true, state, technology)) {
                noteRegistration(kRegisters[i], state, technology);
                return;
            }
        }
    }

    if (_driver->parseOperatorScan(line, length, _network.operators())) { return; }

    {
        bool anyForbidden = false;
        if (_driver->parseForbiddenNetworks(line, length, anyForbidden)) {
            _network.forbiddenNetworks(anyForbidden);
            return;
        }
    }

    if (startsWith(line, length, "+CSQ")) {
        GsmSignal reading = _diagnostics.signal();
        GsmDriver3gpp::parseSignalQuality(line, length, reading);
        _diagnostics.signal(reading);      // judged there, not here
        return;
    }

    // What the radio says about itself. Asked once per bring-up in the
    // RadioReadback phase; parsed here because the reply arrives like any
    // other line and the driver decides which prefixes are its own.
    if (_driver->parseRadioConfig(line, length, _diagnostics.radio())) { return; }

    if (startsWith(line, length, "+CPSI")) {
        {
            GsmSignal reading = _diagnostics.signal();
            GsmTechnology seen = _network.technology();
            _driver->parseSystemInfo(line, length, reading, seen);
            _network.technology(seen);
            _diagnostics.signal(reading);
        }
        return;
    }

    if (startsWith(line, length, "+COPS")) {
        GsmField name;
        if (GsmDriver3gpp::parseOperator(line, length, name)) {
            copyBounded(_operatorName, sizeof _operatorName, name.data, name.length);
        }
        return;
    }

    {
        GsmField address;
        if (_driver->parseAddress(line, length, address)) {
            _bearer.address(address.data, address.length);
            return;
        }
    }

    {
        GsmField iccid;
        if (_driver->parseIccid(line, length, iccid)) {
            copyBounded(_identity.iccid, sizeof _identity.iccid,
                        iccid.data, iccid.length);
            return;
        }
    }
}

// =============================================================================
// REGISTRATION
// =============================================================================

void GsmModem::noteRegistration(uint8_t which, GsmRegistration state,
                                GsmTechnology technology) {
    const bool registerWasRegistered = gsm_isRegistered(_network.reported(which));
    _network.report(which, state);

    if (_state == GsmState::Registering) {
        // A better answer never loses to a worse one inside a single
        // rotation: being told "searching" by CREG does not undo CEREG having
        // just said "home". A denial is counted inside registration(), the
        // one place that hears it - so one answer is one count, however the
        // answer arrived.
        if (gsm_isRegistered(state) || !_network.isRegistered()) {
            _network.registration(state);
            _network.technology(technology);
        }
        return;
    }

    if (_state != GsmState::Ready) { return; }

    if (gsm_isRegistered(state)) {
        _network.technology(technology);
        return;
    }

    // READY IS RE-ENTRANT. Losing the network drops back to waiting for it,
    // without tearing the module down - and this is the code that makes that
    // true. It used to sit in the URC handler, testing a registration that
    // nothing in Ready ever updated, so it could not run; a module the
    // network had dropped went on reporting Ready with a bearer until a
    // socket tried to use it and failed for no stated reason.
    //
    // A LOSS IS A CHANGE. This register must have been carrying registration
    // and must have stopped, and neither packet register may still be
    // carrying it (see GsmNetwork::packetRegistered for why CREG gets no vote
    // and why either packet register is enough). A register that never said
    // yes saying no again is not news: on an LTE-only module GPRS can report
    // "not registered" for as long as it is powered.
    if (which == GsmNetwork::kRegisterCircuit) { return; }
    if (!registerWasRegistered || _network.packetRegistered()) { return; }

    _counters.registrationLosses++;
    _bearer.markDown();

    // A fresh search, with a fresh ladder: the rungs already climbed belong
    // to the search that succeeded, not to this one.
    _network.restartLadder();
    _network.lose(state);
    _operatorAttempts = 0;
    _registeringSince = 0;
    _registerPolls = 0;
    _networkPollAt = gsm_millis();
    enter(GsmState::Registering, stateBudgetMs(GsmState::Registering));
}

// =============================================================================
// UNSOLICITED
// =============================================================================

bool GsmModem::onUrc(void* context, const char* line, size_t length) {
    GsmModem* self = (GsmModem*)context;
    if (self->_driver == nullptr) { return false; }

    GsmUrcInfo info;
    const GsmUrcKind kind = self->_driver->classifyUrc(line, length, info);

    // DECLINED. On one family the same prefix serves both a URC and a reply,
    // told apart only by a field - so an unrecognised line must fall through
    // to whatever command is in flight rather than being swallowed here.
    if (kind == GsmUrcKind::Unrecognised) { return false; }

    // WHAT THE REPORT SAID, not only that there was one. The driver says a
    // line is a registration report; the stat in it is standard, and an
    // unsolicited report carries no <n> field, so it parses one place to the
    // left of a read reply's. Before this the kind was passed on and the
    // stat thrown away, so nothing anywhere knew the registration had gone.
    if (kind == GsmUrcKind::RegistrationChanged) {
        static const char* const kPrefixes[] = { "+CEREG", "+CGREG", "+CREG" };
        static const uint8_t kRegisters[] = {
            GsmNetwork::kRegisterEps, GsmNetwork::kRegisterPacket,
            GsmNetwork::kRegisterCircuit,
        };
        for (size_t i = 0; i < 3; i++) {
            GsmRegistration state = self->_network.registration();
            GsmTechnology technology = self->_network.technology();
            if (GsmDriver3gpp::parseRegistration(line, length, kPrefixes[i],
                                                 false, state, technology)) {
                self->noteRegistration(kRegisters[i], state, technology);
                break;
            }
        }
    }

    self->handleUrcInfo(info);
    return true;
}

void GsmModem::handleUrcInfo(const GsmUrcInfo& info) {
    switch (info.kind) {
        case GsmUrcKind::BearerDown:
            // EVERY SOCKET IS DEAD, not one. Reported as its own kind so the
            // socket layer reopens the bearer rather than reopening sockets
            // over a context that is gone - which never succeeds and never
            // explains itself.
            if (_bearer.markDown()) { _counters.bearerLosses++; }
            if (_state == GsmState::Ready) {
                enter(GsmState::BearerUp, stateBudgetMs(GsmState::BearerUp));
            }
            break;

        case GsmUrcKind::BearerUp:
            _bearer.markUp();
            break;

        case GsmUrcKind::RegistrationChanged:
            // Already acted on in onUrc(), which had the line to read the stat
            // from - see noteRegistration(). Here only the event goes out.
            break;

        case GsmUrcKind::SupplyWarning:
            // The earliest warning of a marginal supply there is, and it costs
            // no extra hardware. Counted so a site with an intermittent rail
            // shows up as a number rather than as folklore.
            _counters.supplyWarnings++;
            break;

        case GsmUrcKind::PowerDown:
            _bearer.markDown();
            enter(GsmState::Off, 0);
            if (_wantUp) { start(); }
            break;

        case GsmUrcKind::Ready:
            // A boot banner. It means two completely different things
            // depending on when it turns up, and only one of them was handled
            // before.
            //
            // WHILE POWERING UP it is an accelerator: the module is up, so
            // stop waiting. Its absence changes nothing, because one family
            // never sends one and the other suppresses it under auto-bauding.
            if (_state == GsmState::PoweringUp || _state == GsmState::Syncing) {
                enter(GsmState::Syncing, syncTimeoutMs());
                break;
            }

            // ANY LATER, IT MEANS THE MODULE RESTARTED UNDERNEATH US - and
            // everything we believed about it is now false. Echo is back on,
            // the socket mode is back to its default, any context is gone.
            //
            // This is not hypothetical. A real A7670G emits *ATREADY: 1, then
            // boots a SECOND time about seven seconds later, every single
            // cold start. The library configured the first boot and the second
            // one threw it away - including AT+CIPRXGET=1, which is what puts
            // the module into Buffer Access. Nothing failed, no command
            // errored: the module simply stopped being the one we had set up,
            // and a socket opened later returned zero bytes because its data
            // was being pushed at us in a mode we do not read.
            //
            // Worth noting against the manual: A76XX V1.12 documents NO boot
            // banner at all - no RDY, no PB DONE, no SMS DONE, verified by
            // searching all 684 pages. This firmware sends *ATREADY: 1,
            // +CPIN: READY and SMS DONE. The manual is not wrong so much as
            // incomplete, which is the whole argument for capturing hardware.
            //
            // So: go back to Syncing and configure it again. Cheap, idempotent
            // and the only correct response to "that is a different module
            // now".
            if (_state != GsmState::Off && _state != GsmState::Failed) {
                _counters.moduleRestarts++;
                _bearer.markDown();

                // Everything we believed about it, forgotten - registration
                // included. Keeping that one sent a module that came back
                // UNREGISTERED straight past Registering to bring a bearer up
                // on no network. See resetBringUpMemory().
                resetBringUpMemory();

                // A GUARD, BECAUSE "CONFIGURE IT AGAIN" IS A LOOP IF THE
                // CONFIGURATION IS WHAT RESTARTS IT.
                //
                // That is not a hypothetical either. A command well inside its
                // documented parameter ranges reset this exact module on every
                // attempt, and restarting bring-up simply sent it again - the
                // library span for minutes, doing the same four seconds of
                // work over and over, with every individual step succeeding.
                //
                // So a module that keeps restarting during bring-up is
                // reported as a fault rather than retried forever. Failed is
                // itself retried on a backoff, so this recovers by itself if
                // the cause was transient; what it will not do is hide.
                if (_restartsThisBringUp >= kMaxRestartsPerBringUp) {
                    _lastError = GsmStatus::NotSupportedByModule;
                    enter(GsmState::Failed, 30000);
                } else {
                    _restartsThisBringUp++;
                    enter(GsmState::Syncing, syncTimeoutMs());
                }

                // What the old module was asked is abandoned only NOW, with
                // the state already moved on - so the step it concludes is
                // recognisably a straggler and is dropped, rather than failing
                // the sequence it belonged to and starting the recovery ladder
                // on top of the restart.
                _engine.abandonAll(GsmStatus::Cancelled);
            }
            break;

        default:
            break;
    }

    if (_eventHandler != nullptr) { _eventHandler(_eventContext, info.kind, info); }
}

bool GsmModem::onFinalResult(void* context, const char* line, size_t length,
                             GsmResult& out) {
    GsmModem* self = (GsmModem*)context;
    if (self->_driver == nullptr) { return false; }
    return self->_driver->isFinalResult(line, length, out);
}

// =============================================================================
// STATUS
// =============================================================================

void GsmModem::refreshStatus() {
    if (_driver == nullptr || _state == GsmState::Off) { return; }

    GsmCmdBuf command;
    GsmCommand request;
    request.onReply = &GsmModem::onReplyLine;
    request.onDone = &GsmModem::onStatusDone;
    request.context = this;
    // [A76XX §4.2.9 AT+CPSI] and [§15.2.7 AT+IPADDR] both give a Max Response
    // Time of 9000 ms, and this one request object is reused for every status
    // query. The slowest of them sets the deadline for all.
    request.timeoutMs = kMaxResponseMs;

    // Superseded, so an application that calls this every scan sends one query
    // rather than a backlog of identical ones - and the caller whose query was
    // replaced is told rather than left waiting.
    _driver->signalCommand(command);
    request.text = command.text();
    request.supersedeKey = kKeySignal;
    _engine.submit(request);

    GsmCmdBuf registration;
    _driver->registrationCommand(0, registration);
    request.text = registration.text();
    request.supersedeKey = kKeyRegistration;
    _engine.submit(request);

    GsmCmdBuf systemInfo;
    _driver->systemInfoCommand(systemInfo);
    request.text = systemInfo.text();
    request.supersedeKey = kKeySystemInfo;
    _engine.submit(request);
}
