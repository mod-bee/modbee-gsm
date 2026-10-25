// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#pragma once

// Power, as seven optional hooks. THIS FILE IS WHY THE LIBRARY HAS NO PINS.
//
// Boards differ in every respect here - a load switch, a direct PWRKEY, a
// relay, a STATUS line or none, a USB modem with no pin at all.
//
//   THE APPLICATION owns the pins and implements whichever of these calls
//   its board can answer.
//   THIS LIBRARY owns the ORDER and the DEADLINES, which come from the
//   module's hardware design and are declared by the driver.
//
// Order is not optional: hardware designs state that cutting the supply while
// the module runs can corrupt its flash. So powerOff() is never first - the
// module's own power-down command goes out, the library waits for it to
// settle or for isPoweredOn() to go false, and only then calls the hook.
//
// Supply no hooks and the library assumes the modem is already on, which is
// correct for a USB modem and for a board that wires the module live.
#include "GsmPlatform.h"

// Three-valued, because "I do not know" is a real and common answer.
//
// A board with no STATUS line cannot say whether the module is powered; one
// with a STATUS line can say so authoritatively. Collapsing that to a bool
// would make "no wire fitted" indistinguishable from "definitely off", and the
// bring-up machine treats those completely differently: one means poll for the
// modem, the other means power it on first.
enum class GsmTristate : uint8_t { Unknown = 0, No, Yes };

class GsmPowerControl {
public:
    virtual ~GsmPowerControl() = default;

    // --- THE LINE-LEVEL HOOKS. PREFER THESE. ---
    //
    // Most modules are started and stopped by a PULSE on a power key: hold the
    // line at its active level for a documented time, then release it. It is a
    // push button, not a switch, and the module toggles on the edge.
    //
    // THE PULSE WIDTH IS A MODULE FACT, NOT A BOARD FACT, so the library times
    // it and the application only moves the line. That split is not tidiness:
    //
    //   - The widths differ by more than 10x between families (see
    //     GsmProfile). A board that picked one would work on one module and
    //     silently fail on the next.
    //   - One family's minimum on-pulse is half a SECOND, so a board that
    //     pulsed inside a single call would have to block for that long -
    //     which this library does not do anywhere.
    //   - The module cannot be identified until it is running, so at the
    //     moment the first pulse is needed NOBODY knows which module it is.
    //     The library handles that (see GsmBootstrap); an application could
    //     not.
    //   - Holding the line indefinitely is explicitly forbidden on at least
    //     one family, where an internal timer resets the module if the key
    //     stays down. Timing it centrally means that hazard is honoured once,
    //     here, rather than in every board file.
    //
    // Return false for "this board has no such line", which is not an error.

    // Hold the module's power key at its ACTIVE level, or release it.
    //
    // "Asserted" means the module sees its key pressed. Which way round that
    // is in volts is entirely the board's business - a key is usually active
    // low at the module and is often driven through an inverting transistor,
    // so asserted may well mean driving a GPIO high. The library never knows
    // and never needs to.
    virtual bool setPowerKey(bool asserted) { (void)asserted; return false; }

    // Apply or remove the module's supply, where the board switches one.
    // Separate from the key: a board may have a rail and no key, a key and no
    // rail, both, or neither.
    virtual bool setSupply(bool on) { (void)on; return false; }

    // Hold the module's reset line at its active level, or release it. Same
    // reasoning as the key, and the recovery ladder's late rung.
    virtual bool setReset(bool asserted) { (void)asserted; return false; }

    // --- THE WHOLE-OPERATION HOOKS. A FALLBACK. ---
    //
    // For boards where power really is a simple switch with no key at all - a
    // load switch, a relay, a USB port - and for boards that have their own
    // sequencing they would rather keep. Used only when setPowerKey() says the
    // board has no key.

    // Bring the module up. Return false for "I cannot do that", which is the
    // default and is not an error - it means the caller should assume the
    // module is already on and try to talk to it.
    //
    // MUST NOT BLOCK for the module's boot time. Assert whatever the board
    // needs and return; the library holds the deadline and waits across
    // successive poll() calls.
    virtual bool powerOn() { return false; }

    // Remove power. Called ONLY after the library has issued the module's own
    // power-down command and waited - see the note above about flash.
    virtual bool powerOff() { return false; }

    // Assert the module's reset. The recovery ladder's late rung, used when
    // the module has stopped answering and a software restart did not work.
    //
    // Deliberately separate from powerOn()/powerOff(): on most modules reset
    // and power are different pins with different pulse widths, and the
    // manuals are consistent that reset is for emergencies rather than for
    // ordinary restarts.
    virtual bool reset() { return false; }

    // Whether the module is powered, if the board can tell.
    //
    // Where this returns Yes or No it is authoritative and the library uses it
    // instead of guessing - a STATUS line that goes high only when the module
    // has finished initialising is a better boot signal than any amount of
    // polling, and several modules document exactly that behaviour.
    //
    // Unknown is the default and costs nothing: the library falls back to
    // asking the modem whether it is there.
    virtual GsmTristate isPoweredOn() const { return GsmTristate::Unknown; }
};

// The default. Says it cannot control power, which makes the library assume
// the module is already on - correct for a USB modem and for a board that
// wires the module live.
//
// A named class rather than a null pointer, so that the no-hardware case is
// something the code says rather than something it omits.
class GsmNoPowerControl : public GsmPowerControl {};
