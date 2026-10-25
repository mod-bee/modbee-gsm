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
// THE COMMANDS SENT BEFORE A DRIVER EXISTS
// =============================================================================
//
// A chicken and egg, and this file is the egg.
//
// The core chooses a driver from what the module says about itself - but
// asking it that is an AT command, and there is no driver yet to supply one.
// An earlier draft solved it by putting four commands directly in the bring-up
// machine. That is the wrong answer: the moment the core knows how to say
// anything, it has learnt a dialect, and the second module starts needing
// special cases in places nobody would look.
//
// So they live here, in the drivers directory, with everything else that
// speaks AT. The core calls these; it does not contain them.
//
// ALL FOUR ARE UNIVERSAL. Two are ITU-T V.25ter basic commands that predate
// every module this library will ever meet, and two are 3GPP commands that
// every cellular modem implements by definition. Nothing vendor-specific
// belongs in this file - anything that needs a manual with a part number on
// the cover belongs in a driver.

#include "../GsmCmdBuf.h"

namespace GsmBootstrap {

// Is anything there? The probe the sync state repeats until something answers.
void probeCommand(GsmCmdBuf& out);

// What are you? Answered with free-form text that a driver's claims() reads.
void identifyCommand(GsmCmdBuf& out);

// Which firmware? Two facts in a profile vary by revision rather than by part,
// so the answer is part of choosing a driver, not merely a diagnostic.
void revisionCommand(GsmCmdBuf& out);

// The MODEL, asked the standardised way.
//
// ATI is free-form and two real modules prove how little that guarantees:
//
//   A7670G-LABE  ATI -> "Manufacturer: SIMCOM INCORPORATED" then
//                        "Model: A7670G-LABE" on the NEXT line
//   SIM7070G     ATI -> "R1951.05"    - a firmware build number, and no
//                        model anywhere in the reply at all
//
// The second cannot be rescued by any amount of parsing: the answer simply
// does not contain what a driver needs to recognise the module. AT+CGMM does,
// on both - "A7670G-LABE" and "SIMCOM_SIM7070" - because unlike ATI it is
// specified rather than free-form.
void modelCommand(GsmCmdBuf& out);

// Restart the radio without restarting the module.
//
// A rung of the recovery ladder, and a cheap one: it clears most registration
// faults without touching the module's flash, its sockets or its
// configuration. Tried before anything that resets hardware.
void radioRestartCommand(GsmCmdBuf& out);

// THE FIRST PULSE, sent before anyone knows which module this is - the key
// must be pulsed before the module talks, and it will not say what it is
// until it does. These figures are the OVERLAP of what the families document:
//
//   [A7670 HW Design V1.00 3.2.1 T9]   Ton on-pulse    50 ms typical
//   [A7670 HW Design V1.00 3.2.2 T10]  Toff off-pulse  2.5 s minimum
//   [SIM7070G-HP HW V1.02 3.2.1 T12]   Ton on-pulse    0.5-12 s
//                                      Ton(wait)       1 s minimum
//   [SIM7070G-HP HW V1.02 3.2.2 T13]   Toff off-pulse  1.2 s minimum
//
// Window 500..1200 ms: below 500 the SIM707x sees no press; at 1200 it reads
// as a shutdown. 1000 sits inside with room either side.
//
// Because 1000 ms is under BOTH families' power-off minimums, a bootstrap
// pulse cannot shut down a module that is already running - so it is safe to
// pulse first and ask questions afterwards.
constexpr uint32_t kBootstrapOnPulseMs = 1000;

// The longer family's requirement, applied to both. Harmless where it is not
// required and mandatory where it is.
//   [SIM7070G-HP §3.2.1, Table 12] Ton(wait) >= 1 s from VBAT stable.
constexpr uint32_t kBootstrapSupplySettleMs = 1000;

// The LONGER of the two families' UART-ready figures, because giving up early
// on a slow module looks exactly like a dead one.
//   [A7670 §3.2.1, Table 9]        Ton(uart) 11.2 s typical
//   [SIM7070G-HP §3.2.1, Table 12] Ton(uart) 3 s typical, 4 s maximum
constexpr uint32_t kBootstrapUartReadyMs = 11200;

// Never hold the key longer than this before a profile is available.
//   [SIM7070G-HP §3.2.1] "The PWRKEY pin has its own reset function. The reset
//   time is determined by the internal timer (default is 12 seconds). After
//   the PWRKEY is pulled low, the module will be reset after 12 seconds.
//   Therefore, it is not recommended to connect PWRKEY to ground all the time
//   in external circuit design."
constexpr uint32_t kBootstrapMaxHoldMs = 12000;

} // namespace GsmBootstrap
