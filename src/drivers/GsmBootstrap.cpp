// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#include "GsmBootstrap.h"
#include "../GsmNoExcept.h"

namespace GsmBootstrap {

void probeCommand(GsmCmdBuf& out) {
    // ITU-T V.25ter - the bare attention sequence.
    //
    // The oldest command in the file and the most important one. Because one
    // module family documents NO boot banner at all and the other suppresses
    // its banner whenever auto-bauding is active, repeating this until it
    // answers is the only portable way to learn that an AT port is alive.
    out.clear();
    out.add("AT");
}

void identifyCommand(GsmCmdBuf& out) {
    // ITU-T V.25ter - request identification information.
    //
    // Free-form: modules answer with one line or several, in whatever order
    // suits them, and the model may not be first. So every line is offered to
    // the driver registry rather than one field being picked out here.
    out.clear();
    out.add("ATI");
}

void revisionCommand(GsmCmdBuf& out) {
    // 3GPP TS 27.007 5.3 - request revision identification.
    out.clear();
    out.add("AT+CGMR");
}

void modelCommand(GsmCmdBuf& out) {
    // 3GPP TS 27.007 5.2 - request model identification.
    //
    // Sent BEFORE a driver exists, so like the others here it cannot come
    // from one. See the header for the two real replies that made it
    // necessary.
    out.clear();
    out.add("AT+CGMM");
}

void radioRestartCommand(GsmCmdBuf& out) {
    // 3GPP TS 27.007 8.2 - set phone functionality.
    //
    // AT+CFUN=<fun>,<rst> with fun=1 full functionality and rst=1 "reset the
    // MT before setting it to <fun> power level". The radio restarts; the
    // module does not.
    out.clear();
    out.add("AT+CFUN=1,1");
}

} // namespace GsmBootstrap
