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
// WHAT THIS MODULE CAN ACTUALLY DO
// =============================================================================
//
// Facts, not behaviour. The core NEVER ASSUMES ANY OF THIS - it asks.
//
// That is what lets one binary drive modules with genuinely different limits,
// and it is why the numbers below are not constants somewhere in the socket
// code. Two modules this library ships drivers for differ on almost every line
// of this struct: 10 sockets against 13, a 1500-byte send against 1459, TLS as
// a separate sixteen-command family against TLS as a flag on the ordinary
// open, a documented boot banner against none at all.
//
// --- KEYED ON FIRMWARE, NOT ON THE PART NUMBER ---
//
// Two of these fields vary by FIRMWARE REVISION on the same silicon:
//
//   maxTlsSessions   one manual says 2, and notes that a later baseline
//                    supports 3.
//   closeNotice      one module polls for socket state on an older firmware
//                    and pushes a URC on a newer one.
//
// So a driver is selected from the identification string AND the revision, not
// from the part alone. A profile chosen on the part number would be wrong on
// half the stock in the field.

#include "GsmPlatform.h"

// How received socket data reaches us. The names are the vendor's own, from
// the TCP/IP application note, rather than ones this library invented.
enum class GsmDataAccessMode : uint8_t {
    // The module emits data unsolicited as it arrives. NEVER USED: there is no
    // back pressure at all, and on a three-wire link that loses bytes and
    // desynchronises the AT stream rather than merely dropping a reading.
    DirectPush = 0,

    // The module buffers, notifies, and we fetch what we have room for. The
    // default everywhere, and the only mode this library drives.
    BufferAccess,

    // The socket becomes a raw pipe and the AT channel is gone. One socket
    // only, no multiplexing, no diagnostics. Not supported.
    Transparent,
};

// How TLS is reached, and the two shapes are not variations of each other.
enum class GsmTlsStyle : uint8_t {
    None = 0,

    // A parallel command family with its own open, send, receive and close,
    // and usually its own smaller session count and its own send limit. Plain
    // TCP and TLS share nothing but the bearer.
    SeparateFamily,

    // A configuration flag applied to the same socket that plain TCP uses.
    // Every socket can be either.
    FlagOnOpen,
};

// How a module accounts for what it has sent.
enum class GsmSendAccounting : uint8_t {
    // The send reply states requested and confirmed lengths. When they differ
    // THE SOCKET IS DEAD - the manual says it "cannot be used further" - so a
    // short send is a terminal error rather than something to retry.
    ConfirmedLength = 0,

    // Free buffer space can be queried before sending, and unacknowledged
    // bytes after. A better shape, because it prevents the overrun rather than
    // reporting it.
    LeftSizeAndAck,
};

// How we learn a socket closed.
enum class GsmCloseNotice : uint8_t {
    Urc = 0,     // the module tells us
    Poll,        // we have to ask
};

// What the module says when it has finished booting, if anything.
enum class GsmBootBanner : uint8_t {
    // NOTHING. One manual documents no boot banner at all - zero occurrences
    // in 684 pages - so boot completion is detected by polling a bare AT until
    // it answers. Any library that waited for a banner would hang forever.
    None = 0,
    Ready,       // a banner is emitted, and may be used as an accelerator
};

struct GsmProfile {
    // What this driver calls itself, for diagnostics. Not parsed by anything.
    const char* family = "generic";

    // --- Sockets ---
    uint8_t  maxSockets = 1;
    uint8_t  firstSocketId = 0;      // some modules number from 0, others from 1
    uint8_t  maxTlsSessions = 0;

    // Hard limits from the manual. Exceeding them is refused before anything
    // reaches the modem, with TooLarge, rather than producing a truncated
    // send that the module answers in a way nobody expects.
    uint16_t maxSendChunk = 1024;
    uint16_t maxRecvChunk = 1024;
    uint16_t maxTlsSendChunk = 0;    // 0 = same as maxSendChunk

    // A module may cap the whole command line far below what we ever build,
    // but it is recorded because a driver that starts assembling certificate
    // data would meet it.
    uint16_t maxAtLineLength = 512;

    GsmDataAccessMode dataAccessMode = GsmDataAccessMode::BufferAccess;
    GsmTlsStyle       tlsStyle = GsmTlsStyle::None;
    GsmSendAccounting sendAccounting = GsmSendAccounting::ConfirmedLength;
    GsmCloseNotice    closeNotice = GsmCloseNotice::Urc;
    GsmBootBanner     bootBanner = GsmBootBanner::None;

    // --- What the module supports at all ---
    bool hasUdp = true;
    bool hasTcpServer = false;
    bool hasGnss = false;
    bool hasPsm = false;
    bool hasEdrx = false;
    bool hasSmsPdu = true;
    bool hasMultiContext = false;    // several PDP contexts, addressed by id
    bool hasPacketRegistration = true;   // a separate GPRS registration query
    bool hasEpsRegistration = true;      // a separate LTE registration query

    // --- Timings ---
    //
    // From the module's own documentation, never guessed. The spread is
    // enormous and a single global timeout is either far too short for the
    // long ones or so long that a wedged modem is undetectable.
    uint32_t defaultCommandTimeoutMs = 1000;
    uint32_t bearerUpTimeoutMs = 60000;
    uint32_t socketOpenTimeoutMs = 120000;
    uint32_t socketSendTimeoutMs = 120000;
    uint32_t socketFetchTimeoutMs = 8000;

    // How long an open socket may sit with no data notice before the module
    // is ASKED whether it is holding any. Zero turns the check off.
    //
    // The notice is edge-triggered: the module announces data once and then
    // waits to be read. A notice that never arrives - its bytes lost on a
    // serial link with no flow control, measured on a host board whose UART
    // FIFO overflowed while flash was being written - leaves the bytes in the
    // module for ever, and the application waits on a socket that looks open
    // and never delivers again. Only the driver knows the question
    // (unreadQueryCommand); a driver that has none leaves this inert.
    uint32_t unreadCheckMs = 2000;
    uint32_t registrationTimeoutMs = 90000;
    uint32_t operatorScanTimeoutMs = 180000;

    // Payload delivery deadline, where the module imposes one. A send command
    // on one module takes an input-time parameter and abandons the transfer if
    // the bytes do not arrive inside it. Zero means the module imposes none.
    uint32_t sendInputTimeMs = 0;

    // --- Power sequencing, from the hardware design document ---
    //
    // The LIBRARY owns the order and the deadlines; the APPLICATION owns the
    // pins. These are what the state machine waits for between calling one of
    // its hooks and expecting the module to respond.
    uint32_t powerOnToUartMs = 15000;    // until the AT port answers at all
    uint32_t powerOffSettleMs = 3000;    // after the power-down command
    uint32_t powerOffToOnMs = 2000;      // before a restart may begin
    uint32_t resetSettleMs = 3000;

    // How long to hold the module's RESET line to reset it - the width of the
    // pulse GsmPowerControl::setReset() is asserted for. A module fact, timed
    // by the library for the same reasons as the power key below.
    //
    // ZERO MEANS THE MODULE HAS NO RESET LINE, and the library then never
    // drives one, whatever the board offers. That is a real answer, not a
    // missing one: one supported family brings out no RESET pin at all and
    // resets through its power key's internal timer instead.
    uint32_t resetPulseMs = 0;

    // --- THE POWER KEY IS A PULSE, AND ITS WIDTH IS A MODULE FACT ---
    //
    // This is the reason GsmPowerControl::setPowerKey() is a line-level hook
    // rather than a "power on for me" call. How long the key is held is
    // written in the MODULE's hardware design, not the board's schematic, and
    // the numbers are not close between families:
    //
    //     on-pulse    50 ms on one, 500 ms MINIMUM on the other - 10x apart
    //     off-pulse   2.5 s on one, 1.2 s on the other
    //
    // An application asked to "power on the module" would have to know which
    // module it is holding, and it CANNOT: identification needs the AT port,
    // and the AT port needs the module powered. So the application asserts and
    // releases a line, and the library - which reads the profile - decides for
    // how long. See GsmBootstrap for the pulse used before identity is known.
    //
    // Zero means "not documented for this module", and the state machine then
    // falls back to the bootstrap figures rather than inventing one.

    // How long to hold the key to power a module ON.
    uint32_t powerKeyOnPulseMs = 0;

    // How long to hold it to power a running module OFF. Used only after the
    // module's own power-down command has been tried and has not worked.
    uint32_t powerKeyOffPulseMs = 0;

    // LONGEST the key may be held, where the module documents a hazard for
    // holding it longer. One family's design says the key "cannot be shorted
    // all the way to ground" and that an internal timer resets the module if
    // it is held - so a stuck line is not merely ineffective, it is a reset
    // loop that looks exactly like a dead module. Zero means none documented.
    uint32_t powerKeyMaxHoldMs = 0;

    // How long after the supply is applied before the key may be pulled at
    // all. One family requires a settling period and the other does not
    // mention one. Zero means no requirement.
    uint32_t supplySettleMs = 0;

    // --- Signal ---
    //
    // How to turn this module's reported reference signal power into dBm.
    // NOT a constant: one manual defines it as "report - 140" and another
    // gives no conversion at all. An earlier draft of this library carried a
    // third convention taken from the wrong manual entirely, which would have
    // been a tenfold error in every signal alarm.
    bool hasRsrpConversion = false;
    int  rsrpOffsetDbm = -140;           // dBm = report + rsrpOffsetDbm
};
