// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#pragma once

// A conclusion, and where its number came from.
//
// A STRUCT, NOT AN ENUM, because a modem reports failure in four independent
// number spaces and the same integer means different things in each:
//
//   +CME ERROR   3GPP TS 27.007 Annex A. Universal.
//   +CMS ERROR   3GPP TS 27.005. SMS only.
//   <err_info>   0..22, the A76XX socket table.
//   <err>        0..12, a DIFFERENT A76XX table, same chapter.
//   <result>     0..27, the SIM707x table.
//
// Flattening them loses what decides the next action: <result> 25 is
// "certificate's common name does not match", which no +CME code expresses.
// So a result carries WHICH SPACE and WHAT CODE; the driver that owns the
// space renders the sentence and the core never interprets the number.
//
// <err_info> can be reported numerically or as English prose, by module
// setting. Bring-up pins it to NUMERIC: a number has one representation, a
// sentence has as many as the firmware has revisions.
#include "GsmPlatform.h"

// Which of the module's number spaces a code belongs to.
//
// Ordered so that None is zero and a default-constructed result carries no
// claim about a module it never reached.
enum class GsmErrorSpace : uint8_t {
    None = 0,

    // 3GPP, and therefore the same on every modem ever made. The core may
    // render these itself; everything below belongs to a driver.
    Cme,          // TS 27.007 Annex A - equipment errors
    Cms,          // TS 27.005 - message service errors

    // Vendor tables. The driver that emitted the code renders it.
    SocketInfo,   // A76XX-style <err_info>, 0..22
    SocketErr,    // A76XX-style <err>, 0..12
    ConnResult,   // SIM707x-style <result>, 0..27

    // A driver's own invention, for a module whose table matches none of the
    // above. Carried rather than refused, so adding a module never needs this
    // enumeration edited.
    Driver,
};

// What became of a command, an operation or a socket.
//
// Grouped by WHERE THE OUTCOME CAME FROM, because that is what decides what a
// caller does next. A refusal that never reached the modem is a configuration
// problem; one the modem sent is a capability or state problem; a link fault
// is neither.
enum class GsmStatus : uint8_t {
    // --- Accepted ---
    Queued = 0,          // Accepted; the outcome arrives on a later poll()
    Ok,                  // Completed, and completed successfully

    // --- Refused here; nothing reached the modem ---
    NotInitialised,      // begin() has not been called
    InvalidArgument,
    NotReady,            // the modem is not through bring-up yet
    NoBearer,            // registered, perhaps, but no data context is up
    NoSocketAvailable,   // every socket the PROFILE declares is in use
    QueueFull,           // past the queue capacity; never a silent drop
    Superseded,          // replaced by a later command to the same effect
    Cancelled,           // the application asked, or the modem was ended
    TooLarge,            // over what this module's profile says it can carry

    // Refused because THIS MODULE cannot do it - not because the request was
    // wrong. Distinct from InvalidArgument on purpose: one means fix your
    // code, the other means fit a different modem or accept the limit.
    NotSupportedByModule,

    // --- Concluded by the modem ---
    Error,               // a bare ERROR, with no reason offered
    ModuleError,         // a reason WAS offered - see space() and code()

    // --- The link or the lifecycle, not the command ---
    Timeout,             // no conclusion inside this command's own deadline
    SocketClosed,        // this socket went; others may be fine
    BearerLost,          // the data context went; every socket is dead
    Unresponsive,        // the modem stopped answering at all
    PoweredOff,          // it is off, and we are the ones who turned it off

    // The network said NO. <stat> 3 on CREG/CGREG/CEREG.
    //
    // Distinct from Timeout on purpose, and the distinction is the whole
    // point: a timeout sends somebody to look at the antenna, the bands and
    // the signal; a denial sends them to the operator. Measured on a
    // SIM7070G with CSQ 17-31 - excellent signal, explicit refusal - while
    // the library reported a timeout and said nothing about the refusal.
    RegistrationDenied,
};

// A conclusion, and where its number came from.
//
// Four bytes, trivially copyable, no allocation. It is returned by value from
// the poll path and stored one per queued command, so it may not grow.
class GsmResult {
public:
    constexpr GsmResult() : _status(GsmStatus::Ok), _space(GsmErrorSpace::None), _code(0) {}
    constexpr GsmResult(GsmStatus status)   // NOLINT - implicit on purpose
        : _status(status), _space(GsmErrorSpace::None), _code(0) {}
    constexpr GsmResult(GsmStatus status, GsmErrorSpace space, uint16_t code)
        : _status(status), _space(space), _code(code) {}

    // A module reported a numbered reason. The usual way a driver concludes a
    // command that failed on the far side of the serial link.
    static constexpr GsmResult fromModule(GsmErrorSpace space, uint16_t code) {
        return GsmResult(GsmStatus::ModuleError, space, code);
    }

    constexpr GsmStatus status() const { return _status; }
    constexpr GsmErrorSpace space() const { return _space; }
    constexpr uint16_t code() const { return _code; }

    // Whether this is a conclusion at all. Queued is not: it says the command
    // was accepted and the real answer is still to come.
    constexpr bool isConcluded() const { return _status != GsmStatus::Queued; }
    constexpr bool isOk() const { return _status == GsmStatus::Ok; }

    // Whether retrying the identical command could plausibly succeed.
    //
    // Worth having in one place rather than as a switch copied into every
    // caller: the rule is subtle in exactly one spot. A short send is NOT
    // retryable even though it looks like a transient failure - the module's
    // own manual says the socket cannot be used further afterwards - and that
    // is expressed by the driver concluding SocketClosed rather than Timeout.
    bool isRetryable() const;

    // The library's own vocabulary, always available and never null.
    // "timeout", "no bearer", "queue full".
    //
    // It does NOT render code(): that number belongs to a module's table and
    // only the driver holding that table can say what it means. Ask the driver
    // for the sentence; ask this for the category.
    const char* describe() const;

    // The 3GPP spaces only, which are universal, or null for a vendor space.
    // Kept separate from describe() so that a caller which wants the standard
    // reason does not accidentally get a driver's guess at one.
    const char* describeStandardCode() const;

    constexpr bool operator==(const GsmResult& other) const {
        return _status == other._status && _space == other._space &&
               _code == other._code;
    }
    constexpr bool operator!=(const GsmResult& other) const { return !(*this == other); }

private:
    GsmStatus     _status;
    GsmErrorSpace _space;
    uint16_t      _code;
};
