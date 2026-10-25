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
// PLATFORM
// =============================================================================
//
// The only file in the library that knows whether it is on an Arduino or a
// host. Everything above it asks for the time and yields; nothing above it
// includes a platform header.
//
// Two things are settled here and they are settled once: how wide a time value
// is, and where it comes from.
//
// THIS FILE KNOWS NOTHING ABOUT ANY BOARD, and neither does anything else in
// src/. Pins, enable lines, power rails and product names live in the
// application. See GsmPowerControl - the library defines the ORDER and the
// DEADLINES of a power sequence and calls back; what a callback does with a
// pin is not this library's business.

#include <stdint.h>
#include <stddef.h>

#if defined(ARDUINO)
    #define GSM_PLATFORM_ARDUINO 1
#else
    #define GSM_PLATFORM_HOST 1
#endif

// --- Time ---

// Fixed at 32 bits on every target.
//
// `unsigned long` is 32-bit on ESP32 and MSVC but 64-bit on Linux and macOS,
// so a library that used it would roll over after 49.7 days on the board and
// never on the host that tests it. Pinning the width keeps every timeout
// calculation identical everywhere, which means a rollover defect shows up on
// every target or on none - and the one that shows up nowhere is the one that
// takes a site down in seven weeks.
//
// It matters more here than in a fieldbus library. A modem carries timeouts
// measured in whole minutes - registration on NB-IoT, an operator scan at
// AT+COPS=? - so the arithmetic has further to go before it is checked.
typedef uint32_t gsm_time_t;

// Milliseconds since start. Wraps about every 49.7 days.
gsm_time_t gsm_millis();

// Microseconds since start. Wraps about every 71.6 minutes.
//
// That wrap is short enough to hit in an afternoon, so every subtraction on
// this clock goes through gsm_elapsed() below.
gsm_time_t gsm_micros();

// How long has passed between two readings of the same clock.
//
// Unsigned arithmetic wraps, so this is correct across the rollover for any
// interval shorter than the clock's full range. Prefer it over a bare
// subtraction everywhere: writing it this way says the wrap was considered.
inline gsm_time_t gsm_elapsed(gsm_time_t since, gsm_time_t now) {
    return (gsm_time_t)(now - since);
}

// Whether a deadline is still ahead of us.
//
// Half the range is treated as the future and half as the past, which is the
// only test that works on a clock that wraps. A plain `now < deadline` reads a
// deadline just past the wrap as long expired and fires everything at once -
// which on this library would abandon every in-flight AT command at the same
// instant and then run the whole recovery ladder for no reason.
inline bool gsm_isFuture(gsm_time_t now, gsm_time_t deadline) {
    const gsm_time_t remaining = gsm_elapsed(now, deadline);
    return remaining != 0 && remaining < 0x80000000u;
}

// --- Clock source ---

// The clock is replaceable, for two reasons.
//
// PORTING. A target this library has never seen supplies its own two functions
// and needs no #ifdef here.
//
// TESTING. A test drives time by hand, so a 90 second registration timeout, a
// 180 second operator scan and the 11.2 second power-up of an A76XX cost no
// wall clock at all - and so the 49.7 day rollover can be exercised in
// milliseconds rather than never being exercised.
typedef gsm_time_t (*gsm_clock_fn)();

// Either argument may be null, which restores the platform clock for that one.
//
// A replacement must be monotonic within its width and must WRAP rather than
// saturate, matching the platform clocks it stands in for. A source that
// saturates makes gsm_elapsed() return a constant and every timeout stops -
// which presents as a modem that never answers and never times out either.
void gsm_setClockSource(gsm_clock_fn millisSource, gsm_clock_fn microsSource);

void gsm_resetClockSource();

// --- Storage taken once ---

// The buffers a setting sizes - the socket rings, the AT command queue - are
// taken once, when the modem is opened, through these, and never on the poll
// path. On an ESP32 with PSRAM they go there first: nothing in them is touched
// from an interrupt or with the flash cache off, and internal RAM is what the
// PLC program's own variables need. Internal RAM otherwise.
//
// Null when there is no room. The caller says so out loud; it never carries
// on with less than it was asked for.
void* gsm_allocLarge(size_t bytes);
void  gsm_freeLarge(void* memory);

// a * b, or false when it would not fit in a size_t - so a setting large
// enough to wrap is refused as too large rather than allocated small.
inline bool gsm_multiply(size_t a, size_t b, size_t& out) {
    if (a != 0 && b > (size_t)-1 / a) { return false; }
    out = a * b;
    return true;
}

// --- Yielding ---

// Give the rest of the system a turn.
//
// Called where the library would otherwise spin without making progress.
// There is exactly one such place on the running path - GsmClient::connect(),
// which must return a verdict because Arduino's Client says so, and therefore
// pumps poll() until the socket opens or the deadline passes. It is documented
// as the one blocking call and it is the only caller of this function. (The
// shutdown in end() also waits, for a bounded time, while the module powers
// down; it does not call this.)
//
// IT IS NOT THE SEND PATH. A write that the transport cannot take RETURNS, and
// the rest goes out on the next poll() from the cursor where it stopped.
// Nothing in the send path spins, so nothing in the send path yields.
//
// On Arduino this is the core's own yield(), which feeds the watchdog and runs
// the network stack on an ESP8266 - except on ESP32, where it is a one-tick
// delay: yield() there only lets tasks of the same priority run, so a pump
// would starve every lower-priority task on its core. See GsmPlatform.cpp.
void gsm_yield();
