// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#include "GsmPlatform.h"

#if defined(GSM_PLATFORM_ARDUINO)

#include <Arduino.h>
#include "GsmNoExcept.h"

namespace {

gsm_time_t platformMillis() { return (gsm_time_t)millis(); }
gsm_time_t platformMicros() { return (gsm_time_t)micros(); }

} // namespace

#if defined(ARDUINO_ARCH_ESP32)
// A REAL SLEEP ON ESP32, NOT yield(). There yield() is vPortYield(), which
// hands the CPU only to tasks of the SAME priority - so GsmClient::connect(),
// pumping for a second or five, starved every lower-priority task on its core
// for that long: other PLC tasks in an RTOS build, and the idle task the task
// watchdog checks. One tick lets them all run; the pump still turns a
// thousand times a second, which is far faster than any module answers.
void gsm_yield() { delay(1); }
#else
void gsm_yield() { yield(); }
#endif

#else

#include <chrono>
#include <thread>

namespace {

// Anchored at first use, so both clocks start near zero the way Arduino's do.
//
// std::chrono::steady_clock counts from an arbitrary point that is typically
// system uptime - a large number that is nowhere near the 32-bit wrap. Left
// alone it would put the host permanently in a different part of the range
// from the board, which hides rollover defects on the machine that runs the
// tests and exposes them only on the one in the field.
std::chrono::steady_clock::time_point startupTime() {
    static const std::chrono::steady_clock::time_point t0 =
        std::chrono::steady_clock::now();
    return t0;
}

gsm_time_t platformMillis() {
    const auto since = std::chrono::steady_clock::now() - startupTime();
    return (gsm_time_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        since).count();
}

gsm_time_t platformMicros() {
    const auto since = std::chrono::steady_clock::now() - startupTime();
    return (gsm_time_t)std::chrono::duration_cast<std::chrono::microseconds>(
        since).count();
}

} // namespace

void gsm_yield() { std::this_thread::yield(); }

#endif

#if defined(ARDUINO_ARCH_ESP32)
#include <esp_heap_caps.h>

void* gsm_allocLarge(size_t bytes) {
    if (bytes == 0) { bytes = 1; }
    void* memory = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (memory == nullptr) {
        memory = heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    return memory;
}
void gsm_freeLarge(void* memory) { heap_caps_free(memory); }
#else
#include <stdlib.h>

void* gsm_allocLarge(size_t bytes) { return malloc(bytes != 0 ? bytes : 1); }
void gsm_freeLarge(void* memory) { free(memory); }
#endif

namespace {

// Constant-initialised with the platform clocks, so there is no static
// initialisation order to reason about and no null check on a path that runs
// on every poll().
gsm_clock_fn g_millisSource = &platformMillis;
gsm_clock_fn g_microsSource = &platformMicros;

} // namespace

gsm_time_t gsm_millis() { return g_millisSource(); }
gsm_time_t gsm_micros() { return g_microsSource(); }

void gsm_setClockSource(gsm_clock_fn millisSource, gsm_clock_fn microsSource) {
    g_millisSource = millisSource ? millisSource : &platformMillis;
    g_microsSource = microsSource ? microsSource : &platformMicros;
}

void gsm_resetClockSource() {
    g_millisSource = &platformMillis;
    g_microsSource = &platformMicros;
}
