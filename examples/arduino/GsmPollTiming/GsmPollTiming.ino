// ModBee GSM - how long poll() actually takes, on the real chip.
//
// THIS IS THE MEASUREMENT THE HOST CANNOT MAKE.
//
// Timing poll() on a development machine gets figures in the tens of
// nanoseconds - but a desktop CPU runs perhaps twenty times more instructions
// per second than a 240 MHz microcontroller and has cache the controller does
// not. That makes a host number useful for the SHAPE of the curve (bounded?
// does it grow over time?) and useless as an absolute.
//
// So this sketch does the only honest thing: calls micros() either side of
// poll() on the actual silicon, and prints what it finds.
//
// It needs no SIM and no antenna. With nothing attached at all it measures the
// busiest state there is - Syncing, where the library is sending a probe every
// second and parsing whatever noise comes back - which is a fair worst case.
// With a real modem it goes on to measure the steady state, which is the one a
// scan cycle actually lives in.
//
// Needs: nothing beyond ModBee GSM. Change MODEM_SERIAL to suit your board.

#include <ModBeeGsm.h>

#define MODEM_SERIAL Serial1

GsmDevice gsm;

// Kept as running values rather than an array: 100000 samples at four bytes
// each is more RAM than the boards this library targets have spare, and the
// median is not the interesting number here - the WORST one is.
struct Window {
    const char* name;
    uint32_t count;
    uint32_t worst;
    uint64_t total;
    uint32_t over100us;
    uint32_t over1ms;

    // A CONSTRUCTOR, NOT DEFAULT MEMBER INITIALISERS. Brace-initialising an
    // aggregate that has NSDMIs is C++14; this library builds at gnu++11 and
    // several Arduino cores - SAMD among them - hold their sketches to the
    // same standard.
    explicit Window(const char* label)
        : name(label), count(0), worst(0), total(0), over100us(0), over1ms(0) {}

    void add(uint32_t microseconds) {
        count++;
        total += microseconds;
        if (microseconds > worst) { worst = microseconds; }
        if (microseconds > 100) { over100us++; }
        if (microseconds > 1000) { over1ms++; }
    }

    void report() const {
        if (count == 0) { return; }
        Serial.print(name);
        Serial.print(": n=");        Serial.print(count);
        Serial.print("  mean=");     Serial.print((uint32_t)(total / count));
        Serial.print("us  worst=");  Serial.print(worst);
        Serial.print("us  >100us="); Serial.print(over100us);
        Serial.print("  >1ms=");     Serial.println(over1ms);
    }
};

Window syncing{"syncing (no modem, or booting)"};
Window ready{"ready (steady state)"};
Window afterHour{"ready, after an hour"};

unsigned long startedAt = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) { }

    // THE RECEIVE BUFFER, AND THE TRUTH ABOUT IT.
    //
    // Two separate things, and they must agree. setRxBufferSize() actually
    // makes the buffer bigger, and only ESP cores have it - SAMD, STM32 and
    // AVR compile their buffer size in. The third argument to gsm.begin() is
    // what you TELL the library the buffer holds; it caps every fetch to that,
    // because most modem wiring has no flow control and an overrun does not
    // lose a reading, it desynchronises the AT stream.
    //
    // So claim what you actually have. A small buffer is safe but slow; a
    // number larger than the truth is the one that breaks.
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
    const size_t rxBuffer = 2048;
    MODEM_SERIAL.setRxBufferSize(rxBuffer);
#else
    const size_t rxBuffer = 256;    // the Arduino default on most cores
#endif
    MODEM_SERIAL.begin(115200);

    gsm.apn("internet");
    gsm.begin(&MODEM_SERIAL, 115200, rxBuffer);

    startedAt = millis();

    Serial.println();
    Serial.println("ModBee GSM - poll() timing");
    Serial.print("CPU ");
#if defined(ESP32) || defined(ESP8266)
    Serial.print(ESP.getCpuFreqMHz());
    Serial.println(" MHz");
#else
    Serial.println("(unknown clock)");
#endif
    Serial.println("Runs for an hour. No SIM or antenna needed.");
    Serial.println();
}

void loop() {
    const uint32_t before = micros();
    gsm.poll();
    const uint32_t elapsed = micros() - before;   // unsigned: wraps correctly

    const unsigned long upFor = millis() - startedAt;

    if (!gsm.isReady()) {
        syncing.add(elapsed);
    } else if (upFor < 3600000UL) {
        ready.add(elapsed);
    } else {
        afterHour.add(elapsed);
    }

    // Every ten seconds, so the numbers are watchable rather than a wall.
    static unsigned long lastReport = 0;
    if (millis() - lastReport > 10000) {
        lastReport = millis();

        Serial.print("["); Serial.print(upFor / 1000); Serial.print("s] ");
        Serial.println(gsm.stateName());
        syncing.report();
        ready.report();
        afterHour.report();
        Serial.println();
    }

    // NOT delayed. A scan cycle calls poll() as often as it runs, and
    // inserting a wait here would measure the wait.
}
