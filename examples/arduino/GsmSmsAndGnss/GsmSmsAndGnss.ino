// ModBee GSM - SMS and location, with no network connection at all.
//
// Neither of these needs a data bearer, which makes them the fallback when
// one cannot be had: a site with no usable data service can still be asked
// where it is and told what to do.
//
// Needs: nothing beyond ModBee GSM. Change MODEM_SERIAL, the APN and the phone
// number below to suit your board, your SIM and yourself.
//
// Not every module has a satellite receiver, even within one family: on the
// A7670, for instance, it depends on the order code. gsm.hasGnssReceiver()
// says whether yours answered.

#include <ModBeeGsm.h>

#define MODEM_SERIAL Serial1

GsmDevice gsm;

// REPLACE with your own number, in international format. This one is from a
// range reserved for fiction, so it reaches nobody.
const char* kAlertNumber = "+447700900123";

void onSms(void* /*context*/, const GsmSmsMessage& message) {
    Serial.print("SMS from "); Serial.print(message.sender);
    Serial.print(": "); Serial.println(message.text);

    // A message that is one part of several says so, rather than arriving as
    // a fragment nobody can tell from a whole one.
    if (message.isPart) {
        Serial.print("  (part "); Serial.print(message.partIndex);
        Serial.print(" of "); Serial.print(message.partCount); Serial.println(")");
    }
}

void setup() {
    Serial.begin(115200);
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
    gsm.onSms(onSms, nullptr);
}

void loop() {
    gsm.poll();

    static bool started = false;
    if (!started && gsm.isRegistered()) {
        started = true;
        gsm.gnss(true);                       // a cold fix can take minutes
        gsm.sendSms(kAlertNumber, "modbee-gsm is up");
    }

    static unsigned long lastFix = 0;
    if (started && millis() - lastFix > 10000) {
        lastFix = millis();
        gsm.refreshFix();

        if (gsm.hasFix()) {
            Serial.print(gsm.fix().latitude, 6); Serial.print(", ");
            Serial.print(gsm.fix().longitude, 6);

            // THE NUMBER THAT MATTERS. A receiver that lost the sky keeps
            // reporting its last position perfectly happily, so the age is
            // what says whether to believe it.
            Serial.print("  age "); Serial.print(gsm.fixAgeMillis() / 1000);
            Serial.println("s");
        } else {
            Serial.println("no fix yet");
        }
    }
}
