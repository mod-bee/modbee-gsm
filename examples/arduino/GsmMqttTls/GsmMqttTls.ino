// ModBee GSM - MQTT over TLS, encrypted by the modem.
//
// The same sketch as GsmMqtt with two characters changed: GsmClientSecure
// instead of GsmClient, and 8883 instead of 1883. PubSubClient never learns
// which it got.
//
// THE HANDSHAKE HAPPENS IN THE MODULE. That is why this costs no flash and no
// RAM on the controller - there is no TLS library here at all. What it costs
// instead is control: the module chooses the cipher suites, and some modules
// offer as few as two concurrent TLS sessions against ten plain sockets
// (gsm.profile()->maxTlsSessions says how many yours has). Certificate
// pinning, your own trust store, or more sessions than that means wolfSSL over
// a plain GsmSocket instead.
//
// A module whose driver reports no TLS refuses the connect by name rather than
// falling back to plain, so this sketch cannot silently send in clear.
//
// Needs: PubSubClient (Arduino Library Manager).
//
// Change MODEM_SERIAL, the APN and the broker below to suit your board, your
// SIM and your network. broker.example.com is a placeholder.

#include <ModBeeGsm.h>
#include <PubSubClient.h>

#define MODEM_SERIAL Serial1

GsmDevice       gsm;
GsmClientSecure net(gsm.sockets());     // <- the only difference
PubSubClient    mqtt(net);

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

    mqtt.setServer("broker.example.com", 8883);
}

void loop() {
    gsm.poll();
    if (!gsm.isReady()) { return; }

    // WORTH CHECKING ONCE, and worth reporting rather than retrying forever.
    // A module with no TLS at all cannot be made to do this, and the profile
    // says so before a single byte goes out.
    static bool checked = false;
    if (!checked && gsm.profile() != nullptr) {
        checked = true;
        if (gsm.profile()->tlsStyle == GsmTlsStyle::None) {
            Serial.println("this modem has no module TLS - use wolfSSL over "
                           "a plain GsmSocket");
        } else {
            Serial.print("TLS sessions available: ");
            Serial.println(gsm.profile()->maxTlsSessions);
        }
    }

    if (!mqtt.connected()) {
        // Wait between attempts WITHOUT delay(): poll() must keep running, or
        // the modem's replies pile up in the serial buffer while nobody reads.
        static unsigned long lastAttempt = 0;
        static bool attempted = false;
        if (attempted && millis() - lastAttempt < 5000) { return; }
        attempted = true;
        lastAttempt = millis();

        // Blocks, bounded, because Arduino's Client::connect must return a
        // verdict - and a TLS handshake is the slowest part of it. Measured at
        // about five seconds on an A7670G over a live network.
        if (mqtt.connect("modbee-tls")) {
            mqtt.publish("modbee/hello", "encrypted by the modem");
        } else {
            Serial.print("connect failed: ");
            Serial.println(net.lastError().describe());
        }
        return;
    }

    mqtt.loop();
}
