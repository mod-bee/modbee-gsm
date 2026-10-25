// ModBee GSM - MQTT over cellular, with PubSubClient.
//
// Again: no MQTT code here. PubSubClient takes a Client&, GsmClient is one,
// and the two have never heard of each other.
//
// PubSubClient's connect() blocks, because Arduino's Client interface demands
// a verdict. A client written against GsmSocket directly does not - see
// docs/05, "Carrying another protocol over it".
//
// Needs: PubSubClient (Arduino Library Manager).
//
// Change MODEM_SERIAL, the APN and the broker below to suit your board, your
// SIM and your network. broker.example.com is a placeholder.

#include <ModBeeGsm.h>
#include <PubSubClient.h>

#define MODEM_SERIAL Serial1

GsmDevice    gsm;
GsmClient    net(gsm.sockets());
PubSubClient mqtt(net);

void onMessage(char* topic, byte* payload, unsigned int length) {
    Serial.print("message on "); Serial.print(topic); Serial.print(": ");
    for (unsigned int i = 0; i < length; i++) { Serial.write(payload[i]); }
    Serial.println();
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

    mqtt.setServer("broker.example.com", 1883);
    mqtt.setCallback(onMessage);
}

void loop() {
    gsm.poll();
    if (!gsm.isReady()) { return; }

    if (!mqtt.connected()) {
        // PubSubClient's connect blocks, because Arduino's Client::connect
        // must return a verdict. The library documents that as its one
        // blocking call; modbee-gsm's own socket has no such limit.
        if (mqtt.connect("modbee-demo")) {
            mqtt.subscribe("plant/demo/command");
        }
        return;
    }

    mqtt.loop();

    static unsigned long lastPublish = 0;
    if (millis() - lastPublish > 30000) {
        lastPublish = millis();
        char payload[48];
        // rssiDbm() is GSM_UNKNOWN_DBM until the modem has been asked, never
        // zero: zero dBm would be a superb signal, not a missing one.
        snprintf(payload, sizeof payload, "{\"rssi\":%d}", gsm.rssiDbm());
        mqtt.publish("plant/demo/status", payload);
    }
}
