// ModBee GSM - an HTTP request over cellular.
//
// The point of this sketch is what is NOT in it: no HTTP code.
//
// GsmClient implements Arduino's own Client, so ArduinoHttpClient works over
// a modem exactly as it does over WiFi or Ethernet, with no source change.
// That is the whole shape of this library - one transport, and the ecosystem
// on top of it.
//
// Needs: ArduinoHttpClient (Arduino Library Manager).
//
// Change MODEM_SERIAL and the APN below to suit your board and your SIM.

#include <ModBeeGsm.h>
#include <ArduinoHttpClient.h>

// Whatever serial port your modem is wired to. This library does not know and
// does not need to - it is handed a stream and nothing else.
#define MODEM_SERIAL Serial1

GsmDevice  gsm;
GsmClient  net(gsm.sockets());
HttpClient http(net, "example.com", 80);

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

    gsm.apn("internet");            // whatever your SIM's operator wants
    gsm.begin(&MODEM_SERIAL, 115200, rxBuffer);

    Serial.println("bringing the modem up...");
}

void loop() {
    // The only call that does work. Nothing in this library blocks, so this
    // returns promptly whatever the network is doing.
    gsm.poll();

    static bool fetched = false;
    if (!fetched && gsm.isReady()) {
        fetched = true;

        Serial.print("module   "); Serial.println(gsm.module());
        Serial.print("operator "); Serial.println(gsm.operatorName());
        Serial.print("address  "); Serial.println(gsm.ipAddress());

        // From here on this is ordinary ArduinoHttpClient code.
        http.get("/");
        Serial.print("status ");
        Serial.println(http.responseStatusCode());
        Serial.println(http.responseBody());
        http.stop();
    }
}
