// ModBee GSM - a UDP datagram over cellular, asking an NTP server the time.
//
// GsmUDP implements Arduino's UDP, so anything written against WiFiUDP or
// EthernetUDP works unchanged. NTP is the example because it is the smallest
// honest one: 48 bytes out, 48 bytes back, and an answer you can check.
//
// A DATAGRAM IS BUILT WHOLE AND SENT ONCE. beginPacket() starts buffering,
// write() adds to the buffer, endPacket() sends it. That is UDP's own
// contract, not a limitation here - a datagram sent in pieces is several
// datagrams, which is not what anybody means.
//
// The modem also knows the time by itself, from the network, with no server
// and no data at all: gsm.refreshTime() and gsm.networkTime(). Use that if all
// you need is a clock - it is free, and this example spends bytes.
//
// Needs: nothing beyond ModBee GSM. Change MODEM_SERIAL and the APN below to
// suit your board and your SIM.

#include <ModBeeGsm.h>

#define MODEM_SERIAL Serial1

GsmDevice gsm;
GsmUDP    udp(gsm.sockets());

const char* kNtpHost = "pool.ntp.org";
const uint16_t kNtpPort = 123;

// Seconds between 1900 (NTP's epoch) and 1970 (Unix's).
const uint32_t kNtpToUnix = 2208988800UL;

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
}

void askTheTime() {
    uint8_t request[48] = {0};
    request[0] = 0x1B;          // LI 0, version 3, client mode

    // A MODULE WITHOUT UDP REFUSES HERE, rather than opening something else.
    if (udp.begin(0) == 0) {
        Serial.println("no UDP on this modem");
        return;
    }
    udp.beginPacket(kNtpHost, kNtpPort);
    udp.write(request, sizeof request);
    udp.endPacket();
}

void loop() {
    gsm.poll();
    if (!gsm.isReady()) { return; }

    static bool asked = false;
    if (!asked) { asked = true; askTheTime(); return; }

    const int size = udp.parsePacket();
    if (size < 48) { return; }

    uint8_t reply[48];
    udp.read(reply, sizeof reply);

    // Transmit timestamp: seconds since 1900, big-endian, at offset 40.
    const uint32_t seconds = ((uint32_t)reply[40] << 24) |
                             ((uint32_t)reply[41] << 16) |
                             ((uint32_t)reply[42] << 8)  |
                              (uint32_t)reply[43];

    Serial.print("unix time: ");
    Serial.println(seconds - kNtpToUnix);

    udp.stop();
    asked = false;      // ask again next time round, if you want a clock
}
