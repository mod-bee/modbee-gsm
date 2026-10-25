# 02 — Getting started

## Words you will meet

Cellular has its own vocabulary, and the library uses it where there is no
plainer word. Each of these comes up again below.

| | |
| --- | --- |
| **AT command** | A line of text sent to the modem, beginning with `AT`. The modem answers with zero or more lines and then a final `OK` or `ERROR`. |
| **SIM PIN** | A code that locks the SIM card. Most IoT SIMs have none; if yours does, `gsm.pin("1234")`. |
| **Registration** | The network accepting the module onto a tower. Roughly "bars on the phone". Needed for SMS and for data. |
| **APN** | Access point name: which of the operator's gateways the data goes through. Comes from your SIM's documentation. |
| **Bearer** | The data connection itself, also called a PDP context. Brought up after registration, using the APN. Having one means having an IP address. |
| **URC** | Unsolicited result code: a line the modem sends without being asked, such as "an SMS arrived" or "the network went away". |
| **Cat-1, LTE-M, NB-IoT** | Kinds of LTE radio. Cat-1 is ordinary LTE at modest speed; LTE-M (Cat-M1) and NB-IoT are low-power variants that not every network or SIM carries. |

## On a board

```cpp
#include <ModBeeGsm.h>

GsmDevice gsm;

void setup() {
    Serial1.setRxBufferSize(2048);     // see "the buffer" below
    Serial1.begin(115200);

    gsm.apn("internet");               // whatever your SIM's operator wants
    gsm.begin(&Serial1, 115200, 2048);
}

void loop() {
    gsm.poll();                        // the only call that does work

    if (gsm.isReady()) {
        // sockets may be opened
    }
}
```

No module is named anywhere. The library sends `ATI` ("identify yourself")
and the driver that recognises the answer drives the module.

## An HTTP request

The point of this is what is *not* in it — there is no HTTP code, because
`GsmClient` is an Arduino `Client` and ArduinoHttpClient already knows how to
speak HTTP over one.

```cpp
#include <ModBeeGsm.h>
#include <ArduinoHttpClient.h>

GsmDevice  gsm;
GsmClient  net(gsm.sockets());
HttpClient http(net, "example.com", 80);
bool       fetched = false;

void setup() {
    Serial.begin(115200);
    Serial1.setRxBufferSize(2048);
    Serial1.begin(115200);
    gsm.apn("internet");
    gsm.begin(&Serial1, 115200, 2048);
}

void loop() {
    gsm.poll();
    if (gsm.isReady() && !fetched) {
        fetched = true;
        http.get("/");
        Serial.println(http.responseStatusCode());
        Serial.println(http.responseBody());
    }
}
```

PubSubClient, WebSockets and anything else written against `Client` work the
same way. See [05 — Sockets and clients](05-sockets-and-clients.md).

## On a host

The same source builds for Linux, macOS and Windows. `GsmSerialTransport` opens
a serial device directly: a USB modem, a hardware serial port, or a
pseudo-terminal, which looks exactly like a modem from the library's side.

```cpp
GsmSerialTransport link;
link.configure("/dev/ttyUSB0", 115200);

GsmDevice gsm;
gsm.apn("internet");
gsm.begin(&link);

while (!gsm.isReady()) { gsm.poll(); }

GsmSocket* socket = gsm.openTcp("example.com", 80);
socket->begin();                       // starts the open and returns
while (socket->state() == GsmSocketState::Opening) { gsm.poll(); }
```

On a host, waiting in a loop like this is fine; on a board, the same checks
belong in `loop()`. `examples/cpp/` has a complete program that brings the
modem up, opens a socket, sends a line and prints the reply:

```bash
cmake -S . -B build -DGSM_BUILD_EXAMPLES=ON && cmake --build build -j
./build/examples/cpp/socket_on_a_host /dev/ttyUSB0 example.com 80
```

The host and port are whatever TCP server you want to reach over the cellular
link; the modem's own network is on the far side, so a local address such as
`127.0.0.1` means nothing to it.

Nothing about the library changes between the two. The state machines, the
drivers and the socket layer are the same code; only the transport differs.

## The buffer

**Raise your serial receive buffer.** The default on most cores is 256 bytes,
which is about 22 ms of traffic at 115200.

Most modem wiring is three wires. RTS and CTS exist on nearly every module but
are very often not brought out, and on a USB modem there is no hardware
handshake at all — so **the modem cannot be told to wait**. Ask it for 1500
bytes and 1500 bytes arrive, as fast as the line carries them.

An overrun does not lose a reading. It **desynchronises the AT stream**,
because the bytes that went missing were part of a length-delimited payload,
and everything after them is read as the wrong thing until something times out.

The library never asks for more than the transport says it can hold — in fact
slightly less, because the payload arrives wrapped in a header and an `OK` that
land in the same buffer. So a small buffer is *safe*, just slower. A larger one
is simply faster.

**Two separate things, and they must agree.** `setRxBufferSize()` actually
makes the buffer bigger, and only the ESP cores have it — SAMD, STM32 and AVR
compile their size in and cannot be changed at run time. The third argument to
`begin()` is what you *tell* the library the buffer holds:

```cpp
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
    const size_t rxBuffer = 2048;
    MODEM_SERIAL.setRxBufferSize(rxBuffer);
#else
    const size_t rxBuffer = 256;      // whatever your core compiled in
#endif
    gsm.begin(&MODEM_SERIAL, 115200, rxBuffer);
```

Claiming more than you have is the one that breaks. Leave the argument out
entirely and it assumes 256, which is the pessimistic answer and always safe.

This is not speculative: TinyGSM's own HTTP example carries the warning in its
header comment.

## What `isReady()` means

The modem is registered **and** a data bearer is up. Those are different
things: you can be registered with no data context at all, which is exactly
what "full bars, no internet" looks like.

| | |
| --- | --- |
| `isRegistered()` | a tower has accepted us |
| `hasBearer()` | we have an IP address |
| `isReady()` | both |
