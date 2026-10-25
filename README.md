# ModBee GSM

**Cellular modems over AT commands, for Arduino and for hosts. Nothing blocks,
and no hardware lives here.**

A cellular modem is a separate computer on the end of a serial cable. You talk
to it by sending lines of text that begin with `AT` (an "AT command"), and it
answers in lines of text. This library does that talking for you, and hands
back the network interfaces your code already knows: Arduino's `Client` and
`UDP`, so a library written for WiFi or Ethernet works over cellular unchanged.

```
 library:  esp32 · esp8266 · samd · mbed · rp2040 · stm32 · Linux · macOS · Windows
 modules:  SIMCom A76XX (LTE Cat-1) · SIMCom SIM707x (LTE-M / NB-IoT)
```

> **Status: 0.1.0, working on real hardware.** The A76XX driver has been
> brought up on an A7670G over a live network: registration, a data bearer,
> TCP, module TLS, SMS and a clean five-minute soak. On an ESP32-S3, `poll()`
> measured a mean of 24 µs and a worst case of 1.2 ms across five million
> calls.
>
> The SIM707x driver is complete and has been checked against recorded
> sessions from a real SIM7070G, but has not yet carried traffic over a live
> LTE-M network. Treat it as less proven than the A76XX driver until it has.

---

## What it is

```cpp
GsmDevice gsm;
GsmClient net(gsm.sockets());

HttpClient   http(net, "example.com", 80);    // ArduinoHttpClient, unchanged
PubSubClient mqtt(net);                       // PubSubClient, unchanged

GsmUDP udp(gsm.sockets());
udp.beginPacket("pool.ntp.org", 123);         // NTP, syslog, anything
```

`GsmClient` and `GsmUDP` implement Arduino's own `Client` and `UDP` base
classes, so libraries written for WiFi or Ethernet work over cellular with no
source change. On a host, `GsmSocket` is the same thing without Arduino.

For TLS, swap one word. The handshake happens inside the modem, so it costs
no flash and no RAM on the controller:

```cpp
GsmClientSecure net(gsm.sockets());           // and connect to 8883, or 443
```

There is no `GsmServer`. Accepting inbound connections needs a fixed-IP APN,
which ordinary IoT SIMs do not give you, and none of the established modem
libraries implements it either. `profile()->hasTcpServer` tells you whether the
module could, and `sendAt()` lets you drive it yourself.

## The three things that make it different

**Nothing waits on the modem.** Opening a socket starts the attempt and
returns; a state tells you how it is going. The same source runs in an Arduino
`loop()`, in a PLC scan cycle and in a Linux daemon. The library it most
resembles, TinyGSM, funnels every operation through a busy-wait that can hold
for sixty seconds, which is a stutter in a sketch and a dead PLC in a scan
cycle.

**Adding a modem is a table, not a `#define`.** Every AT string lives in
`src/drivers/`; the state machines, the queue, the retry policy and the
one-conclusion contract are written once and inherited. One firmware can carry
several drivers and pick at run time from what the module says it is (its
reply to `ATI`, the "identify yourself" command), so a host build adapts to
whatever is plugged in. Nothing outside `src/drivers/` knows a modem family
exists.

**It contains no hardware at all.** There is no pin number, no board name and
no product name anywhere in the core. Power and reset are callbacks your
application supplies; the library decides only *when* to call them and how
long to wait afterwards, because that comes from the module's manual rather
than from your board.

## Quick start

```cpp
#include <ModBeeGsm.h>

GsmDevice gsm;

void setup() {
    Serial1.setRxBufferSize(2048);     // ESP32 / ESP8266 only; see docs/02
    Serial1.begin(115200);

    gsm.apn("internet");               // your operator's APN - see below
    gsm.begin(&Serial1, 115200, 2048); // returns immediately
}

void loop() {
    gsm.poll();                        // the only call that does work

    if (gsm.isReady()) {
        // registered, with a data connection: sockets may be opened
    }
}
```

The **APN** (access point name) is the name of your operator's gateway to the
internet. It is printed in your SIM's documentation or on the operator's
website; `"internet"` is a common value but not a universal one, and the wrong
APN is the commonest reason for "registered, but no data".

No module is named anywhere. The library asks the module what it is and picks
the driver that recognises it. [Getting started](docs/02-getting-started.md)
walks through the rest, including the serial buffer setting that matters most.

## Supported hardware

| Driver | Modules | Radio | Status |
| --- | --- | --- | --- |
| `GsmDriverA76XX` | SIMCom A7670, A7672, A7678, A7608 | LTE Cat-1 | Proven on an A7670G over a live network |
| `GsmDriverSIM707x` | SIMCom SIM7070, SIM7080, SIM7090 | LTE-M and NB-IoT | Checked against recorded sessions; not yet on a live network |

Both drivers are compiled into every build and chosen at run time. A module
neither recognises is refused by name rather than driven with the wrong
dialect; [adding a modem](docs/04-adding-a-modem.md) explains how to write a
driver for it.

| Platform | |
| --- | --- |
| **Arduino** | 32-bit boards. `library.properties` declares `esp32`, `esp8266`, `samd`, `mbed`, `rp2040` and `stm32`. The modem is handed any `Stream` you have opened (`Serial1`, a `HardwareSerial` on chosen pins). |
| **Linux, macOS, Windows** | CMake 3.16 or later. `GsmSerialTransport` opens a POSIX tty or a Windows COM port: a USB modem, a hardware serial port, or a pseudo-terminal. |

The library is C++11 throughout, because several Arduino cores are (SAMD among
them), and a library that needed anything newer would simply not build on the
boards it claims to support. The host build itself uses C++17 for its examples.

## Install

### Arduino IDE

Download this repository as a ZIP and add it with **Sketch → Include Library →
Add .ZIP Library…**, or clone it into the `libraries` folder of your sketchbook.
Then:

```cpp
#include <ModBeeGsm.h>
```

There is nothing else to configure. The sketches in `examples/arduino` appear
in the IDE under **File → Examples → ModBee GSM**.

### CMake

The library builds as a static library with the target `ModBee::gsm`:

```cmake
set(GSM_BUILD_EXAMPLES OFF)
add_subdirectory(modbee-gsm)

target_link_libraries(my_app PRIVATE ModBee::gsm)
```

| Option | Default | |
| --- | --- | --- |
| `GSM_BUILD_EXAMPLES` | `ON` | build the host example in `examples/cpp` |
| `GSM_SANITIZE` | `OFF` | build with the address and undefined-behaviour sanitizers |
| `GSM_SANITIZE_THREAD` | `OFF` | build with the thread sanitizer (cannot be combined with the one above) |
| `GSM_NO_EXCEPTIONS` | `OFF` | build without exceptions or RTTI, as a wrapper would |

To build it on its own:

```bash
cmake -S . -B build && cmake --build build -j
./build/examples/cpp/socket_on_a_host /dev/ttyUSB0 example.com 80   # any TCP server you can reach
```

## Examples

| | |
| --- | --- |
| [GsmHttpClient](examples/arduino/GsmHttpClient) | An HTTP request through ArduinoHttpClient, with no HTTP code in the sketch |
| [GsmMqtt](examples/arduino/GsmMqtt) | MQTT through PubSubClient |
| [GsmMqttTls](examples/arduino/GsmMqttTls) | The same over TLS, encrypted by the modem |
| [GsmUdpNtp](examples/arduino/GsmUdpNtp) | One UDP datagram each way: asking an NTP server the time |
| [GsmSmsAndGnss](examples/arduino/GsmSmsAndGnss) | Text messages and satellite position, with no data connection at all |
| [GsmPollTiming](examples/arduino/GsmPollTiming) | How long `poll()` really takes on your board; needs no SIM |
| [`examples/cpp`](examples/cpp) | A TCP socket from a Linux host, over a USB modem |

`GsmHttpClient` needs the **ArduinoHttpClient** library, and `GsmMqtt` and
`GsmMqttTls` need **PubSubClient**; both are in the Arduino Library Manager.
The others need nothing beyond this library. Every example uses `Serial1` for
the modem and the placeholder APN `"internet"`: change both to suit your board
and your SIM.

## Documentation

| | |
| --- | --- |
| [01 — What it is](docs/01-what-it-is.md) | And what it deliberately is not |
| [02 — Getting started](docs/02-getting-started.md) | On a board or a host, and the words you will meet |
| [03 — Architecture](docs/03-architecture.md) | Five layers, and why each boundary is where it is |
| [04 — Adding a modem](docs/04-adding-a-modem.md) | A table, not a rewrite |
| [05 — Sockets and clients](docs/05-sockets-and-clients.md) | `GsmSocket`, `GsmClient`, `GsmUDP` |
| [06 — Power](docs/06-power.md) | Hooks your application implements, and not one pin |
| [07 — Troubleshooting](docs/07-troubleshooting.md) | What each failure actually means |
| [API](docs/api.md) | Every public call |

## Everything it says, it can cite

Nothing here is written from memory of AT commands. Every command carries the
manual and section that defines it, in the comment beside it.

That rule has already paid for itself. An early draft was written against the
wrong manual for one of the two modules: it reported four sockets where the
part has ten, and an RSRP (signal strength) conversion that was wrong by a
factor of ten. Every one of those would have reached a site.

## Licence

GNU Affero General Public License, Version 3, **with additional ModBee terms**
(see [LICENSE](LICENSE)). Use is non-commercial; **commercial use requires a
separate licence**: contact info@modbee.org.

The source deliberately carries no SPDX identifier: the additional terms mean
this is not plain AGPL-3.0, and stamping that identifier on it would tell
automated tooling the opposite of what LICENSE says.
