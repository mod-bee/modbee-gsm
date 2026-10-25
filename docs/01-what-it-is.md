# 01 — What it is

**modbee-gsm drives an AT-command cellular modem over a serial link, and
produces standard TCP and UDP client interfaces.** That is the whole of it.

An *AT command* is a line of text sent to the modem, beginning with `AT` and
ending in a carriage return: `AT+CSQ` asks for signal strength, and the modem
answers with a line such as `+CSQ: 17,99` followed by `OK`. Every cellular
module speaks this way, but each vendor adds its own commands for the parts
the standards leave out, sockets above all. This library speaks the dialect so
your code does not have to.

```
  your application
        │   Arduino Client / UDP, or GsmSocket
  ┌─────┴──────────┐
  │   modbee-gsm   │
  └─────┬──────────┘
        │   a byte stream
    the modem
```

## Three things that make it different

### Nothing waits on the modem

Opening a socket starts the attempt and returns; a state says how it is going.
The same source runs in an Arduino `loop()`, in a PLC scan cycle and in a Linux
daemon.

The library everyone uses, TinyGSM, funnels every operation through a
busy-wait:

```cpp
do { TINY_GSM_YIELD(); ... } while (millis() - start < timeout_ms);
```

`TINY_GSM_YIELD()` is `delay(0)`. It feeds the watchdog; it does not return to
your code. `gprsConnect()` can hold for sixty seconds, a socket open for
seventy-five. In a sketch that is a stutter. In a scan cycle it is a dead PLC
with frozen outputs.

### Adding a modem is a table, not a `#define`

TinyGSM selects its module with the preprocessor, before the include:

```cpp
#define TINY_GSM_MODEM_SIM7600
#include <TinyGsmClient.h>
```

One firmware, one modem family, chosen at compile time — and a host build that
adapts to whatever is on `/dev/ttyUSB0` cannot be written at all.

Here every registered driver is linked, the module is asked what it is, and the
one that recognises it is installed. See [04 — Adding a modem](04-adding-a-modem.md).

### It contains no hardware

Not a pin, not a board name, not a wiring assumption. Power and reset are
callbacks your application supplies; the library decides only *when* to call
them and how long to wait afterwards, because that comes from the module's
manual rather than from your board. See [06 — Power](06-power.md).

## What it deliberately does not do

**No application protocols.** No HTTP, no WebSocket, no MQTT, no CoAP. It
carries bytes and controls the modem.

That is not an omission, it is the same answer TinyGSM reached: its module list
is Battery, Bluetooth, Calling, GPRS, GPS, GSMLocation, Modem, NTP, SMS, SSL,
TCP, Temperature, Time, Wifi — and its own HTTP example is four lines of
layering:

```cpp
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
TinyGsmClient client(modem);
HttpClient    http(client, server, port);
```

One transport, and the whole ecosystem on top of it. Whatever else TinyGSM got
wrong, this it got right.

**No module protocol stacks.** Both modules this library ships drivers for have
MQTT, HTTP and FTP clients in firmware. We use none of them: they cannot do
MQTT 5.0, cannot be driven non-blocking, and would put the protocol inside a
black box that cannot be tested or fuzzed.

**No TLS by default.** A socket is a byte pipe; whether those bytes are a TLS
record is not its business. `GsmSecurity::Plain` is the default and the library
links no crypto. Module-side TLS is opt-in for boards where a software engine
plus a certificate chain will not fit.

## Everything it says, it can cite

Nothing here is written from memory of AT commands. Every command carries the
manual and section that defines it, in the comment beside it.

That rule has already paid for itself. An early draft was written against the
wrong manual for one of the two modules — it reported four sockets where the
part has ten, and an RSRP conversion wrong by a factor of ten. Every one of
those would have reached a site.
