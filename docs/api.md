# API

Everything public, in the order you meet it.

```cpp
#include <ModBeeGsm.h>
```

---

## `GsmDevice` — the one object

### Opening

```cpp
bool begin(GsmTransport* transport);                  // host, or a test

template <typename StreamType>                        // Arduino
bool begin(StreamType* stream, uint32_t bitsPerSecond = 115200,
           size_t receiveCapacity = 256);

void end();                                           // orderly power-down
```

`receiveCapacity` is the size of your core's serial receive buffer, and it
matters — see [02](02-getting-started.md#the-buffer). The default is the
pessimistic one. The Arduino form does not open the port: call
`Serial1.begin()` yourself first.

`begin()` returns false when the link will not open, or when the memory
`setCapacity()` asked for cannot be found; `beginError()` then says which.

`end()` sends the module its own power-down command and gives it, at most, the
module's documented settling time to answer before calling your `powerOff()`
hook. It is the one call that may spend a few seconds; see
[06](06-power.md#the-ordering-rule-that-destroys-hardware-if-you-get-it-wrong).

### Configuration

```cpp
bool apn(const char* name, const char* user = nullptr,
         const char* password = nullptr, GsmAuth auth = GsmAuth::Auto);
                                                      // APN up to 100 characters (3GPP),
                                                      // user and password up to 127; false,
                                                      // and nothing changed, when longer
void setCapacity(uint8_t sockets, size_t socketTxBytes, size_t socketRxBytes,
                 size_t commands);                    // before begin(); 0 = default
const char* beginError() const;                       // "out of memory for the sockets" /
                                                      // "... for the AT command queue"
void pin(const char* pin);                            // the SIM's PIN, if it has one
void powerControl(GsmPowerControl* hooks);            // optional; see 06
bool addDriver(GsmDriver* driver);                    // a module we do not ship

void radioPreference(GsmRadioPreference wanted);      // Automatic, GsmOnly, LteOnly,
                                                      // GsmAndLte, CatM, NbIot, CatMAndNbIot
void bands(const GsmBandList& catM, const GsmBandList& nbIot);
void operatorCode(const char* numericCode);           // MCC+MNC, e.g. "53001"; null = automatic
GsmResult clearForbiddenNetworks();
```

The APN, user and password are **copied, not borrowed** — a sketch may pass a
temporary, and the bearer is brought up again on every reconnect.

`addDriver()` returns false for a driver already registered, so calling it
twice costs nothing, and false when the registry is full (`GSM_DRIVERS_MAX`).
An added driver is asked **after** the two shipped ones; see
[04](04-adding-a-modem.md#writing-one).

**`radioPreference()`** matters on an LPWA module (LTE-M / NB-IoT): one
searching for a technology that is not on air never registers and never says
why. Left alone, the module keeps its own setting. **`bands()`** narrows the
search to the 3GPP band numbers a site actually uses; a `GsmBandList` is a
pointer to an array of band numbers and a count, and an empty list leaves the
module's configuration alone. Naming one or two bands turns a cold search of
minutes into seconds.

**`operatorCode()` is not a default.** Left null, the module searches — which is
what a controller on a pole needs, because pinning it to one operator means a
site where that operator is absent simply never connects, and nothing about the
failure says why. Name one to shorten a known cold start.

**`clearForbiddenNetworks()`** empties `EF_FPLMN`, the list of networks that
have refused this SIM. The list lives **on the card** and survives power
cycles, so a handful of rejected attempts can leave a module skipping the only
operator that would have worked, for good. The library clears it by itself when
registration will not happen; this is the manual lever. If naming an operator
works and automatic does not, this is why.

### Driving

```cpp
void poll();       // the only call that does work; never blocks
void start();
void stop();
```

### State

```cpp
bool isReady()        const;   // registered AND a bearer is up
bool isRegistered()   const;
bool hasBearer()      const;
GsmState state()      const;
const char* stateName() const;

const char* module()        const;   // the driver's family, once identified
const GsmIdentity& identity() const; // IMEI, ICCID, model, revision
const GsmSignal& signal()   const;
GsmRegistration registration() const;
GsmTechnology technology()  const;
const char* operatorName()  const;
const char* ipAddress()     const;

GsmResult lastError()       const;
const char* lastErrorText() const;   // the module's own words, never a number
const GsmModem::Counters& counters() const;
void refreshStatus();                // superseded; safe every scan
```

The same facts one value at a time, because a PLC function-block pin takes a
scalar and a wrapper is easiest to build from this surface:

```cpp
const char* imei()     const;
const char* iccid()    const;
const char* imsi()     const;
const char* firmware() const;

int rssiDbm() const;                 // GSM_UNKNOWN_DBM if never read
int rsrpDbm() const;
int rsrqDb()  const;
GsmQuality signalQuality() const;
```

An unread reading returns `GSM_UNKNOWN_DBM`, **not zero** — zero dBm is a real
and excellent signal level, so reporting "nobody has asked" as zero is how an
alarm built on it never fires.

### What the radio is actually set to

```cpp
const GsmRadioConfig& radioConfig() const;
GsmDiagnostics& diagnostics();       // signal quality bands, radio read-back
```

`radioConfig()` reports what the module says, not what was asked for. Mode,
technology and bands are saved in the module's own non-volatile memory, so a
value left by a previous firmware or a diagnostic session outlives every power
cycle. Read once per bring-up, after the library has finished configuring it.

### Network time

```cpp
GsmResult refreshTime();             // queued, non-blocking
bool hasNetworkTime() const;
const GsmParse::GsmDateTime& networkTime() const;
gsm_time_t networkTimeAgeMillis() const;
```

The network sets the module's clock at registration, so this needs no NTP, no
socket and no bearer — it is the only time source available to a device that
is registered but has no data. Asked for, never polled.

### What `poll()` costs

```cpp
void   setMaxBytesPerPoll(size_t bytes);
size_t maxBytesPerPoll();
gsm_time_t serviceIntervalUs() const;
gsm_time_t serviceIntervalMs() const;
```

The library promises **bounded work** per `poll()`, not a wall-clock figure —
the same work is ~1150 µs on a 240 MHz ESP32-S3 and several times that on a
48 MHz part, so one number cannot be a contract across every target.

At most `maxBytesPerPoll()` bytes come off the link per call, at most one
command is submitted, and no loop is sized by anything the modem chose.
Measure it once on your board and budget for it; if that is still too much for
your scan cycle, turn the budget down and call `poll()` more often — same
throughput, smaller bites.

`serviceIntervalUs()` is how often the application actually calls `poll()`,
smoothed. **Every timeout in the library is floored by it**, so a caller
polling slower than a timeout does not manufacture timeouts out of its own
scheduling.

### Sockets

```cpp
GsmSocket* openTcp(const char* host, uint16_t port,
                   GsmSecurity security = GsmSecurity::Plain);
GsmSocket* openUdp(uint16_t localPort);
void close(GsmSocket* socket);

uint8_t socketsOpen()      const;
uint8_t socketsAvailable() const;
```

Returns `nullptr` when every slot is in use, or when the module cannot do what
was asked — refused at the moment you ask, not by a timeout later.

### SMS

```cpp
GsmResult sendSms(const char* number, const char* text,
                  GsmDoneFn done = nullptr, void* context = nullptr);
void onSms(GsmSmsHandler handler, void* context);

typedef void (*GsmSmsHandler)(void* context, const GsmSmsMessage& message);
typedef void (*GsmDoneFn)(void* context, GsmResult result);
```

Refused rather than truncated: text outside the default alphabet, or longer
than one message, returns `InvalidArgument`. SMS needs registration but no
data bearer; location needs neither.

A `GsmSmsMessage` carries `sender`, `text`, `textLength`, `timestamp` (with
`hasTimestamp`) and `alphabet`. A message that is one part of several says so
with `isPart`, `reference`, `partIndex` (from 1) and `partCount`; reassembling
the parts is left to the application.

The SMS layer is attached by the first call that uses it, so a program that
never sends or reads a message links none of the SMS code. Until then an
incoming message is left on the SIM, unread.

`onSms()` stays in force across module restarts. A module that reboots forgets
that it was asked to announce new messages, so after every restart the device
asks it again before `sms().isReady()` reports true; `isReady()` means
"configured in this life of the module", not "configured once".

### Reading and deleting messages

```cpp
GsmResult readSms(uint16_t index);
GsmResult deleteSms(uint16_t index);
uint32_t  smsReceived() const;
uint32_t  smsSent()     const;
```

### TLS

```cpp
GsmSocket* openTcp(const char* host, uint16_t port,
                   GsmSecurity security = GsmSecurity::Plain);
```

`GsmSecurity::Plain` is the default and the library does no crypto — a socket
is a byte pipe, which is what lets wolfSSL run **above** it. That is the
recommended arrangement, since `modbee-mqtt` already carries wolfSSL on both
host and Arduino.

`GsmSecurity::Module` uses the **modem's own** TLS engine, for boards where a
software engine plus a certificate chain will not fit. The two families
implement it completely differently and the application cannot tell:

| | SIM707x | A76XX |
| --- | --- | --- |
| Shape | a flag on an ordinary socket | a **separate command family** |
| Setup | `AT+CSSLCFG` context, then `AT+CASSLCFG=<cid>,"SSL",1` | `AT+CCHSET`, `AT+CCHSTART`, `AT+CCHSSLCFG` |
| Open | `AT+CAOPEN`, unchanged | `AT+CCHOPEN` |
| Send / fetch / close | `AT+CASEND` / `CARECV` / `CACLOSE` | `AT+CCHSEND` / `CCHRECV` / `CCHCLOSE` |
| Sessions | 13, the same as plain | **2** (`+CCHOPEN: (0,1)`) against 10 plain |
| Send limit | 1460, the same as plain | **2048**, larger than the 1500 plain allows |

Asking for more TLS sessions than the module has is refused **when you ask**,
not by an open that fails twenty seconds later. Asking for module TLS on a
module that has none is refused the same way.

### Location

**Ask before you wait.** `hasGnssReceiver()` reports what the module actually
answered, not what the family can do — GNSS is an order-code option on the
A7670 (`-F` has it, `-L` does not), so two modules with the same model string
differ. `profile()->hasGnss` means only "the driver knows these commands";
`hasGnssReceiver()` means "this one replied".

```cpp
bool hasGnssReceiver() const;

GsmResult gnss(bool on);
GsmResult refreshFix();
bool hasFix() const;
const GsmFix& fix() const;
gsm_time_t fixAgeMillis() const;      // the number to build an alarm on
```

`GsmFix` carries `latitude` and `longitude` in signed decimal degrees,
`altitudeMetres`, `speedKnots`, `courseDegrees`, `satellitesInView`, `hdop`
and `utc`, each with a `has...` flag where the receiver may leave it out, and
a `quality`. A receiver that has lost the sky keeps reporting its last
position perfectly happily, which is why `fixAgeMillis()` is the number to
alarm on.

### The escape hatch

```cpp
GsmResult sendAt(const char* command, uint32_t timeoutMs,
                 GsmReplyFn onReply, GsmDoneFn onDone, void* context);

typedef void (*GsmReplyFn)(void* context, const char* line, size_t length);
```

`onReply` is handed each reply line as it arrives; `onDone` is called once,
with the conclusion.

There is always one more vendor command. Queued, timed and concluded like
anything else. A library that forced a fork to send one would be forked.

### Nothing is a wall

```cpp
GsmModem&    modem();
GsmSockets&  sockets();
GsmSms&      sms();
GsmGnss&     gnssLayer();
GsmAtEngine& engine();
GsmDriver*   driver()  const;
const GsmProfile* profile() const;    // what THIS module can do
```

---

## `GsmSocket`

```cpp
bool   begin();                       // starts the open and RETURNS; false if refused at once
void   end();                         // slot returns once the modem has answered
size_t readable();
size_t read(uint8_t* into, size_t length);
size_t write(const uint8_t* from, size_t length);
bool   isConnected() const;
bool   peerClosed()  const;

GsmSocketState state() const;         // Closed, Opening, Open, Closing, Failed
GsmResult lastError()  const;
size_t pendingOut()    const;
```

---

## `GsmClient` / `GsmUDP` *(Arduino)*

Implement Arduino's own `Client` and `UDP`. Drop-in for `WiFiClient`,
`WiFiUDP` and their Ethernet equivalents.

**There is no `GsmServer`.** Arduino's `Server` is not implemented, and the
omission is deliberate rather than pending: an inbound connection needs a
fixed-IP APN, which ordinary IoT SIMs do not issue — they are typically handed
an address behind the operator's NAT, which nothing outside can reach. None of
the established modem libraries implements a server either, so there is
nothing proven to adapt, and a state machine written from the manuals alone
and never exercised is exactly the kind of code that fails in the field.

`profile()->hasTcpServer` reports whether the module could do it
(`AT+SERVERSTART` on A76XX, `AT+CASERVER` on SIM707x), and `sendAt()` will
send those if you have an APN that makes them useful.

```cpp
GsmClient       net(gsm.sockets());       // plain TCP
GsmClientSecure tls(gsm.sockets());       // the modem's own TLS
GsmUDP          udp(gsm.sockets());

int connect(const char* host, uint16_t port);                       // 75 s deadline
int connect(const char* host, uint16_t port, int timeoutSeconds);
GsmResult  lastError() const;             // why the last connect or socket failed
GsmSocket* socket()    const;             // the socket underneath, or null
```

`GsmClient::connect()` blocks — bounded and documented — because Arduino's
interface demands a verdict. Everything else does not. See
[05](05-sockets-and-clients.md).

---

## `GsmResult`

Four bytes. `{status, space, code}`.

```cpp
GsmStatus status() const;
GsmErrorSpace space() const;
uint16_t code() const;

bool isOk()          const;
bool isConcluded()   const;   // Queued is NOT a conclusion
bool isRetryable()   const;
const char* describe() const;              // the library's own words
const char* describeStandardCode() const;  // +CME / +CMS only, or null
```

`GsmStatus`: `Queued`, `Ok`, `NotInitialised`, `InvalidArgument`, `NotReady`,
`NoBearer`, `NoSocketAvailable`, `QueueFull`, `Superseded`, `Cancelled`,
`TooLarge`, `NotSupportedByModule`, `Error`, `ModuleError`, `Timeout`,
`SocketClosed`, `BearerLost`, `Unresponsive`, `PoweredOff`,
`RegistrationDenied`.

`GsmErrorSpace`: `None`, `Cme`, `Cms`, `SocketInfo`, `SocketErr`,
`ConnResult`, `Driver` — see
[03](03-architecture.md#error-spaces-kept-apart).

A vendor code is never rendered by the core — only the driver that owns that
table can say what it means, and guessing would be worse than saying nothing.

---

## `GsmProfile` — what this module can actually do

Asked, never assumed.

```cpp
const GsmProfile* p = gsm.profile();      // null until the module is identified
p->family;
p->maxSockets;  p->maxSendChunk;  p->maxRecvChunk;
p->maxTlsSessions;  p->maxTlsSendChunk;  p->tlsStyle;  p->sendAccounting;
p->hasUdp;  p->hasTcpServer;  p->hasGnss;  p->hasPsm;  p->hasEdrx;
p->hasMultiContext;
p->registrationTimeoutMs;  p->socketOpenTimeoutMs;  p->unreadCheckMs;
p->powerOnToUartMs;  p->powerKeyOnPulseMs;  p->powerKeyOffPulseMs;
p->hasRsrpConversion;  p->rsrpOffsetDbm;
```

Every field is commented in `src/GsmProfile.h`, with the manual section its
value comes from in each driver.

---

## `GsmPowerControl`

```cpp
virtual bool setPowerKey(bool asserted);     // preferred: move a line, return
virtual bool setSupply(bool on);
virtual bool setReset(bool asserted);

virtual bool powerOn();                      // fallback, for a board with no key
virtual bool powerOff();
virtual bool reset();

virtual GsmTristate isPoweredOn() const;     // Unknown, No, Yes
```

Seven hooks, all optional; each returns false for "this board cannot". The
library owns the order and the deadlines, including how long the power key is
held; your application owns the pins. See [06](06-power.md).

---

## Lower layers

All public, all documented in their headers.

| | |
| --- | --- |
| `GsmModem` | the bring-up state machine and the recovery ladder; `stateBudgetMs(state)` is each state's deadline, `moduleEpoch()` moves on every time the module starts afresh |
| `GsmAtEngine` | the queue; `submit()` a `GsmCommand` directly |
| `GsmUrcRouter` | register a prefix and a handler |
| `GsmAtParser` | the four-mode scanner; no I/O, no clock |
| `GsmParse` | fields, numbers, addresses, timestamps — refuses, never guesses |
| `GsmSmsPdu` | TS 23.040 / 23.038, pure |
| `GsmTransport` | the byte channel to the modem |
| `GsmDriver` | a dialect; see [04](04-adding-a-modem.md) |

Four of them are reachable straight off the device, and each returns the **live
object** rather than a copy:

```cpp
GsmNetwork&     network();        // registration, operator, technology, bands
const GsmApn&   bearer() const;   // the data context as configured
GsmClock&       clock();          // the module's clock; see "Network time" above
GsmDiagnostics& diagnostics();    // signal, quality thresholds, radio read-back
```

---

## Capacity, set at run time

Taken once, in `begin()`, from `setCapacity()` — on an ESP32 in PSRAM first —
and never on the poll path. Too much for the board makes `begin()` fail with
`beginError()` saying which; it never carries on with less.

| `setCapacity()` | Default | |
| --- | --- | --- |
| `sockets` | 2 (`GSM_SOCKET_SLOTS`) | sockets open at once; the module's own limit still applies |
| `socketTxBytes` / `socketRxBytes` | 1500 (`GSM_SOCKET_TX_BUFFER` / `_RX_`) | per socket |
| `commands` | 12 (`GSM_QUEUE_CAPACITY`) | AT commands held at once |

`GsmUDP(sockets, packetBytes)` takes its datagram size the same way (default
`GSM_UDP_PACKET_MAX`, 512); its `begin()` returns 0 when there is no room.

## Build-time knobs

Every one has a default that works. None selects a modem — that is decided at
run time.

| | Default | |
| --- | --- | --- |
| `GSM_AT_COMMAND_MAX` | 192 | room for each queued command |
| `GSM_AT_COMMAND_LONG_MAX` | 384 | the one buffer for a longer command (the SIM707x bearer set-up with a long APN and credentials); a second long one waits its turn as `QueueFull` |
| `GSM_APN_MAX` / `_USER_MAX` / `_PASSWORD_MAX` | 100 / 127 / 127 | the APN, user name and password held |
| `GSM_AT_LINE_MAX` | 512 | longest reply held in full |
| `GSM_POLL_BYTE_BUDGET` | 256 | bytes drained per `poll()` |
| `GSM_URC_HANDLERS_MAX` | 16 | |
| `GSM_DRIVERS_MAX` | 6 | |
| `GSM_UDP_PACKET_MAX` | 512 | one datagram, unless the constructor says otherwise |
