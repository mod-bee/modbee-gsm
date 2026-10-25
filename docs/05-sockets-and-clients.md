# 05 — Sockets and clients

There are two doorways, and the difference between them is stated honestly.

```
  ArduinoHttpClient · WebSockets · PubSubClient · Blynk · anything
                      │  Arduino Client / UDP
                    GsmClient · GsmUDP
                      │
                  GsmSocket        ◀──  anything that wants raw bytes
                      │
                  GsmAtEngine
```

## `GsmSocket` — the native form

Non-blocking end to end. `begin()` starts the open and **returns**; `state()`
says how it is going.

```cpp
GsmSocket* socket = gsm.openTcp("example.com", 80);
if (socket == nullptr) { /* every slot is in use */ }

socket->begin();                    // does not wait
while (socket->state() == GsmSocketState::Opening) { gsm.poll(); }

socket->write(data, length);        // buffered; returns what it took
size_t taken = socket->read(buffer, sizeof buffer);
```

Seven methods, and none of them knows what is being carried. That is what
makes it usable by anything — see "Carrying another protocol over it" below.

### It carries opaque bytes and never looks at them

No hex mode, no text mode, no escaping, exact byte counts both ways, NUL
passes through untouched. That is not merely tidy: it is what lets wolfSSL run
*above* this rather than inside it, which is how TLS works on this stack by
default.

### Writes are buffered, and that is not just convenience

One manual states that once a send command has been issued, "when the total
size of the inputted data reaches `<len>`, TA will report the following code.
**Otherwise, the serial port will be blocked.**"

A send issued before its bytes are in hand can wedge the AT channel outright.
So `write()` buffers, and the command is only issued when the whole chunk can
be completed.

### A short send is fatal on one module

> "If the `<cnfSendLength>` is not equal to the `<reqSendLength>`, the socket
> then cannot be used further." — A76XX §15.2.4

It *looks* transient — the buffer was full, try again — and it is not. The
driver concludes `SocketClosed`, and the socket must be reopened rather than
the send repeated. `GsmResult::isRetryable()` encodes it in one place.

### Receiving is polled, never pushed

The modules offer three data access modes, and the vendor names them: **Direct
Push**, **Buffer Access**, **Transparent Access**.

Direct Push — the module's own default — emits data unsolicited as it arrives,
with **no back pressure at all**. On a link with no flow control that
desynchronises the AT stream. So bring-up selects Buffer Access on every
module, and the library fetches what it has room for.

The notification is **edge-triggered**: the module says "data waiting" once,
however much there is. Clearing that on any successful fetch strands the second
half of a large reply; never clearing it fetches forever. The library clears it
when the module says nothing is left — one family reports a remaining count,
and on the other a short read means the same thing.

## `GsmClient` — Arduino's `Client`

A **first-class deliverable**, not a compatibility shim. It is the doorway to
every library already written against Arduino's networking.

```cpp
GsmClient  net(gsm.sockets());
HttpClient http(net, "example.com", 80);     // ArduinoHttpClient, unchanged
PubSubClient mqtt(net);                       // PubSubClient, unchanged
```

**One call blocks, and it is not ours.** `int connect(const char*, uint16_t)`
must return a verdict — there is nowhere in that signature to put "still
trying" — so it pumps `poll()` until the socket opens or its deadline passes.
It is bounded, it is documented, and in normal running it is the only one —
nothing else waits (the exception is `end()`, which gives a shutting-down
module its few documented seconds), and this one exists because Arduino's
signature leaves no room for anything else.

The deadline is 75 seconds by default. `connect(host, port, timeoutSeconds)`
takes a shorter one, and `net.lastError()` says why a connect failed.

Everything after `connect()` — `available()`, `read()`, `write()`, `stop()` —
is non-blocking as usual.

### TLS from the Arduino side

`GsmClientSecure` is a `GsmClient` that asks for the modem's own TLS engine
(`GsmSecurity::Module`). Connect it to 443 or 8883 and the library above it
never knows the difference. A module with no TLS refuses the connect rather
than quietly falling back to plain text.

### `connected()` is deliberately generous

A peer that closed while bytes were still buffered reads as **still
connected**, because those bytes are the answer somebody is waiting for.
Reporting the close first is how the last reply of every exchange that ends in
a hang-up gets thrown away — which is most HTTP responses without a content
length.

## `GsmUDP` — Arduino's `UDP`

A separate class, not a flag on `GsmClient`, and that is deliberate.

Datagrams have boundaries, no ordering and no connection. Squeezing them
through a stream interface loses the boundaries, and a 512-byte datagram
arriving as reads of 300 and 212 is not UDP any more.

```cpp
GsmUDP udp(gsm.sockets());
udp.begin(5000);                         // bind a local port
udp.beginPacket("pool.ntp.org", 123);
udp.write(request, sizeof request);
udp.endPacket();                         // sent whole, or not at all
```

A datagram is assembled before it goes. A payload that does not fit one send
(1400 bytes, or the module's own limit if that is smaller) is refused rather
than split — two halves would be two datagrams, and the far end would read them
as such.

One datagram is in flight at a time. While the previous one is still with the
modem, `endPacket()` returns 0 rather than merging the new one into it or
sending it to the old one's address; call it again on a later pass of `loop()`.

## Carrying another protocol over it

`GsmSocket` is a byte pipe, and **this library knows nothing about what travels
through it.** That is deliberate: a GSM library that included an MQTT header
could not be built without MQTT, and would then need a header for every protocol
anybody ever wanted.

So the adaptation belongs on the other side. Seven methods is all a protocol
library needs, and they are the shape most byte-channel interfaces already have:

```cpp
GsmSocket* socket = gsm.openTcp("broker.example.com", 8883, GsmSecurity::Module);
socket->begin();                          // starts the open and RETURNS

socket->readable();                       // what has arrived
socket->read(into, length);               // take it, never blocks
socket->write(from, length);              // queue it, never blocks
socket->isConnected();
socket->peerClosed();                     // not the same question
socket->end();
```

`begin()` returns false when the open is refused at once (no free slot at the
module, a TLS setup the module turned down); a refusal that comes later shows
as `state()` reaching `Failed`, with the reason in `lastError()`. `end()` on a
socket with a command still out at the modem moves it to `Closing`, and its slot
comes back once the modem has answered — until then that socket id is still in
use at the module, and reusing it would hand the old connection's bytes to the
new one.

A protocol library writes a twenty-line class in front of that and is
non-blocking end to end — which is the point of using `GsmSocket` rather than
`GsmClient`, whose `connect()` has to wait because Arduino's interface demands
a verdict.

## How many sockets

`setCapacity()` (default 2 sockets, 1500 bytes each way) is what this device
holds — set before `begin()`, taken then, and refused by `begin()` with
`beginError()` when the board has no room. The profile is what the module could
manage, and the smaller wins. A profile may declare thirteen, and thirteen times
two 1500-byte buffers is 39 kB that few small boards have spare.

```cpp
gsm.setCapacity(4, 1500, 1500, 0);   // four sockets; the AT queue at its default
```

```cpp
gsm.socketsAvailable()          // free slots in this build
gsm.profile()->maxSockets       // what the module itself allows
```

Asking for one more than either is refused **by name**, at the moment you ask —
not by a timeout twenty seconds later.
