# 03 — Architecture

Five layers, and why each boundary is where it is.

This is the document to read if you want to know *why* the library is shaped
the way it is, or if you are about to reach past the one-object API into the
layer underneath.

```
 ┌──────────────────────────────────────────────────────────────────────────┐
 │ L5  API       GsmDevice           one object, one poll(), hides nothing  │
 ├──────────────────────────────────────────────────────────────────────────┤
 │ L4  SERVICES  GsmSockets/GsmSocket   the byte pipe — the product         │
 │               GsmClient · GsmUDP     Arduino's own interfaces over it    │
 │               GsmSms · GsmGnss                                           │
 ├──────────────────────────────────────────────────────────────────────────┤
 │ L3  SESSION   GsmModem           power → sync → SIM → network → bearer   │
 ├──────────────────────────────────────────────────────────────────────────┤
 │ L2  ENGINE    GsmAtEngine        one in flight, queued, concluded once   │
 │               GsmUrcRouter       unsolicited lines, routed out of band   │
 ├──────────────────────────────────────────────────────────────────────────┤
 │ L1  PROTOCOL  GsmAtParser · GsmParse · GsmSmsPdu   no I/O, no clock      │
 ├──────────────────────────────────────────────────────────────────────────┤
 │ L0  LINK      GsmTransport       one byte channel TO the modem           │
 │               GsmPowerControl    power hooks, and not one pin            │
 └──────────────────────────────────────────────────────────────────────────┘

      DRIVERS    GsmDriver + GsmProfile + src/drivers/*
                 the only files in the library containing AT text
```

Dependencies point downward only.

---

## The two byte channels point opposite ways

This library has one at each end, and confusing them would be fatal — so only
one of them is called a transport.

```
  application
       ▲  GsmSocket        UP: the shape MqttTransport expects. We PRODUCE it.
  ┌────┴──────────┐
  │  modbee-gsm   │
  └────┬──────────┘
       ▼  GsmTransport     DOWN: Arduino Stream, POSIX tty, PTY, loopback.
   the modem                   We CONSUME it.
```

`modbee-gsm` produces exactly what `modbee-mqtt` consumes. That is the point of
the family, and it is why `GsmSocket` is shaped to `MqttTransport` rather than
to Arduino's `Client` — `Client::connect()` must return a verdict and therefore
cannot be non-blocking.

---

## L1 has no I/O and no clock

`GsmAtParser`, `GsmParse` and `GsmSmsPdu` are fed bytes and emit answers. They
never read the clock and never touch a link.

That is not tidiness. It is what makes them **exhaustively fuzzable in
isolation**, which is where parser defects live — and everything in this layer
parses bytes we did not produce, over a link that can corrupt them, at a baud
rate that might be wrong. A modem at the wrong rate does not send an error; it
sends plausible-looking rubbish, forever.

### The AT channel is not a stream of lines

Every library that treats it as "text, delimited by CRLF" eventually corrupts
data, and the corruption is silent. The channel is in one of four modes, and
the parser must be **told** which:

| Mode | The bytes are | Ends on |
| --- | --- | --- |
| Line | text, CRLF delimited | a terminator |
| Prompt | waiting for a bare `>` | the `>` |
| Payload-in | exactly N raw bytes from the modem | N bytes |
| Payload-out | exactly N raw bytes to the modem | N bytes |

**In payload mode it counts bytes and looks at nothing.** A socket payload is
opaque — it can contain CR, LF and zero — and a TLS record contains all three
within its first few hundred bytes essentially always. A line-scanning parser
cuts such a payload into pieces at every CRLF inside it and reads the remains
as replies. Nothing about that is visible until something binary goes through,
which on a plant is the first TLS handshake at three in the morning.

### The channel is also asymmetric

A **command** ends with a bare CR. A **reply** ends with CRLF. That is ITU-T
V.25ter, not a vendor quirk.

`GsmAtParser` reads the reply direction — the only one a library needs, since
it writes commands rather than parsing them. So a bare CR is *dropped* rather
than treated as a terminator, because inside a reply a lone CR appears in some
modules' informational text.

---

## L2: one command in flight, and the manuals require it

> "you need to wait the final response (for Examples OK, CME error, CMS error)
> of last AT Command you entered before you enter the next AT Command."
> — A76XX §1.5.2, SIM707x §1.4.5

So the engine serialises, and the queue makes that invisible. This is not a
conservative choice; it is the documented contract. Two commands at once
produce interleaved replies that no parser can untangle.

### The contract

**Every accepted command reaches exactly one conclusion.** Not "usually". Not
"unless the modem reset". A command that is accepted and then silently never
concluded does not look like a bug — it looks like a cellular link that quietly
stopped publishing, weeks after commissioning, on a site nobody can drive to.

What the queue gives you with no configuration: **stacked** (nothing is sent
inside the call that asked for it), **ordered**, **superseded** (a query that
would replace one still queued replaces it, and the replaced caller is *told*),
and **concluded exactly once**.

### URCs are routed before anything else

A URC is a line the modem sends when nobody asked — `RING`, `+CMTI:` for an
SMS, `+CEREG: 0` when the network goes. **They arrive at any moment, including
in the middle of a reply.**

That is the single thing naive AT libraries get wrong, and it produces the
classic defect: the application asked for signal strength, an SMS arrived at
that instant, and the parser read `+CMTI: "SM",3` as the answer. The signal
reading is nonsense and the SMS is lost.

The fix is one line of code in the right place — every complete line is offered
to the router *first*.

**A handler may decline.** On one module the same prefix is both a URC and a
reply (`+CIPRXGET: 1,<link>` is unsolicited, `+CIPRXGET: 2,...` is an answer),
told apart only by a field value. And `+CEREG:` is *both* on every module — the
read reply carries `<n>` first, the unsolicited form does not. A router that
claimed the reply form would swallow the answer to the registration query and
the bring-up machine would wait forever for something it had already been sent.
That happened; the decline mechanism is why it does not now.

### Two-stage conclusions are not an edge case

`AT+CIPOPEN` returns `OK` and then, up to 120 seconds later, `+CIPOPEN:
<link>,<err>` with the real answer. `AT+NETOPEN` does the same. **`OK` is not
the conclusion**, and an engine that treated it as one would report every
socket as open the instant it was asked for.

So a command may declare a deferred prefix. It concludes on that line, with its
own deadline — and it **holds the queue while it waits**. That is deliberate:
two sockets opening at once would both be waiting for a `+CIPOPEN:` line, and
only the driver can tell whose it is. An engine that guessed would conclude the
wrong socket. The deadline bounds the wait, so a reply that never comes costs
time, never a wedged queue.

---

## L3 is re-entrant, and every state has an exit

```
 Off → PoweringUp → Syncing → Identifying → Configuring → SimWait
     → ReadingIdentity → NetworkSetup → Registering → BearerUp → Ready

 any state that runs out of options → Recovering → (back to Syncing) or Failed
```

`stateName()` gives each one in words: `"syncing"`, `"waiting for SIM"`,
`"registering"`, and so on.

- **Every state has a deadline and every deadline has an exit.** A state with
  no timeout is a hang, and it will happen in a basement with one bar of
  signal.
- **A state's deadline covers the worst case of what runs inside it.** The
  budgets for `Configuring` and `BearerUp` are added up from the driver's own
  steps — each step's timeout, its deferred timeout, and its retries — so a
  module that is slow but within its manual is never cut off mid-step.
  `modem().stateBudgetMs(state)` reports the figure.
- **`Ready` is re-entrant.** Losing registration — an unsolicited `+CEREG` or
  `+CGREG` report saying the module is no longer registered — drops back to
  `Registering` without tearing the module down, and counts in
  `counters().registrationLosses`. Reconnection is not a special path — it is
  the normal path, run again.
- **The module is not assumed to announce itself.** One family documents no
  boot banner at all; the other documents one and then says it is suppressed
  under auto-bauding. So `Syncing` polls a bare `AT` until it answers, and a
  banner only cuts the wait short.

### The recovery ladder

Core-owned and identical for every module, cheapest rung first:

```
  probe again  →  restart the radio  →  the module's reset
               →  the application's power hooks  →  report, and back off
```

Every rung ends in the whole bring-up running again from its first step. A
required step that failed is asked again, never stepped over — a module that
refused its radio mode must not reach `Ready` merely because the retry started
one step later. One failure skips the ladder entirely: a transport that reports
itself gone (an unplugged USB modem) goes straight to `Failed`, because every
rung is a command or a power cycle sent to something that can no longer be
reached, and the port is reopened on the backoff instead.

Backoff doubles with each bring-up that fails in a row — 2 s, 4 s, and so on up
to a ceiling of 60 s — and starts again from the bottom only when the modem
reaches `Ready` or the application calls `begin()`. It also carries jitter. A
hundred sites that lose a tower together must not re-register in lockstep and
bring it down again as it comes back.

---

## Three conditions that are not the same

| | Means | What to do |
| --- | --- | --- |
| Socket closed | this socket went; others may be fine | reopen this socket |
| Bearer lost | the data context went; **every** socket is dead | re-establish the bearer, then the sockets |
| Modem gone | it stopped answering at all: the port went, or three commands in a row timed out with nothing heard from it | the ladder |

A library that reports all three as "not connected" makes the application retry
the wrong thing — reopening a socket over a context that no longer exists,
forever, without ever explaining itself. `lastError()` says which.

---

## Error spaces, kept apart

A modem reports failure in several independent number spaces, and the same
integer means different things in each:

| Space | `GsmErrorSpace` | Range | Defined by |
| --- | --- | --- | --- |
| `+CME ERROR` | `Cme` | — | 3GPP TS 27.007 Annex A; universal |
| `+CMS ERROR` | `Cms` | — | 3GPP TS 27.005; SMS only |
| `<err_info>` | `SocketInfo` | 0–22 | one module's socket layer |
| `<err>` | `SocketErr` | 0–12 | a *different* table on the same module |
| `<result>` | `ConnResult` | 0–27 | a third table, on the other module |
| a driver's own | `Driver` | — | a module whose table matches none of the above |

Flattening them costs information that decides what somebody does next. A
`<result>` of 25 is "certificate's common name does not match" — a
commissioning mistake fixable in a minute. No `+CME` code says it, so a library
that mapped everything onto `+CME` would report it as a generic failure and
send somebody to look at the antenna.

`GsmResult` carries `{space, code}` in four bytes. The core renders the two
3GPP spaces because they are universal; a vendor table belongs to the driver
that owns it.
