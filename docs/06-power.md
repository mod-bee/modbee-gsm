# 06 — Power

**This library owns the order and the deadlines. Your application owns the
pins.**

## Why it is split that way

Boards differ in every respect that would tempt an assumption. One switches a
rail from a GPIO through a load switch; one drives PWRKEY directly; one wires a
relay; one brings out a STATUS line and one does not; a USB modem on a host has
no pin at all and is simply on.

A library that knew about any single arrangement would be wrong on the next
board. So `GsmPowerControl` is an **interface your application implements**,
and hand to the library with `gsm.powerControl(&hooks)` before `begin()`:

```cpp
class GsmPowerControl {
public:
    // Line-level hooks. Prefer these: move a line and return at once.
    virtual bool setPowerKey(bool asserted) { return false; }  // false = "no such line"
    virtual bool setSupply(bool on)         { return false; }
    virtual bool setReset(bool asserted)    { return false; }

    // Whole-operation hooks. The fallback, for a board with no power key.
    virtual bool powerOn()  { return false; }                  // false = "I cannot"
    virtual bool powerOff() { return false; }
    virtual bool reset()    { return false; }

    virtual GsmTristate isPoweredOn() const { return GsmTristate::Unknown; }
};
```

Seven hooks, all optional, all defaulted to "this board cannot". There is no
pin number, no GPIO call and no board name anywhere in the core.

**Supply none and the module is assumed already on** — correct for a USB modem,
and for a board that wires the module live.

### Why the power key is a line, not a "power on" call

Most modules are switched on and off by a **power key** (often labelled
PWRKEY): a line you hold at its active level for a set time and then release,
like pressing a button. The module toggles on that press.

**How long to hold it is a fact about the module, not the board**, and the two
shipped families differ by more than ten times: 50 ms on one, around a second
on the other. Worse, the module cannot be identified until it is running, so
at the moment the first press is needed nobody knows which module it is. So
your hook only moves the line (`setPowerKey(true)`, later `setPowerKey(false)`)
and the library decides how long to hold it, across `poll()` calls, without
blocking. "Asserted" means the module sees its key pressed; which way round
that is in volts, through whatever transistor your board uses, is yours alone.

Holding the key down indefinitely is not harmless either: one family resets
itself if the key stays pressed, which looks exactly like a dead module. The
library times the press, so that hazard is handled once rather than in every
board file.

Use `powerOn()` and `powerOff()` instead when the board has no key at all — a
load switch, a relay, a USB port. `powerOn()` is called only when
`setPowerKey()` has returned false; `powerOff()` is the last step of `end()`,
below. `setSupply()` is called with `true` at power-up, before the key, on a
board that switches the module's rail.

## What the library does own

The *ordering and the deadlines*, because those come from the module and are in
its manual. The A76XX driver declares, from its hardware design document:

| Symbol | Meaning | Value |
| --- | --- | --- |
| `Ton` | power-on pulse width | 50 ms |
| `Ton(uart)` | power-on → **the AT port answers** | **11.2 s** typ |
| `Toff` | power-off pulse width | ≥ 2.5 s |
| `Toff(status)` | power-off settling | 1.9 s |
| `Toff-on` | before a restart may begin | ≥ 2 s |
| `Treset` | reset pulse width (`resetPulseMs`) | 1.5 s |

The SIM707x driver declares its own figures from its own manual. It has no
reset pin at all — the module resets itself through the power key's internal
timer — so its `resetPulseMs` is 0, which means "never drive one".

**The UART is not ready for 11.2 seconds.** A bring-up budgeted for a couple of
seconds would declare a healthy module dead on every cold start.

The library does *not* sit silently through that, though: probing a module that
has not finished booting is harmless, so `Syncing` starts immediately and
retries until it answers. A module that is already awake — the ordinary case on
a warm restart — answers on the first probe and costs none of the wait.

## The ordering rule that destroys hardware if you get it wrong

> "the customer cannot shut down Vbat by disconnecting it, **which may cause
> damage to flash**" — A7670 Series Hardware Design §3.2.2

So `end()` runs a sequence and never asks for a rail drop first:

```
  the module's power-down command
      → wait for it to answer, at most the module's own Toff(status)
      → call powerOff()
```

`end()` is the one place the library spends a few seconds inside a call: it
is a shutdown, and the module has to be given its documented time to save its
state before power goes. It is bounded by that figure and never by the module
answering.

When the recovery ladder has to restart a module that has stopped answering, it
tries, in order:

1. the power key, where the board has one, held for the module's power-off
   pulse width;
2. otherwise the reset line, through `setReset()`, held for the module's
   `Treset` — timed by the library across `poll()` calls, like the key, and
   only on a module whose manual documents a reset pin (the profile's
   `resetPulseMs`; the SIM707x family has none, so it is never driven there);
3. otherwise `powerOff()` and then `reset()`.

Whichever it uses, it then leaves the module off for `Toff-on` before powering
it up again, and the whole bring-up runs again from the first step. The reset
line is always released before the power key is pressed: the A7670 manual
forbids holding the two down together.

An application that wants to drop its own rail early is free to. The library
will not do it on the application's behalf.

`reset()` is for the recovery ladder's late rungs only — the same document says
reset "is recommended ... only in case of emergency, such as the module is not
responding".

## `GsmTristate`, and why it is not a bool

```cpp
enum class GsmTristate { Unknown, No, Yes };
```

A board with no STATUS line cannot say whether the module is powered; one with
a STATUS line can say so authoritatively. Collapsing that to a bool makes "no
wire fitted" indistinguishable from "definitely off" — and the bring-up machine
treats those completely differently. One means poll for the modem; the other
means power it on first.

Where a STATUS line exists it is the **better boot signal than any amount of
polling**: several modules document that it goes high only once initialisation
is complete.

## The module watches its own supply

Both modules report under- and over-voltage, and one auto-powers-off outside
−40…+85 °C with a warning from −30…+80 °C. On any board running from a
switching supply this is the earliest warning of a marginal rail there is, and
it costs no extra hardware.

```cpp
gsm.modem().counters().supplyWarnings
```

One manual prints the same URC as `WARNNING` in one chapter and `WARNING` in
another. The driver matches **both spellings** — a hand-written table that
picked one would silently miss half the events.
