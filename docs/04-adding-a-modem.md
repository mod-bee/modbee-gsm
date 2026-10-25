# 04 — Adding a modem

**A driver is a table and a handful of parse functions. It contains no state
machine.**

Retry policy, backoff, reconnection, the queue, the socket lifecycle, the
bring-up sequence and the exactly-one-conclusion contract all live in the core
and are written once. A new modem inherits every one of them.

That is the difference between "adding a module is a day" and "adding a module
is a month", and it is why TinyGSM needs twenty-odd near-duplicate headers of
several hundred lines each.

---

## The rule that makes it true

**No AT command text outside `src/drivers/`.**

It is not a style rule. The day a core file grows an `"AT+CSQ"`, the core has
learnt one module's dialect, and the second module starts needing special cases
in places nobody would look.

There is a second half to it, because a core file can learn a module's identity
without containing an AT command — a branch on a part number, a constant named
after a family. The rule is that the core asks the **driver**, never the part:

```cpp
profile().maxSockets                    // not  (family == A76XX) ? 10 : 13
driver().openCommand(id, endpoint, ...) // not  if (isSimcom7070) ...
```

---

## Seventy per cent is already written

`GsmDriver3gpp` is a **concrete base class**, not an interface. It implements
everything defined by 3GPP TS 27.007, TS 27.005 and ITU-T V.25ter — SIM status,
signal, registration, operator selection, context definition, clock, SMS, echo,
error verbosity.

A new driver subclasses it and overrides **the IP stack and the location
commands**. That is the remainder, and it is the part that is genuinely vendor
dialect.

---

## What differs, in practice

The two drivers this library ships share a vendor and almost nothing else. Every
row below is cited in the driver source:

| | A76XX | SIM707x |
| --- | --- | --- |
| Sockets | **10** | **13** |
| Bearer | `AT+CGDCONT` + `AT+NETOPEN` | `AT+CNCFG` + `AT+CNACT` |
| Open | `AT+CIPOPEN=…` | `AT+CACID` then `AT+CAOPEN=…` |
| Open result | `OK`, then `+CIPOPEN:` **later** | `+CAOPEN:` **then** `OK` |
| Send limit | 1500 | 1460 |
| Receive | `+CIPRXGET: 2,<id>,<len>,<rest>` then bytes on the next line | `+CARECV: <len>,` then bytes **on the same line** |
| Short send | **fatal to the socket** | free space queryable in advance |
| TLS | a **separate 16-command family**, 2 sessions, 2048-byte limit | a **flag** on the ordinary open, 13 sessions, 1460 |
| SIM serial | `AT+CICCID` → `+ICCID: …` | `AT+CCID` → **bare digits, no prefix** |
| GNSS | `AT+CGNSSPWR`, `AT+CGNSSINFO`, 17 fields, NMEA coordinates | `AT+CGNSPWR`, `AT+CGNSINF`, 18 fields, **signed decimal degrees** |
| Power off | `AT+CPOF` | `AT+CPOWD=1` |
| AT line limit | 3071 | 559 |
| Boot banner | **none documented** | `RDY`, suppressed under auto-bauding |

Look at the TLS row. On one module TLS means a *different set of commands*; on
the other it is a *flag on the same command*. Any abstraction that assumed "TLS
is a flag" is broken by these two alone.

And the GNSS row is the one that would fail silently: read NMEA
degrees-and-minutes as decimal degrees and a device lands a few hundred
kilometres away, in a plausible-looking place.

---

## Keyed on firmware, not on the part number

Two facts vary by **firmware revision** on the same silicon:

- TLS session count — one manual says 2, and notes that a later baseline
  supports 3.
- Whether socket closure is pushed as a URC or must be polled — V1.03 says
  polled, V1.04 added the URC.

So `claims()` is given the identification text **and** the revision. A profile
selected on the part alone would be wrong on half the stock in the field.

---

## Writing one

```cpp
class GsmDriverYours : public GsmDriver3gpp {
public:
    GsmDriverYours() {
        _profile.family = "YOURS";
        _profile.maxSockets = 6;          // [manual §x.y]
        _profile.maxSendChunk = 1024;     // [manual §x.y]
        // ... every field a citation
    }

    const GsmProfile& profile() const override { return _profile; }

    bool claims(const char* identification, const char* revision) const override;

    void openCommand(uint8_t socketId, const GsmEndpoint& endpoint,
                     const GsmApn& apn, GsmCmdBuf& out) const override;
    bool parseOpenResult(const char* line, size_t length, GsmOpenResult& out,
                         const GsmEndpoint& endpoint) const override;
    // ... the IP stack and GNSS; everything else is inherited

private:
    GsmProfile _profile;
};
```

`src/GsmDriver.h` lists every method, and which ones are pure virtual (the
compiler will tell you if one is missing). The two shipped drivers in
`src/drivers/` are the worked examples.

Register it before `begin()`:

```cpp
GsmDriverYours yours;
gsm.addDriver(&yours);
gsm.begin(&Serial1);
```

The registry asks each driver in the order it was added, and the first whose
`claims()` says yes wins. `GsmDevice` registers its two shipped drivers when it
is constructed, so a driver added with `addDriver()` is asked **after** them:
it takes any module they do not recognise, but cannot take one away from them.
Make `claims()` match only the modules your driver is for.

**Every command cites the manual section that defines it:**

```cpp
// [YOURS §12.4.2] AT+XOPEN=<id>,"TCP",<host>,<port>
out.add("AT+XOPEN=");
```

That is not bureaucracy. It makes the claim checkable by somebody who was not
there, and it makes a firmware change survivable — when a module behaves
differently from what the code expects, the comment says which document and
which section the expectation came from.

---

## A measurement

`GsmDriverSIM707x`, a complete second dialect including its bearer, sockets,
TLS, identity, GNSS and URC handling, is **under 700 lines of code**, and about
1,250 with its comments and citations.

That is the right order of magnitude: a table and some parsing, not a
reimplementation.
