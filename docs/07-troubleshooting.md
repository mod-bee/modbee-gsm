# 07 — Troubleshooting

Each heading is a symptom. `gsm.stateName()` gives the bring-up state in
words, and `gsm.lastError()` says what went wrong last: its `status()` is the
library's category, and `describe()` is the same in words.

## Nothing happens; `stateName()` stays `syncing`

The module is not answering a bare `AT`.

- **It may not be up yet.** One family takes 11.2 seconds from power-on before
  its UART answers. `Syncing` allows thirty.
- **Wrong baud rate.** A modem at the wrong rate does not report an error; it
  sends plausible-looking rubbish. `gsm.engine().parser().truncatedLines()`
  climbing is the symptom.
- **Wrong port.** On a board with several, `Serial1` is not always the one on
  the header.
- **Nothing powered it on.** If your board switches the module's supply, the
  library does not know — supply a `GsmPowerControl`. See [06 — Power](06-power.md).

## `stateName()` reaches `failed` straight after `identifying`, with `NotSupportedByModule`

No registered driver recognised what `ATI` returned.

Nothing is guessed: driving an unrecognised module with somebody else's dialect
fails in ways that look like a hardware fault. Capture the identity and either
pick the right driver or write one — see [04](04-adding-a-modem.md).

Ask the module directly. Any serial terminal will do — the two questions that
matter are `ATI` and `AT+CGMR`, and on a SIM707x also `AT+CGMM`, because that
family answers `ATI` with a build number containing no model at all:

```
ATI
AT+CGMR
AT+CGMM
```

From inside a sketch, `gsm.sendAt()` asks the same questions without a
terminal, and hands you each reply line as it arrives.

## Stuck in `waiting for SIM`

`+CME ERROR: <n>` is the modem's standard way of saying "that failed, and here
is the reason number"; the numbers are defined by 3GPP and mean the same on
every module.

- **`+CME ERROR: 14` — SIM busy — is normal** for some seconds after power-on.
  The library retries; the state's own deadline governs.
- `+CME ERROR: 10` is no SIM inserted. `13` is SIM failure — often a holder
  that is marginal rather than a dead card.
- A SIM needing a **PUK** is refused outright and not retried. Three wrong PIN
  entries is a card that needs an operator, and guessing would burn the PUK
  retries too.

## Stuck in `registering`

The module is fine; the network has not accepted it. **Read `lastError()`
first** — the library tells these cases apart and they need different answers.

| `lastError().status()` | What it means | Where to look |
| --- | --- | --- |
| `RegistrationDenied` | The network **refused**, `<stat> 3` | The subscription, not the radio |
| `Timeout` | Nobody refused; nothing answered | Coverage, bands, antenna |

### It says `RegistrationDenied`

Asking again gets the same answer. Before giving up, the library clears the
SIM's forbidden-network list, then scans and tries each operator on air in
turn — so by the time you see this, all of that has already failed.

Check the **subscription**, in this order:

1. **Which networks is the SIM entitled to?** A roaming IoT SIM carries a
   network list. A network that is on air but not on that list refuses you,
   and the operator's dashboard usually says so in as many words.
2. **Which TECHNOLOGY is it entitled to on each?** This is the one that costs
   days. LTE and LTE-M are different entitlements. A SIM can be provisioned
   for `4G` on a network and still be refused LTE-M on the same network —
   and a Cat-M1-only module has no other way in.
3. **Is roaming enabled at all** for the country you are in?

Diagnostic that settles it quickly: fit a **Cat-1** module in the same place
with the same SIM. If Cat-1 registers and Cat-M1 does not, the radio and the
antenna are fine and the difference is an entitlement.

### It says `Timeout`

Nothing refused you, so nothing heard you.

- **Narrow the bands.** `gsm.bands()` is the single most effective thing available
  against a slow cold start — an LPWA module left to search every band it
  supports can take many minutes per pass, and the state may expire mid-sweep.
- **Narrow the technology.** `gsm.radioPreference(GsmRadioPreference::CatM)`
  stops the module spending half its time searching for a technology the
  country does not deploy. Scanning for an absent technology is not free: on
  one module, enabling NB-IoT alongside Cat-M1 took `AT+CSQ` (the signal
  query) from a steady 17 to a permanent 99, which means "no reading".
- `AT+CSQ` is nearly meaningless on NB-IoT — a 2G-era indicator that bottoms
  out long before the radio does. Use `rsrpDbm()` where the driver converts it.
- Roaming counts as registered. Code that accepts only
  `GsmRegistration::Home` refuses to work abroad.

### Ask the module rather than guessing

`radioConfig()` reports what the radio is **actually** set to, which is not
always what anyone set. The mode, technology and band list are saved in the
module's own non-volatile memory, so a value left by a previous firmware or a
diagnostic session outlives every power cycle.

For anything beyond this, get a live prompt on the module — `gsm.sendAt()`
from a sketch, or a pass-through sketch that copies bytes between the USB port
and the modem in both directions. A question you can ask in a second instead of
a rebuild is worth the half hour it takes to set up.

## Registered, but `isReady()` is false

Registered and having a data bearer are **different things**. This is exactly
what "full bars, no internet" looks like.

Usually the APN. `lastError()` often carries `+CME ERROR: 149` — PDP
authentication failure — when credentials are wrong or absent.

## A socket opens and then fails with `SocketClosed` after a send

On one module a short send is **fatal to the socket**, by design:

> "If the `<cnfSendLength>` is not equal to the `<reqSendLength>`, the socket
> then cannot be used further."

Reopen it. Do not retry the send — `GsmResult::isRetryable()` returns false for
exactly this reason.

## Every socket fails at once with `BearerLost`

The data context went, not one socket. Reopening sockets over a context that no
longer exists never succeeds and never explains itself.

The library re-establishes the bearer by itself; `gsm.counters().bearerLosses`
records how often. A site where that number climbs has a coverage problem, not
a software one.

## Data arrives corrupted, or the link "goes mad" under load

Almost always the **receive buffer**.

Most modem wiring is three wires, so the modem cannot be told to wait. An
overrun does not lose a reading — it desynchronises the AT stream, because the
missing bytes were part of a length-delimited payload.

```cpp
Serial1.setRxBufferSize(2048);     // BEFORE begin()
Serial1.begin(115200);
gsm.begin(&Serial1, 115200, 2048); // tell the library what it is
```

The library never asks for more than the transport says it can hold, so an
honest small number is safe. A number larger than the truth is not.

A big enough buffer is not the whole answer on an ESP32 whose core runs the
UART interrupt from flash (`CONFIG_UART_ISR_IN_IRAM` off, the Arduino default).
While flash is erased or written (a firmware update, a save of retained
values) that interrupt cannot run, and bytes arriving then overflow the UART's own
128-byte FIFO (`UART_FIFO_OVF_ERROR`) before they ever reach the buffer. The
library survives the loss rather than preventing it:

- a lost data notice is found by asking: an open socket with nothing announced
  asks the module every `GsmProfile::unreadCheckMs` (2 s) how many bytes it is
  holding, and fetches them;
- a fetch cut short ends the socket with `Timeout`, because a stream with a
  hole in it is not a stream; the application reconnects.

## A reply seems to vanish

Check whether a URC handler is claiming it. On every module `+CEREG:` is
**both** a URC and a read reply, told apart only by whether `<n>` leads. The
drivers decline the read form; a driver you wrote may not.

`gsm.modem().urcRouter().unhandledCount()` and `lastUnhandled()` show what
matched nothing.
An unexpected URC arriving every few seconds is a real diagnostic signal — a
repeating `+CPIN: NOT READY` means a marginal SIM holder.

## An SMS arrives with its last characters missing

The user-data length is in **septets** for the default alphabet and **octets**
for everything else. Writing the octet count is the commonest defect in SMS
code, and it only shows on text long enough for the two to differ.

If you are using this library's codec, that is handled. If you are building
PDUs yourself, that is where to look.

## A USB modem was unplugged, and the state went to `failed`

On a host, `GsmSerialTransport` notices the port going away — the operating
system reports a hang-up or an I/O error — and the modem goes to `failed` with
`Unresponsive`, abandoning whatever was in flight. Nothing on the recovery
ladder can help a module that cannot be reached, so the ladder is skipped.

It is not the end. Each time the backoff comes round (growing to at most a
minute) the library tries to reopen the port, and once the modem is plugged
back in, a normal bring-up follows by itself. A custom `GsmTransport` gets the
same treatment through its `isConnected()`, so that answer must mean what it
says: one that always returns false will never come up.

## The module reports a voltage warning

Take it seriously. On a board running from a switching supply this is the
earliest warning of a marginal rail there is. One manual spells the same URC
two different ways in two chapters; the drivers match both.

## Everything works on the bench and fails in the field

The usual three, in order:

1. **The receive buffer**, under real traffic rather than test traffic.
2. **A timeout sized for a good signal.** Registration and socket opens take
   far longer at the edge of coverage than on a desk.
3. **Reconnection never exercised.** Pull the antenna and watch it recover
   before shipping — `Ready` is re-entrant precisely so that path is the
   normal one, but only a test proves it.
