// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#pragma once

// =============================================================================
// A MODEM DIALECT
// =============================================================================
//
// THE ONE RULE THAT MAKES THIS LIBRARY WORTH BUILDING:
//
//   A DRIVER CONTAINS NO STATE MACHINE. It is consulted; it does not drive.
//
// Retry policy, backoff, reconnection, the queue, the socket lifecycle, the
// bring-up sequence and the exactly-one-conclusion contract all live in the
// core and are written once. A new modem inherits every one of them. That is
// the difference between "adding a module is a day" and "adding a module is a
// month", and it is why TinyGSM needs twenty-odd near-duplicate headers of
// several hundred lines each while a driver here is a table and a handful of
// parse functions.
//
// Every method below either BUILDS a command into a caller's buffer or PARSES
// a line the caller already has. None of them do I/O, hold state, sleep, or
// loop. A driver method that waited would put blocking back into the library
// through the one door nobody would think to check.
//
// --- WHAT IS 3GPP AND WHAT IS NOT ---
//
// Roughly seventy per cent of what this library sends is standardised and
// identical on every modem ever made: SIM status, signal, registration,
// operator selection, context definition, SMS, echo, error verbosity. All of
// that lives in GsmDriver3gpp, a CONCRETE base class. A new driver subclasses
// it and overrides only the IP stack and the location commands - the quarter
// that is genuinely vendor dialect.

#include "GsmPlatform.h"
#include "GsmResult.h"
#include "GsmTypes.h"
#include "GsmProfile.h"
#include "GsmCmdBuf.h"

// One step of a sequence the core drives. A driver answers "what is step N",
// and says when there are no more.
struct GsmStep {
    bool     present = false;      // false ends the sequence
    uint32_t timeoutMs = 1000;
    bool     ignoreFailure = false;  // an optional step; carry on if it errors

    // How many EXTRA attempts this step gets before its failure counts.
    //
    // Some steps are known to need more than one go at real hardware, and
    // saying so here is better than papering over it with ignoreFailure. Both
    // TinyGSM forks wrap AT+CNACT in `while (!res && ntries < 5)`; that
    // survived review in two independently maintained forks, which is not an
    // accident.
    uint8_t  retries = 0;

    // A step that answers TWICE: "OK" first, and the real outcome later on a
    // line of its own. Bringing a bearer up on one family works this way, and
    // a sequence that advanced on the OK would declare the bearer ready before
    // the module had finished deciding - after which every socket open fails
    // for a reason that makes no sense.
    //
    // Null for the ordinary case, which is most steps.
    const char* deferredPrefix = nullptr;
    uint32_t    deferredTimeoutMs = 0;
};

class GsmDriver {
public:
    virtual ~GsmDriver() = default;

    // --- Identity ---

    virtual const GsmProfile& profile() const = 0;

    // Whether this driver recognises the module that answered.
    //
    // Given BOTH the identification text and the firmware revision, because
    // two facts in the profile vary by firmware rather than by part - a TLS
    // session count and whether socket closure is pushed or polled. A driver
    // selected on the part alone would be wrong on half the stock.
    //
    // Either argument may be empty if the module did not answer that query.
    virtual bool claims(const char* identification, const char* revision) const = 0;

    // --- Bring-up ---
    //
    // The core walks these from index 0 until one returns present=false. What
    // the steps DO is the driver's business; WHEN they run, what happens if
    // one fails, and how long each may take are the core's.

    virtual GsmStep initStep(uint8_t index, GsmCmdBuf& out) const = 0;

    // --- Identity queries ---

    virtual void identificationCommand(GsmCmdBuf& out) const = 0;
    virtual void revisionCommand(GsmCmdBuf& out) const = 0;
    virtual void imeiCommand(GsmCmdBuf& out) const = 0;
    virtual void imsiCommand(GsmCmdBuf& out) const = 0;

    // The SIM's serial number. Two modules this library drives use DIFFERENT
    // COMMANDS AND DIFFERENT REPLY SHAPES for it - one answers with a prefixed
    // line, the other with the bare digits - so both the command and the parse
    // belong here.
    virtual void iccidCommand(GsmCmdBuf& out) const = 0;
    virtual bool parseIccid(const char* line, size_t length, GsmField& out) const = 0;

    // --- Network ---

    virtual void signalCommand(GsmCmdBuf& out) const = 0;
    // Registration status, asked one register at a time.
    //
    // NOBODY TRUSTS A SINGLE REGISTER. TinyGSM's SIM70xx and a board
    // vendor's A76xx driver both ask CEREG and fall back to CGREG, for the
    // reason TinyGSM states: "We could be using GPRS fall-back or the board
    // could be being moody." TinyGSM's A7672x and a hand-written SIM7070G
    // driver from a production gateway ask CREG only. Four proven
    // implementations, three strategies, and not one of them stakes
    // registration on one answer.
    //
    // This library did, and the Registering state would sit out its whole
    // window while a module that was in fact attached reported so on a
    // register nobody was reading.
    //
    // <index> walks the preference order; an empty buffer means there is no
    // register at that index for this module and the caller should wrap.
    virtual void registrationCommand(uint8_t index, GsmCmdBuf& out) const = 0;
    virtual void operatorCommand(GsmCmdBuf& out) const = 0;
    virtual void simStatusCommand(GsmCmdBuf& out) const = 0;

    // Supply the PIN a locked SIM is waiting for.
    //
    // Separate from simStatusCommand because asking and answering are
    // different acts with different consequences: asking is free, and getting
    // this wrong three times bricks the card until somebody finds the PUK.
    virtual void simUnlockCommand(const char* pin, GsmCmdBuf& out) const = 0;

    // System information, where the module reports reference signal power.
    // Not every module does, and the conversion differs between those that do.
    virtual void systemInfoCommand(GsmCmdBuf& out) const = 0;
    virtual bool parseSystemInfo(const char* line, size_t length,
                                 GsmSignal& signal, GsmTechnology& technology) const = 0;

    // --- The data bearer ---

    virtual GsmStep bearerUpStep(uint8_t index, const GsmApn& apn, GsmCmdBuf& out) const = 0;
    virtual GsmStep bearerDownStep(uint8_t index, const GsmApn& apn, GsmCmdBuf& out) const = 0;
    virtual void addressCommand(const GsmApn& apn, GsmCmdBuf& out) const = 0;
    virtual bool parseAddress(const char* line, size_t length, GsmField& out) const = 0;

    // --- Sockets ---

    virtual void openCommand(uint8_t socketId, const GsmEndpoint& endpoint,
                             const GsmApn& apn, GsmCmdBuf& out) const = 0;

    // Cycle the radio so that configuration changes take effect.
    //
    // Band lists, network mode and Cat-M/NB-IoT preference are read by the
    // protocol stack when the radio comes up, not when the command is
    // accepted. A module told to search different bands while it is already
    // searching will answer OK and carry on doing exactly what it was doing -
    // and every tested sequence published for this family cycles the radio
    // afterwards for that reason.
    //
    // 3GPP TS 27.007 8.2 - AT+CFUN=0 disables the radio, AT+CFUN=1 restores
    // full functionality. Two steps rather than AT+CFUN=1,1, which resets the
    // whole module and would throw away everything just configured.
    virtual GsmStep radioCycleStep(uint8_t index, GsmCmdBuf& out) const;

    // Narrow the band search, one step at a time.
    //
    // Absent when the module has no such control or no bands were named, so
    // the base implementation does nothing and a Cat-1 driver need not care.
    virtual GsmStep bandConfigStep(uint8_t index, const GsmBandList& catM,
                                   const GsmBandList& nbIot,
                                   GsmCmdBuf& out) const {
        (void)index; (void)catM; (void)nbIot; (void)out;
        return GsmStep();
    }

    // Define the default PDP context, BEFORE the module registers.
    //
    // THE ORDER IS THE WHOLE POINT, and getting it wrong is invisible on some
    // networks and fatal on others.
    //
    // On LTE - and Cat-M1 and NB-IoT are LTE - attaching to the network and
    // opening a data context are ONE procedure, not two. The attach request
    // itself carries a PDN connectivity request naming an APN. A module whose
    // context 1 is still empty therefore attaches with no APN, and what
    // happens next is entirely the network's choice: some assign a default,
    // some ignore it, and some refuse the attach outright. Nothing in the
    // refusal says "APN", so it reads as searching or as a denial.
    //
    // This library used to define the context in bearerUpStep(), which runs
    // AFTER registration has succeeded - correct for the 2G/3G model where
    // attach and context activation really are separate, and too late for
    // LTE. A Cat-1 module on a permissive network hid it: it attached with an
    // empty APN, the network assigned one, and everything worked.
    //
    // Every tested SIM7070 sequence in the wild sets AT+CGDCONT before it
    // asks the module to attach, and so does this one now.
    virtual void defaultContextCommand(const GsmApn& apn, GsmCmdBuf& out) const;

    // Attach to the packet service.
    //
    // REGISTERED IS NOT ATTACHED. 3GPP TS 27.007 10.1.9: registration says
    // the network knows where the module is; attachment says it will carry
    // packets for it. A module can sit registered and unattached indefinitely,
    // and every registration query in the set will report it as fine.
    //
    // Cheap to assert and harmless when it is already true, so it is done
    // once before the bearer is asked for rather than diagnosed later.
    virtual void attachCommand(GsmCmdBuf& out) const;

    // Ask whether the packet service is attached. Separate from registration
    // and reported separately by the module - watching only one of the two
    // hides half the picture.
    virtual void attachQueryCommand(GsmCmdBuf& out) const;

    // --- FORBIDDEN NETWORKS ---
    //
    // A SIM remembers every network that refused it, and a module in
    // AUTOMATIC selection will not try any of them again. That list is
    // EF_FPLMN on the card, and it survives reboots, power cycles and
    // reflashing - it is on the SIM, not in the module.
    //
    // It is a sensible optimisation that becomes a trap: a device powered up
    // before its SIM was activated, or carried into a country the SIM is not
    // allowed in, fills the list with every local network and then refuses to
    // connect anywhere for good. Nothing about that looks like a stored
    // decision; it looks like no coverage.
    //
    // Observed here, self-inflicted: a few attempts on a technology this SIM
    // is not provisioned for were enough to get the two networks that DO work
    // written to the list, after which automatic selection skipped them
    // entirely. Manual selection still reached them, which is the tell - if
    // naming a network works and automatic does not, the list is why.
    //
    // 3GPP TS 27.007 8.18 restricted SIM access, so these are the same on
    // every module and belong to the base driver.
    //
    //   read:  AT+CRSM=176,28539,0,0,12
    //   clear: AT+CRSM=214,28539,0,0,12,"FFFFFFFFFFFFFFFFFFFFFFFF"
    //
    // 176 is READ BINARY, 214 is UPDATE BINARY, 28539 is 0x6F7B - EF_FPLMN.
    // An empty list reads back as 24 hex Fs.
    virtual void forbiddenNetworksReadCommand(GsmCmdBuf& out) const;
    virtual void forbiddenNetworksClearCommand(GsmCmdBuf& out) const;

    // True when a +CRSM reply shows at least one forbidden network.
    virtual bool parseForbiddenNetworks(const char* line, size_t length,
                                        bool& anyForbidden) const;

    // Choose the operator, or hand the choice back to the module.
    //
    // 3GPP TS 27.007 7.3, so it belongs to every module rather than to a
    // vendor - the base driver implements it and nothing needs to override it.
    //
    // Automatic selection picks a network the module can SEE, which on a
    // roaming SIM is not the same as one it may USE. Naming an operator skips
    // the ones that will refuse.
    //
    // `numericCode` is MCC+MNC as digits, or nullptr for automatic.
    // Ask what is on air. Slow - the module sweeps its bands - so the caller
    // must allow profile().operatorScanTimeoutMs for it.
    virtual void operatorScanCommand(GsmCmdBuf& out) const;

    // Pull the numeric operator codes out of a scan reply.
    virtual bool parseOperatorScan(const char* line, size_t length,
                                   GsmOperatorList& out) const;

    virtual void operatorSelectCommand(const char* numericCode,
                                       GsmCmdBuf& out) const;

    // Point the radio at a technology, one step at a time.
    //
    // Returns an absent step when there is nothing (more) to do, which is the
    // default and is correct for any module with one radio. Run during
    // configuration, before registration is waited on, because afterwards is
    // too late - the module is already searching the wrong thing.
    // Read the radio configuration BACK. One query per index; an empty
    // buffer ends the sequence.
    //
    // A library that sets a thing and cannot read it is asking to be trusted
    // about the one fact it has no evidence for. On a real SIM7070G the band
    // list had been narrowed to the wrong operator's band by an earlier
    // session, AUTO_SAVE kept it through every power cycle, and nothing here
    // could see it.
    virtual void radioReadbackCommand(uint8_t index, GsmCmdBuf& out) const {
        (void)index; out.clear();
    }

    // Read one radio-configuration reply into the report. False if this line
    // is not one - the caller tries other parsers.
    //
    // A DRIVER METHOD because the prefixes are dialect, not protocol: +CMNB,
    // +CBANDCFG and +CNSMOD exist on one family and not the other, and the
    // core is not allowed to know which is which.
    virtual bool parseRadioConfig(const char* line, size_t length,
                                  GsmRadioConfig& out) const {
        (void)line; (void)length; (void)out; return false;
    }

    virtual GsmStep radioPreferenceStep(uint8_t index, GsmRadioPreference wanted,
                                        GsmCmdBuf& out) const {
        (void)index; (void)wanted; (void)out;
        return GsmStep();
    }

    // A socket open that answers twice. Empty when the module concludes on its
    // first reply. The core turns this into a deferred conclusion.
    // TLS SETUP, BEFORE THE OPEN. One step per index; an empty buffer ends
    // the sequence.
    //
    // The two families differ completely. One configures an SSL context and
    // then sets a FLAG on an ordinary socket open; the other has a whole
    // parallel command family, so its open, send, fetch and close are
    // different commands with different reply prefixes. That is why security
    // travels with the endpoint into every one of them rather than being
    // decided once here.
    virtual GsmStep tlsSetupStep(uint8_t index, uint8_t socketId,
                                 const GsmEndpoint& endpoint,
                                 GsmCmdBuf& out) const {
        (void)index; (void)socketId; (void)endpoint; out.clear();
        return GsmStep();
    }

    virtual const char* openDeferredPrefix(const GsmEndpoint& endpoint) const {
        (void)endpoint; return nullptr;
    }

    virtual void sendCommand(uint8_t socketId, size_t length,
                             const GsmEndpoint& endpoint, GsmCmdBuf& out) const = 0;

    // A SEND THAT ANSWERS TWICE, and the reason this matters far more than it
    // looks.
    //
    // One family confirms a send with "OK" and then, separately, with a line
    // carrying how many bytes it ACTUALLY took. That second line is the only
    // place the short-send rule can be enforced - and the rule is that a short
    // send is TERMINAL, not retryable:
    //
    //   "If the <cnfSendLength> is not equal to the <reqSendLength>, the
    //    socket then cannot be used further." [A76XX §15.2.4]
    //
    // Without this prefix the command concludes on the OK, the confirmation
    // arrives with nothing in flight to attach to, and it is routed away as an
    // unhandled URC. Everything looks fine: parseSendResult() exists, is
    // called from the right place and is covered by tests - it simply never
    // sees a line on real hardware.
    //
    // Found by a soak counting 838 unhandled URCs and naming the last one:
    //   "+CIPSEND: 0,57,57"
    //
    // Empty where the module confirms before the payload instead of after,
    // which is what the other family does.
    virtual const char* sendDeferredPrefix(const GsmEndpoint& endpoint) const {
        (void)endpoint; return nullptr;
    }
    virtual void fetchCommand(uint8_t socketId, size_t maxBytes,
                              const GsmEndpoint& endpoint, GsmCmdBuf& out) const = 0;
    virtual void closeCommand(uint8_t socketId, const GsmEndpoint& endpoint,
                              GsmCmdBuf& out) const = 0;

    virtual bool parseOpenResult(const char* line, size_t length,
                                 GsmOpenResult& out,
                                 const GsmEndpoint& endpoint) const = 0;

    // The header that precedes received bytes. Returning true tells the core
    // how many bytes to take as an opaque payload rather than as text.
    virtual bool parseFetchHeader(const char* line, size_t length,
                                  GsmFetchInfo& out,
                                  const GsmEndpoint& endpoint) const = 0;

    // For a module whose payload begins PART-WAY THROUGH the header line,
    // after a comma, rather than on a line of its own. Returns the header
    // prefix to watch for, or null for the ordinary shape.
    //
    // See GsmAtParser::expectInlinePayload() - the two shapes cannot both be
    // handled by waiting for a line end, because opaque bytes contain line
    // endings.
    virtual const char* inlineFetchPrefix(const GsmEndpoint& endpoint) const {
        (void)endpoint; return nullptr;
    }

    // For a module whose FETCH concludes after its OK rather than on it.
    //
    // MEASURED, AND THE MANUAL AGREES. A76XX AT+CCHRECV answers OK first,
    // then the data block, then "+CCHRECV: <session_id>,<err>" - which is the
    // real conclusion. Concluding on the OK ends the command before the bytes
    // arrive, and every one of them is then an orphaned line. See A76XX
    // V1.12 SS19.2.15, whose response listing puts OK above the data.
    //
    // Written into out because it carries the SESSION ID: the terminator
    // "+CCHRECV: 0,0" and the data header "+CCHRECV: DATA,0,266" share a
    // prefix, and only the field after the space tells them apart. Empty for
    // the ordinary shape, where OK is the conclusion.
    virtual void fetchDeferredPrefix(uint8_t socketId,
                                     const GsmEndpoint& endpoint,
                                     GsmCmdBuf& out) const {
        (void)socketId; (void)endpoint; out.clear();
    }

    // How many received bytes the module is holding for this socket, asked
    // without taking any. The safety net under the data notice - see
    // GsmProfile::unreadCheckMs. Empty where the module has no such query,
    // which leaves the check off for that family.
    virtual void unreadQueryCommand(uint8_t socketId, const GsmEndpoint& endpoint,
                                    GsmCmdBuf& out) const {
        (void)socketId; (void)endpoint; out.clear();
    }

    // The answer to unreadQueryCommand(), for THIS socket. False for any other
    // line, including the same answer about a different socket.
    virtual bool parseUnread(const char* line, size_t length, uint8_t socketId,
                             const GsmEndpoint& endpoint, size_t& out) const {
        (void)line; (void)length; (void)socketId; (void)endpoint; (void)out;
        return false;
    }

    // Whether a send reply says the transfer completed intact.
    //
    // ONE MODULE MAKES A SHORT SEND FATAL TO THE SOCKET - its manual states
    // that when the confirmed length differs from the requested length "the
    // socket then cannot be used further". So this returns not merely success
    // or failure but the conclusion the core should record, and the socket
    // layer knows to reopen rather than retry.
    virtual bool parseSendResult(const char* line, size_t length,
                                 size_t requested, GsmResult& out,
                                 const GsmEndpoint& endpoint) const = 0;

    // --- Unsolicited ---

    // What this line is, if anything. The core acts on the kind; only the
    // driver knows which text means which.
    //
    // MAY DECLINE by returning Unrecognised even for a prefix it registered -
    // on one module the same prefix is both a URC and a reply, told apart only
    // by a field value, and the line must then fall through to the command in
    // flight.
    virtual GsmUrcKind classifyUrc(const char* line, size_t length,
                                   GsmUrcInfo& out) const = 0;

    // Every prefix this driver wants offered to classifyUrc(). Returns null
    // past the end. Registered with the router at attach time.
    virtual const char* urcPrefix(uint8_t index) const = 0;

    // --- Final results the module invents ---
    //
    // On top of the universal OK / ERROR / +CME ERROR / +CMS ERROR that the
    // engine already knows. One module answers socket faults with a line of
    // its own that is neither of those, and a command that did not recognise
    // it would sit until its deadline instead of failing at once.
    virtual bool isFinalResult(const char* line, size_t length, GsmResult& out) const {
        (void)line; (void)length; (void)out;
        return false;
    }

    // --- Power ---

    virtual void powerDownCommand(GsmCmdBuf& out) const = 0;
    virtual void resetCommand(GsmCmdBuf& out) const = 0;

    // --- Location ---

    virtual void gnssPowerCommand(bool on, GsmCmdBuf& out) const { (void)on; out.clear(); }
    virtual void gnssInfoCommand(GsmCmdBuf& out) const { out.clear(); }
    virtual bool parseGnssInfo(const char* line, size_t length, GsmFix& out) const {
        (void)line; (void)length; (void)out;
        return false;
    }

    // --- Time ---

    virtual void clockCommand(GsmCmdBuf& out) const = 0;
    virtual bool parseClock(const char* line, size_t length,
                            GsmParse::GsmDateTime& out) const = 0;

    // --- SMS ---

    // SMS. AN EMPTY BUFFER MEANS THIS MODULE HAS NO SUCH COMMAND, and callers
    // report that as NotSupportedByModule rather than as a bad argument - one
    // says fit a different modem, the other says fix your code.
    virtual GsmStep smsInitStep(uint8_t index, GsmCmdBuf& out) const {
        (void)index; out.clear();
        return GsmStep();
    }
    virtual void smsSendCommand(const char* number, size_t textLength,
                                GsmCmdBuf& out) const {
        (void)number; (void)textLength; out.clear();
    }
    virtual void smsReadCommand(uint16_t index, GsmCmdBuf& out) const {
        (void)index; out.clear();
    }
    virtual void smsDeleteCommand(uint16_t index, GsmCmdBuf& out) const {
        (void)index; out.clear();
    }
};
