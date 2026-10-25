// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#pragma once

// Unsolicited result codes - lines the modem sends when nobody asked.
//
// They arrive at any moment INCLUDING MID-REPLY, which is the classic defect:
// an SMS lands while a signal query is outstanding and "+CMTI: \"SM\",3" is
// read as the answer to AT+CSQ. The signal is nonsense and the SMS is lost.
//
// So every complete line is offered HERE FIRST. A matching prefix means the
// line is a URC and the in-flight command never sees it.
//
// A prefix alone is not always enough: on some modules the same prefix is both
// a URC and a reply, told apart only by a field - "+CIPRXGET: 1,<link>" is
// unsolicited, "+CIPRXGET: 2,<link>,<len>,<rest>" is an answer. A handler may
// therefore DECLINE a line by returning false, and it falls through to the
// command in flight. That decision belongs to the driver.

#include "GsmPlatform.h"

#ifndef GSM_URC_HANDLERS_MAX
#define GSM_URC_HANDLERS_MAX 16
#endif

// Return true when the line was consumed as a URC, false to let it fall
// through to whatever command is in flight.
typedef bool (*GsmUrcFn)(void* context, const char* line, size_t length);

class GsmUrcRouter {
public:
    // `prefix` is matched at the start of the line and must outlive this
    // router - a string literal, or a member of the driver that registered it.
    //
    // Returns false when the table is full rather than silently ignoring the
    // registration, because a URC nobody is listening for is a fault that
    // reports itself as silence.
    bool add(const char* prefix, GsmUrcFn handler, void* context);

    // Removes every handler registered with this context. Used when a driver
    // or a service is torn down, so a stale handler cannot be called with a
    // dangling context.
    void removeAll(void* context);

    void clear();

    // Offers a line to the table. True when a handler took it.
    bool route(const char* line, size_t length);

    // How many lines NOBODY wanted - not a URC handler, and not a command in
    // flight either.
    //
    // COUNTED, NEVER SILENTLY DROPPED. A modem emitting an unexpected URC
    // every few seconds is a real diagnostic signal - an unhandled
    // "+CPIN: NOT READY" says the SIM holder is marginal, and if it is
    // swallowed somebody chases that for a week.
    //
    // THIS IS NOT INCREMENTED BY route(), AND THAT IS THE POINT. route()
    // returning false only means no URC handler claimed the line; the engine
    // then offers it to whatever command is in flight, and the overwhelming
    // majority of lines are claimed there. Counting at the router made every
    // ordinary "OK" an unhandled URC.
    //
    // The damage was not the number itself but what it hid: a soak reporting
    // "838 unhandled URCs" looked like noise to be ignored, and buried in it
    // was one line that genuinely was orphaned - "+CIPSEND: 0,57,57", the
    // send confirmation that the short-send rule depends on. A diagnostic
    // that cries wolf is worse than none.
    //
    // So the engine calls noteUnhandled() at the one place a line is truly
    // orphaned, and this counts only those.
    uint32_t unhandledCount() const { return _unhandled; }

    // Called by the engine when a line was claimed by nothing at all.
    void noteUnhandled(const char* line, size_t length);

    // The most recent unmatched line, for a diagnostic page. Never null.
    const char* lastUnhandled() const { return _lastUnhandled; }

    uint32_t routedCount() const { return _routed; }
    size_t handlerCount() const { return _count; }

private:
    struct Entry {
        const char* prefix = nullptr;
        GsmUrcFn handler = nullptr;
        void* context = nullptr;
    };

    Entry _entries[GSM_URC_HANDLERS_MAX];
    size_t _count = 0;

    uint32_t _routed = 0;
    uint32_t _unhandled = 0;

    // Bounded copy: a modem at the wrong baud rate can emit very long lines
    // and this is a diagnostic aid, not a log.
    char _lastUnhandled[64] = {0};
};
