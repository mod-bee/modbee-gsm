// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#include "GsmUrcRouter.h"
#include "GsmNoExcept.h"

namespace {

bool startsWith(const char* line, size_t length, const char* prefix) {
    for (size_t i = 0; prefix[i] != '\0'; i++) {
        if (i >= length || line[i] != prefix[i]) { return false; }
    }
    return true;
}

} // namespace

bool GsmUrcRouter::add(const char* prefix, GsmUrcFn handler, void* context) {
    if (prefix == nullptr || handler == nullptr) { return false; }
    if (_count >= GSM_URC_HANDLERS_MAX) { return false; }

    _entries[_count].prefix = prefix;
    _entries[_count].handler = handler;
    _entries[_count].context = context;
    _count++;
    return true;
}

void GsmUrcRouter::removeAll(void* context) {
    size_t kept = 0;
    for (size_t i = 0; i < _count; i++) {
        if (_entries[i].context == context) { continue; }
        _entries[kept++] = _entries[i];
    }
    _count = kept;
}

void GsmUrcRouter::clear() { _count = 0; }

bool GsmUrcRouter::route(const char* line, size_t length) {
    if (line == nullptr) { return false; }

    // An empty line is not a URC. Modems emit them liberally between replies
    // and a handler registered on an empty prefix would match everything.
    if (length == 0) { return false; }

    for (size_t i = 0; i < _count; i++) {
        if (!startsWith(line, length, _entries[i].prefix)) { continue; }

        // MATCHING IS NOT THE SAME AS CLAIMING. A handler may decline - on
        // some modules one prefix is both a URC and a reply, distinguished
        // only by a field the driver knows how to read. A declined line falls
        // through to the next handler and then to the in-flight command, as if
        // it had never matched here.
        if (_entries[i].handler(_entries[i].context, line, length)) {
            _routed++;
            return true;
        }
    }

    // No URC handler wanted it - which is the ordinary case, because most
    // lines are replies to a command in flight. Whether it is genuinely
    // orphaned is the ENGINE's question, not this one's; see noteUnhandled().
    return false;
}

void GsmUrcRouter::noteUnhandled(const char* line, size_t length) {
    _unhandled++;
    size_t i = 0;
    for (; i + 1 < sizeof _lastUnhandled && i < length; i++) {
        _lastUnhandled[i] = line[i];
    }
    _lastUnhandled[i] = '\0';
}
