// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#include "GsmResult.h"
#include "GsmNoExcept.h"

bool GsmResult::isRetryable() const {
    switch (_status) {
        // Transient by nature. The modem was busy, or the queue was, or the
        // link had a moment. The identical command may well work next time.
        case GsmStatus::Timeout:
        case GsmStatus::QueueFull:
        case GsmStatus::NoSocketAvailable:
            return true;

        // Not retryable, and each for a different reason worth keeping apart.
        //
        // SocketClosed is the subtle one and it is why this function exists.
        // A partial send looks transient - the buffer was full, try again -
        // but the module's own manual says that once the confirmed length
        // differs from the requested length "the socket then cannot be used
        // further". So the driver concludes SocketClosed, and retrying the
        // send on that socket would write into something that is already
        // gone. Reopen first; the reopen is retryable, the send is not.
        case GsmStatus::SocketClosed:
        case GsmStatus::BearerLost:
        case GsmStatus::PoweredOff:
        case GsmStatus::Unresponsive:

        // A refusal by the network. Asking again produces the same refusal -
        // that is what makes it a decision rather than a delay - so the only
        // useful moves are somebody else's: a different operator, a different
        // SIM, or a call to whoever issued it.
        case GsmStatus::RegistrationDenied:
            return false;

        // Refused because of what was asked or what this module is. Retrying
        // an identical request changes nothing; the caller has to change.
        case GsmStatus::InvalidArgument:
        case GsmStatus::NotSupportedByModule:
        case GsmStatus::TooLarge:
        case GsmStatus::NotInitialised:
            return false;

        // A module said no with a reason. Whether that reason is transient
        // depends on a table this file does not own, so the honest answer is
        // no - a caller that knows the module can ask its driver instead.
        case GsmStatus::ModuleError:
        case GsmStatus::Error:
            return false;

        // States, not failures.
        case GsmStatus::Queued:
        case GsmStatus::Ok:
        case GsmStatus::NotReady:
        case GsmStatus::NoBearer:
        case GsmStatus::Superseded:
        case GsmStatus::Cancelled:
            return false;
    }
    return false;
}

const char* GsmResult::describe() const {
    switch (_status) {
        case GsmStatus::Queued:               return "queued";
        case GsmStatus::Ok:                   return "ok";
        case GsmStatus::NotInitialised:       return "not initialised";
        case GsmStatus::InvalidArgument:      return "invalid argument";
        case GsmStatus::NotReady:             return "modem not ready";
        case GsmStatus::NoBearer:             return "no data bearer";
        case GsmStatus::NoSocketAvailable:    return "no socket available";
        case GsmStatus::QueueFull:            return "queue full";
        case GsmStatus::Superseded:           return "superseded";
        case GsmStatus::Cancelled:            return "cancelled";
        case GsmStatus::TooLarge:             return "too large for this module";
        case GsmStatus::NotSupportedByModule: return "not supported by this module";
        case GsmStatus::Error:                return "modem said ERROR";
        case GsmStatus::ModuleError:          return "modem reported an error code";
        case GsmStatus::Timeout:              return "timeout";
        case GsmStatus::SocketClosed:         return "socket closed";
        case GsmStatus::BearerLost:           return "data bearer lost";
        case GsmStatus::Unresponsive:         return "modem unresponsive";
        case GsmStatus::PoweredOff:           return "modem powered off";
        case GsmStatus::RegistrationDenied: return "registration denied by the network";
    }
    return "unknown";
}

const char* GsmResult::describeStandardCode() const {
    // Only the 3GPP spaces, and only the codes a controller actually meets.
    //
    // NOT the whole of Annex A. A table of 150 rarely-seen strings costs flash
    // on every board that links this and earns nothing: an unlisted code still
    // reaches the operator as its number, which is what they would search for
    // anyway. The ones here are the ones that change what somebody does next.
    if (_space == GsmErrorSpace::Cme) {
        switch (_code) {
            case 3:   return "operation not allowed";
            case 4:   return "operation not supported";
            case 10:  return "SIM not inserted";
            case 11:  return "SIM PIN required";
            case 12:  return "SIM PUK required";
            case 13:  return "SIM failure";
            case 14:  return "SIM busy";          // normal for seconds after boot
            case 15:  return "SIM wrong";
            case 16:  return "incorrect password";
            case 17:  return "SIM PIN2 required";
            case 18:  return "SIM PUK2 required";
            case 20:  return "memory full";
            case 21:  return "invalid index";
            case 22:  return "not found";
            case 23:  return "memory failure";
            case 30:  return "no network service";
            case 31:  return "network timeout";
            case 32:  return "network not allowed - emergency calls only";
            case 50:  return "incorrect parameters";
            case 100: return "unknown";           // the modem's own shrug
            case 103: return "illegal MS";
            case 106: return "illegal ME";
            case 107: return "GPRS services not allowed";
            case 111: return "PLMN not allowed";
            case 112: return "location area not allowed";
            case 113: return "roaming not allowed in this location area";
            case 132: return "service option not supported";
            case 133: return "requested service option not subscribed";
            case 134: return "service option temporarily out of order";
            case 148: return "unspecified GPRS error";
            case 149: return "PDP authentication failure";
            case 150: return "invalid mobile class";
            default:  return nullptr;
        }
    }

    if (_space == GsmErrorSpace::Cms) {
        switch (_code) {
            case 300: return "ME failure";
            case 301: return "SMS service of ME reserved";
            case 302: return "operation not allowed";
            case 303: return "operation not supported";
            case 304: return "invalid PDU mode parameter";
            case 305: return "invalid text mode parameter";
            case 310: return "SIM not inserted";
            case 311: return "SIM PIN required";
            case 313: return "SIM failure";
            case 314: return "SIM busy";
            case 315: return "SIM wrong";
            case 321: return "invalid memory index";
            case 322: return "memory full";
            case 330: return "SMSC address unknown";
            case 331: return "no network service";
            case 332: return "network timeout";
            case 500: return "unknown error";
            default:  return nullptr;
        }
    }

    // A vendor space. Only the driver that emitted it can say what it means,
    // and guessing here would be worse than saying nothing.
    return nullptr;
}
