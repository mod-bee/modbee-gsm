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
// ModBee GSM
// =============================================================================
//
// An AT-command cellular modem, driven over a serial link, presented as
// standard TCP and UDP clients. For Arduino and for hosts.
//
// THE ONE RULE: NOTHING BLOCKS. No call waits on the modem - opening a socket
// included. One object, one poll() in your loop.
//
// THE SECOND RULE: NO HARDWARE LIVES HERE. Not a pin, not a board name. Power
// and reset are callbacks your application supplies; this library decides only
// WHEN to call them and how long to wait afterwards. See GsmPowerControl.h.
//
// WHAT YOU GET ON TOP, and it is the reason to use this:
//
//   GsmClient   implements Arduino's Client  -> ArduinoHttpClient, PubSubClient,
//                                               modbee-mqtt, anything
//   GsmUDP      implements Arduino's UDP     -> NTP, syslog, plain datagrams
//   GsmSocket   the native, non-blocking form: open, read, write, close
//
// NO SERVER. Arduino's Server is not implemented: an inbound connection needs
// a fixed-IP APN, and none of the established modem libraries implements one
// to adapt. The drivers report the capability through profile()->hasTcpServer,
// so a sketch can drive AT+SERVERSTART / AT+CASERVER itself through sendAt()
// if it has such an APN. Claimed nowhere else - see docs/05.
//
// ADDING A MODEM IS A TABLE, NOT A #define. Every AT string in this library
// lives in src/drivers/; the state machines, the queue, the retry policy and
// the one-conclusion contract are written once and inherited by every driver.

#include "GsmPlatform.h"
#include "GsmResult.h"
#include "GsmTypes.h"
#include "GsmTransport.h"
#include "GsmPowerControl.h"
#include "GsmParse.h"
#include "GsmAtParser.h"
#include "GsmUrcRouter.h"
#include "GsmAtEngine.h"
#include "GsmProfile.h"
#include "GsmDriver.h"
#include "GsmDriverRegistry.h"
#include "GsmModem.h"
#include "GsmSocket.h"
#include "GsmSmsPdu.h"
#include "GsmSms.h"
#include "GsmGnss.h"
#include "GsmDevice.h"

// The drivers this library ships. Both are always compiled; which one drives a
// module is decided at run time from what the module says about itself.
#include "drivers/GsmDriverA76XX.h"
#include "drivers/GsmDriverSIM707x.h"

// Arduino's own networking base classes, over the modem. Compile to nothing
// on a host. These are the doorway to every library already written against
// them - ArduinoHttpClient, PubSubClient, WebSockets and the rest.
#include "GsmClient.h"
#include "GsmUDP.h"

// Transports. Each compiles to nothing on the platforms it does not serve.
#include "transport/GsmStreamTransport.h"
#include "transport/GsmSerialTransport.h"
