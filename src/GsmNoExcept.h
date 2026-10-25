// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#pragma once

// Included last by each library source file, after every other include.
//
// The library throws nothing and catches nothing (it builds with
// GSM_NO_EXCEPTIONS too), so its own functions need no exception tables or
// unwind code: about 2.5 kB of flash on a board. Placed after the standard
// headers so they keep their own settings; an application handler that throws
// through the library ends the program, which on a controller is what an
// escaped exception does anyway.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC optimize("no-exceptions")
#endif
