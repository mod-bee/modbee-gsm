// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

#include "GsmTypes.h"
#include "GsmNoExcept.h"

const char* gsm_technologyName(GsmTechnology technology) {
    switch (technology) {
        case GsmTechnology::Gsm:         return "GSM";
        case GsmTechnology::GsmEgprs:    return "EDGE";
        case GsmTechnology::Utran:       return "UMTS";
        case GsmTechnology::UtranHs:     return "HSPA";
        case GsmTechnology::Eutran:      return "LTE";
        case GsmTechnology::EutranNbS1:  return "NB-IoT";
        case GsmTechnology::EutranCatM:  return "LTE-M";
        case GsmTechnology::Unknown:     break;
    }
    return "unknown";
}
