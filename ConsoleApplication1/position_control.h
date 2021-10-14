/***********************************************************************************************************************
* position_control.h -
*
* Author : Dorian Verdel
* Creation  date : 03/2021
*
* Description :
* Manages the declaration of position control of ABLE
***********************************************************************************************************************/

#pragma once

#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

// Project includes
#include "communication_struct_ABLE.h"
#include "utils_for_ABLE_Com.h"
#include "data_recording_functions.h"

// Position control for static identifications
void able_PositionAsserv(ThreadInformations* ableInfos);
// Regulation of interaction force during minimum jerk trajectory control
void able_RegulateIFPos(ThreadInformations* ableInfos);
// Position control for dynamic identifications
void able_DynIdentAsserv(ThreadInformations* ableInfos, int iter_counter);

#endif // !POSITION_CONTROL_H