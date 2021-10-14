/***********************************************************************************************************************
* set_ABLEParameters.h -
*
* Author : Dorian Verdel
* Creation  date : 03/2021
*
* Description :
* Manages the definition of the functions for orders management.
***********************************************************************************************************************/

#pragma once

#ifndef SET_ABLEPARAMETERS_H
#define SET_ABLEPARAMETERS_H

// Project includes
#include "communication_struct_ABLE.h"
#include "control_struct.h"
#include "motors_type_params.h"

// Centralised function
void able_SetAllParams(AbleControlStruct* ctrl_ABLE);

// Functions setting all parameters
void able_SetMotorParams(AbleControlStruct* ctrl_ABLE);
void able_SetReductionsValues(AbleControlStruct* ctrl_ABLE);
void able_SetMotionRanges(AbleControlStruct* ctrl_ABLE);
void set_IdentifiedDynamics(AbleControlStruct* ctrl_ABLE);
#endif // !SET_ABLEPARAMETERS_H