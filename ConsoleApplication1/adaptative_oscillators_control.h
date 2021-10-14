/***********************************************************************************************************************
* adaptative_oscillators_control.h -
*
* Authors : Dorian Verdel, Abdelwaheb Hafs
* Creation  date : 05/2021
*
* Description :
* Manages the declaration of functions to achieve adaptative oscillators control of ABLE
***********************************************************************************************************************/

#pragma once

#ifndef ADAPTATIVE_OSCILLATORS_CONTROL_H
#define ADAPTATIVE_OSCILLATORS_CONTROL_H

// Include project files
#include "communication_struct_ABLE.h"
#include "utils_for_ABLE_Com.h"
#include "data_recording_functions.h"

// Define Hopf parameters
#define ETA 5;			// TODO : Give meaning
#define HOPF_V 20;		// TODO : Give meaning

// Adaptative oscillators control main function
void able_Hopf_PositionAsserv(ThreadInformations* ableInfos);

#endif // !ADAPTATIVE_OSCILLATORS_CONTROL_H