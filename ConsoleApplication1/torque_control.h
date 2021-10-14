/***********************************************************************************************************************
* torque_control.h -
*
* Author : Dorian Verdel
* Creation  date : 03/2021
*
* Description :
* Manages the declaration of functions to achieve torque control of ABLE
***********************************************************************************************************************/

#pragma once

#ifndef TORQUE_CONTROL_H
#define TORQUE_CONTROL_H

#include "communication_struct_ABLE.h"
#include "utils_for_ABLE_Com.h"
#include "data_recording_functions.h"

// Torque control main function
void able_TorqueAsserv(ThreadInformations* ableInfos);

// Dynamic model compensations computation
void able_ComputeDynModelCompensation(ThreadInformations* ableInfos, int i);
float able_ComputeDynModelStat(ThreadInformations* ableInfos, int i);

// FT control functions
void able_TransparentFT_Control(ThreadInformations* ableInfos, int i);
void able_AntigravFT_Control(ThreadInformations* ableInfos, int i);
void able_FatigueTestFT_Control(ThreadInformations* ableInfos, int i, int counter_FatigueTest);

// Computational functions
float speed_MeanFilter(ThreadInformations* ableInfos, float art_speed_i);
#endif // !TORQUE_CONTROL_H