/***********************************************************************************************************************
* communication_struct_ABLE.h -
*
* Author : Dorian Verdel
* Creation  date : 03/2021
*
* Description :
* Manages the declaration of the ThreadInformations struct
***********************************************************************************************************************/

#pragma once

#ifndef COMMUNICATION_STRUCT_ABLE_H
#define COMMUNICATION_STRUCT_ABLE_H

// Project includes
extern "C"
{
#include "carte_variateur\carte_variateur_V3.h"
}
#include "control_struct.h"

// ------------------------------------------------- GLOBAL CONTROL STRUCT ---------------------------------------------
struct ThreadInformations
{
	ServoComEth* eth_ABLE;
	AbleControlStruct* ctrl_ABLE;
	FILE* err_file;
	FILE* out_file;
	FILE* identification_file;
	FILE* currents_file;
	FILE* artpos_file;
	FILE* speeds_file;
	FILE* xs_slider_file;
	FILE* fz_Arm_file;
	FILE* ft_Arm_sensor_file;
	FILE* fz_Wrist_file;
	FILE* ft_Wrist_sensor_file;
	FILE* times_file;
	FILE* times;
};

#endif // !COMMUNICATION_STRUCT_ABLE_H