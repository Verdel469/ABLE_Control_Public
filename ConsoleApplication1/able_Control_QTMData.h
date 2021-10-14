/***********************************************************************************************************************
* able_Control_QTMData.h -
*
* Author : Dorian Verdel
* Creation  date : 03/2021
*
* Description :
* Manages the declaration of functions establishing communication between ABLE thread and QTM thread.
***********************************************************************************************************************/

#pragma once

#ifndef ABLE_CONTROL_QTMDATA_H
#define ABLE_CONTROL_QTMDATA_H

#include "communication_struct_ABLE.h"

// ------------------------------------------------- READ FROM QTM STRUCT ----------------------------------------------
struct QTMToReadStruct
{
	// Current position of the slider to transmit
	float s_pos;
	// Variable to send for BicepsBrachial activation (0 : no activation detected, 1 : activation detected)
	int ping_BB;
	// Variable to send for BrachioRadialis activation (0 : no activation detected, 1 : activation detected)
	int ping_BR;
	// Variable to send for TricepsBrachialLongH activation (0 : no activation detected, 1 : activation detected)
	int ping_TBLongH;
	// Variable to send for TricepsBrachialLatH activation (0 : no activation detected, 1 : activation detected)
	int ping_TBLatH;
};

// ------------------------------------------------- WRITE TO QTM STRUCT -----------------------------------------------
struct QTMToWriteStruct
{
	int iter_counter;
	int robot_activated;
	int mvt_ended;
};

// Write function definition
void qtm_WriteData(AbleControlStruct* ctrl_ABLE, int iter_counter);

// Read function definition
void qtm_ReadData(AbleControlStruct* ctrl_ABLE);
#endif // !ABLE_CONTROL_QTMDATA_H