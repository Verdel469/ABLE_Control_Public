/***********************************************************************************************************************
* utils_for_ABLE_Com.h -
*
* Author : Dorian Verdel
* Creation  date : 03/2021
*
* Description :
* Manages the declaration of utils functions to communicate with ABLE
***********************************************************************************************************************/

#pragma once

#ifndef UTILS_FOR_ABLE_COM_H
#define UTILS_FOR_ABLE_COM_H

#include "communication_struct_ABLE.h"

// Health functions definition
void check_state(ThreadInformations* ableInfos);			// Function to check the healthy behaviour of ABLE

// Data extraction functions definition
void able_ExtractData(ThreadInformations* ableInfos, int i);		// Extract current data from ETH struct
float able_ComputeCurrentPosition(int coder_pos, float reduction);	// Compute current position of an axis

// Particular orders functions definition
void able_SendNullSpeedOrder(ServoComEth* eth_ABLE, AbleControlStruct* ctrl_ABLE, int inhibition);
void able_SetPowerOff(ServoComEth* eth_ABLE, AbleControlStruct* ctrl_ABLE);
void sature_SpeedOrders(ThreadInformations* ableInfos);

// Timing functions definition
void able_OneMsWait();														// Function to wait 1ms (sampling frequency)
void able_WaitPower(ServoComEth* eth_ABLE, AbleControlStruct* ctrl_ABLE);	// Wait for 48V
void end_time_wait();														// Wait at the end of execution

#endif // !UTILS_FOR_ABLE_COM_H
