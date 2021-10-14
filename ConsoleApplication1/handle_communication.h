/***********************************************************************************************************************
* handle_communication.h -
*
* Author : Dorian Verdel
* Creation  date : 02/2019
*
* Description :
* Manages the definition of the communication with the variator card.
***********************************************************************************************************************/

#pragma once

#ifndef HANDLE_COMMUNICATION_H
#define HANDLE_COMMUNICATION_H

// Project includes
extern "C"
{
	#include "carte_variateur\carte_variateur_V3.h"
}
#include "control_struct.h"
#include "compute_orders.h"
#include "motors_type_params.h"
#include "communication_struct_ABLE.h"
#include "utils_for_ABLE_Com.h"
#include "data_recording_functions.h"
#include "position_control.h"
#include "torque_control.h"
#include "able_Control_QTMData.h"
#include "able_Control_FTData.h"
#include "able_OrdersManagement.h"
#include <sal.h>

// Constants of communication definition
#define IP_ABLE "192.168.100.53"
#define PORT_ABLE 53210

// Functions declaration
// Communication functions
int able_InitCommunication(ServoComEth *eth_ABLE, AbleControlStruct *ctrl_ABLE, FILE* err_file, FILE* out_file);
int able_CloseCommunication(ServoComEth *eth_ABLE, AbleControlStruct *ctrl_ABLE);
void able_SendMotorParameters(ServoComEth *eth_ABLE, AbleControlStruct *ctrl_ABLE);
void check_OrderTransmission(ThreadInformations* ableInfos);
void deadman_CheckButtons(ThreadInformations* ableInfos);
int able_SendOrders(ThreadInformations* ableInfos);

// Command functions
DWORD WINAPI able_UpdateRealTimeProcess(LPVOID ableArgs);
#endif // !HANDLE_COMMUNICATION_H