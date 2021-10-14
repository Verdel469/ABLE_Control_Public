/***********************************************************************************************************************
* communication_struct.h -
*
* Author : Dorian Verdel
* Creation  date : 02/2020
*
* Description :
* Defines the communication struct to interact with Qualisys and EMGs.
***********************************************************************************************************************/

#pragma once

#ifndef COMMUNICATION_STRUCT_H
#define COMMUNICATION_STRUCT_H

#include <vector>
#include "low_level_command_1DoF_main.h"

#define PIPE_BUFFER_LENGTH 512		// Define length of the buffer for pipe communications

// -------------------------------------------------- SOCKET SUBSTRUCT -------------------------------------------------
struct socketStruct
{
	SOCKET socket_com;            // Socket identifier for codes communication
	int socket_port;              // Socket port for commuunication with other codes
	sockaddr_in recvAddr;         // Structure for connection
};

// --------------------------------------------------- PIPE SUBSTRUCT --------------------------------------------------
struct pipeStruct
{
	HANDLE readPipeCtrl;	// Handle of the read extremity of the pipe mastered by the control thread
	HANDLE writePipeMeas;	// Handle of the write extremity of the pipe mastered by the qtm measures thread
	int robot_activated;	// Int determining if the robot is still controlled by the control code 1 : YES; 0 : NO
	int iter_counter;		// Number of iterations of the control thread
};

// -------------------------------------------- QUALISYS MEASURES SUBSTRUCT --------------------------------------------
struct qualisysMeasures
{
	float sliderPos; // Current position of the slider
};

// ----------------------------------------------- EMG MEASURES SUBSTRUCT ----------------------------------------------
struct emgMeasures
{
	// Variable to send for BicepsBrachial activation (0 : no activation detected, 1 : activation detected)
	int ping_BicepsBrachial;
	// Variable to send for BrachioRadialis activation (0 : no activation detected, 1 : activation detected)
	int ping_BrachioRadialis;
	// Variable to send for TricepsBrachialLongH activation (0 : no activation detected, 1 : activation detected)
	int ping_TricepsBrachialLongH;
	// Variable to send for TricepsBrachialLatH activation (0 : no activation detected, 1 : activation detected)
	int ping_TricepsBrachialLatH;
	// Store all passed values of EMG
	std::vector<float> emg_BB;		// Vector stroring the EMG values of the Biceps Brachii
	std::vector<float> emg_BR;		// Vector stroring the EMG values of the Brachio Radialis
	std::vector<float> emg_TBLoH;	// Vector stroring the EMG values of the Triceps Brachial Long Head
	std::vector<float> emg_TBLaH;	// Vector stroring the EMG values of the Triceps Brachial Lateral Head
};

// --------------------------------------------- GLOBAL COMMUNICATION STRUCT -------------------------------------------
struct ComStruct
{
	// Definition of substruct containing the socket parameters to communicate with Python
	socketStruct sockStruct;
	// Communication pipes handles
	pipeStruct pipeHandles;
	// Definition of substruct containing the pipe parameters to communicate between threads (control/communication)
	pipeStruct comPipeStruct;
	// Definition of substruct containing the qualisys measures to send to the control thread of ABLE
	qualisysMeasures qualMeas;
	// Definition of substruct containing the EMG measures to send to the control thread of ABLE
	emgMeasures emgMeas;
};

// -------------------------------------------- RECEIVED COMMUNICATION STRUCT ------------------------------------------
struct ReadStruct
{
	int iter_counter;
	int robot_activated;
	int mvt_ended;
};

// ------------------------------------------- TRANSMITTED COMMUNICATION STRUCT ----------------------------------------
struct TransStruct
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
#endif // !COMMUNICATION_STRUCT_H