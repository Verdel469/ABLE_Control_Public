/***********************************************************************************************************************
* low_level_command_1DoF_main.h -
*
* Author : Dorian Verdel
* Creation  date : 02/2019
*
* Description :
* Manages the includes and main functions definition of the project.
***********************************************************************************************************************/

#pragma once

#ifndef LOW_LEVEL_COMMAND_1DOF_MAIN_H
#define LOW_LEVEL_COMMAND_1DOF_MAIN_H

// General includes
#include <iostream>
#include <thread>
#include <errno.h>					// Standard errors librairy to get sockets outputs
#include <stdio.h>
#include <winsock2.h>				// Socket librairy
#pragma comment(lib, "Ws2_32.lib")	// Use of socket


// Project includes
#include "shared_FT_struct.h"				// Header containing the shared FT measures struct definition
#include "set_ABLEParameters.h"				// Header of the file containing the robot parameters
#include "limb_identification.h"			// Header containing the functions to identify, store and extract the limb parameters
#include "control_struct.h"					// Header containing the full declaration of the control struct
#include "communication_struct.h"			// Header containing the full declaration of the communication struct
#include "compute_orders.h"					// Header of the orders computation and values definition
#include "handle_communication.h"			// Header of the code containing the functions to communicate with ABLE
#include "get_qtm_measures.h"				// Header of the file containing the thread communicating with python
#include "get_FT_measures_WinAPI.h"			// Header of the file communicating with the FT sensor

struct ComStruct;

// Thread handles struct
struct ThreadHandles
{
	HANDLE control_thread;
	HANDLE digital_FT_thread_Arm;
	HANDLE digital_FT_thread_Wrist;
	HANDLE qtm_thread;
};

// Functions declaration
int main(int argc, char *argv[]);										        // Declaration of the main command function
void extract_InputData(int argc, char* argv[], FILE* out_file, FILE* err_file); // Extract input data from python script
void preallocate_memory();														// Pre allocate vectors memory
void initialise_FT_Shared();													// Initialise shared structs
void prefill_FT_Comm_Structs();													// Pre-fill structs with their respective data
void prefill_ABLE_Thread_Comm_Struct(ThreadInformations* ableInformations, FILE* out_file, FILE* err_file);
void initQTMComPipes(FILE* err_file, FILE* out_file);							// Init QTM communication pipes
int initComPython(FILE* err_file, FILE* out_file);                              // Function initializing the communication
void waitNomVoltage(FILE* err_file, FILE* out_file);					        // Wait nominal voltage to protect ABLE
int launch_FT_Measures_Arm();													// Function launching the FT measures at arm Thread
int launch_FT_Measures_Wrist();													// Function launching the FT measures at wrist Thread
int launch_QTM_Measures(ComStruct* qtm_ComStruct, FILE* xs_slider_file2);		// Function launching the QTM measures Thread
int executeMotions(ThreadInformations* ableInfos);						        // Function executing the Thread of command
int getErrorMessage(int err, FILE* err_file, FILE* out_file);			        // Get error message associated with Thread exit
void clean_Files(ThreadInformations* ableInfos);                                // Clean all files used during command
#endif // !LOW_LEVEL_COMMAND_1DOF_MAIN_H