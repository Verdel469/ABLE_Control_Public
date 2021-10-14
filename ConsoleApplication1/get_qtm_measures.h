/***********************************************************************************************************************
* get_qtm_measures.h -
*
* Author : Dorian Verdel
* Creation  date : 02/2019
*
* Description :
* File containing the functions allowing to get the real time measures of the Qualisys and the EMG.
***********************************************************************************************************************/

#pragma once

#ifndef GET_QTM_MEASURES_H
#define GET_QTM_MEASURES_H

// Project includes
#include "communication_struct.h"
#include <strsafe.h>

// Constants definition
#define LENGTH_BUFFER 100    // Length of the buffer for recv (receive python values)

// Functions declaration
DWORD WINAPI get_RealTimeQTMMeas(LPVOID comArgs);	// Main thread function to get real-time QTM measures
void get_SocketContent(ComStruct* comParams);		// Read and translate the content of the socket if needed
void readPipeContent(ComStruct* comParams);			// Read messages sent by control thread in the pipe
void writePipeContent(ComStruct* comParams, FILE* out_qtm_file);		// Write messages to robot control thread
void check_MovInitialisation(ComStruct* comParams);	// Check if a movement is about to happen
void ErrorExit(LPTSTR lpszFunction, FILE* out_qtm_file);
#endif // !GET_QTM_MEASURES_H