/***********************************************************************************************************************
* able_Control_QTMData.cpp -
*
* Author : Dorian Verdel
* Creation  date : 03/2021
*
* Description :
* Establishes the communication between ABLE thread and QTM thread.
***********************************************************************************************************************/

#include "able_Control_QTMData.h"

/*---------------------------------------------------------------------------------------------------------------------
| qtm_ReadData - Read current QTM data
|
| Syntax --
|	void qtm_ReadData(AbleControlStruct* ctrl_ABLE)
|
| Inputs --
|	AbleControlStruct *ctrl_ABLE -> pointer towards control of ABLE struct
----------------------------------------------------------------------------------------------------------------------*/
void qtm_ReadData(AbleControlStruct* ctrl_ABLE)
{
	// Variables declaration
	LPVOID buffer_pipe = new QTMToReadStruct;
	bool err;
	DWORD nBytesRead;
	DWORD nBytesToRead;
	QTMToReadStruct* readData;

	// Compute number of bytes to read
	nBytesToRead = sizeof(QTMToReadStruct);

	// Read the pipe
	err = ReadFile(ctrl_ABLE->pipeHandlesQTM.readPipeMeas, buffer_pipe, nBytesToRead, &nBytesRead, NULL);
	readData = (QTMToReadStruct*)buffer_pipe;

	// Store the data into control struct
	ctrl_ABLE->aDynamics.axis4_mod.x_slider = readData->s_pos;
	ctrl_ABLE->emgPing.ping_BB = readData->ping_BB;
	ctrl_ABLE->emgPing.ping_BR = readData->ping_BR;
	ctrl_ABLE->emgPing.ping_TBLatH = readData->ping_TBLatH;
	ctrl_ABLE->emgPing.ping_TBLongH = readData->ping_TBLongH;

	// Free memory from buffer_pipe
	delete buffer_pipe;
}

/*---------------------------------------------------------------------------------------------------------------------
| qtm_WriteData - Write messages to the qtm measures thread
|
| Syntax --
|	void qtm_WriteData(AbleControlStruct* ctrl_ABLE)
|
| Inputs --
|	AbleControlStruct *ctrl_ABLE -> pointer towards control of ABLE struct
|   int iter_counter -> number of the current iteration
----------------------------------------------------------------------------------------------------------------------*/
void qtm_WriteData(AbleControlStruct* ctrl_ABLE, int iter_counter)
{
	// Variables declaration
	bool err;
	DWORD nBytesWritten;
	DWORD nBytesToWrite;
	QTMToWriteStruct struct_ToWrite;

	// Retrieve useful informations
	struct_ToWrite.iter_counter = iter_counter;
	struct_ToWrite.robot_activated = ctrl_ABLE->rtParams.able_Connected;
	struct_ToWrite.mvt_ended = 0;

	// Compute number of bytes to write
	nBytesToWrite = sizeof(struct_ToWrite);

	// Write informations into communication pipe
	err = WriteFile(ctrl_ABLE->pipeHandlesQTM.writePipeCtrl, &struct_ToWrite, nBytesToWrite, &nBytesWritten, NULL);
}