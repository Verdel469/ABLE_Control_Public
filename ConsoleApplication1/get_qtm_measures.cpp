/***********************************************************************************************************************
* get_qtm_measures.cpp -
*
* Author : Dorian Verdel
* Creation  date : 02/2020
*
* Description :
* File containing the recuperation of the Qualisys and EMG measures in real-time
***********************************************************************************************************************/

#include "get_qtm_measures.h"

/*---------------------------------------------------------------------------------------------------------------------
| get_RealTimeQTMMeas - Main thread function to get real-time QTM measures
|
| Syntax --
|	DWORD WINAPI get_RealTimeQTMMeas(LPVOID comArgs)
|
| Inputs --
|	LPVOID comArgs -> void type pointer towards the structure containing all the required communication parameters
|
| Outputs --
|	DWORD -> 0 : Success
----------------------------------------------------------------------------------------------------------------------*/
DWORD WINAPI get_RealTimeQTMMeas(LPVOID comArgs)
{
	// Variables declaration and transformation
	int i = 0;
	static ComStruct* comParams = (ComStruct*)comArgs;
	FILE* out_qtm_file = NULL;
	fopen_s(&out_qtm_file, "out_qtm_thread.txt", "w");

	// Start to run
	comParams->comPipeStruct.robot_activated = 1;
	while (comParams->comPipeStruct.robot_activated && i < 500)
	{
		// Read the pipe from control thread
		//readPipeContent(comParams);
		// Get the content of the socket
		get_SocketContent(comParams);
		// Check if a movement is about to happen
		check_MovInitialisation(comParams);
		// Send measures to the control code
		writePipeContent(comParams, out_qtm_file);
		Sleep(5);
		i++;
	}
	fflush(out_qtm_file);
	return 0;
}

/*---------------------------------------------------------------------------------------------------------------------
| get_SocketContent - Function retreiving socket content if there is some
|
| Syntax --
|	int get_SocketContent(ComStruct* comParams)
|
| Inputs --
|	ComStruct* comParams -> pointer towards the structure containing all communication parameters
----------------------------------------------------------------------------------------------------------------------*/
void get_SocketContent(ComStruct* comParams)
{
	// Variables declaration
	char* endptr;
	char buffer[LENGTH_BUFFER];
	int err = 0;
	float current_pos;
	socketStruct* sockParams = &comParams->sockStruct;

	// Read socket if something has been transmitted
	//if (WSAEventSelect(sockParams->socket_com, recv, FD_READ) == 0)
	//{
	recv(sockParams->socket_com, buffer, LENGTH_BUFFER, NULL);

	// Split transmitted string and fill the structure with new values
	current_pos = strtof(strtok(buffer, " ;"), &endptr);
	if (current_pos != 0.0f)
	{
		comParams->qualMeas.sliderPos = current_pos;
	}
	comParams->emgMeas.emg_BB.push_back(strtof(strtok(buffer, " ;"), &endptr));
	comParams->emgMeas.emg_BR.push_back(strtof(strtok(buffer, " ;"), &endptr));
	comParams->emgMeas.emg_TBLoH.push_back(strtof(strtok(buffer, " ;"), &endptr));
	comParams->emgMeas.emg_TBLaH.push_back(strtof(strtok(buffer, " ;"), &endptr));
	//}
}

/*---------------------------------------------------------------------------------------------------------------------
| readPipeContent - Read messages sent by control thread in the pipe
|
| Syntax --
|	void readPipeContent(ComStruct* comParams)
|
| Inputs --
|	ComStruct* comParams -> pointer towards the structure containing all communication parameters
----------------------------------------------------------------------------------------------------------------------*/
void readPipeContent(ComStruct* comParams)
{
	// Variables declaration
	LPVOID buffer_pipe = new ReadStruct;
	bool err;
	DWORD nBytesRead;
	DWORD nBytesToRead;
	ReadStruct* readData;

	// Compute number of bytes to read
	nBytesToRead = sizeof(ReadStruct);

	// Read the pipe
	err = ReadFile(comParams->comPipeStruct.readPipeCtrl, buffer_pipe, nBytesToRead, &nBytesRead, NULL);
	readData = (ReadStruct*)buffer_pipe;
	comParams->comPipeStruct.iter_counter = readData->iter_counter;
	comParams->comPipeStruct.robot_activated = readData->robot_activated;

	// Free memory from buffer_pipe
	delete buffer_pipe;
}

/*---------------------------------------------------------------------------------------------------------------------
| writePipeContent - Write messages containing measures to the control thread
|
| Syntax --
|	void writePipeContent(ComStruct* comParams)
|
| Inputs --
|	ComStruct* comParams -> pointer towards the structure containing all communication parameters
----------------------------------------------------------------------------------------------------------------------*/
void writePipeContent(ComStruct* comParams, FILE* out_qtm_file)
{
	// Variables declaration
	bool err;
	DWORD nBytesWritten;
	DWORD nBytesToWrite;
	TransStruct struct_ToTransmit;
	LPVOID buffer1 = VirtualAlloc(NULL, sizeof(TransStruct), MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE);

	// Retrieve useful informations
	struct_ToTransmit.ping_BB = comParams->emgMeas.ping_BicepsBrachial;
	struct_ToTransmit.ping_BR = comParams->emgMeas.ping_BrachioRadialis;
	struct_ToTransmit.ping_TBLatH = comParams->emgMeas.ping_TricepsBrachialLatH;
	struct_ToTransmit.ping_TBLongH = comParams->emgMeas.ping_TricepsBrachialLongH;
	struct_ToTransmit.s_pos = comParams->qualMeas.sliderPos;

	// Compute number of bytes to write
	buffer1 = &struct_ToTransmit;
	nBytesToWrite = sizeof(struct_ToTransmit);

	// Write informations into communication pipe
	err = WriteFile(comParams->comPipeStruct.writePipeMeas, &struct_ToTransmit, nBytesToWrite, &nBytesWritten, NULL);
	fprintf(out_qtm_file, "Sortie WriteFile : %i ; nBytesToWrite-nBytesWritten = %i \n", err, nBytesToWrite - nBytesWritten);
}

/*---------------------------------------------------------------------------------------------------------------------
| check_MovInitialisation - Check if a movement is about to happen thanks to EMG measures
|
| Syntax --
|	void check_MovInitialisation(ComStruct* comParams)
|
| Inputs --
|	ComStruct* comParams -> pointer towards the structure containing all communication parameters
----------------------------------------------------------------------------------------------------------------------*/
void check_MovInitialisation(ComStruct* comParams)
{
	comParams->emgMeas.ping_BicepsBrachial = 0;
	comParams->emgMeas.ping_BrachioRadialis = 0;
	comParams->emgMeas.ping_TricepsBrachialLatH = 0;
	comParams->emgMeas.ping_TricepsBrachialLongH = 0;
}

void ErrorExit(LPTSTR lpszFunction, FILE* out_qtm_file)
{
	// Retrieve the system error message for the last-error code
	LPVOID lpMsgBuf;
	LPVOID lpDisplayBuf;
	DWORD dw = GetLastError();

	FormatMessage(
		FORMAT_MESSAGE_ALLOCATE_BUFFER |
		FORMAT_MESSAGE_FROM_SYSTEM |
		FORMAT_MESSAGE_IGNORE_INSERTS,
		NULL,
		dw,
		MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
		(LPTSTR)&lpMsgBuf,
		0, NULL);

	lpDisplayBuf = (LPVOID)LocalAlloc(LMEM_ZEROINIT,
		(lstrlen((LPCTSTR)lpMsgBuf) + lstrlen((LPCTSTR)lpszFunction) + 40) * sizeof(TCHAR));
	StringCchPrintf((LPTSTR)lpDisplayBuf,
		LocalSize(lpDisplayBuf) / sizeof(TCHAR),
		TEXT("%s failed with error %d: %s"),
		lpszFunction, dw, lpMsgBuf);

	// Display the error message and exit the process
	MessageBox(NULL, (LPCTSTR)lpDisplayBuf, TEXT("Error"), MB_OK);
	LocalFree(lpMsgBuf);
	LocalFree(lpDisplayBuf);
}
