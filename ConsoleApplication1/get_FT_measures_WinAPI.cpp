/***********************************************************************************************************************
* get_FT_measures_WinAPI.cpp -
*
* Author : Dorian Verdel
* Creation  date : 06/2020
*
* Description :
* Manages the communication with the ATI Force/Torque (FT) sensor directly through Windows API.
***********************************************************************************************************************/

#include "get_FT_measures_WinAPI.h"
#include <iostream>
#include <Windows.h>
#include <conio.h>
#include <string>
#include<intrin.h>

using namespace std;
using namespace std::chrono;

/*---------------------------------------------------------------------------------------------------------------------
| Definition of the CRC values to use for ModBus protocol.
----------------------------------------------------------------------------------------------------------------------*/
/* Table of CRC values for high-order byte */
static const uint8_t table_crc_hi[] = {
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};
/* Table of CRC values for low-order byte */
static const uint8_t table_crc_lo[] = {
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
	0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
	0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
	0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
	0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
	0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
	0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
	0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
	0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
	0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
	0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
	0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
	0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
	0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
	0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
	0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
	0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
	0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
	0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
	0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
	0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
	0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

/*---------------------------------------------------------------------------------------------------------------------
| crc16 - Function extracted from "Libmodbus" librairy to compute the CRC for modbus communication
|
| Syntax --
|	static uint16_t crc16(uint8_t* buffer, uint16_t buffer_length)
|
| Inputs --
|	uint8_t* buffer -> Buffer containing the request without CRC
|   uint16_t buffer_length -> Length of the request without CRC
|
| Outputs --
|	static uint16_t -> Computed CRC for modbus communication
----------------------------------------------------------------------------------------------------------------------*/
static uint16_t crc16(uint8_t* buffer, uint16_t buffer_length)
{
	uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
	uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
	unsigned int i; /* will index into CRC lookup */

	/* pass through message buffer */
	while (buffer_length--) {
		i = crc_hi ^ *buffer++; /* calculate the CRC  */
		crc_hi = crc_lo ^ table_crc_hi[i];
		crc_lo = table_crc_lo[i];
	}

	return (crc_hi << 8 | crc_lo);
}

/*---------------------------------------------------------------------------------------------------------------------
| initialize_FT_sensor - Completely initialize FT sensor before using it for control
|
| Syntax --
|	BOOL initialize_FT_sensor(FT_Comm_Struct* FT_Comm_params)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the struct containing all relevant communication parameters
|
| Outputs --
|	BOOL -> TRUE : Function finished normally ; FALSE : Error during communication initialization
----------------------------------------------------------------------------------------------------------------------*/
BOOL initialize_FT_sensor(FT_Comm_Struct* FT_Comm_params)
{
	// Initialize serial port communication
	if (!initialize_serial_port(FT_Comm_params)) { return FALSE; }
	// Initialize FT calibration
	if (!get_FT_calibration(FT_Comm_params)) { return FALSE; }
	// Set FT bias
	if (!get_and_set_bias(FT_Comm_params)) { return FALSE; }
	fprintf(FT_Comm_params->out_file_FT, "FT device successfully initialized.\n");
	fflush(FT_Comm_params->err_file_FT);
	fflush(FT_Comm_params->out_file_FT);
	return TRUE;
}

/*---------------------------------------------------------------------------------------------------------------------
| run_real_time_Measures - Runs real time FT measurement process
|
| Syntax --
|	DWORD WINAPI run_real_time_Measures(LPVOID FT_comm)
|
| Inputs --
|	LPVOID FT_comm -> pointer towards the struct containing all relevant communication parameters
|
| Outputs --
|	DWORD -> 0 : Function finished normally ; -1 : Error during communication initialization
----------------------------------------------------------------------------------------------------------------------*/
DWORD WINAPI run_real_time_Measures(LPVOID FT_comm)
{
	// Initialize variables
	FT_Comm_Struct* FT_Comm_params = (FT_Comm_Struct*)FT_comm;
	int ownership_val;
	BOOL buffer = FALSE;

	// Wait starting top from Control thread
	while (1)
	{
		// Try to get the ownership of the critical section
		ownership_val = TryEnterCriticalSection(FT_Comm_params->pCritical_share_FT);

		buffer = FT_Comm_params->FT_measures_Shared->streaming;

		// Release ownership of the critical section if obtained before
		if (ownership_val != 0)
		{
			LeaveCriticalSection(FT_Comm_params->pCritical_share_FT);
		}
		if (buffer)
		{
			break;
		}
		else {
			wait_between_CS_tries();
		}
	}
	// Start streaming
	if (!start_streaming_data(FT_Comm_params))
	{ 
		close_communication(FT_Comm_params);
		return -1;
	}
	close_communication(FT_Comm_params);
	return 0;
}

/*---------------------------------------------------------------------------------------------------------------------
| wait_between_CS_tries - Waiting time between two calls to TryEnterCriticalSection
|
| Syntax --
|	void wait_between_CS_tries()
----------------------------------------------------------------------------------------------------------------------*/
void wait_between_CS_tries()
{
	auto timestamp_1_w = high_resolution_clock::now();
	while (true)
	{
		auto timestamp_2_w = high_resolution_clock::now();
		duration<double> elapsed_w = timestamp_2_w - timestamp_1_w;
		if (elapsed_w.count() > 0.00001f)
		{
			break;
		}
	}
}

/*---------------------------------------------------------------------------------------------------------------------
| initialize_serial_port - Initialize communication parameters in agreement with digital FT sensor documentation
|
| Syntax --
|	BOOL initialize_serial_port(FT_Comm_Struct* FT_Comm_params)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the struct containing all relevant communication parameters
|
| Outputs --
|	BOOL -> TRUE : Function finished normally ; FALSE : Error during communication initialization
----------------------------------------------------------------------------------------------------------------------*/
BOOL initialize_serial_port(FT_Comm_Struct* FT_Comm_params)
{
	DWORD error;
	// Initialise serial port name
	if (FT_Comm_params->general_params_FT.struct_FT_Arm)
	{
		FT_Comm_params->general_params_FT.serial_port_name = SERIAL_PORT_NAME_ARM;
	} else if (FT_Comm_params->general_params_FT.struct_FT_Wrist)
	{
		FT_Comm_params->general_params_FT.serial_port_name = SERIAL_PORT_NAME_WRIST;
	}
	// Create serial port
	FT_Comm_params->serial_port = CreateFile(FT_Comm_params->general_params_FT.serial_port_name,
											 GENERIC_READ | GENERIC_WRITE,
											 0,
											 0,
											 OPEN_EXISTING,
											 FILE_ATTRIBUTE_NORMAL,
											 0);
	if (FT_Comm_params->serial_port == INVALID_HANDLE_VALUE)
	{
		fprintf(FT_Comm_params->err_file_FT,"Error openning serial port\n");
		return FALSE;
	}
	// Get default properties of serial port communication
	if (!GetCommState(FT_Comm_params->serial_port, &FT_Comm_params->FT_Comm_DCB))
	{
		error = GetLastError();
		FT_ErrorExit(FT_Comm_params, error);
		fprintf(FT_Comm_params->err_file_FT, "Error in getting communication state.\n");
		return FALSE;
	}
	// Fill constructor parameters for communication
	FT_Comm_params->FT_Comm_DCB.BaudRate = BAUD_RATE;
	FT_Comm_params->FT_Comm_DCB.Parity = EVENPARITY;
	FT_Comm_params->FT_Comm_DCB.ByteSize = MB_BYTE_SIZE;
	FT_Comm_params->FT_Comm_DCB.StopBits = ONESTOPBIT;
	// Set parameters on serial port communication
	if (!SetCommState(FT_Comm_params->serial_port, &FT_Comm_params->FT_Comm_DCB))
	{
		error = GetLastError();
		FT_ErrorExit(FT_Comm_params, error);
		fprintf(FT_Comm_params->err_file_FT, "Error in setting communication state.\n");
		return FALSE;
	}
	// Get default communication timeouts
	GetCommTimeouts(FT_Comm_params->serial_port, &FT_Comm_params->FT_Comm_timeouts);
	// Fill timeouts of the communication
	FT_Comm_params->FT_Comm_timeouts.ReadIntervalTimeout = 20;
	FT_Comm_params->FT_Comm_timeouts.ReadTotalTimeoutMultiplier = 20;
	FT_Comm_params->FT_Comm_timeouts.ReadTotalTimeoutConstant = 100;
	// Set timeouts on serial port communication
	if (!SetCommTimeouts(FT_Comm_params->serial_port, &FT_Comm_params->FT_Comm_timeouts))
	{
		error = GetLastError();
		FT_ErrorExit(FT_Comm_params, error);
		fprintf(FT_Comm_params->err_file_FT, "Error in setting communication timeouts.\n");
		return FALSE;
	}
	// Initialize communication
	initiate_Com(FT_Comm_params);
	// Flush files and return TRUE if no errors
	fprintf(FT_Comm_params->out_file_FT, "Communication normally established with FT Sensor.\n");
	fflush(FT_Comm_params->err_file_FT);
	fflush(FT_Comm_params->out_file_FT);
	return TRUE;
}

/*---------------------------------------------------------------------------------------------------------------------
| initiate_Com - Send initialization message to the device
|
| Syntax --
|	void initiate_Com(FT_Comm_Struct* FT_Comm_params)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the struct containing all relevant communication parameters
----------------------------------------------------------------------------------------------------------------------*/
void initiate_Com(FT_Comm_Struct* FT_Comm_params)
{
	// Initialize variables
	DWORD error, numTransferred;
	COMSTAT stat = { 0 };
	FT_Comm_params->stat = stat;

	// Initialize communication
	ClearCommError(FT_Comm_params->serial_port, &error, &FT_Comm_params->stat);
	WriteFile(FT_Comm_params->serial_port, "a", 1, &numTransferred, NULL);
	stop_streaming(FT_Comm_params);

	COMMPROP props = { 0 };
	GetCommProperties(FT_Comm_params->serial_port, &props);

	ClearCommError(FT_Comm_params->serial_port, &error, &FT_Comm_params->stat);
}

/*---------------------------------------------------------------------------------------------------------------------
| get_FT_calibration - Get the calibration of the device and apply gauge gains and offsets
|
| Syntax --
|	BOOL get_FT_calibration(FT_Comm_Struct* FT_Comm_params)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the struct containing all relevant communication parameters
|
| Outputs --
|	BOOL -> TRUE : Function finished normally ; FALSE : Error while getting FT calibration
----------------------------------------------------------------------------------------------------------------------*/
BOOL get_FT_calibration(FT_Comm_Struct* FT_Comm_params)
{
	// Initialize calibration
	if (FT_Comm_params->general_params_FT.struct_FT_Arm)
	{
		fill_Calib_struct_Arm(FT_Comm_params);
	}
	else if (FT_Comm_params->general_params_FT.struct_FT_Wrist)
	{
		fill_Calib_struct_Wrist(FT_Comm_params);
	}
	print_calibration(FT_Comm_params);
	// Initialize variables
	DWORD numTransferred, error;
	uint16_t gauge_gain_CRC;
	uint16_t gauge_offset_CRC;
	uint16_t nb_registers = 0x0006;
	// Build write gains request content
	uint8_t gain_pre_request[19];
	gain_pre_request[0] = FT_SENSOR_ADRESS;
	gain_pre_request[1] = MODBUS_WRITE_N_REGISTERS_FN;
	gain_pre_request[2] = GAIN_REGISTERS_ADDRESS_START >> 8;
	gain_pre_request[3] = GAIN_REGISTERS_ADDRESS_START & 0x00FF;
	gain_pre_request[4] = nb_registers >> 8;
	gain_pre_request[5] = nb_registers & 0x00FF;
	gain_pre_request[6] = 0x0C;
	
	for (int i(7), j(0); i < 19; i += 2, j++)
	{
		gain_pre_request[i] = FT_Comm_params->FT_Calib.GaugeGain[j] >> 8;
		gain_pre_request[i+1] = FT_Comm_params->FT_Calib.GaugeGain[j] & 0x00FF;
	}
	// Build write offsets request content
	uint8_t offset_pre_request[19];
	offset_pre_request[0] = FT_SENSOR_ADRESS;
	offset_pre_request[1] = MODBUS_WRITE_N_REGISTERS_FN;
	offset_pre_request[2] = OFFSET_REGISTERS_ADDRESS_START >> 8;
	offset_pre_request[3] = OFFSET_REGISTERS_ADDRESS_START & 0x00FF;
	offset_pre_request[4] = nb_registers >> 8;
	offset_pre_request[5] = nb_registers & 0x00FF;
	offset_pre_request[6] = 0x0C;

	for (int i(7), j(0); i < 19; i += 2, j++)
	{
		offset_pre_request[i] = FT_Comm_params->FT_Calib.GaugeOffset[j] >> 8;
		offset_pre_request[i + 1] = FT_Comm_params->FT_Calib.GaugeOffset[j] & 0x00FF;
	}
	// Initialize full requests and answers
	uint8_t gain_request[21];
	uint8_t offset_request[21];
	uint8_t gain_response[8];
	uint8_t offset_response[8];
	
	// Unlock holding registers
	if (!lock_unlock_holding_registers(FT_Comm_params, FALSE)) { return FALSE; }

	// Compute both CRC to read holding registers
	gauge_gain_CRC = crc16(gain_pre_request, 19);
	gauge_offset_CRC = crc16(offset_pre_request, 19);
	// Add CRC to low request
	memcpy(gain_request, gain_pre_request, 19);
	gain_request[19] = (gauge_gain_CRC >> 8) & 0x00FF;
	gain_request[20] = gauge_gain_CRC & 0x00FF;
	// Add CRC to high request
	memcpy(offset_request, offset_pre_request, 19);
	offset_request[19] = (gauge_offset_CRC >> 8) & 0x00FF;
	offset_request[20] = gauge_offset_CRC & 0x00FF;

	// Clear communication errors
	ClearCommError(FT_Comm_params->serial_port, &error, &FT_Comm_params->stat);

	fprintf(FT_Comm_params->out_file_FT, "Write gauge gain registers command : {");
	for (int i(0); i < 21; i++)
	{
		fprintf(FT_Comm_params->out_file_FT, " %d", gain_request[i]);
	}
	fprintf(FT_Comm_params->out_file_FT, " }\n");
	fflush(FT_Comm_params->out_file_FT);

	// Send request to write gains
	if (!WriteFile(FT_Comm_params->serial_port, gain_request, 21, &numTransferred, NULL))
	{
		error = GetLastError();
		FT_ErrorExit(FT_Comm_params, error);
		fprintf(FT_Comm_params->err_file_FT, "Error in Writing request : write gain registers\n");
		return FALSE;
	}
	// Read answer
	if (!ReadFile(FT_Comm_params->serial_port, gain_response, 8, &numTransferred, NULL))
	{
		error = GetLastError();
		FT_ErrorExit(FT_Comm_params, error);
		fprintf(FT_Comm_params->err_file_FT, "Error reading MODBUS reponse. Transfered bytes : %i\n", numTransferred);
		return FALSE;
	}
	// Clear communication errors
	ClearCommError(FT_Comm_params->serial_port, &error, &FT_Comm_params->stat);

	fprintf(FT_Comm_params->out_file_FT, "Write gauge offset registers command : {");
	for (int i(0); i < 21; i++)
	{
		fprintf(FT_Comm_params->out_file_FT, " %d", offset_request[i]);
	}
	fprintf(FT_Comm_params->out_file_FT, " }\n");

	// Send request to write offsets
	if (!WriteFile(FT_Comm_params->serial_port, offset_request, 21, &numTransferred, NULL))
	{
		error = GetLastError();
		FT_ErrorExit(FT_Comm_params, error);
		fprintf(FT_Comm_params->err_file_FT, "Error in Writing request : write offset registers\n");
		return FALSE;
	}
	// Read answer
	if (!ReadFile(FT_Comm_params->serial_port, offset_response, 8, &numTransferred, NULL))
	{
		error = GetLastError();
		FT_ErrorExit(FT_Comm_params, error);
		fprintf(FT_Comm_params->err_file_FT, "Error reading MODBUS reponse. Transfered bytes : %i\n", numTransferred);
		return FALSE;
	}
	// Lock holding registers
	if (!lock_unlock_holding_registers(FT_Comm_params, TRUE)) {	return FALSE; }
	fprintf(FT_Comm_params->out_file_FT, "FT Calibration successfully retrieved !\n");
	return TRUE;
}

/*---------------------------------------------------------------------------------------------------------------------
| lock_unlock_holding_registers - Function to lock/unlock holding registers (containing calibration, gauge gains, ...)
|
| Syntax --
|	BOOL lock_unlock_holding_registers(FT_Comm_Struct* FT_Comm_params, BOOL lock_unlock)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the struct containing all relevant communication parameters
|   BOOL lock_unlock -> TRUE : LOCK holding registers ; FALSE : UNLOCK holding registers
|
| Outputs --
|	BOOL -> TRUE : Function finished normally ; FALSE : Error while unlocking registers
----------------------------------------------------------------------------------------------------------------------*/
BOOL lock_unlock_holding_registers(FT_Comm_Struct* FT_Comm_params, BOOL lock_unlock)
{
	// Initialize data
	uint8_t data;
	uint8_t response_data;
	if (lock_unlock)
	{
		data = LOCK_FT_REGISTERS_DATA;
		fprintf(FT_Comm_params->out_file_FT, "Lock registers required\n");
	} else {
		data = UNLOCK_FT_REGISTERS_DATA;
		fprintf(FT_Comm_params->out_file_FT, "Unlock registers required\n");
	}
	// Send custom function
	if (!send_custom_function(FT_Comm_params, LOCK_UNLOCK_FT_REGISTERS_FN, &data, 1))
	{
		if (data == LOCK_FT_REGISTERS_DATA)
		{
			fprintf(FT_Comm_params->err_file_FT, "Error while locking holding registers.\n");
		}
		else if (data == UNLOCK_FT_REGISTERS_DATA) {
			fprintf(FT_Comm_params->err_file_FT, "Error while unlocking holding registers.\n");
		}
		return FALSE;
	}
	// Check response
	response_data = FT_Comm_params->response[2];
	if (response_data != 0x01)
	{
		fprintf(FT_Comm_params->err_file_FT, "Modbus error while unlocking holding registers : %d\n", response_data);
		return FALSE;
	}
	fprintf(FT_Comm_params->out_file_FT, "Holding registers successfully locked / unlocked\n");
	return TRUE;
}

/*---------------------------------------------------------------------------------------------------------------------
| send_custom_function - Send functions to unlock/lock registers
|
| Syntax --
|	BOOL send_custom_function(FT_Comm_Struct* FT_Comm_params, uint8_t fn_code, uint8_t* data, int data_length)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the struct containing all relevant communication parameters
|   uint8_t fn_code -> Modbus : code of the function to execute
|   uint8_t* data   -> Modbus : data associated with the function
|   int data_length -> Length of data
|
| Outputs --
|	BOOL -> TRUE : Function finished normally ; FALSE : Error while sending custom function
----------------------------------------------------------------------------------------------------------------------*/
BOOL send_custom_function(FT_Comm_Struct* FT_Comm_params, uint8_t fn_code, uint8_t* data, int data_length)
{
	// Initialize variables and request
	DWORD numTransferred, error;
	uint16_t computed_crc;
	uint16_t pre_req_length = 2 + data_length;
	uint8_t pre_request[MODBUS_MAX_MSG_LENGTH];
	// Fill pre-request
	pre_request[0] = FT_SENSOR_ADRESS;
	pre_request[1] = fn_code;
	memcpy(pre_request + 2, data, data_length);
	// Compute CRC 16
	computed_crc = crc16(pre_request, pre_req_length);
	// Add CRC at the end of the request
	// Compute and add CRC high
	uint8_t crc_high = (computed_crc >> 8) & 0x00FF;      // Transform : 0xABCD to 0xCDAB, keep only 0xAB (CRC High)
	pre_request[pre_req_length] = crc_high;				  // Add CRC high at the end of the request
	// Compute and add CRC low
	uint8_t crc_low = computed_crc & 0x00FF;              // Keep only 0xCD from 0xABCD
	pre_req_length++;                                     // Add one byte to the length of the message
	pre_request[pre_req_length] = crc_low;                // Add CRC low after CRC high

	// Print CRC for verification
	fprintf(FT_Comm_params->out_file_FT, "Computed CRC High : %d\n", crc_high);
	fprintf(FT_Comm_params->out_file_FT, "Computed CRC Low : %d\n", crc_low);

	// Copy into adjusted table
	uint8_t* request = new uint8_t[4 + data_length];
	memcpy(request, pre_request, 4 + data_length);

	fprintf(FT_Comm_params->out_file_FT, "Custom request sent to device : {");
	for (int i(0); i < 4 + data_length; i++)
	{
		fprintf(FT_Comm_params->out_file_FT, " %d", request[i]);
	}
	fprintf(FT_Comm_params->out_file_FT, " }\n");
	// Clear communication errors
	COMSTAT stat;
	ClearCommError(FT_Comm_params->serial_port, &error, &stat);

	// Send request
	if (!WriteFile(FT_Comm_params->serial_port, request, 4 + data_length, &numTransferred, NULL)) {
		error = GetLastError();
		FT_ErrorExit(FT_Comm_params, error);
		fprintf(FT_Comm_params->err_file_FT, "Error in Writing custom request with code : %d\n", fn_code);
		delete[] request;
		return FALSE;
	}
	delete[] request;
	// Read response
	if (!ReadFile(FT_Comm_params->serial_port, FT_Comm_params->response, 2 * (4 + data_length), &numTransferred, NULL)
		|| (4 + data_length != numTransferred))
	{
		error = GetLastError();
		FT_ErrorExit(FT_Comm_params, error);
		fprintf(FT_Comm_params->err_file_FT,
			"Error reading MODBUS reponse. Transfered bytes : %i\n", numTransferred);
		//fprintf(FT_Comm_params->err_file_FT, "La lecture qui plante est ici\n");
		return FALSE;
	}

	fprintf(FT_Comm_params->out_file_FT, "Custom request with code %d sent and response received.\n", fn_code);
	return TRUE;
}

/*---------------------------------------------------------------------------------------------------------------------
| fill_Calib_struct_Arm - Fills the calibration struct of Arm FT sensor according to constructor
|
| Syntax --
|	BOOL fill_Calib_struct_Arm(FT_Comm_Struct* FT_Comm_params)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the struct containing all relevant communication parameters
|
| Outputs --
|	BOOL -> TRUE : Function finished normally ; FALSE : Error while filling calibration struct
----------------------------------------------------------------------------------------------------------------------*/
BOOL fill_Calib_struct_Arm(FT_Comm_Struct* FT_Comm_params)
{
	// Fill serial number
	uint8_t* serial = (uint8_t*)"FT33206";
	for (int i(0); i < sizeof(serial); i++)
	{
		FT_Comm_params->FT_Calib.CalibSerialNumber[i] = serial[i];
	}
	// Fill part number
	uint8_t* part = (uint8_t*)"SI-130-10";
	for (int i(0); i < sizeof(part); i++)
	{
		FT_Comm_params->FT_Calib.CalibPartNumber[i] = part[i];
	}
	// Fill family ID
	uint8_t* fId = (uint8_t*)"Net";
	for (int i(0); i < sizeof(fId); i++)
	{
		FT_Comm_params->FT_Calib.CalibFamilyId[i] = fId[i];
	}
	// Fill time
	uint8_t* time = (uint8_t*)"2021-01-1905:00:00Z";
	for (int i(0); i < sizeof(time); i++)
	{
		FT_Comm_params->FT_Calib.CalibTime[i] = time[i];
	}
	//Fill basic matrix
	FT_Comm_params->FT_Calib.BasicMatrix[0][0] = -1.395115f;
	FT_Comm_params->FT_Calib.BasicMatrix[1][0] = -25.79711f;
	FT_Comm_params->FT_Calib.BasicMatrix[2][0] = 7571.881f;
	FT_Comm_params->FT_Calib.BasicMatrix[3][0] = 0.4456577f;
	FT_Comm_params->FT_Calib.BasicMatrix[4][0] = 253.174f;
	FT_Comm_params->FT_Calib.BasicMatrix[5][0] = 0.1613185f;
	FT_Comm_params->FT_Calib.BasicMatrix[0][1] = 6.06166f;
	FT_Comm_params->FT_Calib.BasicMatrix[1][1] = 4918.066f;
	FT_Comm_params->FT_Calib.BasicMatrix[2][1] = 108.7123f;
	FT_Comm_params->FT_Calib.BasicMatrix[3][1] = 59.43468f;
	FT_Comm_params->FT_Calib.BasicMatrix[4][1] = 3.163722f;
	FT_Comm_params->FT_Calib.BasicMatrix[5][1] = -133.4189f;
	FT_Comm_params->FT_Calib.BasicMatrix[0][2] = -86.49169f;
	FT_Comm_params->FT_Calib.BasicMatrix[1][2] = -42.36156f;
	FT_Comm_params->FT_Calib.BasicMatrix[2][2] = 7652.86f;
	FT_Comm_params->FT_Calib.BasicMatrix[3][2] = -220.4258f;
	FT_Comm_params->FT_Calib.BasicMatrix[4][2] = -125.4127f;
	FT_Comm_params->FT_Calib.BasicMatrix[5][2] = -2.528893f;
	FT_Comm_params->FT_Calib.BasicMatrix[0][3] = -4247.508f;
	FT_Comm_params->FT_Calib.BasicMatrix[1][3] = -2453.942f;
	FT_Comm_params->FT_Calib.BasicMatrix[2][3] = 21.27031f;
	FT_Comm_params->FT_Calib.BasicMatrix[3][3] = -30.04249f;
	FT_Comm_params->FT_Calib.BasicMatrix[4][3] = 51.15543f;
	FT_Comm_params->FT_Calib.BasicMatrix[5][3] = -133.9306f;
	FT_Comm_params->FT_Calib.BasicMatrix[0][4] = -12.23112f;
	FT_Comm_params->FT_Calib.BasicMatrix[1][4] = -1.432269f;
	FT_Comm_params->FT_Calib.BasicMatrix[2][4] = 7602.011f;
	FT_Comm_params->FT_Calib.BasicMatrix[3][4] = 219.9759f;
	FT_Comm_params->FT_Calib.BasicMatrix[4][4] = -127.074f;
	FT_Comm_params->FT_Calib.BasicMatrix[5][4] = 0.7586892f;
	FT_Comm_params->FT_Calib.BasicMatrix[0][5] = 4223.725f;
	FT_Comm_params->FT_Calib.BasicMatrix[1][5] = -2445.755f;
	FT_Comm_params->FT_Calib.BasicMatrix[2][5] = 60.34784f;
	FT_Comm_params->FT_Calib.BasicMatrix[3][5] = -27.8555f;
	FT_Comm_params->FT_Calib.BasicMatrix[4][5] = -51.90575f;
	FT_Comm_params->FT_Calib.BasicMatrix[5][5] = -133.654f;

	// Fill Force units
	FT_Comm_params->FT_Calib.ForceUnits = 2;
	// Fill Torque units
	FT_Comm_params->FT_Calib.TorqueUnits = 3;

	// Fill max rating
	FT_Comm_params->FT_Calib.MaxRating[0] = 130.0f;
	FT_Comm_params->FT_Calib.MaxRating[1] = 130.0f;
	FT_Comm_params->FT_Calib.MaxRating[2] = 400.0f;
	FT_Comm_params->FT_Calib.MaxRating[3] = 10.0f;
	FT_Comm_params->FT_Calib.MaxRating[4] = 10.0f;
	FT_Comm_params->FT_Calib.MaxRating[5] = 10.0f;

	// Fill counts per force
	FT_Comm_params->FT_Calib.CountsPerForce = 1000000;
	// Fill counts per torque
	FT_Comm_params->FT_Calib.CountsPerTorque = 1000000;

	// Fill gauge gain
	FT_Comm_params->FT_Calib.GaugeGain[0] = 543;
	FT_Comm_params->FT_Calib.GaugeGain[1] = 555;
	FT_Comm_params->FT_Calib.GaugeGain[2] = 569;
	FT_Comm_params->FT_Calib.GaugeGain[3] = 579;
	FT_Comm_params->FT_Calib.GaugeGain[4] = 565;
	FT_Comm_params->FT_Calib.GaugeGain[5] = 567;

	// Fill gauge offset
	FT_Comm_params->FT_Calib.GaugeOffset[0] = 34927;
	FT_Comm_params->FT_Calib.GaugeOffset[1] = 33031;
	FT_Comm_params->FT_Calib.GaugeOffset[2] = 32120;
	FT_Comm_params->FT_Calib.GaugeOffset[3] = 32868;
	FT_Comm_params->FT_Calib.GaugeOffset[4] = 32234;
	FT_Comm_params->FT_Calib.GaugeOffset[5] = 31842;

	fprintf(FT_Comm_params->out_file_FT, "Calibration struct successfully filled.\n");
	return TRUE;
}

/*---------------------------------------------------------------------------------------------------------------------
| fill_Calib_struct_Wrist - Fills the calibration struct of Wrist FT sensor according to constructor
|
| Syntax --
|	BOOL fill_Calib_struct_Wrist(FT_Comm_Struct* FT_Comm_params)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the struct containing all relevant communication parameters
|
| Outputs --
|	BOOL -> TRUE : Function finished normally ; FALSE : Error while filling calibration struct
----------------------------------------------------------------------------------------------------------------------*/
BOOL fill_Calib_struct_Wrist(FT_Comm_Struct* FT_Comm_params)
{
	// Fill serial number
	uint8_t* serial = (uint8_t*)"FT29020";
	for (int i(0); i < sizeof(serial); i++)
	{
		FT_Comm_params->FT_Calib.CalibSerialNumber[i] = serial[i];
	}
	// Fill part number
	uint8_t* part = (uint8_t*)"SI-130-10";
	for (int i(0); i <sizeof(part); i++)
	{
		FT_Comm_params->FT_Calib.CalibPartNumber[i] = part[i];
	}
	// Fill family ID
	uint8_t* fId = (uint8_t*)"Net";
	for (int i(0); i < sizeof(fId); i++)
	{
		FT_Comm_params->FT_Calib.CalibFamilyId[i] = fId[i];
	}
	// Fill time
	uint8_t* time = (uint8_t*)"2019-09-2404:00:00Z";
	for (int i(0); i < sizeof(time); i++)
	{
		FT_Comm_params->FT_Calib.CalibTime[i] = time[i];
	}
	//Fill basic matrix
	FT_Comm_params->FT_Calib.BasicMatrix[0][0] = -21.65866f;
	FT_Comm_params->FT_Calib.BasicMatrix[0][1] = -7.05528f;
	FT_Comm_params->FT_Calib.BasicMatrix[0][2] = 179.2194f;
	FT_Comm_params->FT_Calib.BasicMatrix[0][3] = -4130.669f;
	FT_Comm_params->FT_Calib.BasicMatrix[0][4] = -225.4132f;
	FT_Comm_params->FT_Calib.BasicMatrix[0][5] = 4352.427f;
	FT_Comm_params->FT_Calib.BasicMatrix[1][0] = -173.9342f;
	FT_Comm_params->FT_Calib.BasicMatrix[1][1] = 4945.822f;
	FT_Comm_params->FT_Calib.BasicMatrix[1][2] = 41.69561f;
	FT_Comm_params->FT_Calib.BasicMatrix[1][3] = -2389.772f;
	FT_Comm_params->FT_Calib.BasicMatrix[1][4] = 145.0311f;
	FT_Comm_params->FT_Calib.BasicMatrix[1][5] = -2511.788f;
	FT_Comm_params->FT_Calib.BasicMatrix[2][0] = 7580.017f;
	FT_Comm_params->FT_Calib.BasicMatrix[2][1] = 80.29717f;
	FT_Comm_params->FT_Calib.BasicMatrix[2][2] = 7608.285f;
	FT_Comm_params->FT_Calib.BasicMatrix[2][3] = 27.53107f;
	FT_Comm_params->FT_Calib.BasicMatrix[2][4] = 7631.579f;
	FT_Comm_params->FT_Calib.BasicMatrix[2][5] = -136.408f;
	FT_Comm_params->FT_Calib.BasicMatrix[3][0] = -1.765515f;
	FT_Comm_params->FT_Calib.BasicMatrix[3][1] = 59.95308f;
	FT_Comm_params->FT_Calib.BasicMatrix[3][2] = -218.6693f;
	FT_Comm_params->FT_Calib.BasicMatrix[3][3] = -29.81287f;
	FT_Comm_params->FT_Calib.BasicMatrix[3][4] = 220.8024f;
	FT_Comm_params->FT_Calib.BasicMatrix[3][5] = -34.0229f;
	FT_Comm_params->FT_Calib.BasicMatrix[4][0] = 253.399f;
	FT_Comm_params->FT_Calib.BasicMatrix[4][1] = 3.109644f;
	FT_Comm_params->FT_Calib.BasicMatrix[4][2] = -129.8972f;
	FT_Comm_params->FT_Calib.BasicMatrix[4][3] = 49.56017f;
	FT_Comm_params->FT_Calib.BasicMatrix[4][4] = -124.2143f;
	FT_Comm_params->FT_Calib.BasicMatrix[4][5] = -50.96896f;
	FT_Comm_params->FT_Calib.BasicMatrix[5][0] = 5.579048f;
	FT_Comm_params->FT_Calib.BasicMatrix[5][1] = -135.5618f;
	FT_Comm_params->FT_Calib.BasicMatrix[5][2] = 4.468352f;
	FT_Comm_params->FT_Calib.BasicMatrix[5][3] = -130.5733f;
	FT_Comm_params->FT_Calib.BasicMatrix[5][4] = 6.550929f;
	FT_Comm_params->FT_Calib.BasicMatrix[5][5] = -137.4021f;

	// Fill Force units
	FT_Comm_params->FT_Calib.ForceUnits = 2;
	// Fill Torque units
	FT_Comm_params->FT_Calib.TorqueUnits = 3;
	
	// Fill max rating
	FT_Comm_params->FT_Calib.MaxRating[0] = 130.0f;
	FT_Comm_params->FT_Calib.MaxRating[1] = 130.0f;
	FT_Comm_params->FT_Calib.MaxRating[2] = 400.0f;
	FT_Comm_params->FT_Calib.MaxRating[3] = 10.0f;
	FT_Comm_params->FT_Calib.MaxRating[4] = 10.0f;
	FT_Comm_params->FT_Calib.MaxRating[5] = 10.0f;

	// Fill counts per force
	FT_Comm_params->FT_Calib.CountsPerForce = 1000000;
	// Fill counts per torque
	FT_Comm_params->FT_Calib.CountsPerTorque = 1000000;

	// Fill gauge gain
	FT_Comm_params->FT_Calib.GaugeGain[0] = 537;
	FT_Comm_params->FT_Calib.GaugeGain[1] = 577;
	FT_Comm_params->FT_Calib.GaugeGain[2] = 541;
	FT_Comm_params->FT_Calib.GaugeGain[3] = 545;
	FT_Comm_params->FT_Calib.GaugeGain[4] = 541;
	FT_Comm_params->FT_Calib.GaugeGain[5] = 569;

	// Fill gauge offset
	FT_Comm_params->FT_Calib.GaugeOffset[0] = 32754;
	FT_Comm_params->FT_Calib.GaugeOffset[1] = 34484;
	FT_Comm_params->FT_Calib.GaugeOffset[2] = 33526;
	FT_Comm_params->FT_Calib.GaugeOffset[3] = 32794;
	FT_Comm_params->FT_Calib.GaugeOffset[4] = 33185;
	FT_Comm_params->FT_Calib.GaugeOffset[5] = 32083;

	fprintf(FT_Comm_params->out_file_FT, "Calibration struct successfully filled.\n");
	return TRUE;
}

/*---------------------------------------------------------------------------------------------------------------------
| get_and_set_bias - Starts to read the data stream (7kHz) until a stop streaming order is sent
|
| Syntax --
|	BOOL get_and_set_bias(FT_Comm_Struct* FT_Comm_params)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the struct containing all relevant communication parameters
|
| Outputs --
|	BOOL -> TRUE : Function finished normally ; FALSE : Error while streaming data
----------------------------------------------------------------------------------------------------------------------*/
BOOL get_and_set_bias(FT_Comm_Struct* FT_Comm_params)
{
	// Initialize variables
	All_FT_measures* All_FT = &FT_Comm_params->FT_measures;
	All_FT->use_bias = FALSE;
	if (!FT_Comm_params->general_params_FT.bias_identified)
	{
		// Stream data for 1 second to get bias
		if (!start_streaming_data(FT_Comm_params)) { return FALSE; }
		fprintf(FT_Comm_params->out_file_FT, "Number of saved elements after bias streaming : %d\n", All_FT->g0_values.size());
		// Compute bias value and store it
		set_FT_bias(FT_Comm_params);
		// Save bias
		if (FT_Comm_params->general_params_FT.struct_FT_Arm)
		{
			fopen_s(&FT_Comm_params->bias_vector_file, "bias_vector_Arm.txt", "w");
		}
		else if (FT_Comm_params->general_params_FT.struct_FT_Wrist)
		{
			fopen_s(&FT_Comm_params->bias_vector_file, "bias_vector_Wrist.txt", "w");
		}
		for (int i(0); i < 6; i++)
		{
			fprintf(FT_Comm_params->bias_vector_file, "%i ", FT_Comm_params->FT_measures.id_bias[i]);
		}
		fclose(FT_Comm_params->bias_vector_file);
	} else {
		retrieve_IdentifiedBias(FT_Comm_params);
	}
	// Print obtained bias
	fprintf(FT_Comm_params->out_file_FT, " Bias vector identified : { ");
	for (int i(0); i < 6; i++)
	{
		fprintf(FT_Comm_params->out_file_FT, "%d ", All_FT->id_bias[i]);
	}
	fprintf(FT_Comm_params->out_file_FT, "}\n");
	All_FT->use_bias = TRUE;
	return TRUE;
}

/*---------------------------------------------------------------------------------------------------------------------
| retrieve_IdentifiedBias - Retrieve identified bias vector
|
| Syntax --
|	void retrieve_IdentifiedBias(FT_Comm_Struct* FT_Comm_params)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the structure containing all the informations
----------------------------------------------------------------------------------------------------------------------*/
void retrieve_IdentifiedBias(FT_Comm_Struct* FT_Comm_params)
{
	// Initialize variables
	All_FT_measures* All_FT = &FT_Comm_params->FT_measures;
	FILE* h_File = FT_Comm_params->bias_vector_file;

	if (fscanf(h_File, "%hi %hi %hi %hi %hi %hi", &All_FT->id_bias[0], &All_FT->id_bias[1], &All_FT->id_bias[2],
		                                          &All_FT->id_bias[3], &All_FT->id_bias[4], &All_FT->id_bias[5]) != 6)
	{
		fprintf(FT_Comm_params->err_file_FT, "Error reading pre-existing bias.\n");
		FT_Comm_params->general_params_FT.bias_identified = 0;
		get_and_set_bias(FT_Comm_params);
	}
	fclose(h_File);
}


/*---------------------------------------------------------------------------------------------------------------------
| start_reading_data_stream - Starts to read the data stream (7kHz) until a stop streaming order is sent
|
| Syntax --
|	BOOL start_streaming_data(FT_Comm_Struct* FT_Comm_params)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the struct containing all relevant communication parameters
|
| Outputs --
|	BOOL -> TRUE : Function finished normally ; FALSE : Error while streaming data
----------------------------------------------------------------------------------------------------------------------*/
BOOL start_streaming_data(FT_Comm_Struct* FT_Comm_params)
{
	// Initialize variables
	int numsamples;
	unsigned char streamCommand[] = { 10, 70, 0x55, 0xA3, 0x9D };
	DWORD numTransferred, error;
	Current_FT_measures* Current_FT = &FT_Comm_params->FT_measures.Current_measures;

	// Initialize communication
	//initiate_Com(FT_Comm_params);
	// Require diagnosis
	//if (!get_FT_diagnosis(FT_Comm_params)) { return FALSE; }
	fprintf(FT_Comm_params->out_file_FT, "Start streaming data called...\n");
	// Send start streaming command
	WriteFile(FT_Comm_params->serial_port, streamCommand, sizeof(streamCommand), &numTransferred, NULL);
	fprintf(FT_Comm_params->out_file_FT, "Start streaming data command sent to FT sensor...\n");
	// Check modbus response
	if (!ReadFile(FT_Comm_params->serial_port, &Current_FT->sample, 5, &numTransferred, NULL) || (5 != numTransferred))
	{
		error = GetLastError();
		FT_ErrorExit(FT_Comm_params, error);
		fprintf(FT_Comm_params->err_file_FT,
			"Error reading MODBUS reponse. Transfered bytes : %i\n", numTransferred);
		return FALSE;
	}
	fprintf(FT_Comm_params->out_file_FT, "Response to start streaming command successfully received...\n");
	// Check if bias identification phase or FT streaming to control ABLE
	if (!FT_Comm_params->FT_measures.use_bias)
	{
		numsamples = BIAS_ID_N_SAMPLES;
	}else
	{
		numsamples = FT_Comm_params->FT_measures.nb_measures;
	}
	// Start streaming loop
	fprintf(FT_Comm_params->out_file_FT, "Starting streaming loop with %d samples...\n", numsamples);
	for (int i(0); i < numsamples; i++)
	{
		//auto timestamp_0_w = high_resolution_clock::now();
		if (!ReadFile(FT_Comm_params->serial_port, &Current_FT->sample, SAMPLE_SIZE, &numTransferred, NULL)
			|| (SAMPLE_SIZE != numTransferred))
		{
			return FALSE;
		}
		//auto timestamp_2_w = high_resolution_clock::now();
		//duration<double> elapsed_readfile_w = timestamp_2_w - timestamp_0_w;

		//auto timestamp_1_w = high_resolution_clock::now();
		if (!checksum_verification(FT_Comm_params))
		{
			fprintf(FT_Comm_params->err_file_FT, "Checksum verification failed at %d.\n", i);
			fflush(FT_Comm_params->err_file_FT);
			return FALSE;
		}
		//timestamp_2_w = high_resolution_clock::now();
		//duration<double> elapsed_chsverif_w = timestamp_2_w - timestamp_1_w;
		// Clear error flag
		//timestamp_1_w = high_resolution_clock::now();
		ClearCommError(FT_Comm_params->serial_port, &error, &FT_Comm_params->stat);
		//timestamp_2_w = high_resolution_clock::now();
		//duration<double> elapsed_clearcom_w = timestamp_2_w - timestamp_1_w;
		// Get gauges values
		//timestamp_1_w = high_resolution_clock::now();
		get_gauges_values(FT_Comm_params);
		//timestamp_2_w = high_resolution_clock::now();
		//duration<double> elapsed_getgauges_w = timestamp_2_w - timestamp_1_w;
		//Resolve FT values
		//timestamp_1_w = high_resolution_clock::now();
		if (FT_Comm_params->FT_measures.use_bias)
		{
			resolve_FT_components(FT_Comm_params);
			//store_current_FT(FT_Comm_params);
		}
		//timestamp_2_w = high_resolution_clock::now();
		//duration<double> elapsed_resFT_w = timestamp_2_w - timestamp_1_w;
		if (!FT_Comm_params->FT_measures.use_bias)
		{
			store_current_gauges(FT_Comm_params);
		}
		// Send data to Control thread if needed
		if (FT_Comm_params->general_params_FT.use_FT_for_Ctrl && i % 5 == 0 && FT_Comm_params->FT_measures.use_bias) // i % 21
		{
			//timestamp_1_w = high_resolution_clock::now();
			if (!send_current_FT(FT_Comm_params)) { return FALSE; }
			//timestamp_2_w = high_resolution_clock::now();
		}
		/*duration<double> elapsed_sendFT_w = timestamp_2_w - timestamp_1_w;
		timestamp_2_w = high_resolution_clock::now();
		duration<double> elapsed_all_w = timestamp_2_w - timestamp_0_w;
		if (FT_Comm_params->FT_measures.use_bias)
		{
			fprintf(FT_Comm_params->times, "%f ; %f ; %f ; %f ; %f ; %f ; %f ;",
				elapsed_readfile_w.count(), elapsed_chsverif_w.count(), elapsed_clearcom_w.count(),
				elapsed_getgauges_w.count(), elapsed_resFT_w.count(), elapsed_sendFT_w.count(),
				elapsed_all_w.count());
		}
		fflush(FT_Comm_params->times);*/
	}
	fprintf(FT_Comm_params->out_file_FT, "Read %d samples and saw %d error codes.\n",
		                                 numsamples, FT_Comm_params->FT_measures.status_bit_errors);
	if (FT_Comm_params->FT_measures.use_bias)
	{
		send_null_frame(FT_Comm_params);
	}
	stop_streaming(FT_Comm_params);
	return TRUE;
}

/*---------------------------------------------------------------------------------------------------------------------
| get_FT_diagnosis - Get diagnostic from FT sensor
|
| Syntax --
|	BOOL get_FT_diagnosis(FT_Comm_Struct* FT_Comm_params)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the struct containing all relevant communication parameters
|
| Outputs --
|	BOOL -> TRUE : Function finished normally ; FALSE : Error while stopping data stream
----------------------------------------------------------------------------------------------------------------------*/
BOOL get_FT_diagnosis(FT_Comm_Struct* FT_Comm_params)
{
	// Initialize variables
	unsigned char readDiagCommand[] = { 0x0a, 0x04, 0x00, 0x27, 0x00, 0x06, 0xc1, 0x78 };
	unsigned char readDiagResponse[17];
	DWORD numTransferred, error;

	// Ask for diagnostic
	WriteFile(FT_Comm_params->serial_port, readDiagCommand, sizeof(readDiagCommand), &numTransferred, NULL);
	if (!ReadFile(FT_Comm_params->serial_port, readDiagResponse,
		sizeof(readDiagResponse), &numTransferred, NULL) || (sizeof(readDiagResponse) != numTransferred))
	{
		fprintf(FT_Comm_params->err_file_FT, "Invalid response to diagnostics query.\n");
		return FALSE;
	} else
	{
		// Print received diagnostic
		for (int i(0); i < 6; i++)
		{
			unsigned int reading = (readDiagResponse[3 + 2 * i] << 8) + readDiagResponse[3 + 2 * i + 1];
			fprintf(FT_Comm_params->out_file_FT, "Diagnostic reading %d: %5d (%4x)\n", i, reading, reading);
		}
	}
	// Clear communication errors
	ClearCommError(FT_Comm_params->serial_port, &error, &FT_Comm_params->stat);
	return TRUE;
}

/*---------------------------------------------------------------------------------------------------------------------
| checksum_verification - Verify checksum according to constructor documentation
|
| Syntax --
|	BOOL checksum_verification(FT_Comm_Struct* FT_Comm_params)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the struct containing all relevant communication parameters
|
| Outputs --
|	BOOL -> TRUE : Function finished normally ; FALSE : Error while stopping data stream
----------------------------------------------------------------------------------------------------------------------*/
BOOL checksum_verification(FT_Comm_Struct* FT_Comm_params)
{
	// Initialize variables
	DWORD numTransferred, error;
	Current_FT_measures* Current_FT = &FT_Comm_params->FT_measures.Current_measures;
	Current_FT->check_sum = 0;
	// Compute sum of gauge bytes
	for (int j = 0; j < SAMPLE_SIZE - 1; j++)
	{
		Current_FT->check_sum += Current_FT->sample[j];
	}
	Current_FT->check_sum &= 0x7f;
	// Check validity
	if (Current_FT->sample[SAMPLE_SIZE - 1] >> 7)
	{
		FT_Comm_params->FT_measures.status_bit_errors++;
		fprintf(FT_Comm_params->err_file_FT, "Status bit error...");
		stop_streaming(FT_Comm_params);
		if (!lock_unlock_holding_registers(FT_Comm_params, FALSE)) {
			fprintf(FT_Comm_params->err_file_FT,"Erreur de registre dans le Checksum !!!");
		}
		unsigned char readRegPreCommand[] = { 10, 0x03, 0x00, 0x1D, 0x00, 0x01 };
		uint16_t readRegCRC = crc16(readRegPreCommand, 6);
		uint8_t readRegCommand[] = { 10, 0x03, 0x00, 0x1D, 0x00, 0x01, (uint8_t)((readRegCRC >> 8) & 0x00FF), (uint8_t)(readRegCRC & 0x00FF) };
		unsigned char readRegAnswer[10];
		WriteFile(FT_Comm_params->serial_port, readRegCommand, sizeof(readRegCommand), &numTransferred, NULL);
		if (!ReadFile(FT_Comm_params->serial_port, &readRegAnswer, 7, &numTransferred, NULL))
		{
			fprintf(FT_Comm_params->err_file_FT, "Error reading holding registers.\n");
			return FALSE;
		}else
		{
			uint16_t err = readRegAnswer[3] << 8 | readRegAnswer[4];
			fprintf(FT_Comm_params->err_file_FT, "Error number: %d\n", err);
		}
		return FALSE;
	} else if ((Current_FT->sample[SAMPLE_SIZE - 1] & 0x7f) != Current_FT->check_sum)
	{
		ClearCommError(FT_Comm_params->serial_port, &error, &FT_Comm_params->stat);
		fprintf(FT_Comm_params->err_file_FT, "Checksum error : %x.\n", error);
		return FALSE;
	}
	return TRUE;
}

/*---------------------------------------------------------------------------------------------------------------------
| get_gauges_values - Computes forces and torques from gauge values without bias
|
| Syntax --
|	void get_gauges_values(FT_Comm_Struct* FT_Comm_params)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the struct containing all relevant communication parameters
----------------------------------------------------------------------------------------------------------------------*/
void get_gauges_values(FT_Comm_Struct* FT_Comm_params)
{
	// Initialize variables
	Current_FT_measures* Current_FT = &FT_Comm_params->FT_measures.Current_measures;
	// Get gauges values
	Current_FT->gauge_0 = Current_FT->sample[0] << 8 | Current_FT->sample[1];
	Current_FT->gauge_1 = Current_FT->sample[6] << 8 | Current_FT->sample[7];
	Current_FT->gauge_2 = Current_FT->sample[2] << 8 | Current_FT->sample[3];
	Current_FT->gauge_3 = Current_FT->sample[8] << 8 | Current_FT->sample[9];
	Current_FT->gauge_4 = Current_FT->sample[4] << 8 | Current_FT->sample[5];
	Current_FT->gauge_5 = Current_FT->sample[10] << 8 | Current_FT->sample[11];
}

/*---------------------------------------------------------------------------------------------------------------------
| resolve_FT_components_nbias - Computes forces and torques from gauge values without bias
|
| Syntax --
|	void resolve_FT_components_nbias(FT_Comm_Struct* FT_Comm_params)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the struct containing all relevant communication parameters
----------------------------------------------------------------------------------------------------------------------*/
void resolve_FT_components(FT_Comm_Struct* FT_Comm_params)
{
	// Initialize variables
	All_FT_measures* All_FT = &FT_Comm_params->FT_measures;
	Current_FT_measures* Current_FT = &FT_Comm_params->FT_measures.Current_measures;
	Calib_Struct* Cal = &FT_Comm_params->FT_Calib;
	// Store gauges values and take account of bias
	int16_t gauges[] = { Current_FT->gauge_0 - All_FT->id_bias[0], Current_FT->gauge_1 - All_FT->id_bias[1],
		                 Current_FT->gauge_2 - All_FT->id_bias[2], Current_FT->gauge_3 - All_FT->id_bias[3],
		                 Current_FT->gauge_4 - All_FT->id_bias[4], Current_FT->gauge_5 - All_FT->id_bias[4] };
	// Compute forces and torques as : FT = BasicMatrix*transpose(gauges)
	// Compute fx
	Current_FT->f_x_c = Cal->BasicMatrix[0][0] * (float)gauges[0] + Cal->BasicMatrix[0][1] * (float)gauges[1]
						+ Cal->BasicMatrix[0][2] * (float)gauges[2] + Cal->BasicMatrix[0][3] * (float)gauges[3]
						+ Cal->BasicMatrix[0][4] * (float)gauges[4] + Cal->BasicMatrix[0][5] * (float)gauges[5];
	Current_FT->f_x = Current_FT->f_x_c / Cal->CountsPerForce;
	// Compute fy
	Current_FT->f_y_c = Cal->BasicMatrix[1][0] * (float)gauges[0] + Cal->BasicMatrix[1][1] * (float)gauges[1]
						+ Cal->BasicMatrix[1][2] * (float)gauges[2] + Cal->BasicMatrix[1][3] * (float)gauges[3]
						+ Cal->BasicMatrix[1][4] * (float)gauges[4] + Cal->BasicMatrix[1][5] * (float)gauges[5];
	Current_FT->f_y = Current_FT->f_y_c / Cal->CountsPerForce;
	// Compute fz
	Current_FT->f_z_c = Cal->BasicMatrix[2][0] * (float)gauges[0] + Cal->BasicMatrix[2][1] * (float)gauges[1]
						+ Cal->BasicMatrix[2][2] * (float)gauges[2] + Cal->BasicMatrix[2][3] * (float)gauges[3]
						+ Cal->BasicMatrix[2][4] * (float)gauges[4] + Cal->BasicMatrix[2][5] * (float)gauges[5];
	Current_FT->f_z = Current_FT->f_z_c / Cal->CountsPerForce;
	// Compute tx
	Current_FT->t_x_c = Cal->BasicMatrix[3][0] * (float)gauges[0] + Cal->BasicMatrix[3][1] * (float)gauges[1]
						+ Cal->BasicMatrix[3][2] * (float)gauges[2] + Cal->BasicMatrix[3][3] * (float)gauges[3]
						+ Cal->BasicMatrix[3][4] * (float)gauges[4] + Cal->BasicMatrix[3][5] * (float)gauges[5];
	Current_FT->t_x = Current_FT->t_x_c / Cal->CountsPerTorque;
	// Compute ty
	Current_FT->t_y_c = Cal->BasicMatrix[4][0] * (float)gauges[0] + Cal->BasicMatrix[4][1] * (float)gauges[1]
						+ Cal->BasicMatrix[4][2] * (float)gauges[2] + Cal->BasicMatrix[4][3] * (float)gauges[3]
						+ Cal->BasicMatrix[4][4] * (float)gauges[4] + Cal->BasicMatrix[4][5] * (float)gauges[5];
	Current_FT->t_y = Current_FT->t_y_c / Cal->CountsPerTorque;
	// Compute tz
	Current_FT->t_z_c = Cal->BasicMatrix[5][0] * (float)gauges[0] + Cal->BasicMatrix[5][1] * (float)gauges[1]
						+ Cal->BasicMatrix[5][2] * (float)gauges[2] + Cal->BasicMatrix[5][3] * (float)gauges[3]
						+ Cal->BasicMatrix[5][4] * (float)gauges[4] + Cal->BasicMatrix[5][5] * (float)gauges[5];
	Current_FT->t_z = Current_FT->t_z_c / Cal->CountsPerTorque;
}

/*---------------------------------------------------------------------------------------------------------------------
| set_FT_bias - Compute bias and set it into commnication struct
|
| Syntax --
|	void set_FT_bias(FT_Comm_Struct* FT_Comm_params)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the struct containing all relevant communication parameters
----------------------------------------------------------------------------------------------------------------------*/
void set_FT_bias(FT_Comm_Struct* FT_Comm_params)
{
	// Initialize variables
	All_FT_measures* All_FT = &FT_Comm_params->FT_measures;
	// Set bias
	All_FT->id_bias[0] = All_FT->g0_values.at(3500);
	All_FT->id_bias[1] = All_FT->g1_values.at(3500);
	All_FT->id_bias[2] = All_FT->g2_values.at(3500);
	All_FT->id_bias[3] = All_FT->g3_values.at(3500);
	All_FT->id_bias[4] = All_FT->g4_values.at(3500);
	All_FT->id_bias[5] = All_FT->g5_values.at(3500);
}

/*---------------------------------------------------------------------------------------------------------------------
| store_current_FT - Store current force and torque values
|
| Syntax --
|	void store_current_FT(FT_Comm_Struct* FT_Comm_params)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the struct containing all relevant communication parameters
----------------------------------------------------------------------------------------------------------------------*/
void store_current_FT(FT_Comm_Struct* FT_Comm_params)
{
	// Initialize variables
	All_FT_measures* All_FT = &FT_Comm_params->FT_measures;
	Current_FT_measures* Current_FT = &FT_Comm_params->FT_measures.Current_measures;
	// Store current values into vectors
	All_FT->fx_values.push_back(Current_FT->f_x);
	All_FT->fy_values.push_back(Current_FT->f_y);
	All_FT->fz_values.push_back(Current_FT->f_z);
	All_FT->tx_values.push_back(Current_FT->t_x);
	All_FT->ty_values.push_back(Current_FT->t_y);
	All_FT->tz_values.push_back(Current_FT->t_z);
}

/*---------------------------------------------------------------------------------------------------------------------
| store_current_gauges - Store current gauges values
|
| Syntax --
|	void store_current_gauges(FT_Comm_Struct* FT_Comm_params)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the struct containing all relevant communication parameters
----------------------------------------------------------------------------------------------------------------------*/
void store_current_gauges(FT_Comm_Struct* FT_Comm_params)
{
	// Initialize variables
	All_FT_measures* All_FT = &FT_Comm_params->FT_measures;
	Current_FT_measures* Current_FT = &FT_Comm_params->FT_measures.Current_measures;
	// Store current values into vectors
	All_FT->g0_values.push_back(Current_FT->gauge_0);
	All_FT->g1_values.push_back(Current_FT->gauge_1);
	All_FT->g2_values.push_back(Current_FT->gauge_2);
	All_FT->g3_values.push_back(Current_FT->gauge_3);
	All_FT->g4_values.push_back(Current_FT->gauge_4);
	All_FT->g5_values.push_back(Current_FT->gauge_5);
}

/*---------------------------------------------------------------------------------------------------------------------
| send_current_FT - Send current resolved measures to Control thread
|
| Syntax --
|	BOOL send_current_FT(FT_Comm_Struct* FT_Comm_params)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the struct containing all relevant communication parameters
|
| Outputs --
|	BOOL -> TRUE : Function finished normally ; FALSE : Error while stopping data stream
----------------------------------------------------------------------------------------------------------------------*/
BOOL send_current_FT(FT_Comm_Struct* FT_Comm_params)
{
	// Extract substructs
	Current_FT_measures* Current_FT = &FT_Comm_params->FT_measures.Current_measures;

	// Initialise variables
	int ownership_val;

	// Try to get ownership of the critical section
	ownership_val = TryEnterCriticalSection(FT_Comm_params->pCritical_share_FT);

	// Extract current measures to send
	FT_Comm_params->FT_measures_Shared->fx = Current_FT->f_x;
	FT_Comm_params->FT_measures_Shared->fy = Current_FT->f_y;
	FT_Comm_params->FT_measures_Shared->fz = Current_FT->f_z;
	FT_Comm_params->FT_measures_Shared->tx = Current_FT->t_x;
	FT_Comm_params->FT_measures_Shared->ty = Current_FT->t_y;
	FT_Comm_params->FT_measures_Shared->tz = Current_FT->t_z;
	if (Current_FT->f_x == Current_FT->f_z && Current_FT->t_z == 0.0f)
	{
		FT_Comm_params->FT_measures_Shared->streaming = FALSE;
	}

	// Release ownership of the critical section if obtained before
	if (ownership_val != 0)
	{
		LeaveCriticalSection(FT_Comm_params->pCritical_share_FT);
	}
	return TRUE;
}

/*---------------------------------------------------------------------------------------------------------------------
| send_null_frame - Send null frame to stop control
|
| Syntax --
|	BOOL send_null_frame(FT_Comm_Struct* FT_Comm_params)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the struct containing all relevant communication parameters
|
| Outputs --
|	BOOL -> TRUE : Function finished normally ; FALSE : Error while stopping data stream
----------------------------------------------------------------------------------------------------------------------*/
BOOL send_null_frame(FT_Comm_Struct* FT_Comm_params)
{
	// Initialize variables
	Current_FT_measures* Current_FT = &FT_Comm_params->FT_measures.Current_measures;

	// Set all values to 0
	Current_FT->f_x = 0.0f;
	Current_FT->f_y = 0.0f;
	Current_FT->f_z = 0.0f;
	Current_FT->t_x = 0.0f;
	Current_FT->t_y = 0.0f;
	Current_FT->t_z = 0.0f;

	// Send null frame
	if (!send_current_FT(FT_Comm_params))
	{
		fprintf(FT_Comm_params->err_file_FT, "Error sending null frame.\n");
		return FALSE;
	}
	return TRUE;
}

/*---------------------------------------------------------------------------------------------------------------------
| stop_streaming - Stop Digital FT streaming
|
| Syntax --
|	BOOL stop_streaming(FT_Comm_Struct* FT_Comm_params)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the struct containing all relevant communication parameters
|
| Outputs --
|	BOOL -> TRUE : Function finished normally ; FALSE : Error while stopping data stream
----------------------------------------------------------------------------------------------------------------------*/
BOOL stop_streaming(FT_Comm_Struct* FT_Comm_params)
{
	// Initialize variables
	DWORD numTransferred, error;
	unsigned char sample[13];
	unsigned char jammingSequence[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 };

	// Send stop streaming sequence
	WriteFile(FT_Comm_params->serial_port, jammingSequence, sizeof(jammingSequence), &numTransferred, NULL);
	do {
		if (!ReadFile(FT_Comm_params->serial_port, sample, 13, &numTransferred, NULL))
		{
			error = GetLastError();
			FT_ErrorExit(FT_Comm_params, error);
			fprintf(FT_Comm_params->err_file_FT, "Error reading file after jamming sequence : %d\n", error);
			return FALSE;
		}
	} while (13 == numTransferred);
	return TRUE;
}

/*---------------------------------------------------------------------------------------------------------------------
| close_communication - Close Communication with serial port
|
| Syntax --
|	void close_communication(FILE* out_file, modbus_t* modbus_context)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the struct containing all relevant communication parameters
----------------------------------------------------------------------------------------------------------------------*/
void close_communication(FT_Comm_Struct* FT_Comm_params)
{
	CloseHandle(FT_Comm_params->serial_port);
	DeleteFile(FT_Comm_params->general_params_FT.serial_port_name);
	fprintf(FT_Comm_params->out_file_FT, "Communication with FT Sensor closed.\n");
}

/*---------------------------------------------------------------------------------------------------------------------
| print_calibration - Print retrieved calibration data
|
| Syntax --
|	void print_calibration(FT_Comm_Struct* FT_Comm_params)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the struct containing all relevant communication parameters
----------------------------------------------------------------------------------------------------------------------*/
void print_calibration(FT_Comm_Struct* FT_Comm_params)
{
	// Print serial number
	char* serial = (char*)FT_Comm_params->FT_Calib.CalibSerialNumber;
	fprintf(FT_Comm_params->out_file_FT, "\nCalibration serial number : %s\n", serial);
	// Print part number
	char* part = (char*)FT_Comm_params->FT_Calib.CalibPartNumber;
	fprintf(FT_Comm_params->out_file_FT, "Calibration part number : %s\n", part);
	// Print family identifier
	char* family = (char*)FT_Comm_params->FT_Calib.CalibFamilyId;
	fprintf(FT_Comm_params->out_file_FT, "Calibration family ID : %s\n", family);
	// Print calibration time
	char* time = (char*)FT_Comm_params->FT_Calib.CalibTime;
	fprintf(FT_Comm_params->out_file_FT, "Calibration time : %s\n", time);
	// Print basic matrix
	fprintf(FT_Comm_params->out_file_FT, "Basic matrix read from calibration : \n{");
	for (int i(0); i < 6; i++)
	{
		for (int j(0); j < 6; j++)
		{
			fprintf(FT_Comm_params->out_file_FT, " %f", FT_Comm_params->FT_Calib.BasicMatrix[i][j]);
		}
		fprintf(FT_Comm_params->out_file_FT, " ;\n");
	}
	fprintf(FT_Comm_params->out_file_FT, " }\n");
	// Print force unit
	fprintf(FT_Comm_params->out_file_FT, "Force unit : %d\n", FT_Comm_params->FT_Calib.ForceUnits);
	// Print torque unit
	fprintf(FT_Comm_params->out_file_FT, "Torque unit : %d\n", FT_Comm_params->FT_Calib.TorqueUnits);
	// Print max rating
	fprintf(FT_Comm_params->out_file_FT, "Max rating : {");
	for (int i(0); i < 6; i++)
	{
		fprintf(FT_Comm_params->out_file_FT, " %f", FT_Comm_params->FT_Calib.MaxRating[i]);
	}
	fprintf(FT_Comm_params->out_file_FT, " }\n");
	// Print counts per force
	fprintf(FT_Comm_params->out_file_FT, "Counts per force : %d\n", FT_Comm_params->FT_Calib.CountsPerForce);
	// Print counts per torque
	fprintf(FT_Comm_params->out_file_FT, "Counts per torque : %d\n", FT_Comm_params->FT_Calib.CountsPerTorque);
	// Print gauge gains
	fprintf(FT_Comm_params->out_file_FT, "Gauge gains : {");
	for (int i(0); i < 6; i++)
	{
		fprintf(FT_Comm_params->out_file_FT, " %d", FT_Comm_params->FT_Calib.GaugeGain[i]);
	}
	fprintf(FT_Comm_params->out_file_FT, " }\n");
	// Print offsets
	fprintf(FT_Comm_params->out_file_FT, "Offsets : {");
	for (int i(0); i < 6; i++)
	{
		fprintf(FT_Comm_params->out_file_FT, " %d", FT_Comm_params->FT_Calib.GaugeOffset[i]);
	}
	fprintf(FT_Comm_params->out_file_FT, " }\n");
	// Print resolution
	fprintf(FT_Comm_params->out_file_FT, "Resolution : {");
	for (int i(0); i < 6; i++)
	{
		fprintf(FT_Comm_params->out_file_FT, " %d", FT_Comm_params->FT_Calib.Resolution[i]);
	}
	fprintf(FT_Comm_params->out_file_FT, " }\n");
}

/*---------------------------------------------------------------------------------------------------------------------
| print_FT_measures - Print all FT measures
|
| Syntax --
|	void print_FT_measures(FT_Comm_Struct* FT_Comm_params)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the struct containing all relevant communication parameters
----------------------------------------------------------------------------------------------------------------------*/
void print_FT_measures(FT_Comm_Struct* FT_Comm_params)
{
	// Initialize variables
	All_FT_measures* All_FT = &FT_Comm_params->FT_measures;
	// Print loop
	for (int i(0); i < (int)All_FT->fx_values.size(); i++)
	{
		fprintf(FT_Comm_params->f_t_sensor_file, "%f;", All_FT->fx_values.at(i));
		fprintf(FT_Comm_params->f_t_sensor_file, "%f;", All_FT->fy_values.at(i));
		fprintf(FT_Comm_params->f_t_sensor_file, "%f;", All_FT->fz_values.at(i));
		fprintf(FT_Comm_params->f_t_sensor_file, "%f;", All_FT->tx_values.at(i));
		fprintf(FT_Comm_params->f_t_sensor_file, "%f;", All_FT->ty_values.at(i));
		fprintf(FT_Comm_params->f_t_sensor_file, "%f;", All_FT->tz_values.at(i));
	}
}

/*---------------------------------------------------------------------------------------------------------------------
| FT_ErrorExit - Retrieve error string in case something fails
|
| Syntax --
|	BOOL FT_ErrorExit(FT_Comm_Struct* FT_Comm_params, DWORD error)
|
| Inputs --
|	FT_Comm_Struct* FT_Comm_params -> pointer towards the struct containing all relevant communication parameters
|   DWORD error -> Last error message retrieved by GetLastError()
|
| Outputs --
|	BOOL -> TRUE : Function finished normally ; FALSE : Error while formating error message
----------------------------------------------------------------------------------------------------------------------*/
BOOL FT_ErrorExit(FT_Comm_Struct* FT_Comm_params, DWORD error)
{
	// Retrieve the system error message for the last-error code

	LPVOID lpMsgBuf;

	FormatMessage(
		FORMAT_MESSAGE_ALLOCATE_BUFFER |
		FORMAT_MESSAGE_FROM_SYSTEM |
		FORMAT_MESSAGE_IGNORE_INSERTS,
		NULL,
		error,
		MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
		(LPTSTR)&lpMsgBuf,
		0, NULL);

	// Display the error message and exit the process
	reinterpret_cast<char*>(lpMsgBuf);
	char* err_string = (char*)lpMsgBuf;
	fprintf(FT_Comm_params->err_file_FT, "Function failed with error code %d => %s. ", error, err_string);
	return TRUE;
}
