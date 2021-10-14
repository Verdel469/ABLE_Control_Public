/***********************************************************************************************************************
* get_FT_measures_WinAPI.h -
*
* Author : Dorian Verdel
* Creation  date : 06/2020
*
* Description :
* Defines the communication with the ATI Force/Torque (FT) sensor directly through Windows API.
***********************************************************************************************************************/

#pragma once

#ifndef GET_FT_MEASURES_WINAPI
#define GET_FT_MEASURES_WINAPI

// Project dependencies
#include "NiSerial.h"
#include "low_level_command_1DoF_main.h"
#include "shared_FT_struct.h"				// Header containing the shared FT measures struct definition

// Communication parameters
#define SERIAL_PORT_NAME_ARM L"COM4"
#define SERIAL_PORT_NAME_WRIST L"COM5"
#define BAUD_RATE 1250000
#define MB_BYTE_SIZE 8

// Important addresses and constants
#define FT_SENSOR_ADRESS 10
#define LOCK_UNLOCK_FT_REGISTERS_FN 0x6A
#define UNLOCK_FT_REGISTERS_DATA 0xAA
#define LOCK_FT_REGISTERS_DATA 0x18
#define CALIB_REGISTER_ADDRESS 0x00e3
#define GAIN_REGISTERS_ADDRESS_START 0x0000
#define GAIN_REGISTERS_ADDRESS_END 0x0005
#define OFFSET_REGISTERS_ADDRESS_START 0x0006
#define OFFSET_REGISTERS_ADDRESS_END 0x000B
#define CALIBRATION_LENGTH 169
#define SAMPLE_SIZE 13

// Modbus constants
#define MODBUS_MAX_MSG_LENGTH 260
#define MODBUS_READ_REGISTERS_FN 0x03
#define MODBUS_READ_REGISTER_LIMIT 125
#define MODBUS_WRITE_N_REGISTERS_FN 0x10
#define MIN_REQ_LENGTH 12

// Custom constants
#define BIAS_ID_N_SAMPLES 7000
#define NB_SAMPLES_TO_CTRL 7

// --------------------------------------------------- PIPE SUBSTRUCT --------------------------------------------------
struct Params_FT
{
	BOOL use_FT_for_Ctrl;		// Input boolean to check if measures are required in Control thread
	BOOL struct_FT_Arm;			// Input boolean to identify the arm sensor
	BOOL struct_FT_Wrist;		// Input boolean to identify the wrist sensor
	LPCWSTR serial_port_name;	// Name of the serial port to use
	int bias_identified;		// Input boolean to check if bias has been previously identified
};

// ------------------------------------------------ CALIBRATION SUBSTRUCT ----------------------------------------------
struct Calib_Struct
{
	uint8_t CalibSerialNumber[8];		// Serial number of the calibration
	uint8_t CalibPartNumber[32];        // Part number of the calibration
	uint8_t CalibFamilyId[4];           // Family of the calibration
	uint8_t CalibTime[20];              // Data of the calibration
	float BasicMatrix[6][6];            // Calibrated matrix to resolve F/T values
	uint8_t ForceUnits;					// Units of force components
	uint8_t TorqueUnits;                // Units of torque components
	float MaxRating[6];                 // Maximum forces supported by the sensor
	int32_t CountsPerForce;				// Force multiplier to resolve F/T values
	int32_t CountsPerTorque;            // Torque multiplier to resolve F/T values
	uint16_t GaugeGain[6];              // Gains to write in the "active gains" register
	uint16_t GaugeOffset[6];            // Offsets to write in the "active offsets" register
	uint8_t Resolution[6];              // Unused
	uint8_t Range[6];                   // Unused
	uint16_t ScaleFactor16[6];          // Unused
	uint8_t UserField1[16];             // Unused
	uint8_t UserField2[16];             // Unused
	uint8_t SpareData[16];              // Unused
};
// ---------------------------------------------- CURRENT MEASURES SUBSTRUCT -------------------------------------------
struct Current_FT_measures
{
	uint8_t sample[13];		// Received sample
	int16_t gauge_0;        // Value returned by gauge 0
	int16_t gauge_1;        // Value returned by gauge 1
	int16_t gauge_2;        // Value returned by gauge 2
	int16_t gauge_3;        // Value returned by gauge 3
	int16_t gauge_4;        // Value returned by gauge 4
	int16_t gauge_5;        // Value returned by gauge 5
	uint8_t check_sum;      // Current computed checksum
	uint8_t check_byte;     // Verification Byte sent by sensor
	float f_x_c;            // Computed force count along x axis
	float f_y_c;            // Computed force count along y axis
	float f_z_c;            // Computed force count along z axis
	float t_x_c;            // Computed torque count along x axis
	float t_y_c;            // Computed torque count along y axis
	float t_z_c;            // Computed torque count along z axis
	float f_x;              // Computed force along x axis
	float f_y;              // Computed force along y axis
	float f_z;              // Computed force along z axis
	float t_x;              // Computed torque along x axis
	float t_y;              // Computed torque along y axis
	float t_z;              // Computed torque along z axis
};

// ------------------------------------------------ ALL MEASURES SUBSTRUCT ---------------------------------------------
struct All_FT_measures
{
	Current_FT_measures Current_measures;  // Current data received from FT sensor
	int16_t id_bias[6];                    // Identified bias vector
	std::vector<int16_t> g0_values;        // All G0 values
	std::vector<int16_t> g1_values;        // All G1 values
	std::vector<int16_t> g2_values;        // All G2 values
	std::vector<int16_t> g3_values;        // All G3 values
	std::vector<int16_t> g4_values;        // All G4 values
	std::vector<int16_t> g5_values;        // All G5 values
	std::vector<float> fx_values;          // All computed forces along x axis
	std::vector<float> fy_values;          // All computed forces along y axis
	std::vector<float> fz_values;          // All computed forces along z axis
	std::vector<float> tx_values;          // All computed torques along x axis
	std::vector<float> ty_values;          // All computed torques along y axis
	std::vector<float> tz_values;          // All computed torques along z axis
	BOOL use_bias;						   // TRUE : Use bias ; FALSE : Do not use bias
	int status_bit_errors;				   // Status errors counter
	int nb_measures;					   // Number of measures to do
};

// ----------------------------------------- GLOBAL SERIAL COMMUNICATION STRUCT ----------------------------------------
struct FT_Comm_Struct
{
	HANDLE serial_port;									// Handle of the serial port file
	DCB FT_Comm_DCB;									// DCB struct for serial communication
	COMMTIMEOUTS FT_Comm_timeouts;						// Timeouts struct for serial communication
	COMSTAT stat;										// Stat struct for serial communication errors
	Params_FT general_params_FT;						// Pipe substruct to send data to Control thread
	unsigned char response[MODBUS_MAX_MSG_LENGTH];		// Buffer storing modbus answers
	Calib_Struct FT_Calib;								// Calibration struct containing all data
	All_FT_measures FT_measures;						// Substruct containing all previous measures
	FILE* f_t_sensor_file;								// File to write all measured forces and torques
	FILE* bias_vector_file;								// File containing the identified bias vector to apply
	FT_meas_Global* FT_measures_Shared;					// Interlocked measures struct
	CRITICAL_SECTION* pCritical_share_FT;				// Critical section
	FILE* out_file_FT;									// Out file dedicated to digital FT communication
	FILE* err_file_FT;									// Err file dedicated to digital FT communication
	FILE* times;										// Debug file for time measurements
};


// ------------------------------------------------- FUNCTIONS DECLARATION ---------------------------------------------
// Initializations
BOOL initialize_FT_sensor(FT_Comm_Struct* FT_Comm_params);
BOOL initialize_serial_port(FT_Comm_Struct* FT_Comm_params);
void initiate_Com(FT_Comm_Struct* FT_Comm_params);
BOOL get_FT_calibration(FT_Comm_Struct* FT_Comm_params);
BOOL fill_Calib_struct_Arm(FT_Comm_Struct* FT_Comm_params);
BOOL fill_Calib_struct_Wrist(FT_Comm_Struct* FT_Comm_params);
BOOL get_and_set_bias(FT_Comm_Struct* FT_Comm_params);
void set_FT_bias(FT_Comm_Struct* FT_Comm_params);
void retrieve_IdentifiedBias(FT_Comm_Struct* FT_Comm_params);
BOOL get_FT_diagnosis(FT_Comm_Struct* FT_Comm_params);

// Real time loop
DWORD WINAPI run_real_time_Measures(LPVOID FT_comm);
void wait_between_CS_tries();
BOOL send_current_FT(FT_Comm_Struct* FT_Comm_params);

// Custom functions
static uint16_t crc16(uint8_t* buffer, uint16_t buffer_length);
BOOL lock_unlock_holding_registers(FT_Comm_Struct* FT_Comm_params, BOOL lock_unlock);
BOOL send_custom_function(FT_Comm_Struct* FT_Comm_params, uint8_t fn_code, uint8_t* data, int data_length);
BOOL start_streaming_data(FT_Comm_Struct* FT_Comm_params);
BOOL checksum_verification(FT_Comm_Struct* FT_Comm_params);
void get_gauges_values(FT_Comm_Struct* FT_Comm_params);
void resolve_FT_components(FT_Comm_Struct* FT_Comm_params);
void store_current_FT(FT_Comm_Struct* FT_Comm_params);
void store_current_gauges(FT_Comm_Struct* FT_Comm_params);

// Outputs and closures
BOOL send_null_frame(FT_Comm_Struct* FT_Comm_params);
BOOL stop_streaming(FT_Comm_Struct* FT_Comm_params);
void close_communication(FT_Comm_Struct* FT_Comm_params);
void print_calibration(FT_Comm_Struct* FT_Comm_params);
void print_FT_measures(FT_Comm_Struct* FT_Comm_params);
BOOL FT_ErrorExit(FT_Comm_Struct* FT_Comm_params, DWORD error);

#endif // !GET_FT_MEASURES_WINAPI
