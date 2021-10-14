/***********************************************************************************************************************
* get_FT_sensor_measures.h -
*
* Author : Dorian Verdel
* Creation  date : 06/2020
*
* Description :
* Defines the communication with the ATI Force/Torque (FT) sensor.
***********************************************************************************************************************/

#pragma once

#ifndef GET_FT_SENSOR_MEASURES
#define GET_FT_SENSOR_MEASURES

// External dependencies
#include <modbus.h>
#include <winioctl.h>
// Project dependencies
#include "NiSerial.h"
#include "low_level_command_1DoF_main.h"

#define MODBUS_READ_REGISTER_LIMIT 125
#define CALIBRATION_LENGTH 169

// Define Calibration struct as described by ATI
struct Calibration_Struct {
	uint8_t CalibSerialNumber[8];
	uint8_t CalibPartNumber[32];
	uint8_t CalibFamilyId[4];
	uint8_t CalibTime[20];
	float BasicMatrix[6][6];
	uint8_t ForceUnits;
	uint8_t TorqueUnits;
	float MaxRating[6];
	int32_t CountsPerForce;
	int32_t CountsPerTorque;
	uint16_t GageGain[6];
	uint16_t GageOffset[6];
	uint8_t Resolution[6];
	uint8_t Range[6];
	uint16_t ScaleFactor16[6];
	uint8_t UserField1[16];
	uint8_t UserField2[16];
	uint8_t SpareData[16];
};

// Functions declaration
// Initialize COM3 port
modbus_t* initialize_serial_port(FILE* err_file, FILE* out_file, modbus_t* modbus_context);
modbus_t* get_FT_calibration(FILE* err_file, FILE* out_file, modbus_t* modbus_context, Calibration_Struct* calib_FT_sensor);
void close_communication(FILE* out_file, modbus_t* modbus_context);

#endif // !GET_FT_SENSOR_MEASURES