/***********************************************************************************************************************
* get_FT_sensor_measures.cpp -
*
* Author : Dorian Verdel
* Creation  date : 06/2020
*
* Description :
* Manages the communication with the ATI Force/Torque (FT) sensor.
***********************************************************************************************************************/

#include "get_FT_sensor_measures.h"

/*---------------------------------------------------------------------------------------------------------------------
| initialize_serial_port - Initialize communication parameters in agreement with digital FT sensor documentation
|
| Syntax --
|	modbus_t* initialize_serial_port(FILE* err_file, FILE* out_file, modbus_t* modbus_context)
|
| Inputs --
|	FILE* err_file -> pointer towards "errors.txt" (stderr)
|	FILE* out_file -> pointer towards "outputs.txt" (stdout)
|   modbus_t* modbus_context -> Modbus context for "ModBus over Serial Port" communication
|
| Outputs --
|	modbus_t* -> Modbus context
----------------------------------------------------------------------------------------------------------------------*/
modbus_t* initialize_serial_port(FILE* err_file, FILE* out_file, modbus_t* modbus_context)
{
	// Handle of serial port creation and cleansing of buffers
	const char* sp_name = "COM3";
	int baud_rate = 1250000;
	int data_size = 8;
	int stop_size = 1;
	char parity = 'E';
	int slave_adress = 10;
	int err, length;

	// Create ModBus context
	modbus_context = modbus_new_rtu(sp_name, baud_rate, parity, data_size, stop_size);
	if (modbus_context == NULL)
	{
		fprintf(err_file, "Unable to create the libmodbus context\n");
		fflush(err_file);
		return NULL;
	}
	// Set timeouts
	modbus_set_response_timeout(modbus_context, 0, 200000);
	modbus_set_byte_timeout(modbus_context, 0, 200000);
	// Set ModBus controlled device
	err = modbus_set_slave(modbus_context, slave_adress);
	if (err == -1)
	{
		fprintf(err_file, "Invalid slave ID\n");
		fflush(err_file);
		return NULL;
	}
	int sa = modbus_get_slave(modbus_context);
	fprintf(out_file, "Slave address : %i\n", sa);
	// Connect to device
	if (modbus_connect(modbus_context) == -1)
	{
		fprintf(err_file, "Connection failed: %s\n", modbus_strerror(errno));
		fflush(err_file);
		return NULL;
	}else
	{
		fprintf(out_file, "Connection established with Force/Torque sensor !\n");
		fflush(out_file);
	}
	// Flush files and return modbus context if no errors
	fflush(err_file);
	fflush(out_file);
	return modbus_context;
}

/*---------------------------------------------------------------------------------------------------------------------
| get_FT_calibration - Get the calibration of the device and apply gauge gains and offsets
|
| Syntax --
|	void initialize_serial_port(FILE* err_file, FILE* out_file, HANDLE comp_port_handle, DCB* dcb)
|
| Inputs --
|	FILE* err_file -> pointer towards "errors.txt" (stderr)
|	FILE* out_file -> pointer towards "outputs.txt" (stdout)
|   modbus_t* -> Modbus context
|   Calibration_Struct* calib_FT_sensor -> Pre-allocated space for calibration struct recording
----------------------------------------------------------------------------------------------------------------------*/
modbus_t* get_FT_calibration(FILE* err_file, FILE* out_file, modbus_t* modbus_context, Calibration_Struct* calib_FT_sensor)
{
	int calib_register_addr = 0x00e3;
	const int highregisters_length = CALIBRATION_LENGTH - MODBUS_READ_REGISTER_LIMIT + 1;
	uint16_t calib_LowRegisters[MODBUS_READ_REGISTER_LIMIT];
	uint16_t calib_HighRegisters[highregisters_length];
	const int nb_bits = 8 * sizeof(Calibration_Struct);
	const int nb_reg = 1;
	uint8_t bit_destination[nb_bits * sizeof(uint8_t)];
	int err = 0;
	uint8_t unlock_command[] = { 10, 0x6A, 0xAA };
	uint8_t answer[MODBUS_RTU_MAX_ADU_LENGTH];

	// Unlock holding registers
	if (modbus_send_raw_request(modbus_context, unlock_command, 6 * sizeof(uint8_t)) == -1)
	{
		fprintf(err_file, "Error during unlocking of registers : %s\n", modbus_strerror(errno));
	}else
	{
		fprintf(out_file, "Unlock holding registers request sent \n");
	}
	modbus_receive_confirmation(modbus_context, answer);
	fprintf(out_file, "Received confirmation :");
	for (int i(0); i < MODBUS_RTU_MAX_ADU_LENGTH; i++) { fprintf(out_file, " %i", answer[i]); }
	fprintf(out_file, "\n");

	if (modbus_context == NULL) {
		fprintf(err_file, "Modbus context has been destroyed !\n");
	}
	// Read holding register containing the information
	err = modbus_read_registers(modbus_context, calib_register_addr, MODBUS_READ_REGISTER_LIMIT, calib_LowRegisters);
	fprintf(err_file, "Error during reading of calibration Low registers : %s\n", modbus_strerror(err));
		//return FALSE;
	err = modbus_read_registers(modbus_context, calib_register_addr + MODBUS_READ_REGISTER_LIMIT,
		highregisters_length, calib_HighRegisters);
	fprintf(err_file, "Error during reading of calibration High registers : %s\n", modbus_strerror(err));
		//return FALSE;
	

	return modbus_context;
}

/*---------------------------------------------------------------------------------------------------------------------
| start_reading_data_stream - Starts to read the data stream (7kHz) until a stop streaming order is sent
|
| Syntax --
|	void initialize_serial_port(FILE* err_file, FILE* out_file, HANDLE comp_port_handle, DCB* dcb)
|
| Inputs --
|	FILE* err_file -> pointer towards "errors.txt" (stderr)
|	FILE* out_file -> pointer towards "outputs.txt" (stdout)
|   HANDLE comp_port_handle -> Handle of the serial port communication file
|   DCB* dcb -> Pointer towards dcb struct containing all communication parameters
----------------------------------------------------------------------------------------------------------------------*/
void start_reading_data_stream(FILE* err_file, FILE* out_file, HANDLE comp_port_handle, DCB* dcb)
{

}

/*---------------------------------------------------------------------------------------------------------------------
| close_communication - Close Communication with serial port
|
| Syntax --
|	void close_communication(FILE* out_file, modbus_t* modbus_context)
|
| Inputs --
|	FILE* out_file -> pointer towards "outputs.txt" (stdout)
|   modbus_t* modbus_context -> Modbus context for "ModBus over Serial Port" communication
----------------------------------------------------------------------------------------------------------------------*/
void close_communication(FILE* out_file, modbus_t* modbus_context)
{
	modbus_close(modbus_context);
	modbus_free(modbus_context);
	fprintf(out_file, "Communication with FT Sensor closed.\n");
}