/***********************************************************************************************************************
* low_level_command_1DoF_main.cpp -
*
* Author : Dorian Verdel
* Creation  date : 02/2019
*
* Description :
* Manages the main functions of the project.
***********************************************************************************************************************/

// Includes
#include "low_level_command_1DoF_main.h"

using namespace std::chrono;
// Creation of thread handles struct
static ThreadHandles all_Threads;
// Creation of ServoComEth structure
static ServoComEth eth_ABLE;
// Creation of AbleControlStruct structure to store real time data
static AbleControlStruct ctrl_ABLE;
// Creation of ComStruct structure to store real time data
static ComStruct qtm_ComStruct;
// Creation of the FT_Comm_Struct to communicate with Digital FT sensor
static FT_Comm_Struct FT_Comm_params_Wrist;
static FT_Comm_Struct FT_Comm_params_Arm;
// Creation of the global measures structs for Critical communication
FT_meas_Global FT_measures_Interlocked_Wrist;
FT_meas_Global FT_measures_Interlocked_Arm;
// Creation of the critical section structs
CRITICAL_SECTION Critical_share_FT_Wrist;
CRITICAL_SECTION Critical_share_FT_Arm;

/*---------------------------------------------------------------------------------------------------------------------
| main - Main function
|
| Syntax --
|	int main(int argc, char *argv[])
|
| Inputs --
|	int argc -> length of argv
|	char* argv[] -> name, identification phase, port, char position, duration, friction, id method, human parameters
|
| Outputs --
|	int -> 0 : Success ; else : Failure
----------------------------------------------------------------------------------------------------------------------*/
int main(int argc, char *argv[])
{
	// Variables declaration
	int err = 0, exit_flag = 0;

	// Initialisation of extern files descriptors for errors and out writing
	FILE* err_file = NULL;
	FILE* out_file = NULL;
	
	// Openning of initialised extern files
	freopen_s(&err_file, "errors.txt", "w", stderr);
	freopen_s(&out_file, "outputs.txt", "w", stdout);
	fprintf(out_file, "All files successfully opened\n");

	// Extract and store input data
	extract_InputData(argc, argv, out_file, err_file);
	fprintf(out_file, "Input data extracted\n");

	// Set all robot parameters
	able_SetAllParams(&ctrl_ABLE);

	// Preallocate vectors memory
	preallocate_memory();

	// Initialise FT shared struct in any case
	initialise_FT_Shared();
	fprintf(out_file, "Shared structs initialised\n");
	fflush(out_file);

	// Initialise critical sections
	if (!InitializeCriticalSectionAndSpinCount(&Critical_share_FT_Arm, 0x00040000))
	{
		fprintf(err_file, "Failed to initialise Arm Critical Section \n");
		fflush(err_file);
		fflush(out_file);
		return -1;
	}
	if (!InitializeCriticalSectionAndSpinCount(&Critical_share_FT_Wrist, 0x00040000))
	{
		fprintf(err_file, "Failed to initialise Wrist Critical Section \n");
		fflush(err_file);
		fflush(out_file);
		return -1;
	}

	fprintf(out_file, "Critical sections initialised\n");
	fflush(out_file);
	
	// Initialize communication with Python
	if (ctrl_ABLE.aOrders.ctrl_type >= TORQUE_CTRL && ctrl_ABLE.rtParams.use_QTM)
	{
		err = initComPython(err_file, out_file);
		if (err == -1) { return -1; }
		// Initialize pipes between threads
		initQTMComPipes(err_file, out_file);
	}
	// Prepare identification orders
	setIdentificationOrders(err_file, out_file, &ctrl_ABLE);
	fprintf(out_file, "Orders initialised\n");
	fflush(out_file);

	// Pre-fill structs for FT communications
	prefill_FT_Comm_Structs();

	if (ctrl_ABLE.rtParams.use_FT)
	{
		launch_FT_Measures_Wrist();
		launch_FT_Measures_Arm();
	}

	fprintf(out_file, "FT measures launched\n");
	fflush(out_file);

	// Flush FT files
	fflush(FT_Comm_params_Wrist.err_file_FT);
	fflush(FT_Comm_params_Wrist.out_file_FT);

	// Call the function that initializes the communication with ABLE throught constructor functions
	err = able_InitCommunication(&eth_ABLE, &ctrl_ABLE, err_file, out_file);

	fprintf(out_file, "Communication with ABLE initialised\n");
	fflush(out_file);

	// Check the status of the initialization of the communication
	if (err != 0)
	{
		fprintf(err_file, "Error during initialization of communication !");
		fflush(err_file);
		fflush(out_file);
		return -1;
	}
	fflush(out_file);
	fflush(err_file);

	// Compute targets
	targetsComputationGeomID_1DoF(&ctrl_ABLE);
	
	// Initialise arguments for thread (put to the shape accepted by windows Threads)
	ThreadInformations ableInformations;
	prefill_ABLE_Thread_Comm_Struct(&ableInformations, out_file, err_file);

	fprintf(out_file, "Able informations struct successfully built\n");
	fflush(out_file);
	fflush(err_file);

	// Launch QTM measures thread
	if (ctrl_ABLE.aOrders.ctrl_type > TORQUE_CTRL && ctrl_ABLE.rtParams.use_QTM)
	{
		//err = launch_QTM_Measures(&qtm_ComStruct, xs_slider_file);
	}

	// Wait nominal voltage to protect the system
	waitNomVoltage(err_file, out_file);

	// Move to home position and check orders application status
	err = executeMotions(&ableInformations);
	exit_flag = getErrorMessage(err, err_file, out_file);

	// Wait for all threads to end
	Sleep(1000);
	CloseHandle(_Post_ _Notnull_ all_Threads.digital_FT_thread_Arm);
	CloseHandle(_Post_ _Notnull_ all_Threads.digital_FT_thread_Wrist);
	CloseHandle(_Post_ _Notnull_ all_Threads.qtm_thread);

	// End the communication with ABLE
	able_CloseCommunication(&eth_ABLE, &ctrl_ABLE);

	// Identify human limb mass
	if (ctrl_ABLE.aOrders.ctrl_type == HDYN_IDENT)
	{
		limbIdentification_Main(&ableInformations);
	}
	
	// Release resources used by the critical section object.
	DeleteCriticalSection(&Critical_share_FT_Wrist);

	// Flush and close all files
	clean_Files(&ableInformations);

	// Return 0 if everything went well
	return exit_flag;
}

/*---------------------------------------------------------------------------------------------------------------------
| extract_InputData - Extract data sent by python script
|
| Syntax --
|	void extract_InputData(int argc, char* argv[], FILE* out_file, FILE* err_file)
|
| Inputs --
|	int argc -> length of argv
|	char* argv[] -> name, identification phase, port, char position, duration, friction, id method, human parameters
----------------------------------------------------------------------------------------------------------------------*/
void extract_InputData(int argc, char* argv[], FILE* out_file, FILE* err_file)
{
	int identification, correction;
	char* endptr;
	const char* file_name = "";

	// Set phase defining with motion to apply
	identification = strtol(argv[1], &endptr, 10);
	ctrl_ABLE.aOrders.ctrl_type = identification;
	fprintf(out_file, "Chosen identification phase : %i\n", identification);

	// Set socket port for communication with other codes
	qtm_ComStruct.sockStruct.socket_port = strtol(argv[2], &endptr, 10);
	fprintf(out_file, "Socket port for connection : %i\n", qtm_ComStruct.sockStruct.socket_port);

	// Extract current position of the char given by user
	ctrl_ABLE.aDynamics.axis4_mod.x_slider = -strtof(argv[3], &endptr);
	qtm_ComStruct.qualMeas.sliderPos = -strtof(argv[3], &endptr);
	fprintf(out_file, "Current position of char : %f\n", ctrl_ABLE.aDynamics.axis4_mod.x_slider);

	// Set experimentation time given by user
	ctrl_ABLE.rtParams.limit_iterCom = strtol(argv[4], &endptr, 10) * 1000;
	FT_Comm_params_Wrist.FT_measures.nb_measures = ctrl_ABLE.rtParams.limit_iterCom * 8;
	FT_Comm_params_Arm.FT_measures.nb_measures = ctrl_ABLE.rtParams.limit_iterCom * 8;
	fprintf(out_file, "Requested number of iterations : %i\n", ctrl_ABLE.rtParams.limit_iterCom);

	// Extract compensation of friction presence
	ctrl_ABLE.rtParams.friction_comp = strtol(argv[5], &endptr, 10);
	fprintf(out_file, "Friction compensation activation : %i\n", ctrl_ABLE.rtParams.friction_comp);

	// Extract boolean for use of QTM in Control thread
	ctrl_ABLE.rtParams.use_QTM = strtol(argv[6], &endptr, 10);
	fprintf(out_file, "Real time QTM measures activation : %i\n", ctrl_ABLE.rtParams.use_QTM);

	// Extract boolean for use of digital FT in Control thread
	ctrl_ABLE.rtParams.use_FT = strtol(argv[7], &endptr, 10);
	FT_Comm_params_Wrist.general_params_FT.use_FT_for_Ctrl = ctrl_ABLE.rtParams.use_FT;
	FT_Comm_params_Arm.general_params_FT.use_FT_for_Ctrl = ctrl_ABLE.rtParams.use_FT;
	fprintf(out_file, "Real time FT control activation : %i\n", ctrl_ABLE.rtParams.use_FT);

	// Extract antigravity value
	ctrl_ABLE.aOrders.antiG_value = strtof(argv[8], &endptr);
	fprintf(out_file, "Antigravity value : %f\n", ctrl_ABLE.aOrders.antiG_value);

	// Extract the name of the file containing human parameters
	if (ctrl_ABLE.aOrders.antiG_value != 0 && identification == TORQUE_CTRL)
	{
		file_name = argv[9];
		fopen_s(&ctrl_ABLE.hDynId.idDyn_File, file_name, "rt");
		retrieve_IdentifiedDyn(&ctrl_ABLE);
		// Extract antigravity correction
		correction = strtol(argv[10], &endptr, 10);
		if (correction == 0)
		{
			ctrl_ABLE.rtParams.correct_fz_antigrav = 0;
			ctrl_ABLE.rtParams.correct_q_antigrav = 0;
			ctrl_ABLE.rtParams.correct_q_antigrav_2 = 0;
		} else if (correction == 1)
		{
			ctrl_ABLE.rtParams.correct_fz_antigrav = 1;
			ctrl_ABLE.rtParams.correct_q_antigrav = 0;
			ctrl_ABLE.rtParams.correct_q_antigrav_2 = 0;
		} else if (correction == 2) {
			ctrl_ABLE.rtParams.correct_fz_antigrav = 1;
			ctrl_ABLE.rtParams.correct_q_antigrav = 1;
			ctrl_ABLE.rtParams.correct_q_antigrav_2 = 0;
		}else if (correction == 3) {
			ctrl_ABLE.rtParams.correct_fz_antigrav = 1;
			ctrl_ABLE.rtParams.correct_q_antigrav = 0;
			ctrl_ABLE.rtParams.correct_q_antigrav_2 = 1;
		}
	}

	// Extract boolean for use of digital FT in Control thread
	FT_Comm_params_Wrist.general_params_FT.bias_identified = strtol(argv[11], &endptr, 10);
	FT_Comm_params_Arm.general_params_FT.bias_identified = strtol(argv[11], &endptr, 10);
	fprintf(out_file, "Bias previously identified : %i\n", FT_Comm_params_Wrist.general_params_FT.bias_identified);

	// Extract the name of the file containing the FT sensor bias
	if (FT_Comm_params_Wrist.general_params_FT.bias_identified && FT_Comm_params_Arm.general_params_FT.bias_identified)
	{
		file_name = argv[12];
		fopen_s(&FT_Comm_params_Arm.bias_vector_file, file_name, "rt");
		file_name = argv[13];
		fopen_s(&FT_Comm_params_Wrist.bias_vector_file, file_name, "rt");
	}

	// Extract the name of the file containing pre-computed minimum jerk trajectories and regulation value
	if (identification == MINJERK_TRAJS)
	{
		// Extract trajectories
		file_name = argv[14];
		ctrl_ABLE.minJerk.minjerk_file.open(file_name);
		fprintf(out_file, "Minimimum jerk trajectories file successfully opened !\n");
		extractJerkPositions(err_file, &ctrl_ABLE);
		fprintf(out_file, "Minimimum jerk trajectories extracted and duplicated \n");
		// Extract regulation forces
		ctrl_ABLE.aOrders.max_resistanceIFPos_biceps = strtof(argv[15],&endptr);
		ctrl_ABLE.aOrders.max_resistanceIFPos_triceps = -strtof(argv[16], &endptr);
		// Extract fatigue test value
		ctrl_ABLE.rtParams.jerkBlockWithFatigueTest = strtol(argv[17], &endptr, 10);
		// Extract this block is a familiarisation block
		ctrl_ABLE.rtParams.jerkFamiliarisation = strtol(argv[18], &endptr, 10);
	}

	// Extract boolean for use of digital FT in Control thread
	ctrl_ABLE.mParams.nb_activated_motors = strtol(argv[19], &endptr, 10);
	fprintf(out_file, "Numbers of activated motors : %i\n", ctrl_ABLE.mParams.nb_activated_motors);

	// Extract motors to inhibate (1) or activate (0)
	int i = 1;
	char* str_inhib = argv[20];
	// Get first token
	ctrl_ABLE.mParams.inhibition_State[0] = strtol(strtok(str_inhib, ";"), &endptr, 10);
	fprintf(out_file, "Inhibited / Desinhibited motors :");
	fprintf(out_file, " %i", ctrl_ABLE.mParams.inhibition_State[0]);
	while (i < NB_MOTORS)
	{
		// Get following tokens of the string (using NULL instead of the string name)
		ctrl_ABLE.mParams.inhibition_State[i] = strtol(strtok(NULL, ";"), &endptr, 10);
		fprintf(out_file, " %i", ctrl_ABLE.mParams.inhibition_State[i]);
		i++;
	}
	fprintf(out_file, "\n");
	// Extract boolean for use of digital FT in Control thread
	ctrl_ABLE.rtParams.use_fx_lockedSlider = strtol(argv[21], &endptr, 10);
	fprintf(out_file, "Use Fx Wrist on slider locked and 2DDL : %i\n", ctrl_ABLE.rtParams.use_fx_lockedSlider);
}

/*---------------------------------------------------------------------------------------------------------------------
| preallocate_memory - Pre allocate vectors memory
|
| Syntax --
|	void preallocate_memory()
----------------------------------------------------------------------------------------------------------------------*/
void preallocate_memory()
{
	ctrl_ABLE.aMeasures.able_xs_slider.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.able_currents_1.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.able_currents_2.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.able_currents_3.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.able_currents_4.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.able_artpos_1.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.able_artpos_2.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.able_artpos_3.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.able_artpos_4.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.able_speeds_1.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.able_speeds_2.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.able_speeds_3.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.able_speeds_4.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.execution_times.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.fx_FTA_sensor.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.fy_FTA_sensor.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.fz_FTA_sensor.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.tx_FTA_sensor.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.ty_FTA_sensor.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.tz_FTA_sensor.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.fx_FTW_sensor.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.fy_FTW_sensor.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.fz_FTW_sensor.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.tx_FTW_sensor.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.ty_FTW_sensor.reserve(SIZE_VECS);
	ctrl_ABLE.aMeasures.tz_FTW_sensor.reserve(SIZE_VECS);
}

/*---------------------------------------------------------------------------------------------------------------------
| initialise_FT_Shared - Initialize shared struct for interthread communications
|
| Syntax --
|	void initialise_FT_Shared()
----------------------------------------------------------------------------------------------------------------------*/
void initialise_FT_Shared()
{
	// Initialise arm interlocked struct
	FT_measures_Interlocked_Arm.fx = 0.0f;
	FT_measures_Interlocked_Arm.fy = 0.0f;
	FT_measures_Interlocked_Arm.fz = 0.0f;
	FT_measures_Interlocked_Arm.tx = 0.0f;
	FT_measures_Interlocked_Arm.ty = 0.0f;
	FT_measures_Interlocked_Arm.tz = 0.0f;
	FT_measures_Interlocked_Arm.streaming = FALSE;
	// Initialise wrist interlocked struct
	FT_measures_Interlocked_Wrist.fx = 0.0f;
	FT_measures_Interlocked_Wrist.fy = 0.0f;
	FT_measures_Interlocked_Wrist.fz = 0.0f;
	FT_measures_Interlocked_Wrist.tx = 0.0f;
	FT_measures_Interlocked_Wrist.ty = 0.0f;
	FT_measures_Interlocked_Wrist.tz = 0.0f;
	FT_measures_Interlocked_Wrist.streaming = FALSE;
}

/*---------------------------------------------------------------------------------------------------------------------
| prefill_FT_Comm_Structs - Pre-fill FT structs with their respective data
|
| Syntax --
|	void prefill_FT_Comm_Structs()
----------------------------------------------------------------------------------------------------------------------*/
void prefill_FT_Comm_Structs()
{
	// Create arm output files
	FILE* out_file_FT_Arm = NULL;
	FILE* err_file_FT_Arm = NULL;
	FILE* times_FTsensor_Arm = NULL;
	fopen_s(&out_file_FT_Arm, "FT_Arm_out_file.txt", "w");
	fopen_s(&err_file_FT_Arm, "FT_Arm_err_file.txt", "w");
	fopen_s(&times_FTsensor_Arm, "iter_times_FT_Arm.txt", "w");

	// Pre-fill arm communication struct
	FT_Comm_params_Arm.times = times_FTsensor_Arm;
	FT_Comm_params_Arm.out_file_FT = out_file_FT_Arm;
	FT_Comm_params_Arm.err_file_FT = err_file_FT_Arm;
	FT_Comm_params_Arm.FT_measures_Shared = &FT_measures_Interlocked_Arm;
	FT_Comm_params_Arm.pCritical_share_FT = &Critical_share_FT_Arm;
	FT_Comm_params_Arm.general_params_FT.struct_FT_Arm = TRUE;
	FT_Comm_params_Arm.general_params_FT.struct_FT_Wrist = FALSE;

	// Create wrist output files
	FILE* out_file_FT_Wrist = NULL;
	FILE* err_file_FT_Wrist = NULL;
	FILE* times_FTsensor_Wrist = NULL;
	fopen_s(&out_file_FT_Wrist, "FT_Wrist_out_file.txt", "w");
	fopen_s(&err_file_FT_Wrist, "FT_Wrist_err_file.txt", "w");
	fopen_s(&times_FTsensor_Wrist, "iter_times_FT_Wrist.txt", "w");

	// Pre-fill wrist communication struct
	FT_Comm_params_Wrist.times = times_FTsensor_Wrist;
	FT_Comm_params_Wrist.out_file_FT = out_file_FT_Wrist;
	FT_Comm_params_Wrist.err_file_FT = err_file_FT_Wrist;
	FT_Comm_params_Wrist.FT_measures_Shared = &FT_measures_Interlocked_Wrist;
	FT_Comm_params_Wrist.pCritical_share_FT = &Critical_share_FT_Wrist;
	FT_Comm_params_Wrist.general_params_FT.struct_FT_Arm = FALSE;
	FT_Comm_params_Wrist.general_params_FT.struct_FT_Wrist = TRUE;
}

/*---------------------------------------------------------------------------------------------------------------------
| prefill_ABLE_Thread_Comm_Struct - Pre-fill ThreadInformations struct
|
| Syntax --
|	void prefill_ABLE_Thread_Comm_Struct(ThreadInformations* ableInformations, FILE* out_file, FILE* err_file)
|
| Inputs:
|	ThreadInformations* ableInformations -> pointer towards ABLE thread communication struct
|	FILE* err_file -> pointer towards "errors.txt" (stderr)
|	FILE* out_file -> pointer towards "outputs.txt" (stdout)
----------------------------------------------------------------------------------------------------------------------*/
void prefill_ABLE_Thread_Comm_Struct(ThreadInformations* ableInformations, FILE* out_file, FILE* err_file)
{
	// Initialise files
	FILE* identification_file = NULL;
	FILE* currents_file = NULL;
	FILE* artpos_file = NULL;
	FILE* speeds_file = NULL;
	FILE* xs_slider_file = NULL;
	FILE* times_file = NULL;
	FILE* ft_Arm_sensor_file = NULL;
	FILE* fz_Arm_file = NULL;
	FILE* ft_Wrist_sensor_file = NULL;
	FILE* fz_Wrist_file = NULL;

	// Open files
	fopen_s(&identification_file, "identification.txt", "w");
	fopen_s(&currents_file, "identification_courants.txt", "w");
	fopen_s(&artpos_file, "identification_positions.txt", "w");
	fopen_s(&speeds_file, "identification_vitesses.txt", "w");
	fopen_s(&xs_slider_file, "positions_chariot.txt", "w");
	fopen_s(&times_file, "iteration_times.txt", "w");
	fopen_s(&ft_Arm_sensor_file, "FT_Arm_Sensor_measures.txt", "w");
	fopen_s(&fz_Arm_file, "FZ_Arm_forces_file.txt", "w");
	fopen_s(&ft_Wrist_sensor_file, "FT_Wrist_Sensor_measures.txt", "w");
	fopen_s(&fz_Wrist_file, "FZ_Wrist_forces_file.txt", "w");

	// Initialise time debug file
	FILE* times_CommLoop = NULL;
	fopen_s(&times_CommLoop, "iter_times_ComLoop.txt", "w");

	// Pre-fill struct
	ableInformations->eth_ABLE = &eth_ABLE;
	ableInformations->ctrl_ABLE = &ctrl_ABLE;
	ableInformations->ctrl_ABLE->FT_measures_Shared_Arm = &FT_measures_Interlocked_Arm;
	ableInformations->ctrl_ABLE->pCritical_share_FT_Arm = &Critical_share_FT_Arm;
	ableInformations->ctrl_ABLE->FT_measures_Shared_Wrist = &FT_measures_Interlocked_Wrist;
	ableInformations->ctrl_ABLE->pCritical_share_FT_Wrist = &Critical_share_FT_Wrist;
	ableInformations->err_file = err_file;
	ableInformations->out_file = out_file;
	ableInformations->identification_file = identification_file;
	ableInformations->currents_file = currents_file;
	ableInformations->artpos_file = artpos_file;
	ableInformations->speeds_file = speeds_file;
	ableInformations->xs_slider_file = xs_slider_file;
	ableInformations->fz_Arm_file = fz_Arm_file;
	ableInformations->ft_Arm_sensor_file = ft_Arm_sensor_file;
	ableInformations->fz_Wrist_file = fz_Wrist_file;
	ableInformations->ft_Wrist_sensor_file = ft_Wrist_sensor_file;
	ableInformations->times_file = times_file;
	ableInformations->times = times_CommLoop;

	// Initialise FT sensors gains
	if (ableInformations->ctrl_ABLE->rtParams.use_FT && ableInformations->ctrl_ABLE->aOrders.antiG_value == 0)
	{
		ableInformations->ctrl_ABLE->current_FT_meas_Arm.k_fp = 0.1f;
		ableInformations->ctrl_ABLE->current_FT_meas_Arm.k_fi = 1.9f;
		ableInformations->ctrl_ABLE->current_FT_meas_Arm.k_fd = 0.000f;
		ableInformations->ctrl_ABLE->current_FT_meas_Wrist.k_fp = 0.1f;
		ableInformations->ctrl_ABLE->current_FT_meas_Wrist.k_fi = 0.001f;
	} else if (ableInformations->ctrl_ABLE->rtParams.use_FT && ableInformations->ctrl_ABLE->aOrders.antiG_value != 0)
	{
		ableInformations->ctrl_ABLE->current_FT_meas_Arm.k_fp = 0.5f;
		ableInformations->ctrl_ABLE->current_FT_meas_Arm.k_fi = 0.01f;
		ableInformations->ctrl_ABLE->current_FT_meas_Wrist.k_fp = 0.5f;
		ableInformations->ctrl_ABLE->current_FT_meas_Wrist.k_fi = 0.01f;
	}
}

/*---------------------------------------------------------------------------------------------------------------------
| initQTMComPipes - Initialize Pipes to communicate between threads (QTM -- CTRL)
|
| Syntax --
|	int initQTMComPipes(FILE* err_file, FILE* out_file)
|
| Inputs --
|	FILE* err_file -> pointer towards "errors.txt" (stderr)
|	FILE* out_file -> pointer towards "outputs.txt" (stdout)
----------------------------------------------------------------------------------------------------------------------*/
void initQTMComPipes(FILE* err_file, FILE* out_file)
{
	// Initialize pipe between Control (write) and QTM Measures (read)
	bool returnPipe1 = CreatePipe(&qtm_ComStruct.comPipeStruct.readPipeCtrl, &ctrl_ABLE.pipeHandlesQTM.writePipeCtrl,
		                          NULL, PIPE_BUFFER_LENGTH);
	if (!returnPipe1)
	{
		fprintf(err_file, "Pipe between Control (write) and QTM Measures (read) could not be openned.\n");
	} else {
		fprintf(out_file, "Pipe between Control (write) and QTM Measures (read) is openned.\n");
	}

	// Initialize pipe between Control (read) and QTM Measures (write)
	bool returnPipe2 = CreatePipe(&ctrl_ABLE.pipeHandlesQTM.readPipeMeas, &qtm_ComStruct.comPipeStruct.writePipeMeas,
		                          NULL, PIPE_BUFFER_LENGTH);
	if (!returnPipe2)
	{
		fprintf(err_file, "Pipe between Control (read) and QTM Measures (write) could not be openned.\n");
	}
	else {
		fprintf(out_file, "Pipe between Control (read) and QTM Measures (write) is openned.\n");
	}
}

/*---------------------------------------------------------------------------------------------------------------------
| initComPython - Initialize communication with Python script
|
| Syntax --
|	int initComPython(FILE* err_file, FILE* out_file)
|
| Inputs --
|	FILE* err_file -> pointer towards "errors.txt" (stderr)
|	FILE* out_file -> pointer towards "outputs.txt" (stdout)
|
| Outputs --
|	int -> 0 : success
----------------------------------------------------------------------------------------------------------------------*/
int initComPython(FILE* err_file, FILE* out_file)
{
	// Initialize variables
	char buffer[8];
	int err = 0;

	// Create socket descriptor and check socket openning status
	WSADATA wsaData_com = { 0 };
	if ((err = WSAStartup(MAKEWORD(2, 2), &wsaData_com)) != 0)
	{
		fprintf(err_file, "WSAStartup error code: %i", err);
	}
	qtm_ComStruct.sockStruct.socket_com = socket(AF_INET, SOCK_STREAM, 0);
	if (qtm_ComStruct.sockStruct.socket_com == INVALID_SOCKET)
	{
		fprintf(out_file, "Invalid socket parameters \n");
	}

	// Define network parameters before connection
	qtm_ComStruct.sockStruct.recvAddr.sin_family = AF_INET;
	qtm_ComStruct.sockStruct.recvAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
	qtm_ComStruct.sockStruct.recvAddr.sin_port = htons(qtm_ComStruct.sockStruct.socket_port);

	// Connect to the socket and check connection status
	err = connect(qtm_ComStruct.sockStruct.socket_com, (SOCKADDR*)&qtm_ComStruct.sockStruct.recvAddr,
		sizeof(qtm_ComStruct.sockStruct.recvAddr));
	if (err < 0)
	{
		fprintf(out_file, "Failed to connect to socket: %i!\n", err);
	}

	// Send identification message for communication with Python
	const char* sendstring = "Robot";
	send(qtm_ComStruct.sockStruct.socket_com, sendstring, strlen(sendstring), 0);
	Sleep(1);

	recv(qtm_ComStruct.sockStruct.socket_com, buffer, 8, NULL);
	fprintf(out_file, "Message received from server: %s \n", buffer);
	return err;
}

/*---------------------------------------------------------------------------------------------------------------------
| waitNomVoltage - Wait nominal voltage to protect the system
|
| Syntax --
|	void waitNomVoltage(FILE* err_file, FILE* out_file)
|
| Inputs --
|	FILE* err_file -> pointer towards "errors.txt" (stderr)
|	FILE* out_file -> pointer towards "outputs.txt" (stdout)
----------------------------------------------------------------------------------------------------------------------*/
void waitNomVoltage(FILE* err_file, FILE* out_file)
{
	// Wait for nominal voltage in motors before starting motion
	while (ctrl_ABLE.rtParams.able_RealTimeCommand != true)
	{
		if (eth_ABLE.presence_48v == 1)
		{
			// Set real time command boolean to true
			ctrl_ABLE.rtParams.able_RealTimeCommand = true;
			fprintf(out_file, "Nominal voltage reached !\n");
			fflush(out_file);
		}else
		{
			// Send null speed order, inhibit all motors and wait 10 ms
			able_WaitPower(&eth_ABLE, &ctrl_ABLE);
			fprintf(out_file, "Waiting for nominal voltage in motors...\n");
		}
	}
}

/*---------------------------------------------------------------------------------------------------------------------
| launch_FT_Measures_Arm - Function launching the FT measures thread
|
| Syntax --
|	int launch_FT_Measures_Arm(ComStruct* qtm_ComStruct, FILE* xs_slider2)
|
| Inputs --
|	ComStruct* qtmComParams -> Struct containing all communications informations
|
| Outputs --
|	int -> Error code obtained at the end of the thread
----------------------------------------------------------------------------------------------------------------------*/
int launch_FT_Measures_Arm()
{
	if (initialize_FT_sensor(&FT_Comm_params_Arm))
	{
		// Initialize thread parameters for FT sensor measures
		DWORD ftMeasuresThread_Arm;     // Create identifier for the tread and exit value

		// Launch measures thread
		all_Threads.digital_FT_thread_Arm = CreateThread(NULL,						// Security attributes (default if NULL)
														 0,							// Stack SIZE default if 0
														 &run_real_time_Measures,	// Start address (function to be executed)
														 &FT_Comm_params_Arm,		// Input data (ThreadInformation struct)
														 0,							// Creational flag (start if  0, wait if 4)
														 &ftMeasuresThread_Arm);	// Thread ID

			// Wait the end of the measures (for test versions only)
			//WaitForSingleObject(_Post_ _Notnull_ hThread_FT, INFINITE);
			//// Get execution status
			//int execution_status = static_cast<int>(GetExitCodeThread(_Post_ _Notnull_ hThread_FT, &threadReturnValue));
			// Return status
		return 1;
	}
	return -1;
}

/*---------------------------------------------------------------------------------------------------------------------
| launch_FT_Measures_Wrist - Function launching the FT measures at wrist thread
|
| Syntax --
|	int launch_FT_Measures_Wrist(ComStruct* qtm_ComStruct, FILE* xs_slider2)
|
| Inputs --
|	ComStruct* qtmComParams -> Struct containing all communications informations
|
| Outputs --
|	int -> Error code obtained at the end of the thread
----------------------------------------------------------------------------------------------------------------------*/
int launch_FT_Measures_Wrist()
{
	// Initialise Wrist FT sensor
	if (initialize_FT_sensor(&FT_Comm_params_Wrist))
	{
		// Initialize thread parameters for FT sensor measures
		DWORD ftMeasuresThread_Wrist;     // Create identifier for the tread and exit value

		// Launch measures thread
		all_Threads.digital_FT_thread_Wrist = CreateThread(NULL,					// Security attributes (default if NULL)
			                                               0,						// Stack SIZE default if 0
														   &run_real_time_Measures,	// Start address (function to be executed)
														   &FT_Comm_params_Wrist,	// Input data (ThreadInformation struct)
														   0,						// Creational flag (start if  0, wait if 4)
														   &ftMeasuresThread_Wrist);// Thread ID

		// Wait the end of the measures (for test versions only)
		//WaitForSingleObject(_Post_ _Notnull_ hThread_FT, INFINITE);
		//// Get execution status
		//int execution_status = static_cast<int>(GetExitCodeThread(_Post_ _Notnull_ hThread_FT, &threadReturnValue));
		// Return status
		return 1;
	}
	return -1;
}

/*---------------------------------------------------------------------------------------------------------------------
| launch_QTM_Measures - Function launching the QTM measures thread
|
| Syntax --
|	int launch_QTM_Measures(ComStruct* qtmComParams)
|
| Inputs --
|	ComStruct* qtmComParams -> Struct containing all communications informations
|
| Outputs --
|	int -> Error code obtained at the end of the thread
----------------------------------------------------------------------------------------------------------------------*/
int launch_QTM_Measures(ComStruct* qtm_ComStruct, FILE* xs_slider2)
{
	// Initialize variables
	// int execution_status;

	// Initialize thread parameters for QTM measures
	DWORD qtmMeasuresThread;     // Create identifier for the tread and exit value

	// Launch measures thread
	all_Threads.qtm_thread = CreateThread(NULL,			// Security attributes (default if NULL)
							 0,							// Stack SIZE default if 0
							 &get_RealTimeQTMMeas,		// Start address (function to be executed)
		                     qtm_ComStruct,				// Input data (ThreadInformation struct)
							 0,							// Creational flag (start if  0, wait if 4)
							 &qtmMeasuresThread);		// Thread ID

	// Test code
	//int i = 0;
	//while (i < 500)
	//{
	//	qtm_ReadData(&ctrl_ABLE);
	//	fprintf(xs_slider2, " %f ;", ctrl_ABLE.aDynamics.x_slider);
	//	Sleep(1);
	//	i++;
	//}

	//// Wait the end of the measures (for test versions only)
	//WaitForSingleObject(_Post_ _Notnull_ hThread_QTM, INFINITE);

	// Get execution status
	// execution_status = static_cast<int>(GetExitCodeThread(_Post_ _Notnull_ hThread_QTM, &threadReturnValue));

	// Return value associated with thread exit status for error message description (Test) or 0
	return 0;
}

/*---------------------------------------------------------------------------------------------------------------------
| executeMotions - Function launching the motion thread
|
| Syntax --
|	int executeMotions(ThreadInformations* ableInfos)
|
| Inputs --
|	ThreadInformations* ableInfos -> Struct containing all command informations
|
| Outputs --
|	int -> Error code obtained at the end of the thread
----------------------------------------------------------------------------------------------------------------------*/
int executeMotions(ThreadInformations* ableInfos)
{
	// Variables declaration
	int execution_status;

	// Compute parameters for initialization move
	initializationOrdersComputation(ableInfos->err_file, ableInfos->out_file, &ctrl_ABLE);
	fflush(ableInfos->out_file);

	// Set check orders number of iterations and sampling frequency
	ableInfos->ctrl_ABLE->rtParams.sampling_frequency = 0.001f;
	if (ableInfos->ctrl_ABLE->aOrders.ctrl_type == STATIC_IDENT)
	{
		ableInfos->ctrl_ABLE->rtParams.able_CheckTargetReachedNbIt = 2500;
	} else if (ableInfos->ctrl_ABLE->aOrders.ctrl_type == HDYN_IDENT)
	{
		ableInfos->ctrl_ABLE->rtParams.able_CheckTargetReachedNbIt = 5000;
	} else
	{
		ableInfos->ctrl_ABLE->rtParams.able_CheckTargetReachedNbIt = 500;
	}
	// Initialize thread for orders execution
	DWORD ableCommandThread, threadReturnValue;        // Create identifier for the tread and exit value
	// SetThreadPriority(hThread_ableCommand, THREAD_PRIORITY_HIGHEST); // Not critical now

	all_Threads.control_thread = CreateThread(NULL,                         // Security attributes (default if NULL)
											  0,                            // Stack SIZE default if 0
											  &able_UpdateRealTimeProcess,  // Start address (function to be executed)
											  ableInfos,                    // Input data (ThreadInformation struct)
											  0,                            // Creational flag (start if  0, wait if 4)
											  &ableCommandThread);          // Thread ID

	// Wait the end of the motion
	WaitForSingleObject(_Post_ _Notnull_ all_Threads.control_thread, INFINITE);

	// Get execution status
	execution_status = static_cast<int>(GetExitCodeThread(_Post_ _Notnull_ all_Threads.control_thread, &threadReturnValue));

	// Destroy thread object
	CloseHandle(_Post_ _Notnull_ all_Threads.control_thread);

	// Return value associated with thread exit status for error message description
	return execution_status;
}

/*---------------------------------------------------------------------------------------------------------------------
| getErrorMessage - Print error message into required files
|
| Syntax --
|	int getErrorMessage(int err, FILE* err_file, FILE* out_file)
|
| Inputs --
|	int err -> value returned by the function applying motion orders
|	FILE* err_file -> pointer towards "errors.txt" (stderr)
|	FILE* out_file -> pointer towards "outputs.txt" (stdout)
|
| Outputs --
|	int -> 0 : The orders were correctly applied; 1 : An error occured during orders application
----------------------------------------------------------------------------------------------------------------------*/
int getErrorMessage(int err, FILE* err_file, FILE* out_file)
{
	switch (err)
	{
	case 0:
		fprintf(out_file, "Orders were correctly applied !\n");
		return 0;
	case 1:
		fprintf(out_file, "Error during orders application (see error file) !\n");
		fprintf(err_file, "Connection not established with ABLE !\n");
		return 1;
	case 2:
		fprintf(out_file, "Error during orders application (see error file) !\n");
		fprintf(err_file, "Invalid control mode selected !\n");
		return 1;
	case 3:
		fprintf(out_file, "Error during orders application (see error file) !\n");
		fprintf(err_file, "Error during order transmission !\n");
		return 1;
	}
	fprintf(out_file, "Error during orders application (see error file) !\n");
	fprintf(err_file, "Unknown error occured during orders application !\n");
	return 1;
}

/*---------------------------------------------------------------------------------------------------------------------
| clean_Files - Flush and close all files
|
| Syntax --
|	void clean_Files(ThreadInformations* ableInfos)
|
| Inputs --
|	ThreadInformations* ableInfos -> Struct containing all command informations
----------------------------------------------------------------------------------------------------------------------*/
void clean_Files(ThreadInformations* ableInfos)
{
	// Make sure all messages were flushed
	fflush(ableInfos->err_file);
	fflush(ableInfos->out_file);
	fflush(ableInfos->identification_file);
	fflush(ableInfos->artpos_file);
	fflush(ableInfos->speeds_file);
	fflush(ableInfos->currents_file);
	fflush(ableInfos->xs_slider_file);
	fflush(ableInfos->fz_Arm_file);
	fflush(ableInfos->ft_Arm_sensor_file);
	fflush(ableInfos->fz_Wrist_file);
	fflush(ableInfos->ft_Wrist_sensor_file);
	fflush(ableInfos->times_file);

	// Close all text files
	fclose(ableInfos->err_file);
	fclose(ableInfos->out_file);
	fclose(ableInfos->identification_file);
	fclose(ableInfos->artpos_file);
	fclose(ableInfos->speeds_file);
	fclose(ableInfos->currents_file);
	fclose(ableInfos->xs_slider_file);
	fclose(ableInfos->fz_Arm_file);
	fclose(ableInfos->ft_Arm_sensor_file);
	fclose(ableInfos->fz_Wrist_file);
	fclose(ableInfos->ft_Wrist_sensor_file);
	fclose(ableInfos->times_file);
}