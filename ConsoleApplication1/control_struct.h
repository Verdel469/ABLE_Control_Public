/***********************************************************************************************************************
* control_struct.h -
*
* Author : Dorian Verdel
* Creation  date : 02/2019
*
* Description :
* Defines the control struct of ABLE.
***********************************************************************************************************************/

#pragma once

#ifndef CONTROL_STRUCT_H
#define CONTROL_STRUCT_H

#include <fstream>
#include <iostream>
#include <vector>
#include <atomic>
#include<chrono>
#include <Windows.h>

#include "shared_FT_struct.h"		// Header containing the shared FT measures struct definition

using namespace std;

// Constants declaration
#define NB_MOTORS 4										// Real number of motors on ABLE
#define NB_VALUES_TO_SEND 7								// Size of the tables required by ABLE
#define NB_POINTS_CODERS 1000							// Number of coder points given by constructor
#define M_PI 3.141592653589793238462643383279502884L	// Mathematical constant : pi
#define G_VAL 9.81f                                     // Definition of gravity constant
#define TARGET_ORDER 0									// Type of order 0 : Target Order, step order
#define TRAJECTORY_ORDER 1								// Type of order 1 : Trajectory order, successive orders
#define NB_MEASURES_GEOM_ID 20							// Number of target for static identification
#define NB_MEASURES_SPEED_ID 5							// Number of speeds for constant speed identification
#define NB_MEASURES_DYNAMIC_ID 40000					// Number of successive measures for dynamic identification
#define IDENT_NOT_SPLITED_DATA 0                        // Not-splited data identification method
#define IDENT_SPLITED_DATA 1                            // Splited data identification method
#define END_WAIT 5										// Waiting time at the end of movement (s)
#define NB_JERK_TRAJS 4									// Number of jerk trajectories to extract
#define NB_REP_FAM 2									// Number of repetions of each jerk traj for familiarisation
#define NB_REP 15										// Number of repetions of each jerk traj
#define NB_MEASURES_FATIGUE_TEST 10000					// Number of measures for fatigue tests at the end of jerk trajectories
#define FORCE_FATIGUE_TEST 15.0f						// Constant force to apply during fatigue tests
#define DELAYFORCEFATIGUE 2000							// Number of iterations at home position before launching the test
#define DELAYROBSTOP 2000
#define SIZE_VECS 3000000
#define NB_ANG_LEVELS 29
// Define all different control possibilities
#define STATIC_IDENT 0
#define CSPEED_IDENT 1
#define VCSPEED_IDENT 2
#define DYN_IDENT 3
#define TORQUE_CTRL 4
#define HDYN_IDENT 5
#define MINJERK_TRAJS 6
#define OSCILLATOR_CTRL 7
// Define combination numbers
#define COMBINATIONS_2MOTORS 3
#define COMBINATIONS_3MOTORS 7
#define COMBINATIONS_4MOTORS 15

// ------------------------------------------------- QTM PIPE SUBSTRUCT ------------------------------------------------
struct pipeStructQTMRead
{
	HANDLE writePipeCtrl;	// Handle of the write extremity of the pipe mastered by the control thread
	HANDLE readPipeMeas;	// Handle of the read extremity of the pipe mastered by the qtm measures thread
};

// -------------------------------------------- MOTORS' PARAMETERS SUBSTRUCT -------------------------------------------
struct motorsParams
{
	double Kp_P[NB_VALUES_TO_SEND];
	double Ki_P[NB_VALUES_TO_SEND];
	double Kp_V[NB_VALUES_TO_SEND];
	double Kp_V_r[NB_VALUES_TO_SEND];
	double Ki_V[NB_VALUES_TO_SEND];
	double Kp_I[NB_VALUES_TO_SEND];
	double Ki_I[NB_VALUES_TO_SEND];
	double Kconv_I[NB_VALUES_TO_SEND];
	float able_AxisReductions[NB_MOTORS];	        // Reductions to apply to each axis
	float offset_ADC[NB_VALUES_TO_SEND];			// Current offset to apply
	int nb_activated_motors;						// Number of activated motors (specified by user input) 
	int inhibition_State[NB_VALUES_TO_SEND];		// Inhibition state for each motor
	std::vector<int> activated_motors;				// Vector containing the indices of activated motors
	float kt_gain;									// Value of the force constant of the motors
	float able_NbPointsCoders[NB_VALUES_TO_SEND];   // Coders state variables and parameters
	// Articular range of motion under the form : 2xnb_motors tab -> line 0 : minimums, line 1 : maximums
	float able_ArtMotionsRanges[2][NB_MOTORS];
	// Different motor combinations tables
	std::vector<int> motors_combinations1;
	std::vector<int> motors_combinations2[3];
	std::vector<int> motors_combinations3[7];
	std::vector<int> motors_combinations4[15];
};

// ------------------------------------------------- ORDERS PARAMETERS -------------------------------------------------
struct ableOrders
{
	int ctrl_type;												// 0 : static, 1 : quasi-static, 2 : constant speed
																// 3 : dynamic, 4 : transparent command, 5 : human identification
	float antiG_value;											// 0 : no human compensation, 1 : weight support, 2 : rev gravity
	float able_DynModTorque;									// Torque to apply to compensate ABLE dynamics
	float max_resistanceIFPos_biceps;							// Max resistance to movement to apply during position control (upward moves)
	float max_resistanceIFPos_triceps;							// Max resistance to movement to apply during position control (downward moves)
	int orderType;												// 0 : TARGET_ORDER; 1 : TRAJECTORY_ORDER (defined by speed)
	int asservType;												// 0 : Stop; 1 : Speed; 2 : Current; 3 : Position
	float positionOrder[NB_VALUES_TO_SEND];						// Table containing articular position order to apply
	float positionCoderOrder[NB_MOTORS];						// Table containing coder position order to apply
	float speedOrder[NB_VALUES_TO_SEND];						// Table containing speed order to send to ABLE
	float currentOrder[NB_VALUES_TO_SEND];						// Table containing current order to send to ABLE (not enabled)
	float positionOrdersDoF[NB_MOTORS][NB_MEASURES_GEOM_ID];	// Table of successive positions for geometrical identification
	float dynamicOrdersId[NB_MOTORS][NB_MEASURES_DYNAMIC_ID];	// Table of successive axial positions for dynamic identification
	std::vector<float> dynamicOrdersIdAll[NB_MOTORS];			// Table of successive positions arranged for dynamic identification
	// Dynamic identification properties (dtheta(t)/dt = amplitude*omega*sin(omega*t))
	std::vector<float> omega;									// Pulsation of the speed command law
	float amplitude;											// Amplitude of the speed command law
	float angular_levels[NB_ANG_LEVELS];
};

// ------------------------------------------- IDENTIFIED DYNAMICS SUBSTRUCTS ------------------------------------------
// Substruct containing identified frictions
struct frictions
{
	float adhfric;                  // Adherence friction coefficient
	float dry_frics[2];				// Dry friction coefficients for both movements directions
	float visc_frics[2];			// Viscous friction coefficients for both movements directions
};

// Substruct containing identified inertias (-> inertia not compensated by model while no intention detection)
struct inertias
{
	float inertia_model[3];			// Inertia model (dependant of x_slider)
};

// ---------------------------------------- IDENTIFIED AXIS DYNAMICS SUBSTRUCTS ----------------------------------------
// Dynamic parameters of ABLE third axis (inertia identified but not compensated)
struct able_Axis3_model
{
	float length3;							// Identified  length of ABlE third axis
	frictions frictions3;					// Identified frictions parameters for ABLE third axis
	float gm_stat[2];						// Identified gravity model of axis 3 if speed = 0 (independant of x_slider)
	float gm_top[2];						// Identified gravity model of axis 3 if speed < 0 (independant of x_slider)
	float gm_bot[2];						// Identified gravity model of axis 3 if speed > 0 (independant of x_slider)
};

// Dynamic parameters of ABLE fourth axis (inertia identified but not compensated)
struct able_Axis4_model
{
	float mass4;							// Identified mass of ABLE fourth axis
	float x_slider;							// Position of the slider (TODO : add extern measures for real-time estimation)
	frictions frictions4;					// Identified frictions parameters for ABLE fourth axis
	float cm_stat[3];						// Identified CM position of axis 4 if speed = 0 (dependant of x_slider)
	float cm_top[3];						// Identified CM position of axis 4 if speed < 0 (dependant of x_slider)
	float cm_bot[3];						// Identified CM position of axis 4 if speed > 0 (dependant of x_slider)
};

// Substruct containing all dynamic parameters
struct ableDynamics
{
	able_Axis3_model axis3_mod;				// Identified model of ABLE third axis
	able_Axis4_model axis4_mod;				// Identified model of ABLE fourth axis
};

// ------------------------------------------- REAL TIME PARAMETERS SUBSTRUCTS -----------------------------------------
struct realTimeParams
{
	bool able_Connected;	                    // false : disconnected, true : connected
	int friction_comp;                          // 1 : friction compensated, 0 : friction not compensated
	float sampling_frequency;					// Frequency at which orders are sent to ABLE
	bool able_RealTimeCommand;					// true : Real time command allowed; false : not allowed
	bool able_OrderNotTransmitted;				// true : Error in order transmission; false : transmission OK
	int currentCoderPosition[NB_MOTORS];		// Table containing real time measured positions of coders
	float oldPosition[NB_MOTORS];				// Table containing an old position to check order termination
	float currentPosition[NB_MOTORS];			// Table containing real time computed positions
	float currentSpeed[NB_MOTORS];				// Table containing real time measured speed
	float currentVoltage[NB_MOTORS];			// Table containing real time measured voltage
	float currentADCcurrent[NB_MOTORS];			// Table containing real time measured ADC current
	int able_CheckTargetReachedNbIt;			// Number of iterations between each order state check
	int limit_iterCom;				            // Limit number of iterations in case of transparent command
	int nb_iterations_dyn_ident;				// Number of iterations in case of dynamic identification
	int able_Calibrated;						// 0 : not calibrated, 1 : calibrated
	int order_counter;                          // Order counter to change orders
	int iter_counter;							// Couter of main loop iterations
	int iter_id_start;							// Start iteration of dynamic identification
	int i2t;									// int monitoring motors overload
	float current_startMinJerk;					// Startpoint of the current minimum jerk trajectory
	float current_endMinJerk;					// Endpoint of the current minimum jerk trajectory
	int current_posMinJerk;					    // Current target position index during jerk trajectory execution
	int current_minJerkMove;					// Counter of the minimum jerk trajectories
	BOOL use_QTM;								// True : Use Qualisys measures ; False : Do not use Qualisys measures
	BOOL use_FT;								// True : Use FT measures ; False : Do not use FT measures
	BOOL jerkMove_goStart;						// True : Go to the starting point of next minimum jerk move
	BOOL jerkMove_startReached;					// True : Ready for next minimum jerk move ; False : Not ready
	BOOL jerkMove_started;						// True : Move in progress ; False : Waiting for next move to start
	BOOL jerkTrajs_AllEnded;					// True : Ended all trajectories ; False : Still movements to do
	BOOL jerkHomePosAfterTrajs;					// True : Reached home position after Jerk trajectories ; False : not reached
	BOOL robot_stopAfterTrajs;
	int jerkBlockWithFatigueTest;				// True : Add a test of fatigue at the end of the block ; False : don't
	int jerkFamiliarisation;					// True : Use familiarisation parameters
	int timer_jerk_end_move;					// Timer allowing progressive strengthenning of the position control
	int correct_fz_antigrav;					// 1 : Correction on Fz activated ; 0 : Not activated
	int correct_q_antigrav;						// 1 : Correction on q activated ; 0 : Not activated
	int correct_q_antigrav_2;					// 1 : Correction 2 on q activated ; 0 : Not activated
	int use_fx_lockedSlider;
};

// ----------------------------------------- MEASURED VALUES STORAGE SUBSTRUCTS ----------------------------------------
struct ableMeasures
{
	std::vector<float> able_xs_slider;			// Vector containing measured values of x_slider
	std::vector<float> able_currents_1;			// Vector containing values of the current for identification (axis 1)
	std::vector<float> able_currents_2;			// Vector containing values of the current for identification (axis 2)
	std::vector<float> able_currents_3;			// Vector containing values of the current for identification (axis 3)
	std::vector<float> able_currents_4;			// Vector containing values of the current for identification (axis 4)
	std::vector<float> able_artpos_1;			// Vector containing values of the position for identification (axis 1)
	std::vector<float> able_artpos_2;			// Vector containing values of the position for identification (axis 2)
	std::vector<float> able_artpos_3;			// Vector containing values of the position for identification (axis 3)
	std::vector<float> able_artpos_4;			// Vector containing values of the position for identification (axis 4)
	std::vector<float> able_speeds_1;			// Vector containing values of the speed for identification (axis 1)
	std::vector<float> able_speeds_2;			// Vector containing values of the speed for identification (axis 2)
	std::vector<float> able_speeds_3;			// Vector containing values of the speed for identification (axis 3)
	std::vector<float> able_speeds_4;			// Vector containing values of the speed for identification (axis 4)
	std::vector<double> execution_times;		// Vector containing execution time of each motion thread loop (s)
	std::vector<float> fx_FTA_sensor;			// All Fx forces sent by digital FT arm sensor for human identification
	std::vector<float> fy_FTA_sensor;			// All Fy forces sent by digital FT arm sensor for human identification
	std::vector<float> fz_FTA_sensor;			// All Fz forces sent by digital FT arm sensor for human identification
	std::vector<float> tx_FTA_sensor;			// All Tx torques sent by digital FT arm sensor for human identification
	std::vector<float> ty_FTA_sensor;			// All Ty torques sent by digital FT arm sensor for human identification
	std::vector<float> tz_FTA_sensor;			// All Tz torques sent by digital FT arm sensor for human identification
	std::vector<float> fx_FTW_sensor;			// All Fx forces sent by digital FT wrist sensor for human identification
	std::vector<float> fy_FTW_sensor;			// All Fy forces sent by digital FT wrist sensor for human identification
	std::vector<float> fz_FTW_sensor;			// All Fz forces sent by digital FT wrist sensor for human identification
	std::vector<float> tx_FTW_sensor;			// All Tx torques sent by digital FT wrist sensor for human identification
	std::vector<float> ty_FTW_sensor;			// All Ty torques sent by digital FT wrist sensor for human identification
	std::vector<float> tz_FTW_sensor;			// All Tz torques sent by digital FT wrist sensor for human identification
};

// ----------------------------------------- IDENTIFIED HUMAN DYNAMICS SUBSTRUCT ---------------------------------------
struct humanDyn
{
	float mass;               // Mass of the human forearm
	float delta_theta;		  // Angular difference between human and robot forearms (computed under matlab)
	float farm_FE_inertia;    // Equivalent inertia at the elbow for movements of flexion/extension
	int check_ExtractData;    // int checking the success of data extraction: 0 = FAIL; 1 = SUCCESS
	FILE* idDyn_File = NULL;  // File containing the indentified dynamic parameters
};

// --------------------------------------- DETECTION OF FUTURE MOVEMENT SUBSTRUCT --------------------------------------
struct emg_pings
{
	// BicepsBrachial activation (0 : no activation detected, 1 : activation detected)
	int ping_BB;
	// BrachioRadialis activation (0 : no activation detected, 1 : activation detected)
	int ping_BR;
	// TricepsBrachialLongH activation (0 : no activation detected, 1 : activation detected)
	int ping_TBLongH;
	// TricepsBrachialLatH activation (0 : no activation detected, 1 : activation detected)
	int ping_TBLatH;
};

// --------------------------------------------- CURRENT FT MEASURES STRUCT --------------------------------------------
struct received_FT_meas
{
	float f_x;											// Computed force along x axis
	float f_y;											// Computed force along y axis
	float f_z;											// Computed force along z axis
	float t_x;											// Computed torque along x axis
	float t_y;											// Computed torque along y axis
	float t_z;											// Computed torque along z axis
	float k_fp;											// Proportionnal gain of force correction
	float k_fi;											// Integral gain of force correction
	float k_fd;                                         // Derivative gain of force correction
};

// ----------------------------------------------- DEADMAN BUTTONS STRUCT ----------------------------------------------
struct deadman_buttons
{
	BOOL leftb_pushed;						// True : Left button pushed ; False : Left button not pushed
	BOOL rightb_pushed;						// True : Right button pushed ; False : Right button not pushed
};

// ----------------------------------------- PRE-COMPUTED MINIMUM JERKS STRUCT -----------------------------------------
struct minjerk_trajs
{
	ifstream minjerk_file;											// File containing pre-computed minimum jerk trajectories
	std::vector<float> parsed_movements[NB_JERK_TRAJS];				// Table containing all minimum jerk movements data
	std::vector<float> all_JmovesFam[NB_JERK_TRAJS * NB_REP_FAM];	// Table containing duplicated jerk movements data for familiarisation
	std::vector<float> all_Jmoves[NB_JERK_TRAJS*NB_REP];			// Table containing duplicated jerk movements data
};

//------------------------------------------------- HOPF OSCILLATOR STRUCT----------------------------------------------
struct hopf_oscillator
{
	float alpha_dot;		// TODO : Give meaning
	float phi_dot;			// TODO : Give meaning
	float omega_dot;		// TODO : Give meaning
};

// ------------------------------------------------ GLOBAL CONTROL STRUCT ----------------------------------------------
struct AbleControlStruct
{
	// Communication with QTM pipe handles
	pipeStructQTMRead pipeHandlesQTM;
	// Motors state variables and parameters
	motorsParams mParams;
	// Orders variables
	ableOrders aOrders;
	// Identified dynamic model for torque command on splited data
	ableDynamics aDynamics;
	// Real time parameters
	realTimeParams rtParams;
	// Measures storage
	ableMeasures aMeasures;
	// Human identified dynamics
	humanDyn hDynId;
	// EMG detection of movement
	emg_pings emgPing;
	// Last received measures form FT sensors
	received_FT_meas current_FT_meas_Arm;
	received_FT_meas current_FT_meas_Wrist;
	// Deadman buttons struct
	deadman_buttons dead_buttons;
	// Minimum jerk trajectories
	minjerk_trajs minJerk;
	// Critical section for arm measures
	CRITICAL_SECTION* pCritical_share_FT_Arm;
	// Shared measures struct for arm measures
	FT_meas_Global * FT_measures_Shared_Arm;
	// Critical section for wrist measures
	CRITICAL_SECTION* pCritical_share_FT_Wrist;
	// Shared measures struct for wrist measures
	FT_meas_Global* FT_measures_Shared_Wrist;
	// Hopf oscillator parameters struct
	hopf_oscillator hopfParams;
};
#endif // !CONTROL_STRUCT_H