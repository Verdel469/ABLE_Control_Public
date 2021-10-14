/***********************************************************************************************************************
* torque_control.cpp -
*
* Author : Dorian Verdel
* Creation  date : 03/2021
*
* Description :
* Achieves torque control of ABLE according to control type.
***********************************************************************************************************************/

#include "torque_control.h"

// -------------------------------------------------- MAIN TORQUE CONTROL ----------------------------------------------

/*---------------------------------------------------------------------------------------------------------------------
| able_TorqueAsserv - Updates the contents of the control struct and the orders that will be sent during next iteration
|
| Syntax --
|	void able_TorqueAsserv(ThreadInformations* ableInfos, int counter)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
|   int counter -> index of the order being executed
----------------------------------------------------------------------------------------------------------------------*/
void able_TorqueAsserv(ThreadInformations* ableInfos)
{
	// Substructs extraction
	ableOrders* oValues = &ableInfos->ctrl_ABLE->aOrders;
	realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;
	motorsParams* mValues = &ableInfos->ctrl_ABLE->mParams;

	// Variables declaration
	float gain_Kp_V, gain_Kt;
	static int counter_FatigueTest = 0;

	for (int i(0); i < NB_MOTORS; i++)
	{
		// Extract current data
		able_ExtractData(ableInfos, i);
		
		// Compute torque control if activated motor
		if (mValues->inhibition_State[i] == 0)
		{
			// Compute filtered speed
			able_ComputeDynModelCompensation(ableInfos, i);

			// Set corresponding speed order to ignore the controller speed loop and build a torque control
			gain_Kp_V = (float)mValues->Kp_V[i];
			gain_Kt = mValues->kt_gain;
			oValues->speedOrder[i] = oValues->able_DynModTorque / (gain_Kp_V * gain_Kt) + rtValues->currentSpeed[i];

			// If digital FT sensor measures are used for control add data
			if (rtValues->use_FT && oValues->ctrl_type == TORQUE_CTRL)
			{
				if (oValues->antiG_value == 0)
				{
					able_TransparentFT_Control(ableInfos, i);
				}
				else {
					able_AntigravFT_Control(ableInfos, i);
				}
			}
			else if (rtValues->use_FT && oValues->ctrl_type == MINJERK_TRAJS) {
				able_FatigueTestFT_Control(ableInfos, i, counter_FatigueTest);
				counter_FatigueTest++;
			}
		}
	}
	// Store current values
	storeValuesInVectors(ableInfos);
	// Print current values
	if (oValues->ctrl_type == TORQUE_CTRL || oValues->ctrl_type == MINJERK_TRAJS)
	{
		recordCurrentValues(ableInfos);
	}
}

// ----------------------------------------------- DYNAMIC MODEL COMPENSATION ------------------------------------------

/*----------------------------------------------------------------------------------------------------------------------
| able_ComputeDynModelCompensation - Compute dynamic model compensation to apply (inertial torques are ignored for now)
|
| Syntax --
|	void able_ComputeDynModelCompensation(ThreadInformations* ableInfos, int i)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
|	int i -> Counter corresponding to the motor number
----------------------------------------------------------------------------------------------------------------------*/
void able_ComputeDynModelCompensation(ThreadInformations* ableInfos, int i)
{
	// Extract "real-time" substructs
	ableOrders* oValues = &ableInfos->ctrl_ABLE->aOrders;
	realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;
	motorsParams* mValues = &ableInfos->ctrl_ABLE->mParams;
	// Extract dynamic parameters substructs
	ableDynamics* aDyns = &ableInfos->ctrl_ABLE->aDynamics;
	able_Axis3_model* ax3_mod = &aDyns->axis3_mod;
	able_Axis4_model* ax4_mod = &aDyns->axis4_mod;
	frictions* fmds3 = &aDyns->axis3_mod.frictions3;
	frictions* fmds4 = &aDyns->axis4_mod.frictions4;

	// Initialise variables
	float art_speed_3, art_speed_4;
	float compensation_torque = 0.0f;

	// Compute current speeds
	art_speed_3 = rtValues->currentSpeed[2] * 2 * (float)M_PI / mValues->able_AxisReductions[2];
	art_speed_4 = rtValues->currentSpeed[3] * 2 * (float)M_PI / mValues->able_AxisReductions[3];

	//mean_speed_3 = speed_MeanFilter(ableInfos, art_speed_3);
	//mean_speed_4 = speed_MeanFilter(ableInfos, art_speed_4);

	// Compute compensation according to speed
	compensation_torque = able_ComputeDynModelStat(ableInfos, i);

	// Compute final command torque
	oValues->able_DynModTorque = compensation_torque;
}

/*----------------------------------------------------------------------------------------------------------------------
| able_ComputeDynModelStat - Compute dynamic model compensation to apply if both axis are static
|
| Syntax --
|	float able_ComputeDynModelStat(ThreadInformations* ableInfos, int i)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
|	int i -> Counter corresponding to the motor number
|
| Outputs --
|   float compensation_torque -> Total dynamic compensation to apply
----------------------------------------------------------------------------------------------------------------------*/
float able_ComputeDynModelStat(ThreadInformations* ableInfos, int i)
{
	// Extract "real-time" substructs
	realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;
	motorsParams* mValues = &ableInfos->ctrl_ABLE->mParams;
	// Extract dynamic parameters substructs
	ableDynamics* aDyns = &ableInfos->ctrl_ABLE->aDynamics;
	able_Axis3_model* ax3_mod = &aDyns->axis3_mod;
	able_Axis4_model* ax4_mod = &aDyns->axis4_mod;
	frictions* fmds3 = &aDyns->axis3_mod.frictions3;
	frictions* fmds4 = &aDyns->axis4_mod.frictions4;

	// Initialise variables
	float art_theta_i, art_speed_i, art_theta_ip1, art_speed_ip1;				// Define intermediate variables
	float art_theta_im1, art_speed_im1;											//
	float friction_torque, gravity_torque, coriolis_torque;						// Define intermediate compensation torques
	float compensation_torque;													// Define final compensation torques

	// Initialise values
	friction_torque = 0.0f;
	gravity_torque = 0.0f;
	coriolis_torque = 0.0f;
	art_theta_ip1 = 0.0f;
	art_speed_ip1 = 0.0f;
	art_theta_im1 = 0.0f;
	art_speed_im1 = 0.0f;
	compensation_torque = 0.0f;

	// Compute articular position, speed and mean speed
	art_theta_i = rtValues->currentPosition[i];
	art_speed_i = rtValues->currentSpeed[i] * 2 * (float)M_PI / mValues->able_AxisReductions[i];

	// Compute dynamic model compensation according to current axis
	if (i == NB_MOTORS - 2)
	{
		// Compute data of fourth axis
		art_theta_ip1 = rtValues->currentPosition[i + 1];
		art_speed_ip1 = rtValues->currentSpeed[i + 1] * 2 * (float)M_PI / mValues->able_AxisReductions[i + 1];
		// Compute friction compensation
		friction_torque = (fmds4->adhfric + fmds3->visc_frics[0] * art_speed_i) * rtValues->friction_comp;
		// Compute gravity  compensation
		gravity_torque = (G_VAL * (ax3_mod->gm_stat[0] * sin(art_theta_i) - ax3_mod->gm_stat[1] * cos(art_theta_i)))
		               + G_VAL * ax4_mod->mass4 * (ax3_mod->length3 * sin(art_theta_i));
					   //+ (ax4_mod->cm_stat[0] * ax4_mod->x_slider + ax4_mod->cm_stat[1]) * cos(art_theta_i + art_theta_ip1 +
						//(float)M_PI / 2)
					   //- ax4_mod->cm_stat[2] * sin(art_theta_i + art_theta_ip1 + (float)M_PI / 2)) * 1.0f;
		// Compute coriolis compensation
		coriolis_torque = (ax3_mod->length3 * (pow(art_speed_ip1, 2) + 2 * art_speed_i * art_speed_ip1)
						* ((ax4_mod->cm_bot[0] * ax4_mod->x_slider + ax4_mod->cm_bot[1]) * sin(art_theta_ip1)
						+ ax4_mod->cm_bot[2] * cos(art_theta_ip1))) * 0.0f;
		// Compute compensation torque
		compensation_torque = friction_torque + (gravity_torque + coriolis_torque) /
			                  mValues->able_AxisReductions[i];

	} else if (i == NB_MOTORS - 1) {
		// Compute data of third axis
		art_theta_im1 = rtValues->currentPosition[i - 1];
		art_speed_im1 = rtValues->currentSpeed[i - 1] * 2 * (float)M_PI / mValues->able_AxisReductions[i - 1];
		// Compute friction compensation
		friction_torque = (fmds4->adhfric + fmds4->visc_frics[0] * art_speed_i) * rtValues->friction_comp;
		// Compute gravity  compensation
		gravity_torque = G_VAL
			           * ((ax4_mod->cm_stat[0] * ax4_mod->x_slider + ax4_mod->cm_stat[1]) * cos(art_theta_i + art_theta_im1)
				       - ax4_mod->cm_stat[2] * sin(art_theta_i + art_theta_im1));
		// Compute coriolis compensation
		coriolis_torque = ax3_mod->length3 * art_speed_im1 * (2 * art_speed_im1 + art_speed_i)
						* ((ax4_mod->cm_bot[0] * ax4_mod->x_slider + ax4_mod->cm_bot[1]) * sin(art_theta_i)
						+ ax4_mod->cm_bot[2] * cos(art_theta_i));
		// Compute compensation torque
		compensation_torque = friction_torque + (gravity_torque + coriolis_torque) / mValues->able_AxisReductions[i];
	}
	// Return computed compensation
	return compensation_torque;
}

// -------------------------------------------------- FT CONTROL FUNCTIONS ---------------------------------------------

/*----------------------------------------------------------------------------------------------------------------------
| able_TransparentFT_Control - Achieve transparent control of ABLE
|
| Syntax --
|	void able_TransparentFT_Control(ThreadInformations* ableInfos, int i)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
|	int i -> Counter corresponding to the motor number
----------------------------------------------------------------------------------------------------------------------*/
void able_TransparentFT_Control(ThreadInformations* ableInfos, int i)
{
	// Extract substructs
	ableOrders* oValues = &ableInfos->ctrl_ABLE->aOrders;
	realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;
	received_FT_meas* ftValues_Arm = &ableInfos->ctrl_ABLE->current_FT_meas_Arm;
	received_FT_meas* ftValues_Wrist = &ableInfos->ctrl_ABLE->current_FT_meas_Wrist;

	// Initialise variables
	static float integral_sum[NB_MOTORS];
	float error, art_theta_ip1, art_theta_i;
	
	error = 0.0f;

	// Compute parameters
	if (i == NB_MOTORS - 2)
	{
		art_theta_i = rtValues->currentPosition[i];
		art_theta_ip1 = rtValues->currentPosition[i + 1];
		error = -ftValues_Arm->f_z;
		integral_sum[i] += error * rtValues->sampling_frequency;				// Compute integral error
		// Compute transparent order to apply
		oValues->speedOrder[i] += error * ftValues_Arm->k_fp
			                    + ftValues_Arm->k_fi * integral_sum[i];
	} else if (i == NB_MOTORS - 1)
	{
		error = -ftValues_Wrist->f_z;									// Compute current wrist error
		integral_sum[i] += error * rtValues->sampling_frequency;		// Compute integral error
		// Compute transparent order to apply
		oValues->speedOrder[i] += error * ftValues_Wrist->k_fp + ftValues_Wrist->k_fi * integral_sum[i];
	}
}

/*----------------------------------------------------------------------------------------------------------------------
| able_AntigravFT_Control - Achieve weight compensation / inversion control of ABLE
|
| Syntax --
|	void able_AntigravFT_Control(ThreadInformations* ableInfos, int i)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
|	int i -> Counter corresponding to the motor number
----------------------------------------------------------------------------------------------------------------------*/
void able_AntigravFT_Control(ThreadInformations* ableInfos, int i)
{
	// Extract substructs
	ableOrders* oValues = &ableInfos->ctrl_ABLE->aOrders;
	realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;
	received_FT_meas* ftValues = &ableInfos->ctrl_ABLE->current_FT_meas_Wrist;
	humanDyn* hmds = &ableInfos->ctrl_ABLE->hDynId;

	// Initialise variables
	static float integral_sum[NB_MOTORS];
	float error, theoretical_fz, art_theta_i;

	// Compute parameters
	art_theta_i = rtValues->currentPosition[i];					// Get current axis position
	theoretical_fz = -hmds->mass * G_VAL * cos(art_theta_i + hmds->delta_theta) * oValues->antiG_value;
	error = theoretical_fz - ftValues->f_z;						// Compute current error
	integral_sum[i] += error * rtValues->sampling_frequency;	// Compute integral error

	// Compute antigravity order to apply
	oValues->speedOrder[i] += rtValues->correct_fz_antigrav * (0.205f * theoretical_fz + 0.398f)
							+ rtValues->correct_q_antigrav * (-1.71f * art_theta_i + 0.81f)
							+ rtValues->correct_q_antigrav_2 * (-2.42f * art_theta_i + 1.21f)
							+ error * ftValues->k_fp
							+ integral_sum[i] * ftValues->k_fi;
}

/*----------------------------------------------------------------------------------------------------------------------
| able_FatigueTestFT_Control - Achieve fatigue tests by applying a constant force on the participant
|
| Syntax --
|	void able_FatigueTestFT_Control(ThreadInformations* ableInfos, int i)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
|	int i -> Counter corresponding to the motor number
----------------------------------------------------------------------------------------------------------------------*/
void able_FatigueTestFT_Control(ThreadInformations* ableInfos, int i, int counter_FatigueTest)
{
	// Extract substructs
	ableOrders* oValues = &ableInfos->ctrl_ABLE->aOrders;
	realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;
	received_FT_meas* ftValues = &ableInfos->ctrl_ABLE->current_FT_meas_Wrist;

	// Initialise variables
	static float integral_sum[NB_MOTORS];
	float error;

	// Compute constant force orders for fatigue blocks
	if (counter_FatigueTest < NB_MEASURES_FATIGUE_TEST) {
		// First block with positive force (fatigue of triceps)
		error = FORCE_FATIGUE_TEST - ftValues->f_z;
		integral_sum[i] += error * rtValues->sampling_frequency;
		oValues->speedOrder[i] += error * ftValues->k_fp + ftValues->k_fi * integral_sum[i];
	}
	else if (counter_FatigueTest >= NB_MEASURES_FATIGUE_TEST && counter_FatigueTest < 2 * NB_MEASURES_FATIGUE_TEST) {
		// Second block with negative force (fatigue of biceps)
		error = -FORCE_FATIGUE_TEST - ftValues->f_z;
		integral_sum[i] += error * rtValues->sampling_frequency;
		oValues->speedOrder[i] += error * ftValues->k_fp + ftValues->k_fi * integral_sum[i];
	}
	else {
		// Turn down the robot after orders execution
		able_SetPowerOff(ableInfos->eth_ABLE, ableInfos->ctrl_ABLE);
	}
}

// ------------------------------------------------- COMPUTATIONAL FUNCTIONS -------------------------------------------

/*----------------------------------------------------------------------------------------------------------------------
| speed_MeanFilter - Mean filter for low speeds
|
| Syntax --
|	float speed_MeanFilter(ThreadInformations* ableInfos, float mean_speed, float art_speed_i)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
|	float mean_speed -> Filtered speed to return
|   float art_speed_i -> current articular speed
----------------------------------------------------------------------------------------------------------------------*/
float speed_MeanFilter(ThreadInformations* ableInfos, float art_speed_i)
{
	// Extract substructs
	motorsParams* mValues = &ableInfos->ctrl_ABLE->mParams;
	ableMeasures* measValues = &ableInfos->ctrl_ABLE->aMeasures;

	// Re-initialise variable
	float mean_speed = 0.0f;

	// Compute mean speed
	if (measValues->able_speeds_4.size() >= 49)
	{
		for (int j(measValues->able_speeds_4.size() - 49); j < (int)measValues->able_speeds_4.size(); j++)
		{
			// Compute current mean articular speed of axis i
			mean_speed += static_cast<float>(measValues->able_speeds_4.at(j)) * 2.0f * static_cast<float>(M_PI)
				/ mValues->able_AxisReductions[3];
		}
		mean_speed += art_speed_i;
		mean_speed /= 50;
	}
	else
	{
		for (int j(0); j < (int)measValues->able_speeds_4.size(); j++)
		{
			// Compute current mean articular speed of axis i
			mean_speed += static_cast<float>(measValues->able_speeds_4.at(j)) * 2.0f * static_cast<float>(M_PI)
				/ mValues->able_AxisReductions[3];
		}
		mean_speed += art_speed_i;
		mean_speed /= (measValues->able_speeds_4.size() + 1);
	}
	return mean_speed;
}