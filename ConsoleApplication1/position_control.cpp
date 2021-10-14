/***********************************************************************************************************************
* position_control.cpp -
*
* Author : Dorian Verdel
* Creation  date : 03/2021
*
* Description :
* Manages the position control of ABLE according to control type.
***********************************************************************************************************************/

#include "position_control.h"

/*---------------------------------------------------------------------------------------------------------------------
| able_PositionAsserv - Updates the contents of the control struct and the orders that will be sent during
|                       next iteration
|
| Syntax --
|	void able_PositionAsserv(ThreadInformations* ableInfos)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
----------------------------------------------------------------------------------------------------------------------*/
void able_PositionAsserv(ThreadInformations* ableInfos)
{
	// Variables declaration
	static float pos_dif, gain_Kp_P_i, gain_Ki_P_i, measured_current;
	static float integral_sum[NB_MOTORS] = { 0.0f,0.0f,0.0f,0.0f };
	static int old_counter;
	realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;
	ableOrders* oValues = &ableInfos->ctrl_ABLE->aOrders;
	motorsParams* mValues = &ableInfos->ctrl_ABLE->mParams;

	// Check if order has changed since previous iteration
	if (old_counter != rtValues->order_counter && oValues->ctrl_type != MINJERK_TRAJS)
	{
		// Update old counter to the current order
		old_counter = rtValues->order_counter;
		fprintf(ableInfos->out_file, "Order counter : %d ; Position order : %f\n", rtValues->order_counter,
			ableInfos->ctrl_ABLE->aOrders.positionOrder[3]);
		// Reset the sum for integral correction to avoid uncontrolled behaviour
		for (int i(0); i < NB_MOTORS; i++)
		{
			if (mValues->inhibition_State[i] == 0)
			{
				integral_sum[i] = 0.0f;
			}
		}
	}
	// Asserv position for setting home position
	for (int i(0); i < NB_MOTORS; i++)
	{
		// Extract current data
		able_ExtractData(ableInfos, i);
		// Compute diference in coder position
		pos_dif = oValues->positionOrder[i] - rtValues->currentPosition[i];
		// Compute next iteration speed order based on position diference and proportionnal gain
		// Cast gains into float
		gain_Kp_P_i = static_cast<float>(ableInfos->ctrl_ABLE->mParams.Kp_P[i]);
		gain_Ki_P_i = static_cast<float>(ableInfos->ctrl_ABLE->mParams.Ki_P[i]);
		// Compute sum for integral correction
		integral_sum[i] += pos_dif * ableInfos->ctrl_ABLE->rtParams.sampling_frequency;
		// Store the computed order into the control struct
		oValues->speedOrder[i] = gain_Kp_P_i * pos_dif + gain_Ki_P_i * integral_sum[i];
		// Regulate interaction force for CoT experimentation
		if (i == NB_MOTORS - 1 && oValues->ctrl_type == MINJERK_TRAJS && rtValues->jerkMove_started)
		{
			able_RegulateIFPos(ableInfos);
		}
	}
	fprintf(ableInfos->out_file, "Computed position/speed order value %f\n", oValues->speedOrder[3]);
	if (rtValues->order_counter > 0 || oValues->ctrl_type == MINJERK_TRAJS)
	{
		// Store current values
		storeValuesInVectors(ableInfos);
	}
	// Print current values
	if (oValues->ctrl_type == MINJERK_TRAJS || rtValues->order_counter > 0 && oValues->ctrl_type == 0)
	{
		recordCurrentValues(ableInfos);
	}
}

/*---------------------------------------------------------------------------------------------------------------------
| able_RegulateIFPos - Regulates interaction forces during position control
|
| Syntax --
|	void able_RegulateIFPos(ThreadInformations* ableInfos)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
----------------------------------------------------------------------------------------------------------------------*/
void able_RegulateIFPos(ThreadInformations* ableInfos)
{
	// Initialise variables
	float direction;
	static float integral_sum, speed_sign_old;
	// Extract substructs
	ableOrders* oValues = &ableInfos->ctrl_ABLE->aOrders;
	realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;
	motorsParams* mValues = &ableInfos->ctrl_ABLE->mParams;
	ableMeasures* measValues = &ableInfos->ctrl_ABLE->aMeasures;
	received_FT_meas* ftValues = &ableInfos->ctrl_ABLE->current_FT_meas_Wrist;
	// Re-initialise integral sum if change of direction
	if (speed_sign_old != rtValues->currentSpeed[3] / abs(rtValues->currentSpeed[3]))
	{
		speed_sign_old = rtValues->currentSpeed[3] / abs(rtValues->currentSpeed[3]);
		integral_sum = 0.0f;
	}
	// Regulate interaction forces
	direction = rtValues->current_endMinJerk - rtValues->current_startMinJerk;
	if (rtValues->use_FT && rtValues->currentPosition[3] < oValues->positionOrder[3] - 0.005f && direction < 0)
	{
		integral_sum += (oValues->max_resistanceIFPos_biceps - ftValues->f_z) * rtValues->sampling_frequency;
		oValues->speedOrder[3] = (oValues->max_resistanceIFPos_biceps - ftValues->f_z) * ftValues->k_fp * 0.13f
				                + (double)ftValues->k_fi * 12000.0 * (double)integral_sum;
	}
	else if (rtValues->use_FT && rtValues->currentPosition[3] > oValues->positionOrder[3] + 0.005f && direction > 0)
	{
		integral_sum += (oValues->max_resistanceIFPos_triceps - ftValues->f_z) * rtValues->sampling_frequency;
		oValues->speedOrder[3] = (oValues->max_resistanceIFPos_triceps - ftValues->f_z) * ftValues->k_fp * 0.1f
			                    + (double)ftValues->k_fi * 8000.0 * (double)integral_sum;
	}
}

/*---------------------------------------------------------------------------------------------------------------------
| able_DynIdentAsserv - Updates the contents of the control struct and dynamic identification orders
|
| Syntax --
|	void able_DynIdentAsserv(ThreadInformations* ableInfos, int iter_counter)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
|   int counter -> index of the order being executed
|   int iter_counter -> number of the current iteration for dynamic identification orders indexing
----------------------------------------------------------------------------------------------------------------------*/
void able_DynIdentAsserv(ThreadInformations* ableInfos, int iter_counter)
{
	// Variables declaration
	static float pos_dif, speed_dif, sampling;
	static float gain_Kp_P_i, gain_Ki_P_i, gain_Kp_V, gain_Kp_V_r, gain_Ki_V, integral_sum_speed;
	static float integral_sum[NB_MOTORS];
	static int old_counter;
	ableOrders* oValues = &ableInfos->ctrl_ABLE->aOrders;
	realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;
	motorsParams* mValues = &ableInfos->ctrl_ABLE->mParams;

	// Check if order has changed since previous iteration
	if (old_counter != rtValues->order_counter)
	{
		// Update old counter to the current order
		old_counter = rtValues->order_counter;
		// Reset the sum for integral correction to avoid uncontrolled behaviour
		integral_sum[3] = 0.0f;
		integral_sum_speed = 0.0f;
	}
	for (int i(0); i < NB_MOTORS; i++)
	{
		// Extract current data
		able_ExtractData(ableInfos, i);
		// Compute next iteration speed order and store it into the control struct
		if (mValues->inhibition_State[i] == 0)
		{
			if (iter_counter < (int)oValues->dynamicOrdersIdAll[i].size())
			{
				// Cast gains into float
				gain_Kp_P_i = static_cast<float>(mValues->Kp_P[i]);
				gain_Ki_P_i = static_cast<float>(mValues->Ki_P[i]);
				// Compute position difference (orders are a sinusoïd)
				pos_dif = oValues->dynamicOrdersIdAll[i].at(iter_counter) - rtValues->currentPosition[i];
				// Compute integral sum of error
				integral_sum[i] += pos_dif * rtValues->sampling_frequency;
				// Compute order to send
				oValues->speedOrder[i] = gain_Kp_P_i * pos_dif + gain_Ki_P_i * integral_sum[i];
			}
			else
			{
				able_SendNullSpeedOrder(ableInfos->eth_ABLE, ableInfos->ctrl_ABLE, 1);
			}

		}
	}
	// Store current values
	storeValuesInVectors(ableInfos);
	recordCurrentValues(ableInfos);
}