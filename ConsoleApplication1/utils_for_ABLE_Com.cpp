/***********************************************************************************************************************
* utils_for_ABLE_Com.cpp -
*
* Author : Dorian Verdel
* Creation  date : 03/2021
*
* Description :
* Manages the definition of useful functions for communication with ABLE.
***********************************************************************************************************************/

#include "utils_for_ABLE_Com.h"

// Namespaces
using namespace std::chrono;	// Namespace for high resolution clock

// ---------------------------------------------------- HEALTH FUNCTIONS -----------------------------------------------

/*----------------------------------------------------------------------------------------------------------------------
| check_state - Optionnal function testing all possible errors
|
| Syntax --
|	void check_state(ThreadInformations* ableInfos)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
----------------------------------------------------------------------------------------------------------------------*/
void check_state(ThreadInformations* ableInfos)
{
	// Check i2t value
	fprintf(ableInfos->out_file, "I2t values : ");
	for (int i(0); i < NB_MOTEURS_V3B; i++)
	{
		fprintf(ableInfos->out_file, "%i ", ableInfos->eth_ABLE->i2t[i]);
	}
	fprintf(ableInfos->out_file, "\n");
	// Check overheat
	fprintf(ableInfos->out_file, "OHt values : ");
	for (int i(0); i < NB_MOTEURS_V3B; i++)
	{
		fprintf(ableInfos->out_file, "%i ", ableInfos->eth_ABLE->pwm_surchauffe[i]);
	}
	fprintf(ableInfos->out_file, "\n");

	// Check overheat
	fprintf(ableInfos->out_file, "PWM values : ");
	for (int i(0); i < NB_MOTEURS_V3B; i++)
	{
		fprintf(ableInfos->out_file, "%i ", ableInfos->eth_ABLE->erreur_pwm[i]);
	}
	fprintf(ableInfos->out_file, "\n");

	// Check overheat
	fprintf(ableInfos->out_file, "PCo values : ");
	for (int i(0); i < NB_MOTEURS_V3B; i++)
	{
		fprintf(ableInfos->out_file, "%i ", ableInfos->eth_ABLE->panne_codeur[i]);
	}
	fprintf(ableInfos->out_file, "\n");
	// Check overheat
	fprintf(ableInfos->out_file, "PIn values : ");
	for (int i(0); i < NB_MOTEURS_V3B; i++)
	{
		fprintf(ableInfos->out_file, "%i ", ableInfos->eth_ABLE->panne_courant[i]);
	}
	fprintf(ableInfos->out_file, "\n");
	// Check overheat
	fprintf(ableInfos->out_file, "PPi values : ");
	for (int i(0); i < NB_MOTEURS_V3B; i++)
	{
		fprintf(ableInfos->out_file, "%i ", ableInfos->eth_ABLE->erreur_checksum_piccolo[i]);
	}
	fprintf(ableInfos->out_file, "\n");
	// Check overheat
	fprintf(ableInfos->out_file, "PCO values : ");
	for (int i(0); i < NB_MOTEURS_V3B; i++)
	{
		fprintf(ableInfos->out_file, "%i ", ableInfos->eth_ABLE->erreur_checksum_controleur[i]);
	}
	fprintf(ableInfos->out_file, "\n");
	// Check overheat
	fprintf(ableInfos->out_file, "Inh values : ");
	for (int i(0); i < NB_MOTEURS_V3B; i++)
	{
		fprintf(ableInfos->out_file, "%i ", ableInfos->eth_ABLE->etat_inhibition[i]);
	}
	fprintf(ableInfos->out_file, "\n");
	fprintf(ableInfos->out_file, "48V presence value : %i\n", ableInfos->eth_ABLE->presence_48v);
	fprintf(ableInfos->out_file, "Urgence stop value : %i\n", ableInfos->eth_ABLE->arret_urgence);
	fprintf(ableInfos->out_file, "Relais value : %i\n", ableInfos->eth_ABLE->panne_relais);
}

// ----------------------------------------------- DATA EXTRACTION FUNCTIONS -------------------------------------------

/*----------------------------------------------------------------------------------------------------------------------
| able_ExtractData - Extract data sent by ABLE
|
| Syntax --
|	void able_ExtractData(ThreadInformations* ableInfos, int i)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
|	int i -> Counter corresponding to the motor number
----------------------------------------------------------------------------------------------------------------------*/
void able_ExtractData(ThreadInformations* ableInfos, int i)
{
	// Variables Declaration
	static float art_theta_i, measured_current;
	struct realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;
	struct motorsParams* mValues = &ableInfos->ctrl_ABLE->mParams;
	// Storing current data in the control struct
	rtValues->currentCoderPosition[i] = ableInfos->eth_ABLE->moteur[i].Position_Codeur;
	rtValues->currentSpeed[i] = ableInfos->eth_ABLE->moteur[i].Vitesse_Filtree;
	rtValues->currentVoltage[i] = ableInfos->eth_ABLE->moteur[i].ADC_Potentiometre;
	// Expression of the current given by the constructor of ABLE
	measured_current = static_cast<float>(mValues->Kconv_I[i])*
		(static_cast<float>(ableInfos->eth_ABLE->moteur[i].ADC_Courant) - static_cast<float>(mValues->offset_ADC[i]));
	rtValues->currentADCcurrent[i] = measured_current;
	// Compute current position of axis i
	art_theta_i = able_ComputeCurrentPosition(rtValues->currentCoderPosition[i], mValues->able_AxisReductions[i]);
	rtValues->currentPosition[i] = art_theta_i;
}

/*---------------------------------------------------------------------------------------------------------------------
| able_ComputeCurrentPosition - Compute the current position of an axis thanks to current coder position and the sign
|                               of the position (signed value)
|
| Syntax --
|	float able_ComputeCurrentPosition(int coder_pos)
|
| Inputs --
|	int coder_pos -> current coder position of the considered axis
|   float reduction -> reduction values given by constructor
|
| Outputs --
|	float theta_i -> Value of the motor angle in rad (signed value)
----------------------------------------------------------------------------------------------------------------------*/
float able_ComputeCurrentPosition(int coder_pos, float reduction)
{
	// Compute motor angular position in rad (4000 tops/rev for each coder)
	float theta_i = static_cast<float>((2 * M_PI) / (4 * NB_POINTS_CODERS) * coder_pos);
	// Compute articular position with reduction
	float art_theta_i = theta_i * 1 / reduction;
	// Treat sign of the position
	if (art_theta_i > M_PI / 2)
	{
		art_theta_i = static_cast<float>(-(2 * M_PI - theta_i * 1 / reduction));
	}
	return art_theta_i;
}

// ---------------------------------------------- PARTICULAR ORDERS FUNCTIONS ------------------------------------------

/*----------------------------------------------------------------------------------------------------------------------
| able_SendNullSpeedOrder - (Des)Inhibit all motors and send null speed order to all motors
|
| Syntax --
|	void able_SendNullSpeedOrder(ServoComEth *eth_ABLE,  AbleControlStruct *ctrl_ABLE, int inibition)
|
| Inputs --
|	ServoComEth *eth_ABLE -> pointer towards communication struct
|	AbleControlStruct *ctrl_ABLE -> pointer towards control of ABLE struct
|	int inhibition -> 1 : inhibit all motors , 0 : desinhibit all motors
----------------------------------------------------------------------------------------------------------------------*/
void able_SendNullSpeedOrder(ServoComEth* eth_ABLE, AbleControlStruct* ctrl_ABLE, int inhibition)
{
	// Initialize values of the control struct
	for (int i(0); i < NB_MOTORS; i++)
	{
		ctrl_ABLE->aOrders.speedOrder[i] = 0.0f;
	}
	// Send order to inhibit all motors in communication struct
	ETH_carte_variateur_V3_Inhibitions(eth_ABLE, inhibition);
	// Send speed order to communication struct
	ETH_carte_variateur_V3_Consignes_Vitesse(eth_ABLE, ctrl_ABLE->aOrders.speedOrder);
	// Transmit orders to ABLE
	ETH_carte_variateur_V3_Send_Consignes(eth_ABLE);
	// Wait to respect sampling frequency of ABLE
	auto timestamp_1_w = high_resolution_clock::now();
	while (true)
	{
		auto timestamp_2_w = high_resolution_clock::now();
		duration<double> elapsed_w = timestamp_2_w - timestamp_1_w;
		if (elapsed_w.count() > 0.001f)
		{
			break;
		}
	}
}

/*---------------------------------------------------------------------------------------------------------------------
| able_SetPowerOff - Turn ABLE power off and inhibit motors
|
| Syntax --
|	void able_SetPowerOff(ServoComEth *eth_ABLE, AbleControlStruct *ctrl_ABLE)
|
| Inputs --
|	ServoComEth *eth_ABLE -> pointer towards communication struct
|	AbleControlStruct *ctrl_ABLE -> pointer towards control of ABLE struct
----------------------------------------------------------------------------------------------------------------------*/
void able_SetPowerOff(ServoComEth* eth_ABLE, AbleControlStruct* ctrl_ABLE)
{
	// Set inhibition
	int inhibition = 1;
	// Turn power off
	ETH_carte_variateur_V3_Stor(eth_ABLE, 0x0000);
	// Inhibit motors and send null speed order
	able_SendNullSpeedOrder(eth_ABLE, ctrl_ABLE, inhibition);
}

/*---------------------------------------------------------------------------------------------------------------------
| sature_SpeedOrders - Sature speed orders to protect the robot during identification
|
| Syntax --
|	void sature_SpeedOrders(ThreadInformations* ableInfos)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
----------------------------------------------------------------------------------------------------------------------*/
void sature_SpeedOrders(ThreadInformations* ableInfos)
{
	// Extract substructs
	motorsParams* mValues = &ableInfos->ctrl_ABLE->mParams;
	ableOrders* oValues = &ableInfos->ctrl_ABLE->aOrders;
	realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;
	float direction, timer = 0.0f;

	//fprintf(ableInfos->out_file, "SAT SPEED\n");
	if (oValues->ctrl_type == MINJERK_TRAJS)
	{
		direction = rtValues->current_endMinJerk - rtValues->current_startMinJerk;
		rtValues->timer_jerk_end_move++;
		timer = rtValues->timer_jerk_end_move / 100.0f;
	}

	// Sature speeds
	for (int i(0); i < NB_MOTORS; i++)
	{
		// MAX_SPEED : 35 (Constructor)
		if (oValues->speedOrder[i] > 20.0f && oValues->ctrl_type == TORQUE_CTRL && rtValues->order_counter != 0 &&
			rtValues->iter_counter <= 6000)
		{
			oValues->speedOrder[i] = 20.0f;
		}
		else if (oValues->speedOrder[i] > 40.0f && oValues->ctrl_type == TORQUE_CTRL && rtValues->iter_counter > 6000)
		{
			oValues->speedOrder[i] = 40.0f;
		}
		else if (oValues->speedOrder[i] > 60.0f && oValues->ctrl_type == MINJERK_TRAJS && rtValues->jerkMove_started && direction > 0)
		{
			oValues->speedOrder[i] = 60.0f;
		}
		else if (oValues->speedOrder[i] > 0.0f && oValues->ctrl_type == MINJERK_TRAJS && rtValues->jerkMove_started && direction < 0)
		{
			oValues->speedOrder[i] = 0.0f;
		}
		else if (oValues->speedOrder[i] > 10.0f && oValues->ctrl_type == MINJERK_TRAJS && !rtValues->jerkMove_started
			&& rtValues->current_minJerkMove == 0)
		{
			oValues->speedOrder[i] = 10.0f;
		}
		else if (oValues->speedOrder[i] > 10.0f && oValues->ctrl_type == MINJERK_TRAJS && !rtValues->jerkMove_started
			&& rtValues->current_minJerkMove > 0)
		{
			oValues->speedOrder[i] = 10.0f;
		}
		else if (oValues->speedOrder[i] > 20.0f  && oValues->ctrl_type == DYN_IDENT)
		{
			oValues->speedOrder[i] = 20.0f;
		}
		else if (oValues->speedOrder[i] > 10.0f && rtValues->order_counter != 0 && oValues->ctrl_type == HDYN_IDENT)
		{
			oValues->speedOrder[i] = 10.0f;
		}
		else if (oValues->speedOrder[i] > 20.0f && rtValues->order_counter != 0 && oValues->ctrl_type == OSCILLATOR_CTRL)
		{
			oValues->speedOrder[i] = 20.0f;
		}
		else if (oValues->speedOrder[i] > 10.0f && (rtValues->order_counter == 0 || oValues->ctrl_type == STATIC_IDENT)
			&& oValues->ctrl_type != MINJERK_TRAJS)
		{
			oValues->speedOrder[i] = 10.0f;
		}
		// MIN_SPEED : -35 (Constructor)
		if (oValues->speedOrder[i] < -20.0f && oValues->ctrl_type == TORQUE_CTRL && rtValues->order_counter != 0 &&
			rtValues->iter_counter <= 6000)
		{
			oValues->speedOrder[i] = -20.0f;
		}
		else if (oValues->speedOrder[i] < -40.0f && oValues->ctrl_type == TORQUE_CTRL && rtValues->iter_counter > 6000)
		{
			oValues->speedOrder[i] = -40.0f;
		}
		else if (oValues->speedOrder[i] < -60.0f && oValues->ctrl_type == MINJERK_TRAJS && rtValues->jerkMove_started && direction < 0)
		{
			oValues->speedOrder[i] = -60.0f;
		}
		else if (oValues->speedOrder[i] < 0.0f && oValues->ctrl_type == MINJERK_TRAJS && rtValues->jerkMove_started && direction > 0)
		{
			oValues->speedOrder[i] = 0.0f;
		}
		else if (oValues->speedOrder[i] < -10.0f && oValues->ctrl_type == MINJERK_TRAJS && !rtValues->jerkMove_started
			&& rtValues->current_minJerkMove == 0)
		{
			oValues->speedOrder[i] = -10.0f;
		}
		else if (oValues->speedOrder[i] < -10.0f && oValues->ctrl_type == MINJERK_TRAJS && !rtValues->jerkMove_started
			&& rtValues->current_minJerkMove > 0)
		{
			oValues->speedOrder[i] = -10.0;
		}
		else if (oValues->speedOrder[i] < -20.0f && oValues->ctrl_type == DYN_IDENT)
		{
			oValues->speedOrder[i] = -20.0f;
		}
		else if (oValues->speedOrder[i] < -10.0f && rtValues->order_counter != 0 && oValues->ctrl_type == HDYN_IDENT)
		{
			oValues->speedOrder[i] = -10.0f;
		}
		else if (oValues->speedOrder[i] < -20.0f && rtValues->order_counter != 0 && oValues->ctrl_type == OSCILLATOR_CTRL)
		{
			oValues->speedOrder[i] = -20.0f;
		}
		else if (oValues->speedOrder[i] < -10.0f && (rtValues->order_counter == 0 || oValues->ctrl_type == STATIC_IDENT)
			&& oValues->ctrl_type != MINJERK_TRAJS)
		{
			oValues->speedOrder[i] = -10.0f;
		}
	}
}

// ---------------------------------------------------- TIMING FUNCTIONS -----------------------------------------------

/*----------------------------------------------------------------------------------------------------------------------
| able_OneMsWait - Wait one millisecond
|
| Syntax --
|	void able_OneMsWait()
----------------------------------------------------------------------------------------------------------------------*/
void able_OneMsWait()
{
	auto timestamp_1_w = high_resolution_clock::now();
	while (true)
	{
		auto timestamp_2_w = high_resolution_clock::now();
		duration<double> elapsed_w = timestamp_2_w - timestamp_1_w;
		if (elapsed_w.count() > 0.001f) { break; }
	}
}

/*---------------------------------------------------------------------------------------------------------------------
| able_WaitPower - Updates the status of ABLE by sending a null speed order
|
| Syntax --
|	void able_WaitPower(ServoComEth *eth_ABLE, AbleControlStruct *ctrl_ABLE)
|
| Inputs --
|	ServoComEth *eth_ABLE -> pointer towards communication struct
|	AbleControlStruct *ctrl_ABLE -> pointer towards control of ABLE struct
----------------------------------------------------------------------------------------------------------------------*/
void able_WaitPower(ServoComEth* eth_ABLE, AbleControlStruct* ctrl_ABLE)
{
	// Inhibit all motors and set speed to null
	able_SendNullSpeedOrder(eth_ABLE, ctrl_ABLE, 1);
	Sleep(1);
	// Receive state message and store it in comunication struct
	ETH_carte_variateur_V3_Receive_State(eth_ABLE);
	Sleep(1);
	// Translate state message and store informations in communication struct
	ETH_carte_variateur_V3_Datas(eth_ABLE);
	// Wait
	Sleep(8);
}

/*---------------------------------------------------------------------------------------------------------------------
| end_time_wait - Waiting time at the end of the movement
|
| Syntax --
|	void end_time_wait()
----------------------------------------------------------------------------------------------------------------------*/
void end_time_wait()
{
	auto timestamp_1_w = high_resolution_clock::now();
	while (true)
	{
		auto timestamp_2_w = high_resolution_clock::now();
		duration<double> elapsed_w = timestamp_2_w - timestamp_1_w;
		if (elapsed_w.count() > END_WAIT)
		{
			break;
		}
	}
}