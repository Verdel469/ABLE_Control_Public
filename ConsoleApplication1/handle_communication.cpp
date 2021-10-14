/***********************************************************************************************************************
* handle_communication.cpp -
*
* Author : Dorian Verdel, structure extracted from Recette3, coded by : Paul de Poix
* Creation  date : 02/2019
*
* Description :
* Manages the communication with the variator card.
***********************************************************************************************************************/
// Includes
#include "handle_communication.h"

// Namespaces
using namespace std;
using namespace std::chrono;

/*---------------------------------------------------------------------------------------------------------------------
| able_InitCommunication - Initialization of communication with ABLE
|
| Syntax --
|	int able_InitCommunication(ServoComEth *eth_ABLE, AbleControlStruct *ctrl_ABLE, FILE* err_file, FILE* out_file)
|
| Inputs --
|	ServoComEth *eth_ABLE -> pointer towards communication struct
|	AbleControlStruct *ctrl_ABLE -> pointer towards control of ABLE struct
|	FILE* err_file -> pointer towards standard errors file
|	FILE* out_file -> pointer towards standard outputs file
|
| Outputs --
|	int value -> 0 : Initialization went well; 1: Connection failure
----------------------------------------------------------------------------------------------------------------------*/
int able_InitCommunication(ServoComEth *eth_ABLE, AbleControlStruct *ctrl_ABLE, FILE* err_file, FILE* out_file)
{
	int err = 0;
	// Initialize variables for compatibility with ABLE
	eth_ABLE->nb_moteurs = 7;
	eth_ABLE->activation_checksum = 1;
	eth_ABLE->activation_complement_a_un = 0;
	// Initialise socket parameters
	err += ETH_carte_variateur_V3_Initialise(eth_ABLE, IP_ABLE, PORT_ABLE);
	if (err == 0){fprintf(out_file, "Initialization OK !\n");}
	else{fprintf(err_file, "Failed to initialize !\n");}
	// Start communication with ABLE
	err += ETH_carte_variateur_V3_Connexion(eth_ABLE);
	if (err == 0) { fprintf(out_file, "Connexion message sent to ABLE !\n"); }
	else { fprintf(err_file, "Failed to send message to ABLE !\n"); }
	// Wait 1 ms to make sure the message has arrived
	Sleep(1);
	// Check connection state and update control struct connexion value
	if (ETH_carte_variateur_V3_Receive_Identification(eth_ABLE) == 0)
	{
		fprintf(out_file, "Note: connection to ABLE OK \n");
		ctrl_ABLE->rtParams.able_Connected = true;
	}else
	{
		ETH_carte_variateur_V3_Close(eth_ABLE);
		fprintf(err_file, "ABLE connection failure ! Please check power and communication parameters.\n");
		ctrl_ABLE->rtParams.able_Connected = false;
		return 1;
	}
	// Check ABLE calibration state and update control struct calibration value
	if (eth_ABLE->etat_calibration == 1)
	{
		fprintf(out_file,"Note: ABLE calibration OK \n");
		ctrl_ABLE->rtParams.able_Calibrated = 1;
	}else
	{
		fprintf(err_file, "Note: no calibration of ABLE \n");
		ctrl_ABLE->rtParams.able_Calibrated = 0;
	}
	// Initialize ADC offset of motors
	// Send request to ABLE
	ETH_carte_variateur_V3_Send_Request_ADC(eth_ABLE);
	// Wait for answer
	Sleep(1);
	// Store offset in control struct
	ETH_carte_variateur_V3_Receive_ADC_Offset(eth_ABLE, ctrl_ABLE->mParams.offset_ADC);

	// Inhibit motors and ask for 48V
	// Initialize TOR outputs
	ETH_carte_variateur_V3_Stor(eth_ABLE, 0x0001);
	// Inhibit all motors and set speed to null
	able_SendNullSpeedOrder(eth_ABLE, ctrl_ABLE, 1);
	// Receive state message and store it in comunication struct
	ETH_carte_variateur_V3_Receive_State(eth_ABLE);
	// Translate state message and store informations in communication struct
	ETH_carte_variateur_V3_Datas(eth_ABLE);
	// Get current position of coders
	for (int i(0); i < NB_MOTORS; i++)
	{
		ctrl_ABLE->rtParams.currentCoderPosition[i] = eth_ABLE->moteur[i].Position_Codeur;
	}
	return 0;
}


/*---------------------------------------------------------------------------------------------------------------------
| able_CloseCommunication - End the communication with ABLE
|
| Syntax --
|	int able_CloseCommunication(ServoComEth *eth_ABLE, AbleControlStruct *ctrl_ABLE)
|
| Inputs --
|	ServoComEth *eth_ABLE -> pointer towards communication struct
|	AbleControlStruct *ctrl_ABLE -> pointer towards control of ABLE struct
----------------------------------------------------------------------------------------------------------------------*/
int able_CloseCommunication(ServoComEth *eth_ABLE, AbleControlStruct *ctrl_ABLE)
{
	// Update control struct connexion value
	ctrl_ABLE->rtParams.able_Connected = false;
	// Call function ending communication
	return ETH_carte_variateur_V3_Close(eth_ABLE);
}

/*---------------------------------------------------------------------------------------------------------------------
| able_SendMotorParameters - Sets all gains and motors parameters and sends them to ABLE
|
| Syntax --
|	void able_SendMotorParameters(ServoComEth *eth_ABLE, AbleControlStruct *ctrl_ABLE)
|
| Inputs --
|	ServoComEth *eth_ABLE -> pointer towards communication struct
|	AbleControlStruct *ctrl_ABLE -> pointer towards control of ABLE struct
----------------------------------------------------------------------------------------------------------------------*/
void able_SendMotorParameters(ServoComEth *eth_ABLE, AbleControlStruct *ctrl_ABLE)
{
	realTimeParams* rtValues = &ctrl_ABLE->rtParams;
	motorsParams* mValues = &ctrl_ABLE->mParams;
	ableOrders* oValues = &ctrl_ABLE->aOrders;
	// Set mode of control
	ETH_carte_variateur_V3_Modes(eth_ABLE, MODE_ASSERVISSEMENT_VITESSE);
	// Store number of points of coders and coefficients of conversion for currents into communication struct
	ETH_carte_variateur_V3_Set_Motor_Parameters(eth_ABLE, mValues->able_NbPointsCoders, mValues->Kconv_I);
	// Send number of points of coders and coefficients of conversion for currents to ABLE
	ETH_carte_variateur_V3_Send_Motors_Parameters(eth_ABLE);
	// Wait 1 ms to make sure the parameters are transmitted
	Sleep(1);
	// Receive data from ABLE (response)
	ETH_carte_variateur_V3_Receive(eth_ABLE);
	// Set gains of the PI corrector of the position loop into communication struct
	ETH_carte_variateur_V3_Set_K_B(eth_ABLE, mValues->Kp_P, mValues->Ki_P);
	// Set gains of torque control and PI corrector of the current loop into communication struct
	ETH_carte_variateur_V3_Set_Asserv_Parameters(eth_ABLE, mValues->Kp_V, mValues->Kp_I, mValues->Ki_I);
	// Send all gains to ABLE
	ETH_carte_variateur_V3_Send_Asserv_Parameters(eth_ABLE);
	// Wait 1 ms to make sure the parameters are transmitted
	Sleep(1);
	// Receive data from ABLE (response)
	ETH_carte_variateur_V3_Receive(eth_ABLE);
}

/*---------------------------------------------------------------------------------------------------------------------
| update_RealTimeProcess - Real time process update function
|
| Syntax --
|	void update_RealTimeProcess(ServoComEth *eth_ABLE, AbleControlStruct *ctrl_ABLE, FILE* err_file, FILE* out_file)
|
| Inputs --
|	ServoComEth *eth_ABLE -> pointer towards communication struct
|	AbleControlStruct *ctrl_ABLE -> pointer towards control of ABLE struct
|	FILE* err_file -> pointer towards standard errors file
|	FILE* out_file -> pointer towards standard outputs file
|
| Outputs --
|	int value -> 0: Everything went well; 1: Able was not connected; 2: Control mode not valid;
|                3: Order could not be transmitted
----------------------------------------------------------------------------------------------------------------------*/
DWORD WINAPI able_UpdateRealTimeProcess(LPVOID ableArgs)
{
	// Variables declaration
	static int err = 0;
	
	// Retrieve informations sent by main code
	static ThreadInformations* ableInfos = (ThreadInformations*)ableArgs;
	realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;
	motorsParams* mValues = &ableInfos->ctrl_ABLE->mParams;
	ableOrders* oValues = &ableInfos->ctrl_ABLE->aOrders;

	// Initialise counter
	rtValues->iter_counter = 0;

	// Start command loop
	while ((rtValues->order_counter <= NB_MEASURES_GEOM_ID && oValues->ctrl_type == STATIC_IDENT) ||
		   (rtValues->iter_counter <= rtValues->nb_iterations_dyn_ident && oValues->ctrl_type == DYN_IDENT) ||
		   (rtValues->iter_counter <= rtValues->limit_iterCom && oValues->ctrl_type == TORQUE_CTRL) ||
		   (oValues->ctrl_type == MINJERK_TRAJS && (rtValues->current_minJerkMove < NB_REP * NB_JERK_TRAJS ||
		   rtValues->current_minJerkMove >= NB_REP * NB_JERK_TRAJS && rtValues->jerkBlockWithFatigueTest)))
	{
		auto timestamp_0 = high_resolution_clock::now();
		// Check the real time command boolean
		if (rtValues->able_RealTimeCommand == true)
		{
			// Check connection with ABLE
			//auto timestamp_1 = high_resolution_clock::now();
			if (ableInfos->ctrl_ABLE->rtParams.able_Connected == false)
			{
				fprintf(ableInfos->err_file, "Error at order : %i\n", rtValues->order_counter);
				able_SetPowerOff(ableInfos->eth_ABLE, ableInfos->ctrl_ABLE);
				return 1;
			}
			//auto timestamp_2 = high_resolution_clock::now();
			//duration<double> elapsed_CheckCo = timestamp_2 - timestamp_1;
			// Send data to the QTM thread through Pipe
			// qtm_WriteData(ableInfos->ctrl_ABLE, iter_counter);

			// Saturation of the speed order to protect motors
			//timestamp_1 = high_resolution_clock::now();
			sature_SpeedOrders(ableInfos);
			//timestamp_2 = high_resolution_clock::now();
			//duration<double> elapsed_SatSpeed = timestamp_2 - timestamp_1;

			// Send orders to ABLE according to the asserv type and check exit status
			//timestamp_1 = high_resolution_clock::now();
			err = able_SendOrders(ableInfos);
			//timestamp_2 = high_resolution_clock::now();
			//duration<double> elapsed_SendOrd = timestamp_2 - timestamp_1;
			//timestamp_1 = high_resolution_clock::now();
			if (err != 0)
			{
				fprintf(ableInfos->err_file, "Selected control mode not valid !\n");
				able_SendNullSpeedOrder(ableInfos->eth_ABLE, ableInfos->ctrl_ABLE, 1);
				able_SetPowerOff(ableInfos->eth_ABLE, ableInfos->ctrl_ABLE);
				return 2;
			}
			//timestamp_2 = high_resolution_clock::now();
			//duration<double> elapsed_CheckOrd = timestamp_2 - timestamp_1;
			// Wait to respect sampling frequency
			//timestamp_1 = high_resolution_clock::now();
			able_OneMsWait();
			//timestamp_2 = high_resolution_clock::now();
			//duration<double> elapsed_WaitOneMs = timestamp_2 - timestamp_1;
			// Check if the order was correctly transmitted to ABLE and receive state frame
			//timestamp_1 = high_resolution_clock::now();
			check_OrderTransmission(ableInfos);
			//timestamp_2 = high_resolution_clock::now();
			//duration<double> elapsed_CheckOrdTrans = timestamp_2 - timestamp_1;
			// Translate state frame into current data of ABLE
			//timestamp_1 = high_resolution_clock::now();
			ETH_carte_variateur_V3_Datas(ableInfos->eth_ABLE);
			//timestamp_2 = high_resolution_clock::now();
			//duration<double> elapsed_Data = timestamp_2 - timestamp_1;
			// Get current measures of the QTM API
			if (ableInfos->ctrl_ABLE->aOrders.ctrl_type > TORQUE_CTRL && rtValues->use_QTM)
			{
				qtm_ReadData(ableInfos->ctrl_ABLE);
			}
			// Get current FT measures
			//timestamp_1 = high_resolution_clock::now();
			if (ableInfos->ctrl_ABLE->aOrders.ctrl_type > DYN_IDENT && rtValues->use_FT)
			{
				if (rtValues->iter_counter == 0)
				{
					fprintf(ableInfos->out_file, "Send start streaming order to FT sensors\n");
					digitalFT_WriteData(ableInfos->ctrl_ABLE);
				}
				digitalFT_ReadData(ableInfos);
			}
			//timestamp_2 = high_resolution_clock::now();
			//duration<double> elapsed_RecFT = timestamp_2 - timestamp_1;
			// fprintf(ableInfos->out_file, "AFTER FT SENSOR\n");
			// Check if deadman button activated
			//timestamp_1 = high_resolution_clock::now();
			deadman_CheckButtons(ableInfos);
			//timestamp_2 = high_resolution_clock::now();
			//duration<double> elapsed_CheckButtons = timestamp_2 - timestamp_1;
			// Compute orders for next iteration
			//timestamp_1 = high_resolution_clock::now();
			able_UpdateOrders(ableInfos);
			//timestamp_2 = high_resolution_clock::now();
			//duration<double> elapsed_UpdateOrd = timestamp_2 - timestamp_1;
			// Check order state every "able_CheckTargetReachedNbIt" iterations
			//timestamp_1 = high_resolution_clock::now();
			if ((err = switch_OrderReached(ableInfos)) == 0)
			{
				able_SendNullSpeedOrder(ableInfos->eth_ABLE, ableInfos->ctrl_ABLE, 1);
				end_time_wait();
				return 0;
			}
			auto timestamp_2 = high_resolution_clock::now();
			//duration<double> elapsed_SwitchOrd = timestamp_2 - timestamp_1;
			// Store iteration duration for time analysis
			duration<double> elapsed = timestamp_2 - timestamp_0;
			ableInfos->ctrl_ABLE->aMeasures.execution_times.push_back(elapsed.count());
			//timestamp_1 = high_resolution_clock::now();
			//fflush(ableInfos->times);
			fflush(ableInfos->out_file);
			fflush(ableInfos->err_file);
			/*timestamp_2 = high_resolution_clock::now();
			duration<double> elapsed_fflush = timestamp_2 - timestamp_1;
			duration<double> elapsed = timestamp_2 - timestamp_0;
			fprintf(ableInfos->times, " %f ; %f ; %f ; %f ; %f ; %f ; %f ; %f ; %f ; %f ; %f ; %f ; %f ;",
				elapsed_CheckCo.count(), elapsed_SatSpeed.count(), elapsed_SendOrd.count(), elapsed_CheckOrd.count(),
				elapsed_WaitOneMs.count(), elapsed_CheckOrdTrans.count(), elapsed_Data.count(), elapsed_RecFT.count(),
				elapsed_CheckButtons.count(), elapsed_UpdateOrd.count(), elapsed_SwitchOrd.count(), elapsed_fflush.count(),
				elapsed.count());*/
			rtValues->iter_counter++;
		}else
		{
			break;
		}
	}
	able_SendNullSpeedOrder(ableInfos->eth_ABLE, ableInfos->ctrl_ABLE, 1);
	end_time_wait();

	return 0;
}

/*---------------------------------------------------------------------------------------------------------------------
| deadman_CheckButtons - Check if a deadman button has been pushed
| Syntax --
|	void deadman_CheckButtons(ThreadInformations* ableInfos)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
----------------------------------------------------------------------------------------------------------------------*/
void deadman_CheckButtons(ThreadInformations* ableInfos)
{
	//fprintf(ableInfos->out_file, "CHECK BUTTONS\n");
	realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;
	// Initialize byte variables
	int entree_tor = ableInfos->eth_ABLE->entree_tor;
	uint8_t start_Byte = entree_tor & 0x000F;
	// Check bit 2
	ableInfos->ctrl_ABLE->dead_buttons.leftb_pushed = (((start_Byte >> 2) & 1) == 1);
	// Check bit 3
	ableInfos->ctrl_ABLE->dead_buttons.rightb_pushed = (((start_Byte >> 3) & 1) == 1);
	// Start next minimum jerk move if button pushed and MINJERK_TRAJS
	//if (ableInfos->ctrl_ABLE->aOrders.ctrl_type == MINJERK_TRAJS && !rtValues->jerkMove_started &&
	//	!rtValues->jerkMove_goStart && !rtValues->jerkMove_startReached &&
	//	(ableInfos->ctrl_ABLE->dead_buttons.leftb_pushed || ableInfos->ctrl_ABLE->dead_buttons.rightb_pushed))
	//{
	//	rtValues->jerkMove_goStart = TRUE;
	//} else 
	if (ableInfos->ctrl_ABLE->aOrders.ctrl_type == MINJERK_TRAJS && !rtValues->jerkMove_started && rtValues->jerkMove_startReached &&
	   (ableInfos->ctrl_ABLE->dead_buttons.leftb_pushed || ableInfos->ctrl_ABLE->dead_buttons.rightb_pushed))
	{
		rtValues->jerkMove_started = TRUE;
		//rtValues->jerkMove_goStart = FALSE;
		fprintf(ableInfos->out_file, "Minimum jerk move started !\n");
		fflush(ableInfos->out_file);
	}
}

/*---------------------------------------------------------------------------------------------------------------------
| check_OrderTransmission - Checks if order was transmitted
|
| Syntax --
|	void check_OrderTransmission(ThreadInformations* ableInfos, int iter_counter)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
|   int iter_counter -> counter of iteration
----------------------------------------------------------------------------------------------------------------------*/
void check_OrderTransmission(ThreadInformations* ableInfos)
{
	// Extract real time parameters
	realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;

	// Check transmission
	if (ETH_carte_variateur_V3_Receive_State(ableInfos->eth_ABLE))
	{
		ableInfos->ctrl_ABLE->rtParams.able_OrderNotTransmitted = true;
		fprintf(ableInfos->err_file, "Error in order transmission at iteration %i\n", rtValues->iter_counter);
	}else
	{
		ableInfos->ctrl_ABLE->rtParams.able_OrderNotTransmitted = false;
	}
}

/*---------------------------------------------------------------------------------------------------------------------
| able_SendOrders - Send current orders contained in the control structure to ABLE
|
| Syntax --
|	int able_SendOrders(ThreadInformations* ableInfos, int iter_counter)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
|
| Outputs --
|	int-> 0 : Orders transmitted; 1 : Mode selected not compatible with motion
----------------------------------------------------------------------------------------------------------------------*/
int able_SendOrders(ThreadInformations* ableInfos)
{
	ableOrders* oValues = &ableInfos->ctrl_ABLE->aOrders;
	realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;
	// Sequence of calls to send speed orders to ABLE according to chosen control mode
	ETH_carte_variateur_V3_Stor(ableInfos->eth_ABLE, 0x0001);
	ETH_carte_variateur_V3_Inhibitions2(ableInfos->eth_ABLE, ableInfos->ctrl_ABLE->mParams.inhibition_State);
	switch (oValues->asservType)
	{
	case 0:
		if (rtValues->iter_counter == 0){fprintf(ableInfos->err_file, "Check control mode! Stop mode selected !\n");}
		fflush(ableInfos->err_file);
		return 1;
	case 1:
		if (rtValues->iter_counter == 0){fprintf(ableInfos->out_file,	"Selected control mode : Speed control \n");}
		ETH_carte_variateur_V3_Consignes_Vitesse(ableInfos->eth_ABLE, oValues->speedOrder);
		ETH_carte_variateur_V3_Send_Consignes(ableInfos->eth_ABLE);
		break;
	case 2:
		if (rtValues->iter_counter == 0){fprintf(ableInfos->out_file,	"Selected control mode : ADC Current control \n");}
		ETH_carte_variateur_V3_Consignes_Courant(ableInfos->eth_ABLE, oValues->currentOrder);
		ETH_carte_variateur_V3_Send_Consignes(ableInfos->eth_ABLE);
		break;
	case 3:
		if (rtValues->iter_counter == 0){fprintf(ableInfos->out_file,	"Selected control mode : Position control \n");}
		ETH_carte_variateur_V3_Consignes_Position(ableInfos->eth_ABLE, oValues->positionOrder,
			                                      oValues->speedOrder);
		ETH_carte_variateur_V3_Send_Consignes(ableInfos->eth_ABLE);
		break;
	}
	return 0;
}



