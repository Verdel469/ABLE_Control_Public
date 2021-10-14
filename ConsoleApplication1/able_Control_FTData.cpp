/***********************************************************************************************************************
* able_Control_FTData.cpp -
*
* Author : Dorian Verdel
* Creation  date : 03/2021
*
* Description :
* Establishes the communication between ABLE thread and FT sensor thread.
***********************************************************************************************************************/

#include "able_Control_FTData.h"

/*---------------------------------------------------------------------------------------------------------------------
| digitalFT_ReadData - Read current FT measures
|
| Syntax --
|	void digitalFT_ReadData(AbleControlStruct* ctrl_ABLE)
|
| Inputs --
|	AbleControlStruct *ctrl_ABLE -> pointer towards control of ABLE struct
----------------------------------------------------------------------------------------------------------------------*/
void digitalFT_ReadData(ThreadInformations* ableInfos)
{
	// Request ownership of the arm critical section
	EnterCriticalSection(ableInfos->ctrl_ABLE->pCritical_share_FT_Arm);

	// Retrieve measured data
	ableInfos->ctrl_ABLE->current_FT_meas_Arm.f_x = ableInfos->ctrl_ABLE->FT_measures_Shared_Arm->fx;
	ableInfos->ctrl_ABLE->current_FT_meas_Arm.f_y = ableInfos->ctrl_ABLE->FT_measures_Shared_Arm->fy;
	ableInfos->ctrl_ABLE->current_FT_meas_Arm.f_z = ableInfos->ctrl_ABLE->FT_measures_Shared_Arm->fz;
	ableInfos->ctrl_ABLE->current_FT_meas_Arm.t_x = ableInfos->ctrl_ABLE->FT_measures_Shared_Arm->tx;
	ableInfos->ctrl_ABLE->current_FT_meas_Arm.t_y = ableInfos->ctrl_ABLE->FT_measures_Shared_Arm->ty;
	ableInfos->ctrl_ABLE->current_FT_meas_Arm.t_z = ableInfos->ctrl_ABLE->FT_measures_Shared_Arm->tz;
	ableInfos->ctrl_ABLE->rtParams.able_RealTimeCommand = ableInfos->ctrl_ABLE->FT_measures_Shared_Arm->streaming;

	// Release ownership of the arm critical section
	LeaveCriticalSection(ableInfos->ctrl_ABLE->pCritical_share_FT_Arm);

	// Check streaming state of arm sensor
	if (!ableInfos->ctrl_ABLE->rtParams.able_RealTimeCommand)
	{
		fprintf(ableInfos->out_file, "Digital FT Arm stopped streaming.\n");
	}

	// Request ownership of the wrist critical section
	EnterCriticalSection(ableInfos->ctrl_ABLE->pCritical_share_FT_Wrist);

	// Retrieve measured data
	ableInfos->ctrl_ABLE->current_FT_meas_Wrist.f_x = ableInfos->ctrl_ABLE->FT_measures_Shared_Wrist->fx;
	ableInfos->ctrl_ABLE->current_FT_meas_Wrist.f_y = ableInfos->ctrl_ABLE->FT_measures_Shared_Wrist->fy;
	ableInfos->ctrl_ABLE->current_FT_meas_Wrist.f_z = ableInfos->ctrl_ABLE->FT_measures_Shared_Wrist->fz;
	ableInfos->ctrl_ABLE->current_FT_meas_Wrist.t_x = ableInfos->ctrl_ABLE->FT_measures_Shared_Wrist->tx;
	ableInfos->ctrl_ABLE->current_FT_meas_Wrist.t_y = ableInfos->ctrl_ABLE->FT_measures_Shared_Wrist->ty;
	ableInfos->ctrl_ABLE->current_FT_meas_Wrist.t_z = ableInfos->ctrl_ABLE->FT_measures_Shared_Wrist->tz;
	ableInfos->ctrl_ABLE->rtParams.able_RealTimeCommand = ableInfos->ctrl_ABLE->FT_measures_Shared_Wrist->streaming;

	// Release ownership of the wrist critical section
	LeaveCriticalSection(ableInfos->ctrl_ABLE->pCritical_share_FT_Wrist);

	// Check streaming state of wrist sensor
	if (!ableInfos->ctrl_ABLE->rtParams.able_RealTimeCommand)
	{
		fprintf(ableInfos->out_file, "Digital FT Wrist stopped streaming.\n");
	}

	// Store the sent fz force for human forearm mass identification
	if ((ableInfos->ctrl_ABLE->aOrders.ctrl_type == HDYN_IDENT) &&
		ableInfos->ctrl_ABLE->rtParams.order_counter > 0)
	{
		ableInfos->ctrl_ABLE->aMeasures.fz_FTA_sensor.push_back(ableInfos->ctrl_ABLE->current_FT_meas_Arm.f_z);
		ableInfos->ctrl_ABLE->aMeasures.fz_FTW_sensor.push_back(ableInfos->ctrl_ABLE->current_FT_meas_Wrist.f_z);
	}
}

/*---------------------------------------------------------------------------------------------------------------------
| digitalFT_WriteData - Write messages to the FT measures thread
|
| Syntax --
|	void digitalFT_WriteData(AbleControlStruct* ctrl_ABLE)
|
| Inputs --
|	AbleControlStruct *ctrl_ABLE -> pointer towards control of ABLE struct
----------------------------------------------------------------------------------------------------------------------*/
void digitalFT_WriteData(AbleControlStruct* ctrl_ABLE)
{
	// Request ownership of the critical section
	EnterCriticalSection(ctrl_ABLE->pCritical_share_FT_Arm);

	// Retrieve measured data
	ctrl_ABLE->FT_measures_Shared_Arm->streaming = TRUE;

	// Release ownership of the critical section
	LeaveCriticalSection(ctrl_ABLE->pCritical_share_FT_Arm);

	// Request ownership of the critical section
	EnterCriticalSection(ctrl_ABLE->pCritical_share_FT_Wrist);

	// Retrieve measured data
	ctrl_ABLE->FT_measures_Shared_Wrist->streaming = TRUE;

	// Release ownership of the critical section
	LeaveCriticalSection(ctrl_ABLE->pCritical_share_FT_Wrist);
}