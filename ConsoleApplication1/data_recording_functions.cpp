/***********************************************************************************************************************
* data_recording_functions.cpp -
*
* Author : Dorian Verdel
* Creation  date : 03/2021
*
* Description :
* Manages the recording of data during program execution
***********************************************************************************************************************/

#include "data_recording_functions.h"

/*---------------------------------------------------------------------------------------------------------------------
| storeValuesInVectors - Store current state of ABLE
|
| Syntax --
|	void storeValuesInVectors(ThreadInformations* ableInfos)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
----------------------------------------------------------------------------------------------------------------------*/
void storeValuesInVectors(ThreadInformations* ableInfos)
{
	// Get pointers towards structs
	ableMeasures* cMeasures = &ableInfos->ctrl_ABLE->aMeasures;
	realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;
	ableDynamics* cDyn = &ableInfos->ctrl_ABLE->aDynamics;
	received_FT_meas* rtFTmeas_Arm = &ableInfos->ctrl_ABLE->current_FT_meas_Arm;
	received_FT_meas* rtFTmeas_Wrist = &ableInfos->ctrl_ABLE->current_FT_meas_Wrist;
	// Store current values during motion
	cMeasures->able_currents_1.push_back(rtValues->currentADCcurrent[0]);
	cMeasures->able_currents_2.push_back(rtValues->currentADCcurrent[1]);
	cMeasures->able_currents_3.push_back(rtValues->currentADCcurrent[2]);
	cMeasures->able_currents_4.push_back(rtValues->currentADCcurrent[3]);
	// Store articular positions values during motion
	cMeasures->able_artpos_1.push_back(rtValues->currentPosition[0]);
	cMeasures->able_artpos_2.push_back(rtValues->currentPosition[1]);
	cMeasures->able_artpos_3.push_back(rtValues->currentPosition[2]);
	cMeasures->able_artpos_4.push_back(rtValues->currentPosition[3]);
	// Store speed values during motion
	cMeasures->able_speeds_1.push_back(rtValues->currentSpeed[0]);
	cMeasures->able_speeds_2.push_back(rtValues->currentSpeed[1]);
	cMeasures->able_speeds_3.push_back(rtValues->currentSpeed[2]);
	cMeasures->able_speeds_4.push_back(rtValues->currentSpeed[3]);
	// Store x_slider values measured by Qualisys
	cMeasures->able_xs_slider.push_back(cDyn->axis4_mod.x_slider);
	// Store FT measures
	if (rtValues->use_FT && (ableInfos->ctrl_ABLE->aOrders.ctrl_type == TORQUE_CTRL ||
		ableInfos->ctrl_ABLE->aOrders.ctrl_type == MINJERK_TRAJS))
	{
		// Record arm sensor measures
		cMeasures->fx_FTA_sensor.push_back(rtFTmeas_Arm->f_x);
		cMeasures->fy_FTA_sensor.push_back(rtFTmeas_Arm->f_y);
		cMeasures->fz_FTA_sensor.push_back(rtFTmeas_Arm->f_z);
		cMeasures->tx_FTA_sensor.push_back(rtFTmeas_Arm->t_x);
		cMeasures->ty_FTA_sensor.push_back(rtFTmeas_Arm->t_y);
		cMeasures->tz_FTA_sensor.push_back(rtFTmeas_Arm->t_z);
		// Record wrist sensor measures
		cMeasures->fx_FTW_sensor.push_back(rtFTmeas_Wrist->f_x);
		cMeasures->fy_FTW_sensor.push_back(rtFTmeas_Wrist->f_y);
		cMeasures->fz_FTW_sensor.push_back(rtFTmeas_Wrist->f_z);
		cMeasures->tx_FTW_sensor.push_back(rtFTmeas_Wrist->t_x);
		cMeasures->ty_FTW_sensor.push_back(rtFTmeas_Wrist->t_y);
		cMeasures->tz_FTW_sensor.push_back(rtFTmeas_Wrist->t_z);
	}
}

/*---------------------------------------------------------------------------------------------------------------------
| recordCurrentValues - Print current data to allow killing the robot code when experimentation finished
|
| Syntax --
|	void recordCurrentValues(ThreadInformations* ableInfos)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
----------------------------------------------------------------------------------------------------------------------*/
void recordCurrentValues(ThreadInformations* ableInfos)
{
	// Extract substructs
	ableOrders* oValues = &ableInfos->ctrl_ABLE->aOrders;
	realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;
	ableMeasures* measValues = &ableInfos->ctrl_ABLE->aMeasures;
	if (rtValues->order_counter > 0 || oValues->ctrl_type == MINJERK_TRAJS)
	{
		// Record currents values
		fprintf(ableInfos->currents_file, "%f ; ", measValues->able_currents_1.at(measValues->able_currents_1.size() - 1));
		fprintf(ableInfos->currents_file, "%f ; ", measValues->able_currents_2.at(measValues->able_currents_2.size() - 1));
		fprintf(ableInfos->currents_file, "%f ; ", measValues->able_currents_3.at(measValues->able_currents_3.size() - 1));
		fprintf(ableInfos->currents_file, "%f ; ", measValues->able_currents_4.at(measValues->able_currents_4.size() - 1));
		// Record articular positions values
		fprintf(ableInfos->artpos_file, "%f ; ", measValues->able_artpos_1.at(measValues->able_artpos_1.size() - 1));
		fprintf(ableInfos->artpos_file, "%f ; ", measValues->able_artpos_2.at(measValues->able_artpos_2.size() - 1));
		fprintf(ableInfos->artpos_file, "%f ; ", measValues->able_artpos_3.at(measValues->able_artpos_3.size() - 1));
		fprintf(ableInfos->artpos_file, "%f ; ", measValues->able_artpos_4.at(measValues->able_artpos_4.size() - 1));
		// Record speeds values
		fprintf(ableInfos->speeds_file, "%f ; ", measValues->able_speeds_1.at(measValues->able_speeds_1.size() - 1));
		fprintf(ableInfos->speeds_file, "%f ; ", measValues->able_speeds_2.at(measValues->able_speeds_2.size() - 1));
		fprintf(ableInfos->speeds_file, "%f ; ", measValues->able_speeds_3.at(measValues->able_speeds_3.size() - 1));
		fprintf(ableInfos->speeds_file, "%f ; ", measValues->able_speeds_4.at(measValues->able_speeds_4.size() - 1));
		// Record x_slider values
		fprintf(ableInfos->xs_slider_file, "%f ; ", measValues->able_xs_slider.at(measValues->able_xs_slider.size() - 1));
		// Record FT Sensor
		if (ableInfos->ctrl_ABLE->rtParams.use_FT) {
			// Record Arm measures
			fprintf(ableInfos->ft_Arm_sensor_file, "%f;", measValues->fx_FTA_sensor.at(measValues->fx_FTA_sensor.size() - 1));
			fprintf(ableInfos->ft_Arm_sensor_file, "%f;", measValues->fy_FTA_sensor.at(measValues->fy_FTA_sensor.size() - 1));
			fprintf(ableInfos->ft_Arm_sensor_file, "%f;", measValues->fz_FTA_sensor.at(measValues->fz_FTA_sensor.size() - 1));
			fprintf(ableInfos->ft_Arm_sensor_file, "%f;", measValues->tx_FTA_sensor.at(measValues->tx_FTA_sensor.size() - 1));
			fprintf(ableInfos->ft_Arm_sensor_file, "%f;", measValues->ty_FTA_sensor.at(measValues->ty_FTA_sensor.size() - 1));
			fprintf(ableInfos->ft_Arm_sensor_file, "%f;", measValues->tz_FTA_sensor.at(measValues->tz_FTA_sensor.size() - 1));
			// Record Wrist measures
			fprintf(ableInfos->ft_Wrist_sensor_file, "%f;", measValues->fx_FTW_sensor.at(measValues->fx_FTW_sensor.size() - 1));
			fprintf(ableInfos->ft_Wrist_sensor_file, "%f;", measValues->fy_FTW_sensor.at(measValues->fy_FTW_sensor.size() - 1));
			fprintf(ableInfos->ft_Wrist_sensor_file, "%f;", measValues->fz_FTW_sensor.at(measValues->fz_FTW_sensor.size() - 1));
			fprintf(ableInfos->ft_Wrist_sensor_file, "%f;", measValues->tx_FTW_sensor.at(measValues->tx_FTW_sensor.size() - 1));
			fprintf(ableInfos->ft_Wrist_sensor_file, "%f;", measValues->ty_FTW_sensor.at(measValues->ty_FTW_sensor.size() - 1));
			fprintf(ableInfos->ft_Wrist_sensor_file, "%f;", measValues->tz_FTW_sensor.at(measValues->tz_FTW_sensor.size() - 1));
		}
	}
	// Print times in output file
	if (measValues->able_currents_1.size() > 1)
	{
		fprintf(ableInfos->times_file, "%f;", measValues->execution_times.at(measValues->execution_times.size() - 1));
	}
	else
	{
		fprintf(ableInfos->times_file, "%f;", 0.0f);
	}
	// Flush files
	fflush(ableInfos->currents_file);
	fflush(ableInfos->artpos_file);
	fflush(ableInfos->speeds_file);
	fflush(ableInfos->xs_slider_file);
	fflush(ableInfos->ft_Arm_sensor_file);
	fflush(ableInfos->ft_Wrist_sensor_file);
	//fflush(ableInfos->times_file);
}

/*---------------------------------------------------------------------------------------------------------------------
| recordValuesInFile - Print positions and currents final values for the order that has just been executed
|
| Syntax --
|	void recordValuesInFile(ThreadInformations* ableInfos)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
----------------------------------------------------------------------------------------------------------------------*/
void recordValuesInFile(ThreadInformations* ableInfos)
{
	// Extract substructs
	ableOrders* oValues = &ableInfos->ctrl_ABLE->aOrders;
	realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;
	ableMeasures* measValues = &ableInfos->ctrl_ABLE->aMeasures;

	if (oValues->ctrl_type != STATIC_IDENT)
	{
		able_SetPowerOff(ableInfos->eth_ABLE, ableInfos->ctrl_ABLE);
		fprintf(ableInfos->out_file, "Number of saved currents : %i\n", measValues->able_currents_1.size());
		fprintf(ableInfos->out_file, "Number of saved positions : %i\n", measValues->able_artpos_1.size());
		fprintf(ableInfos->out_file, "Number of saved speeds : %i\n", measValues->able_speeds_1.size());
		fflush(ableInfos->out_file);
	}

	// Print block header
	fprintf(ableInfos->identification_file, "Final position at identification order : %i\n", rtValues->order_counter);

	// Write robot positions
	for (int i(0); i < NB_MOTORS; i++)
	{
		fprintf(ableInfos->identification_file, "Axis %i : %f\n", i, rtValues->currentPosition[i]);
	}
	// Write robot currents
	if (oValues->ctrl_type == STATIC_IDENT)
	{
		// If static identification : get 1000 last current values (1 second)
		for (int i(measValues->able_currents_1.size() - 2000); i < (int)(measValues->able_currents_1.size() - 1000); i++)
		{
			fprintf(ableInfos->currents_file, "%f ; ", measValues->able_currents_1.at(i));
			fprintf(ableInfos->currents_file, "%f ; ", measValues->able_currents_2.at(i));
			fprintf(ableInfos->currents_file, "%f ; ", measValues->able_currents_3.at(i));
			fprintf(ableInfos->currents_file, "%f ; ", measValues->able_currents_4.at(i));
		}
	}
	else if (rtValues->order_counter == 255)
	{
		// If any other control : get all currents and speeds
		for (int i(0); i < (int)(measValues->able_currents_1.size()); i++)
		{
			// Record currents values
			fprintf(ableInfos->currents_file, "%f ; ", measValues->able_currents_1.at(i));
			fprintf(ableInfos->currents_file, "%f ; ", measValues->able_currents_2.at(i));
			fprintf(ableInfos->currents_file, "%f ; ", measValues->able_currents_3.at(i));
			fprintf(ableInfos->currents_file, "%f ; ", measValues->able_currents_4.at(i));
			// Record articular positions values
			fprintf(ableInfos->artpos_file, "%f ; ", measValues->able_artpos_1.at(i));
			fprintf(ableInfos->artpos_file, "%f ; ", measValues->able_artpos_2.at(i));
			fprintf(ableInfos->artpos_file, "%f ; ", measValues->able_artpos_3.at(i));
			fprintf(ableInfos->artpos_file, "%f ; ", measValues->able_artpos_4.at(i));
			// Record speeds values
			fprintf(ableInfos->speeds_file, "%f ; ", measValues->able_speeds_1.at(i));
			fprintf(ableInfos->speeds_file, "%f ; ", measValues->able_speeds_2.at(i));
			fprintf(ableInfos->speeds_file, "%f ; ", measValues->able_speeds_3.at(i));
			fprintf(ableInfos->speeds_file, "%f ; ", measValues->able_speeds_4.at(i));
			// Record x_slider values
			fprintf(ableInfos->xs_slider_file, "%f ; ", measValues->able_xs_slider.at(i));
			// recprd Fz values for human limb identification
			if (oValues->ctrl_type == HDYN_IDENT)
			{
				fprintf(ableInfos->fz_Arm_file, "%f ; ", measValues->fz_FTA_sensor.at(i));
				fprintf(ableInfos->fz_Wrist_file, "%f ; ", measValues->fz_FTW_sensor.at(i));
			}
		}
		if (rtValues->use_FT && ableInfos->ctrl_ABLE->aOrders.ctrl_type == TORQUE_CTRL)
		{
			for (int i(0); i < (int)(measValues->fx_FTA_sensor.size()); i++)
			{
				// Record arm sensor measures
				fprintf(ableInfos->ft_Arm_sensor_file, "%f;", measValues->fx_FTA_sensor.at(i));
				fprintf(ableInfos->ft_Arm_sensor_file, "%f;", measValues->fy_FTA_sensor.at(i));
				fprintf(ableInfos->ft_Arm_sensor_file, "%f;", measValues->fz_FTA_sensor.at(i));
				fprintf(ableInfos->ft_Arm_sensor_file, "%f;", measValues->tx_FTA_sensor.at(i));
				fprintf(ableInfos->ft_Arm_sensor_file, "%f;", measValues->ty_FTA_sensor.at(i));
				fprintf(ableInfos->ft_Arm_sensor_file, "%f;", measValues->tz_FTA_sensor.at(i));
				// Record wrist sensor measures
				fprintf(ableInfos->ft_Wrist_sensor_file, "%f;", measValues->fx_FTW_sensor.at(i));
				fprintf(ableInfos->ft_Wrist_sensor_file, "%f;", measValues->fy_FTW_sensor.at(i));
				fprintf(ableInfos->ft_Wrist_sensor_file, "%f;", measValues->fz_FTW_sensor.at(i));
				fprintf(ableInfos->ft_Wrist_sensor_file, "%f;", measValues->tx_FTW_sensor.at(i));
				fprintf(ableInfos->ft_Wrist_sensor_file, "%f;", measValues->ty_FTW_sensor.at(i));
				fprintf(ableInfos->ft_Wrist_sensor_file, "%f;", measValues->tz_FTW_sensor.at(i));
			}
		}
	}
	// Clear currents vectors
	measValues->able_currents_1.clear();
	measValues->able_currents_2.clear();
	measValues->able_currents_3.clear();
	measValues->able_currents_4.clear();
	// Clear positions vectors
	if (oValues->ctrl_type != HDYN_IDENT)
	{
		measValues->able_artpos_1.clear();
		measValues->able_artpos_2.clear();
		measValues->able_artpos_3.clear();
		measValues->able_artpos_4.clear();
	}
	// Clear speeds vectors
	measValues->able_speeds_1.clear();
	measValues->able_speeds_2.clear();
	measValues->able_speeds_3.clear();
	measValues->able_speeds_4.clear();
	// Clear FT vectors
	if (rtValues->use_FT && ableInfos->ctrl_ABLE->aOrders.ctrl_type == TORQUE_CTRL)
	{
		// Clear arm vectors
		measValues->fx_FTA_sensor.clear();
		measValues->fy_FTA_sensor.clear();
		measValues->fz_FTA_sensor.clear();
		measValues->tx_FTA_sensor.clear();
		measValues->ty_FTA_sensor.clear();
		measValues->tz_FTA_sensor.clear();
		// Clear wrist vectors
		measValues->fx_FTW_sensor.clear();
		measValues->fy_FTW_sensor.clear();
		measValues->fz_FTW_sensor.clear();
		measValues->tx_FTW_sensor.clear();
		measValues->ty_FTW_sensor.clear();
		measValues->tz_FTW_sensor.clear();
	}
}