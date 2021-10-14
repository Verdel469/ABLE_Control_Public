/***********************************************************************************************************************
* compute_orders.cpp -
*
* Author : Dorian Verdel
* Creation  date : 02/2019
*
* Description :
* Compute the identification orders and sets useful values.
***********************************************************************************************************************/

#include "compute_orders.h"
#include <string>

using namespace std;

/*---------------------------------------------------------------------------------------------------------------------
| initializationOrdersComputation - Initialize orders according to command mode
|
| Syntax --
|	void initializationOrdersComputation(FILE* err_file, FILE* out_file, AbleControlStruct* ctrl_ABLE)
|
| Inputs --
|	FILE* err_file -> pointer towards "errors.txt" (stderr)
|	FILE* out_file -> pointer towards "outputs.txt" (stdout)
|   AbleControlStruct* ctrl_ABLE -> Struct containing all ABLE's informations
----------------------------------------------------------------------------------------------------------------------*/
void initializationOrdersComputation(FILE* err_file, FILE* out_file, AbleControlStruct* ctrl_ABLE)
{
	// Set starting position orders
	setGoHomeOrders(err_file, out_file, ctrl_ABLE);
	// Set control mode and order type
	ctrl_ABLE->aOrders.asservType = MODE_ASSERVISSEMENT_VITESSE;
	// Print initial orders
	printInitialOrders(err_file, out_file, ctrl_ABLE);
}

/*---------------------------------------------------------------------------------------------------------------------
| setIdentificationOrders - Set orders for identification phases 1, 2, 3 and dynamics into control struct
|
| Syntax --
|	void setIdentificationOrders(FILE* err_file, FILE* out_file, AbleControlStruct* ctrl_ABLE)
|
| Inputs --
|	FILE* err_file -> pointer towards "errors.txt" (stderr)
|	FILE* out_file -> pointer towards "outputs.txt" (stdout)
|   AbleControlStruct* ctrl_ABLE -> Struct containing all ABLE's informations
----------------------------------------------------------------------------------------------------------------------*/
void setIdentificationOrders(FILE* err_file, FILE* out_file, AbleControlStruct* ctrl_ABLE)
{
	// Variables declaration
	float omega_i;
	// Extract substructs
	realTimeParams* rtValues = &ctrl_ABLE->rtParams;
	motorsParams* mValues = &ctrl_ABLE->mParams;
	ableOrders* oValues = &ctrl_ABLE->aOrders;

	fprintf(out_file, "Current position of each axis :\n");
	if (oValues->ctrl_type == CSPEED_IDENT)
	{
		fprintf(err_file, "Outdated identification type. Use only static or dynamic.");
	}else if (oValues->ctrl_type == VCSPEED_IDENT)
	{
		fprintf(err_file, "Outdated identification type. Use only static or dynamic.");
	}else if (oValues->ctrl_type == DYN_IDENT)
	{
		for (int i(0); i < NB_MEASURES_DYNAMIC_ID; i++)
		{
			omega_i = 1.0f + 1.5f * (float)i / (float)NB_MEASURES_DYNAMIC_ID;
			oValues->omega.push_back(omega_i);
		}
		computeDynIdentOrders(err_file, out_file, ctrl_ABLE);
		oValues->orderType = TRAJECTORY_ORDER;
	}else if (oValues->ctrl_type == MINJERK_TRAJS)
	{
		// Initialize minimum jerk counters
		rtValues->current_minJerkMove = 0;
		rtValues->current_posMinJerk = 0;
		rtValues->jerkMove_startReached = FALSE;
		rtValues->jerkMove_goStart = FALSE;
		rtValues->jerkMove_started = FALSE;
		rtValues->jerkTrajs_AllEnded = FALSE;
		rtValues->jerkHomePosAfterTrajs = FALSE;
	}
}

/*---------------------------------------------------------------------------------------------------------------------
| setGoHomeOrders - Set starting position orders according to identification phase
|
| Syntax --
|	void setGoHomeOrders(FILE* err_file, FILE* out_file, AbleControlStruct* ctrl_ABLE)
|
| Inputs --
|	FILE* err_file -> pointer towards "errors.txt" (stderr)
|	FILE* out_file -> pointer towards "outputs.txt" (stdout)
|   AbleControlStruct* ctrl_ABLE -> Struct containing all ABLE's informations
----------------------------------------------------------------------------------------------------------------------*/
void setGoHomeOrders(FILE* err_file, FILE* out_file, AbleControlStruct* ctrl_ABLE)
{
	// Variables declaration
	float art_theta_i, pos_dif;
	// Extract substructs
	realTimeParams* rtValues = &ctrl_ABLE->rtParams;
	motorsParams* mValues = &ctrl_ABLE->mParams;
	ableOrders* oValues = &ctrl_ABLE->aOrders;

	// Set inhibition state and position order
	for (int i(0); i < NB_MOTORS; i++)
	{
		// Set articular position order (home)
		oValues->positionOrder[i] = 0.0f;
		// Compute current position and motion sign
		art_theta_i = able_ComputeCurrentPosition(rtValues->currentCoderPosition[i], mValues->able_AxisReductions[i]);

		oValues->positionCoderOrder[i] = oValues->positionOrder[i] * mValues->able_AxisReductions[i];
		// Update position and speed order
		rtValues->currentPosition[i] = art_theta_i;
		// Compute diference in articular position
		pos_dif = oValues->positionOrder[i] - art_theta_i;
		// Compute speed order based on position diference
		oValues->speedOrder[i] = (float)(mValues->Kp_P[i] * pos_dif);
		// Print current position in out file
		fprintf(out_file, "Theta %i : %f\n", i + 1, rtValues->currentPosition[i]);
	}
}

/*---------------------------------------------------------------------------------------------------------------------
| printInitialOrders - Print initial motion parameters into out file
|
| Syntax --
|	void printInitialOrders(FILE* err_file, FILE* out_file, AbleControlStruct* ctrl_ABLE)
|
| Inputs --
|	FILE* err_file -> pointer towards "errors.txt" (stderr)
|	FILE* out_file -> pointer towards "outputs.txt" (stdout)
|   AbleControlStruct* ctrl_ABLE -> Struct containing all ABLE's informations
----------------------------------------------------------------------------------------------------------------------*/
void printInitialOrders(FILE* err_file, FILE* out_file, AbleControlStruct* ctrl_ABLE)
{
	fprintf(out_file, "Control mode : MODE_ASSERVISSEMENT_VITESSE\n");
	if (ctrl_ABLE->aOrders.orderType == TRAJECTORY_ORDER)
	{
		fprintf(out_file, "Order type : TRAJECTORY_ORDER\n");
	}else
	{
		fprintf(out_file, "Order type : TARGET_ORDER\n");
	}
	for (int i(0); i < NB_MOTORS; i++)
	{
		fprintf(out_file, "Axis %i : Position order %f \t Speed order : %f\n", i + 1,
			ctrl_ABLE->aOrders.positionOrder[i],
			ctrl_ABLE->aOrders.speedOrder[i]);
	}
}

/*---------------------------------------------------------------------------------------------------------------------
| targetsComputationGeomID_1DoF - Compute successive targets for static identification
|
| Syntax --
|	void targetsComputationGeomID_1DoF(AbleControlStruct* ctrl_ABLE)
|
| Inputs --
|   AbleControlStruct* ctrl_ABLE -> Struct containing all ABLE's informations
----------------------------------------------------------------------------------------------------------------------*/
void targetsComputationGeomID_1DoF(AbleControlStruct* ctrl_ABLE)
{
	// Extract substructs
	motorsParams* mValues = &ctrl_ABLE->mParams;
	ableOrders* oValues = &ctrl_ABLE->aOrders;
	
	// Compute successive positions according to the identification phase for each activated axis
	for (int i(0); i < NB_MOTORS; i++)
	{
		if (ctrl_ABLE->mParams.inhibition_State[i] == 0)
		{
			// Compute amplitude of range of motion
			float amplitude = mValues->able_ArtMotionsRanges[1][i] - mValues->able_ArtMotionsRanges[0][i];

			if (oValues->ctrl_type == STATIC_IDENT)
			{
				// Compute "NB_MEASURES_GEOM_ID" positions equally spread over the articular range
				for (int j(0); j < NB_MEASURES_GEOM_ID; j++)
				{
					if (i == 2)
					{
						oValues->positionOrdersDoF[i][j] = mValues->able_ArtMotionsRanges[0][i]
							                             + j * 5.0f / 6.0f
							                             * amplitude / NB_MEASURES_GEOM_ID;
					} else if (i == 3)
					{
						oValues->positionOrdersDoF[i][j] = mValues->able_ArtMotionsRanges[1][i]
							                             - j * 5.0f / 6.0f
							                             * amplitude / NB_MEASURES_GEOM_ID;
					}
				}
			}
			else if (oValues->ctrl_type == HDYN_IDENT)
			{
				// Compute "NB_MEASURES_GEOM_ID" positions spread over the articular range
				for (int j(0); j < NB_MEASURES_GEOM_ID; j++)
				{
					oValues->positionOrdersDoF[i][j] = mValues->able_ArtMotionsRanges[1][i]
						                             - j * 3 / 4 * amplitude / NB_MEASURES_GEOM_ID;
				}
			}
		}
	}
}

/*---------------------------------------------------------------------------------------------------------------------
| computeDynIdentOrders - Compute successive position orders for dynamic identification
|
| Syntax --
|	void computeDynIdentOrders(FILE* err_file, FILE* out_file, AbleControlStruct* ctrl_ABLE)
|
| Inputs --
|	FILE* err_file -> pointer towards "errors.txt" (stderr)
|	FILE* out_file -> pointer towards "outputs.txt" (stdout)
|	AbleControlStruct* ctrl_ABLE -> Struct containing all ABLE's informations
----------------------------------------------------------------------------------------------------------------------*/
void computeDynIdentOrders(FILE* err_file, FILE* out_file, AbleControlStruct* ctrl_ABLE)
{
	// Extract substructs and initialise variables
	motorsParams* mValues = &ctrl_ABLE->mParams;
	ableOrders* oValues = &ctrl_ABLE->aOrders;
	realTimeParams* rtParams = &ctrl_ABLE->rtParams;

	// Define number of iterations
	if (mValues->nb_activated_motors == 1) {
		rtParams->nb_iterations_dyn_ident = NB_MEASURES_DYNAMIC_ID + 10000;
	} else if (mValues->nb_activated_motors == 2) {
		rtParams->nb_iterations_dyn_ident = NB_MEASURES_DYNAMIC_ID * COMBINATIONS_2MOTORS + 10000;
	} else if(mValues->nb_activated_motors == 3) {
		rtParams->nb_iterations_dyn_ident = NB_MEASURES_DYNAMIC_ID * COMBINATIONS_3MOTORS + 10000;
	} else if(mValues->nb_activated_motors == 4) {
		rtParams->nb_iterations_dyn_ident = NB_MEASURES_DYNAMIC_ID * COMBINATIONS_4MOTORS + 10000;
	}
	

	// Compute successive positions according to the identification phase for each activated axis
	for (int i(0); i < NB_MOTORS; i++)
	{
		if (ctrl_ABLE->mParams.inhibition_State[i] == 0)
		{
			// Compute amplitude of range of motion
			float mid_position = (mValues->able_ArtMotionsRanges[1][i] + mValues->able_ArtMotionsRanges[0][i]) / 2;
			oValues->amplitude = mValues->able_ArtMotionsRanges[1][i] - mValues->able_ArtMotionsRanges[0][i];

			for (int j(0); j < NB_MEASURES_DYNAMIC_ID; j++)
			{
				oValues->dynamicOrdersId[i][j] = mid_position
					                           + oValues->amplitude / 3
											   * sinf(oValues->omega.at(j) * 0.001f * ((float)j + (float)i * (float)j / NB_MEASURES_DYNAMIC_ID));
				//fprintf(out_file, "Axis %i order %f\n", i, oValues->dynamicOrdersId[i][j]);
			}
			mValues->activated_motors.push_back(i);
		}
	}

	// Generate the complete table with the successive activations of the motors
	generateMotorCombinations(out_file, ctrl_ABLE);

	// Fill the dynamic orders table
	fillDynamicOrdersTable(out_file, ctrl_ABLE);
}

/*---------------------------------------------------------------------------------------------------------------------
| generateMotorCombinations - Compute successive position orders for dynamic identification
|
| Syntax --
|	void generateMotorCombinations(FILE* err_file, FILE* out_file, AbleControlStruct* ctrl_ABLE)
|
| Inputs --
|	FILE* out_file -> pointer towards "outputs.txt" (stdout)
|	AbleControlStruct* ctrl_ABLE -> Struct containing all ABLE's informations
----------------------------------------------------------------------------------------------------------------------*/
void generateMotorCombinations(FILE* out_file, AbleControlStruct* ctrl_ABLE)
{
	// Extract substructs
	motorsParams* mValues = &ctrl_ABLE->mParams;

	if (mValues->nb_activated_motors == 0)
	{
		fprintf(out_file, "No motor activated.\n");
	}
	else if (mValues->nb_activated_motors == 1) {
		mValues->motors_combinations1.push_back(mValues->activated_motors.at(0));
	}
	else if (mValues->nb_activated_motors == 2) {
		// One motor
		mValues->motors_combinations2[0].push_back(mValues->activated_motors.at(0));
		mValues->motors_combinations2[1].push_back(mValues->activated_motors.at(1));
		// Two motors
		mValues->motors_combinations2[2].push_back(mValues->activated_motors.at(0));
		mValues->motors_combinations2[2].push_back(mValues->activated_motors.at(1));
	}
	else if (mValues->nb_activated_motors == 3) {
		// One motor
		mValues->motors_combinations3[0].push_back(mValues->activated_motors.at(0));
		mValues->motors_combinations3[1].push_back(mValues->activated_motors.at(1));
		mValues->motors_combinations3[2].push_back(mValues->activated_motors.at(2));
		// Two motors
		mValues->motors_combinations3[3].push_back(mValues->activated_motors.at(0));
		mValues->motors_combinations3[3].push_back(mValues->activated_motors.at(1));
		mValues->motors_combinations3[4].push_back(mValues->activated_motors.at(0));
		mValues->motors_combinations3[4].push_back(mValues->activated_motors.at(2));
		mValues->motors_combinations3[5].push_back(mValues->activated_motors.at(1));
		mValues->motors_combinations3[5].push_back(mValues->activated_motors.at(2));
		// Three motors
		mValues->motors_combinations3[6].push_back(mValues->activated_motors.at(0));
		mValues->motors_combinations3[6].push_back(mValues->activated_motors.at(1));
		mValues->motors_combinations3[6].push_back(mValues->activated_motors.at(2));
	}
	else if (mValues->nb_activated_motors == 4) {
		// One motor
		mValues->motors_combinations4[0].push_back(mValues->activated_motors.at(0));
		mValues->motors_combinations4[1].push_back(mValues->activated_motors.at(1));
		mValues->motors_combinations4[2].push_back(mValues->activated_motors.at(2));
		mValues->motors_combinations4[3].push_back(mValues->activated_motors.at(3));
		// Two motors
		mValues->motors_combinations4[4].push_back(mValues->activated_motors.at(0));
		mValues->motors_combinations4[4].push_back(mValues->activated_motors.at(1));
		mValues->motors_combinations4[5].push_back(mValues->activated_motors.at(0));
		mValues->motors_combinations4[5].push_back(mValues->activated_motors.at(2));
		mValues->motors_combinations4[6].push_back(mValues->activated_motors.at(0));
		mValues->motors_combinations4[6].push_back(mValues->activated_motors.at(3));
		mValues->motors_combinations4[7].push_back(mValues->activated_motors.at(1));
		mValues->motors_combinations4[7].push_back(mValues->activated_motors.at(2));
		mValues->motors_combinations4[8].push_back(mValues->activated_motors.at(1));
		mValues->motors_combinations4[8].push_back(mValues->activated_motors.at(3));
		mValues->motors_combinations4[9].push_back(mValues->activated_motors.at(2));
		mValues->motors_combinations4[9].push_back(mValues->activated_motors.at(3));
		// Three motors
		mValues->motors_combinations4[10].push_back(mValues->activated_motors.at(0));
		mValues->motors_combinations4[10].push_back(mValues->activated_motors.at(1));
		mValues->motors_combinations4[10].push_back(mValues->activated_motors.at(2));
		mValues->motors_combinations4[11].push_back(mValues->activated_motors.at(0));
		mValues->motors_combinations4[11].push_back(mValues->activated_motors.at(1));
		mValues->motors_combinations4[11].push_back(mValues->activated_motors.at(3));
		mValues->motors_combinations4[12].push_back(mValues->activated_motors.at(0));
		mValues->motors_combinations4[12].push_back(mValues->activated_motors.at(2));
		mValues->motors_combinations4[12].push_back(mValues->activated_motors.at(3));
		mValues->motors_combinations4[13].push_back(mValues->activated_motors.at(1));
		mValues->motors_combinations4[13].push_back(mValues->activated_motors.at(2));
		mValues->motors_combinations4[13].push_back(mValues->activated_motors.at(3));
		// Four motors
		mValues->motors_combinations4[14].push_back(mValues->activated_motors.at(0));
		mValues->motors_combinations4[14].push_back(mValues->activated_motors.at(1));
		mValues->motors_combinations4[14].push_back(mValues->activated_motors.at(2));
		mValues->motors_combinations4[14].push_back(mValues->activated_motors.at(3));
	}
	else {
		fprintf(out_file, "Too many motors in the input string.\n");
	}
}

/*---------------------------------------------------------------------------------------------------------------------
| fillDynamicOrdersTable - Compute successive position orders for dynamic identification
|
| Syntax --
|	void fillDynamicOrdersTable(FILE* err_file, FILE* out_file, AbleControlStruct* ctrl_ABLE)
|
| Inputs --
|	FILE* out_file -> pointer towards "outputs.txt" (stdout)
|	AbleControlStruct* ctrl_ABLE -> Struct containing all ABLE's informations
----------------------------------------------------------------------------------------------------------------------*/
void fillDynamicOrdersTable(FILE* out_file, AbleControlStruct* ctrl_ABLE)
{
	// Extract substructs
	motorsParams* mValues = &ctrl_ABLE->mParams;
	ableOrders* oValues = &ctrl_ABLE->aOrders;
	int motor_found, current_motor;

	if (mValues->nb_activated_motors == 0)
	{
		fprintf(out_file, "No motor activated.\n");
	}
	else if (mValues->nb_activated_motors == 1) {
		// Retrieve activated motor
		int activated_motor = mValues->activated_motors.at(0);
		// Fill orders
		for (int i(0); i < NB_MEASURES_DYNAMIC_ID; i++)
		{
			oValues->dynamicOrdersIdAll[activated_motor].push_back(oValues->dynamicOrdersId[activated_motor][i]);
		}
	}
	else if (mValues->nb_activated_motors == 2) {
		for (int i(0); i < COMBINATIONS_2MOTORS; i++)
		{
			// Select a motor
			for (int j(0); j < (int)mValues->activated_motors.size(); j++)
			{
				motor_found = 0;
				current_motor = mValues->activated_motors.at(j);
				// Search the motor in the combination table's case
				for (int k(0); k < (int)mValues->motors_combinations2[i].size(); k++)
				{
					if (current_motor == mValues->motors_combinations2[i].at(k))
					{
						// If motor found add sinusoidal orders
						motor_found = 1;
						for (int l(0); l < NB_MEASURES_DYNAMIC_ID; l++)
						{
							oValues->dynamicOrdersIdAll[current_motor].push_back(oValues->dynamicOrdersId[current_motor][l]);
							
						}
					}
				}
				if (motor_found == 0)
				{
					// If motor not activated during this combination, set orders to 0
					for (int l(0); l < NB_MEASURES_DYNAMIC_ID; l++)
					{
						oValues->dynamicOrdersIdAll[current_motor].push_back(0.0f);
					}
				}
				fprintf(out_file, "%i\n", (int)oValues->dynamicOrdersIdAll[current_motor].size());
			}
		}
	}
	else if (mValues->nb_activated_motors == 3) {
		for (int i(0); i < COMBINATIONS_3MOTORS; i++)
		{
			// Select a motor
			for (int j(0); j < (int)mValues->activated_motors.size(); j++)
			{
				motor_found = 0;
				current_motor = mValues->activated_motors.at(j);
				// Search the motor in the combination table's case
				for (int k(0); k < (int)mValues->motors_combinations3[i].size(); k++)
				{
					if (current_motor == mValues->motors_combinations3[i].at(k))
					{
						// If motor found add sinusoidal orders
						motor_found = 1;
						for (int l(0); l < NB_MEASURES_DYNAMIC_ID; l++)
						{
							oValues->dynamicOrdersIdAll[current_motor].push_back(oValues->dynamicOrdersId[current_motor][l]);
						}
					}
				}
				if (motor_found == 0)
				{
					// If motor not activated during this combination, set orders to 0
					for (int l(0); l < NB_MEASURES_DYNAMIC_ID; l++)
					{
						oValues->dynamicOrdersIdAll[current_motor].push_back(0.0f);
					}
				}
			}
		}
	}
	else if (mValues->nb_activated_motors == 4) {
		for (int i(0); i < COMBINATIONS_4MOTORS; i++)
		{
			// Select a motor
			for (int j(0); j < (int)mValues->activated_motors.size(); j++)
			{
				motor_found = 0;
				current_motor = mValues->activated_motors.at(j);
				// Search the motor in the combination table's case
				for (int k(0); k < (int)mValues->motors_combinations4[i].size(); k++)
				{
					if (current_motor == mValues->motors_combinations4[i].at(k))
					{
						// If motor found add sinusoidal orders
						motor_found = 1;
						for (int l(0); l < NB_MEASURES_DYNAMIC_ID; l++)
						{
							oValues->dynamicOrdersIdAll[current_motor].push_back(oValues->dynamicOrdersId[current_motor][l]);
						}
					}
				}
				if (motor_found == 0)
				{
					// If motor not activated during this combination, set orders to 0
					for (int l(0); l < NB_MEASURES_DYNAMIC_ID; l++)
					{
						oValues->dynamicOrdersIdAll[current_motor].push_back(0.0f);
					}
				}
			}
		}
	}
	else {
		fprintf(out_file, "Too many motors in the input string.\n");
	}

}

/*---------------------------------------------------------------------------------------------------------------------
| extractJerkPositions - Extract all pre-computed minimum jerk trajectories
|
| Syntax --
|	void extractJerkPositions(FILE* err_file, AbleControlStruct* ctrl_ABLE)
|
| Inputs --
|	FILE* err_file -> pointer towards "errors.txt" (stderr)
|	AbleControlStruct* ctrl_ABLE -> Struct containing all ABLE's informations
----------------------------------------------------------------------------------------------------------------------*/
void extractJerkPositions(FILE* err_file, AbleControlStruct* ctrl_ABLE)
{
	// Initialize parsing variables
	string line;
	string delimiter = ";";
	size_t position = 0, size;
	string token;
	float pos_i;
	int i = 0;
	// Extract data
	if (ctrl_ABLE->minJerk.minjerk_file.is_open())
	{
		while (getline(ctrl_ABLE->minJerk.minjerk_file, line))
		{
			while ((position = line.find(delimiter)) != string::npos)
			{
				token = line.substr(0, position);
				pos_i = stof(token, &size);
				line.erase(0, position + delimiter.length());
				ctrl_ABLE->minJerk.parsed_movements[i].push_back(pos_i);
			}
			i++;
		}
		duplicateJerkPositions(ctrl_ABLE);
		ctrl_ABLE->minJerk.minjerk_file.close();
		ctrl_ABLE->rtParams.robot_stopAfterTrajs = FALSE;
	}
	else
	{
		fprintf(err_file, "File containing minimum jerk trajectories not opened !\n");
	}
}

/*---------------------------------------------------------------------------------------------------------------------
| duplicateJerkPositions - Duplicate all pre-computed minimum jerk trajectories for control
|
| Syntax --
|	void duplicateJerkPositions(AbleControlStruct* ctrl_ABLE)
|
| Inputs --
|	AbleControlStruct* ctrl_ABLE -> Struct containing all ABLE's informations
----------------------------------------------------------------------------------------------------------------------*/
void duplicateJerkPositions(AbleControlStruct* ctrl_ABLE)
{
	// Extract substruct
	realTimeParams* rtValues = &ctrl_ABLE->rtParams;
	// Duplicate data
	for (int i(0); i < NB_JERK_TRAJS / 2; i++)
	{
		if (!rtValues->jerkFamiliarisation)
		{
			for (int j(2 * NB_REP * i); j < 2 * NB_REP * (i + 1); j++)
			{
				ctrl_ABLE->minJerk.all_Jmoves[j].resize(ctrl_ABLE->minJerk.parsed_movements[2 * i].size());
				if (j % 2 == 0)
				{
					copy(ctrl_ABLE->minJerk.parsed_movements[2 * i].begin(),
						ctrl_ABLE->minJerk.parsed_movements[2 * i].end(),
						ctrl_ABLE->minJerk.all_Jmoves[j].begin());
				}
				else {
					copy(ctrl_ABLE->minJerk.parsed_movements[2 * i + 1].begin(),
						ctrl_ABLE->minJerk.parsed_movements[2 * i + 1].end(),
						ctrl_ABLE->minJerk.all_Jmoves[j].begin());
				}
			}
		} else if (rtValues->jerkFamiliarisation) {
			for (int j(2 * NB_REP_FAM * i); j < 2 * NB_REP_FAM * (i + 1); j++)
			{
				ctrl_ABLE->minJerk.all_JmovesFam[j].resize(ctrl_ABLE->minJerk.parsed_movements[2 * i].size());
				if (j % 2 == 0)
				{
					copy(ctrl_ABLE->minJerk.parsed_movements[2 * i].begin(),
						ctrl_ABLE->minJerk.parsed_movements[2 * i].end(),
						ctrl_ABLE->minJerk.all_JmovesFam[j].begin());
				}
				else {
					copy(ctrl_ABLE->minJerk.parsed_movements[2 * i + 1].begin(),
						ctrl_ABLE->minJerk.parsed_movements[2 * i + 1].end(),
						ctrl_ABLE->minJerk.all_JmovesFam[j].begin());
				}
			}
		}
	}
}