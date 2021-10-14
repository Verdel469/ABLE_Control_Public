/***********************************************************************************************************************
* able_OrdersManagement.cpp -
*
* Author : Dorian Verdel
* Creation  date : 03/2021
*
* Description :
* Manages orders to be sent to ABLE.
***********************************************************************************************************************/

#include "able_OrdersManagement.h"

// ------------------------------------------------ UPDATE ORDERS FUNCTIONS --------------------------------------------

/*---------------------------------------------------------------------------------------------------------------------
| able_UpdateOrders - Updates the contents of the control struct and the orders that will be sent during next iteration
|
| Syntax --
|	void able_UpdateOrder(ThreadInformations* ableInfos)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
|   int counter -> index of the order being executed
----------------------------------------------------------------------------------------------------------------------*/
void able_UpdateOrders(ThreadInformations* ableInfos)
{
	// Substructs extraction
	ableOrders* oValues = &ableInfos->ctrl_ABLE->aOrders;
	realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;

	// Change next iteration position order if order done
	if (rtValues->order_counter != 0 && oValues->ctrl_type != MINJERK_TRAJS)
	{
		able_UpdateOrderIdent(ableInfos);
	}
	else if (oValues->ctrl_type == MINJERK_TRAJS)
	{
		able_UpdateOrderMinJerk(ableInfos);
	}
	able_SelectAdaptedControl(ableInfos);
}

/*---------------------------------------------------------------------------------------------------------------------
| able_UpdateOrderIdent - Change identification order
|
| Syntax --
|	void able_UpdateOrderIdent(ThreadInformations* ableInfos)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
----------------------------------------------------------------------------------------------------------------------*/
void able_UpdateOrderIdent(ThreadInformations* ableInfos)
{
	// Extract substructs
	motorsParams* mValues = &ableInfos->ctrl_ABLE->mParams;
	ableOrders* oValues = &ableInfos->ctrl_ABLE->aOrders;
	realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;

	// Initialise variables
	static bool first_call = TRUE;

	for (int i(0); i < NB_MOTORS; i++) {
		if (rtValues->order_counter == 1 && first_call == TRUE)
		{
			fprintf(ableInfos->out_file, "Home position reached ! Starting identification... \n");

			if (mValues->inhibition_State[i] == 0)
			{
				// Increase proportionnal gain of position control loop of activated axis
				mValues->Kp_P[i] = 50.0;
				// Set proportionnal gain of speed control loop if dynamica identification
				if (oValues->ctrl_type == DYN_IDENT)
				{
					rtValues->iter_id_start = rtValues->iter_counter;
					mValues->Kp_V_r[i] = 1 / mValues->Kp_V[i];
				}
			}
			first_call = FALSE;
		}
		if (mValues->inhibition_State[i] == 0)
		{
			oValues->positionOrder[i] = oValues->positionOrdersDoF[i][rtValues->order_counter - 1];
		}
	}
}

/*---------------------------------------------------------------------------------------------------------------------
| able_UpdateOrderMinJerk - Change minimum jerk order.
|
| Syntax --
|	void able_UpdateOrderMinJerk(ThreadInformations* ableInfos)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
----------------------------------------------------------------------------------------------------------------------*/
void able_UpdateOrderMinJerk(ThreadInformations* ableInfos)
{
	// Extract substructs
	motorsParams* mValues = &ableInfos->ctrl_ABLE->mParams;
	ableOrders* oValues = &ableInfos->ctrl_ABLE->aOrders;
	realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;
	minjerk_trajs* minJerk = &ableInfos->ctrl_ABLE->minJerk;

	// Compute current startpoint and endpoint of the minimum jerk trajectory
	if (!rtValues->jerkFamiliarisation && !rtValues->jerkTrajs_AllEnded) {
		// Normal block
		rtValues->current_startMinJerk = minJerk->all_Jmoves[rtValues->current_minJerkMove].at(0);
		rtValues->current_endMinJerk = minJerk->all_Jmoves[rtValues->current_minJerkMove]
			.at(minJerk->all_Jmoves[rtValues->current_minJerkMove].size() - 1);
	}
	else if (rtValues->jerkFamiliarisation && !rtValues->jerkTrajs_AllEnded) {
		// Familiarisation block
		rtValues->current_startMinJerk = minJerk->all_JmovesFam[rtValues->current_minJerkMove].at(0);
		rtValues->current_endMinJerk = minJerk->all_JmovesFam[rtValues->current_minJerkMove]
			.at(minJerk->all_JmovesFam[rtValues->current_minJerkMove].size() - 1);
	}

	// If goStart phase, go to current_startMinJerk, else, if min jerk move started, follow trajectory
	if (!rtValues->jerkMove_started && !rtValues->jerkTrajs_AllEnded)
	{
		oValues->positionOrder[3] = rtValues->current_startMinJerk;
	}
	else if (rtValues->jerkMove_started && !rtValues->jerkFamiliarisation && !rtValues->jerkTrajs_AllEnded)
	{
		oValues->positionOrder[3] = minJerk->all_Jmoves[rtValues->current_minJerkMove].at(rtValues->current_posMinJerk);
	}
	else if (rtValues->jerkMove_started && rtValues->jerkFamiliarisation && !rtValues->jerkTrajs_AllEnded)
	{
		oValues->positionOrder[3] = minJerk->all_JmovesFam[rtValues->current_minJerkMove].at(rtValues->current_posMinJerk);
	}
	else if (rtValues->jerkTrajs_AllEnded && rtValues->jerkBlockWithFatigueTest)
	{
		// Go to home position for fatigue test
		fprintf(ableInfos->out_file, "Going to home position after performing all minjerk trajs...\n");
		oValues->positionOrder[3] = 0.0f;
	}
	//fprintf(ableInfos->out_file, "Current start minjerk value computed %f\n", rtValues->current_startMinJerk);
	//fprintf(ableInfos->out_file, "Current end minjerk value computed %f\n", rtValues->current_endMinJerk);
	//fprintf(ableInfos->out_file, "Current value of go start move bool %i\n", rtValues->jerkMove_goStart);
	//fprintf(ableInfos->out_file, "Current value of start move reached bool %i\n", rtValues->jerkMove_startReached);
	//fprintf(ableInfos->out_file, "Current value of start move bool %i\n", rtValues->jerkMove_started);
	//fprintf(ableInfos->out_file, "Current axis 4 position order value %f\n", oValues->positionOrder[3]);
}

/*---------------------------------------------------------------------------------------------------------------------
| able_SelectAdaptedControl - Select relevant control mode
|
| Syntax --
|	void able_SelectAdaptedControl(ThreadInformations* ableInfos)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
----------------------------------------------------------------------------------------------------------------------*/
void able_SelectAdaptedControl(ThreadInformations* ableInfos)
{
	// Extract substructs
	ableOrders* oValues = &ableInfos->ctrl_ABLE->aOrders;
	realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;
	minjerk_trajs* minJerk = &ableInfos->ctrl_ABLE->minJerk;

	// Initialise variables
	int iter_order;

	// Initialisation and static identifications
	if (oValues->ctrl_type == STATIC_IDENT || oValues->ctrl_type == HDYN_IDENT ||
		(rtValues->order_counter < 1 && oValues->ctrl_type != MINJERK_TRAJS) ||
		oValues->ctrl_type == MINJERK_TRAJS && !rtValues->jerkHomePosAfterTrajs)
	{
		able_PositionAsserv(ableInfos);
	}
	else if (oValues->ctrl_type == DYN_IDENT)
	{
		// Speed control identifications
		iter_order = rtValues->iter_counter - rtValues->iter_id_start;
		able_DynIdentAsserv(ableInfos, iter_order);
	}
	else if (oValues->ctrl_type == TORQUE_CTRL || oValues->ctrl_type == MINJERK_TRAJS &&
		rtValues->jerkTrajs_AllEnded && rtValues->jerkHomePosAfterTrajs && rtValues->jerkBlockWithFatigueTest)
	{
		// Torque controls (transparent, antigravity, fatigue tests)
		able_TorqueAsserv(ableInfos);
	}
	else if (oValues->ctrl_type == OSCILLATOR_CTRL)
	{
		// Adaptative oscillators control (Hopf, ...)
		able_Hopf_PositionAsserv(ableInfos);
	}
}

// ------------------------------------------------- CHECK ORDERS FUNCTIONS --------------------------------------------

/*---------------------------------------------------------------------------------------------------------------------
| able_CheckOrderState - Check if ABLE has reached the target or ended the trajectory based on evolution of position
|                        during "able_CheckTargetReachedNbIt" iterations
|
| Syntax --
|	bool able_CheckOrderState(ThreadInformations* ableInfos)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
----------------------------------------------------------------------------------------------------------------------*/
bool able_CheckOrderState(ThreadInformations* ableInfos)
{
	// Variables declaration
	float pos_evolution;
	bool order_end = true;
	
	realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;

	if (ableInfos->ctrl_ABLE->aOrders.ctrl_type != MINJERK_TRAJS)
	{
		// Check displacements of each axis
		for (int i(0); i < NB_MOTORS; i++)
		{
			pos_evolution = abs(rtValues->currentPosition[i] - rtValues->oldPosition[i]);
			// End of order limit definition
			if (pos_evolution > 0.005)
			{
				order_end = false;
				return order_end;
			}
		}
		return order_end;
	}
	else
	{
		// Manage minimum jerk orders
		able_CheckOrderMinJerk(ableInfos);
		return FALSE;
	}
}

/*---------------------------------------------------------------------------------------------------------------------
| able_CheckOrderMinJerk - Check if order has been reached for minimum jerk control
|
| Syntax --
|	void able_CheckOrderMinJerk(ThreadInformations* ableInfos)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
----------------------------------------------------------------------------------------------------------------------*/
void able_CheckOrderMinJerk(ThreadInformations* ableInfos)
{
	// Extract substructs
	realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;
	minjerk_trajs* minJerk = &ableInfos->ctrl_ABLE->minJerk;

	// Initialise variables
	float dist_to_start, dist_to_end;
	static int counter_delayForce = 0;

	// Compute distance to move start and move end
	dist_to_start = abs(rtValues->currentPosition[NB_MOTORS - 1] - rtValues->current_startMinJerk);
	dist_to_end = abs(rtValues->currentPosition[NB_MOTORS - 1] - rtValues->current_endMinJerk);

	// Check state of movement
	if (!rtValues->jerkMove_started && dist_to_start < 0.01)
	{
		// Ready to start movement
		rtValues->jerkMove_startReached = TRUE;
	}
	else if (rtValues->jerkMove_started && dist_to_end < 0.01)
	{
		// Movement is near the end, re-initialise all parameters
		rtValues->jerkMove_started = FALSE;
		rtValues->jerkMove_startReached = FALSE;
		rtValues->timer_jerk_end_move = 0;
		rtValues->current_posMinJerk = 0;
		rtValues->current_minJerkMove++;
	}
	else if (rtValues->jerkMove_started && !rtValues->jerkFamiliarisation &&
		rtValues->current_posMinJerk < (int)(minJerk->all_Jmoves[rtValues->current_minJerkMove].size()) - 1)
	{
		// Update order during movement
		rtValues->current_posMinJerk++;
	}
	else if (rtValues->jerkMove_started && rtValues->jerkFamiliarisation &&
		rtValues->current_posMinJerk < (int)(minJerk->all_JmovesFam[rtValues->current_minJerkMove].size()) - 1)
	{
		// Update order during familiarisation block movement
		rtValues->current_posMinJerk++;
	}
	else if (!rtValues->jerkBlockWithFatigueTest && (rtValues->current_minJerkMove >= NB_REP * NB_JERK_TRAJS ||
			 rtValues->current_minJerkMove >= NB_REP_FAM * NB_JERK_TRAJS && rtValues->jerkFamiliarisation))
	{
		// Delay robot stop
		counter_delayForce++;
		if (counter_delayForce >= DELAYROBSTOP)
		{
			rtValues->robot_stopAfterTrajs = TRUE;
		}
	}
	else if (rtValues->jerkBlockWithFatigueTest && rtValues->jerkTrajs_AllEnded && abs(rtValues->currentPosition[3]) < 0.05)
	{
		// If fatigue block wanted, delay its beginning by 2 seconds once home position is reached
		counter_delayForce++;
		if (counter_delayForce >= DELAYFORCEFATIGUE)
		{
			rtValues->jerkHomePosAfterTrajs = TRUE;
		}
	}
}

// ---------------------------------------------- HANDLE END ORDERS FUNCTIONS ------------------------------------------

/*----------------------------------------------------------------------------------------------------------------------
| switch_OrderReached - Check if order reached according to identification phase
|
| Syntax --
|	int switch_OrderReached(ThreadInformations* ableInfos, int iter_counter)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
|   int iter_counter -> counter of iteration
----------------------------------------------------------------------------------------------------------------------*/
int switch_OrderReached(ThreadInformations* ableInfos)
{
	bool order_done;
	// Extract substructs
	realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;
	motorsParams* mValues = &ableInfos->ctrl_ABLE->mParams;
	ableOrders* oValues = &ableInfos->ctrl_ABLE->aOrders;

	//fprintf(ableInfos->out_file, "SWITCH OR\n");

	if (rtValues->iter_counter % rtValues->able_CheckTargetReachedNbIt == 0 &&
		(oValues->ctrl_type != TORQUE_CTRL || rtValues->order_counter == 0) || oValues->ctrl_type == MINJERK_TRAJS)
	{
		// Check if target position has been reached
		order_done = able_CheckOrderState(ableInfos);
		if (order_done)
		{
			order_done = false;
			// Write final data into identification files if the order is an identification order
			if (rtValues->order_counter > 0 && oValues->ctrl_type == STATIC_IDENT)
			{
				// Take execution time
				recordValuesInFile(ableInfos);
			}
			rtValues->order_counter++;
			// Exit flag associated with number of required measures
			if (rtValues->iter_counter > ableInfos->ctrl_ABLE->rtParams.nb_iterations_dyn_ident&& oValues->ctrl_type == DYN_IDENT)
			{
				rtValues->order_counter = 255;
				return 0;
			}
			else if (rtValues->order_counter > NB_MEASURES_GEOM_ID && oValues->ctrl_type == HDYN_IDENT)
			{
				rtValues->order_counter = 255;
				recordValuesInFile(ableInfos);
				return 0;
			}
			fprintf(ableInfos->out_file, "Order changed...\n");
		}
		else if (oValues->ctrl_type == MINJERK_TRAJS)
		{
			if (rtValues->robot_stopAfterTrajs && (rtValues->current_minJerkMove >= NB_REP * NB_JERK_TRAJS && !rtValues->jerkBlockWithFatigueTest ||
				rtValues->current_minJerkMove >= NB_REP_FAM * NB_JERK_TRAJS && rtValues->jerkFamiliarisation && !rtValues->jerkBlockWithFatigueTest))
			{
				rtValues->order_counter = 255;
				return 0;
			}
			else if ((rtValues->current_minJerkMove >= NB_REP * NB_JERK_TRAJS ||
				      rtValues->current_minJerkMove >= NB_REP_FAM * NB_JERK_TRAJS && rtValues->jerkFamiliarisation) &&
				      rtValues->jerkBlockWithFatigueTest)
			{
				rtValues->jerkTrajs_AllEnded = TRUE;
			}
		}
		// Store current position for further comparison
		for (int i(0); i < NB_MOTORS; i++)
		{
			rtValues->oldPosition[i] = rtValues->currentPosition[i];
		}
	}
	return 1;
}