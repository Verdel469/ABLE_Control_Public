/***********************************************************************************************************************
* compute_orders.h -
*
* Author : Dorian Verdel
* Creation  date : 01/2020
*
* Description :
* Computes orders to send to ABLE
***********************************************************************************************************************/

#ifndef COMPUTE_ORDERS_H
#define COMPUTE_ORDERS_H

// General includes
#include <iostream>
#include <thread>
#include <errno.h>					// Standard errors librairy to get sockets outputs
#include <stdio.h>

// Project includes

#include "control_struct.h"			// Header containing the full declaration of the control struct
#include "handle_communication.h"	// Header of the code containing the functions to communicate with ABLE

// Initialisation of orders function
void initializationOrdersComputation(FILE* err_file, FILE* out_file, AbleControlStruct* ctrl_ABLE);
// Set orders for identification phases 1, 2, 3 and dynamics into control struct
void setIdentificationOrders(FILE* err_file, FILE* out_file, AbleControlStruct* ctrl_ABLE);
// Set starting position orders according to identification phase
void setGoHomeOrders(FILE* err_file, FILE* out_file, AbleControlStruct* ctrl_ABLE);
// Print initial orders
void printInitialOrders(FILE* err_file, FILE* out_file, AbleControlStruct* ctrl_ABLE);
// Compute the targets for static identification
void targetsComputationGeomID_1DoF(AbleControlStruct* ctrl_ABLE);
// Compute positions for dynamic identification
void computeDynIdentOrders(FILE* err_file, FILE* out_file, AbleControlStruct* ctrl_ABLE);
void generateMotorCombinations(FILE* out_file, AbleControlStruct* ctrl_ABLE);
void fillDynamicOrdersTable(FILE* out_file, AbleControlStruct* ctrl_ABLE);
// Extract pre-computed minimum jerk trajectories
void extractJerkPositions(FILE* err_file, AbleControlStruct* ctrl_ABLE);
// Duplicate all pre-computed minimum jerk trajectories for control
void duplicateJerkPositions(AbleControlStruct* ctrl_ABLE);
#endif // !LOW_LEVEL_COMMAND_1DOF_MAIN_H