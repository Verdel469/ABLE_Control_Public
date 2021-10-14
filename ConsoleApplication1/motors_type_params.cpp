/***********************************************************************************************************************
* motors_type_params.cpp -
*
* Copyright (C) 2010, HAPTION
* Author : Paul de Poix
* Creation date : 07/2013
* Modified by : Dorian Verdel
* Modification  date : 02/2019
*
* Description :
* Manages the initialization of Motor_Type struct with gains values. Gains used for all the diferent commands.
***********************************************************************************************************************/

// Includes
#include "motors_type_params.h"

/*---------------------------------------------------------------------------------------------------------------------
| init_Motor_Type - Initialization of the values of gains
|
| Syntax --
|	void init_Motor_Type(Motor_Type* RE40)
|
| Inputs --
|	Motor_Type* RE40 -> pointer towards motor struct
----------------------------------------------------------------------------------------------------------------------*/
void init_Motor_Type(Motor_Type* RE40)
{
	// Init RE40 (values given by constructor and adjusted with experiments)
	RE40->Kp_P = 125.0;
	RE40->Ki_P = 1.50;
	RE40->Kp_V = 0.5;
	RE40->Kp_V_r = 1.0;
	RE40->Ki_V = 0.05;
	RE40->Kp_I = 0.02;
	RE40->Ki_I = 0.1;
	RE40->Kconv_I = 0.0019073486328;	// Value given by constructor
	RE40->Kt = 0.1591549431;
}

/*---------------------------------------------------------------------------------------------------------------------
| create_Motor - Creation of a motor struct and call to the initialization function
|
| Syntax --
|	Motor_Type create_Motor()
|
| Outputs --
|	Motor_Type -> Copy of the initialized motor struct
----------------------------------------------------------------------------------------------------------------------*/
Motor_Type create_Motor()
{
	Motor_Type motor_RE40;
	init_Motor_Type(&motor_RE40);
	return motor_RE40;
}