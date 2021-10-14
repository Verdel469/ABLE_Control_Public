/***********************************************************************************************************************
* motors_type_params.h -
*
* Copyright (C) 2010, HAPTION
* Author : Paul de Poix
* Creation date : 07/2013
* Modified by : Dorian Verdel
* Modification  date : 02/2019
*
* Description :
* Manages the definition of the Motor_Type struct to store motors'
* parameters.
***********************************************************************************************************************/

#pragma once

#ifndef MOTORS_TYPE_PARAMS_H
#define MOTORS_TYPE_PARAMS_H
#ifdef __cplusplus 
extern "C" {
#endif

	struct Motor_Type
	{
		// Gain Bp -> proportionnal gain of the PI corrector of the position loop
		double Kp_P;
		// Gain Bp -> integral gain of the PI corrector of the position loop
		double Ki_P;
		// Gain Kv -> constructor proportionnal gain of the speed loop
		double Kp_V;
		// Gain Kpv -> adjusted proportionnal gain of the speed loop
		double Kp_V_r;
		// Integral gain of the speed loop
		double Ki_V;
		// Gain Kp -> proportionnal gain of the PI corrector of the current loop
		double Kp_I;
		// Gain Kp -> integral gain of the PI corrector of the current loop
		double Ki_I;
		// Coefficient of conversion measured_current[i] = K_convI[i] * (ADC_courant[i] - ADC_Offset[i])
		double Kconv_I;
		// Force / Current gain
		double Kt;
	};

	void init_Motor_Type(Motor_Type* RE40);

#ifdef __cplusplus 
}
#endif
Motor_Type create_Motor();
#endif	/* _ETH_H_ */