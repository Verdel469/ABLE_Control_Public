/***********************************************************************************************************************
* set_ABLEParameters.cpp -
*
* Author : Dorian Verdel
* Creation  date : 03/2021
*
* Description :
* Sets all useful parameters.
***********************************************************************************************************************/

#include "set_ABLEParameters.h"


// ------------------------------------------------- CENTRALISED FUNCTION ----------------------------------------------

/*----------------------------------------------------------------------------------------------------------------------
| able_SetAllParams - Set all useful parameters from a centralised function
|
| Syntax --
|	void able_SetAllParams(AbleControlStruct* ctrl_ABLE)
|
| Inputs --
|	AbleControlStruct* ctrl_ABLE -> Struct containing all ABLE informations
----------------------------------------------------------------------------------------------------------------------*/
void able_SetAllParams(AbleControlStruct* ctrl_ABLE)
{
	// Set motor parameters
	able_SetMotorParams(ctrl_ABLE);
	// Set axis reduction values
	able_SetReductionsValues(ctrl_ABLE);
	// Set articular motion ranges
	able_SetMotionRanges(ctrl_ABLE);
	// Set dynamic model previously identified
	set_IdentifiedDynamics(ctrl_ABLE);
}

// --------------------------------------------------- SETTING FUNCTIONS -----------------------------------------------

/*----------------------------------------------------------------------------------------------------------------------
| able_SetMotorParams - Set motors parameters
|
| Syntax --
|	void able_SetMotorParams(AbleControlStruct* ctrl_ABLE)
|
| Inputs --
|	AbleControlStruct* ctrl_ABLE -> Struct containing all ABLE informations
----------------------------------------------------------------------------------------------------------------------*/
void able_SetMotorParams(AbleControlStruct* ctrl_ABLE)
{
	// Create motor object
	Motor_Type motor_RE40 = create_Motor();
	motorsParams* mValues = &ctrl_ABLE->mParams;
	// Initialize values of the motors and coders in control struct
	for (int i(0); i < NB_MOTORS; ++i)
	{
		mValues->Kp_P[i] = motor_RE40.Kp_P;
		mValues->Ki_P[i] = motor_RE40.Ki_P;
		mValues->Kp_V[i] = motor_RE40.Kp_V;
		mValues->Kp_V_r[i] = motor_RE40.Kp_V_r;
		mValues->Ki_V[i] = motor_RE40.Ki_V;
		mValues->Kp_I[i] = motor_RE40.Kp_I;
		mValues->Ki_I[i] = motor_RE40.Ki_I;
		mValues->Kconv_I[i] = motor_RE40.Kconv_I;
		mValues->kt_gain = static_cast<float>(motor_RE40.Kt);
		mValues->able_NbPointsCoders[i] = NB_POINTS_CODERS;
	}
}

/*----------------------------------------------------------------------------------------------------------------------
| able_SetReductionsValues - Set the reduction value for each axis
|
| Syntax --
|	void able_SetReductionsValues(AbleControlStruct *ctrl_ABLE)
|
| Inputs --
|	AbleControlStruct *ctrl_ABLE -> pointer towards control of ABLE struct
----------------------------------------------------------------------------------------------------------------------*/
void able_SetReductionsValues(AbleControlStruct* ctrl_ABLE)
{
	// Extract motor substruct
	motorsParams* mValues = &ctrl_ABLE->mParams;
	// Values of axis reduction givven by contructor
	mValues->able_AxisReductions[0] = 69.9f;
	mValues->able_AxisReductions[1] = 69.6087f;
	mValues->able_AxisReductions[2] = 70.6858f;
	mValues->able_AxisReductions[3] = 70.6858f;
}

/*----------------------------------------------------------------------------------------------------------------------
| able_SetMotionRanges - Set the identified motion range of each axis of ABLE for identification and avoiding out of
|                        range orders that could break motors (amplitudes slightly reduced compared to real capacity)
|
| Syntax --
|	void able_SetMotionRanges(AbleControlStruct *ctrl_ABLE)
|
| Inputs --
|	AbleControlStruct *ctrl_ABLE -> pointer towards control of ABLE struct
----------------------------------------------------------------------------------------------------------------------*/
void able_SetMotionRanges(AbleControlStruct* ctrl_ABLE)
{
	// Extract motor substruct
	motorsParams* mValues = &ctrl_ABLE->mParams;
	// Set first axis motion range
	mValues->able_ArtMotionsRanges[0][0] = -0.23f;
	mValues->able_ArtMotionsRanges[1][0] = 1.52f;
	// Set second axis motion range
	mValues->able_ArtMotionsRanges[0][1] = -0.92f;
	mValues->able_ArtMotionsRanges[1][1] = 1.0f;
	// Set third axis motion range
	mValues->able_ArtMotionsRanges[0][2] = -2.25f;
	mValues->able_ArtMotionsRanges[1][2] = 0.16f;
	// Set fourth axis motion range
	mValues->able_ArtMotionsRanges[0][3] = -0.69f;
	mValues->able_ArtMotionsRanges[1][3] = 1.50f;
}

/*----------------------------------------------------------------------------------------------------------------------
| set_IdentifiedDynamics - Set the values of the identified dynamic model
|
| Syntax --
|	void set_IdentifiedDynamics(FILE* out_file)
----------------------------------------------------------------------------------------------------------------------*/
void set_IdentifiedDynamics(AbleControlStruct* ctrl_ABLE)
{
	// Extract dynamic parameters substructs
	ableDynamics* aDyns = &ctrl_ABLE->aDynamics;
	able_Axis3_model* ax3_mod = &aDyns->axis3_mod;
	able_Axis4_model* ax4_mod = &aDyns->axis4_mod;
	frictions* fmds3 = &aDyns->axis3_mod.frictions3;
	frictions* fmds4 = &aDyns->axis4_mod.frictions4;

	// Identified coefficients for splited data
	// Identified friction and adherence values of axis 3
	fmds3->adhfric = 0.0f;
	fmds3->dry_frics[0] = 0.0f;
	fmds3->dry_frics[1] = fmds3->dry_frics[0];
	fmds3->visc_frics[0] = 0.122490703f;
	fmds3->visc_frics[1] = fmds3->visc_frics[0];
	// Identified friction and adherence values of axis 4
	fmds4->adhfric = 0.05f;
	fmds4->dry_frics[0] = 0.0f;
	fmds4->dry_frics[1] = fmds4->dry_frics[0];
	fmds4->visc_frics[0] = 0.036568512f;
	fmds4->visc_frics[1] = fmds4->visc_frics[0];
	// Identified gravity models of axis 3
	ax3_mod->length3 = 0.30f;
	ax3_mod->gm_stat[0] = 3.8f * 0.14f;
	ax3_mod->gm_stat[1] = 3.8f * 0.01f;
	ax3_mod->gm_bot[0] = ax3_mod->gm_stat[0];
	ax3_mod->gm_bot[1] = ax3_mod->gm_stat[1];
	ax3_mod->gm_top[0] = ax3_mod->gm_stat[0];
	ax3_mod->gm_top[1] = ax3_mod->gm_stat[1];
	// Identified gravity models of axis 3
	ax4_mod->mass4 = 1.541f;
	ax4_mod->cm_stat[0] = 0.0f;
	ax4_mod->cm_stat[1] = -1.066434f;
	ax4_mod->cm_stat[2] = -0.323876f;
	ax4_mod->cm_bot[0] = ax4_mod->cm_stat[0];
	ax4_mod->cm_bot[1] = ax4_mod->cm_stat[1];
	ax4_mod->cm_bot[2] = ax4_mod->cm_stat[2];
	ax4_mod->cm_top[0] = ax4_mod->cm_stat[0];
	ax4_mod->cm_top[1] = ax4_mod->cm_stat[1];
	ax4_mod->cm_top[2] = ax4_mod->cm_stat[2];
	// Keep old values
	/*ax4_mod->cm_stat[0] = 0.0026141f;
	ax4_mod->cm_stat[1] = -0.251324f;
	ax4_mod->cm_stat[2] = -0.329629f;
	ax4_mod->cm_bot[0] = 0.0019297f;
	ax4_mod->cm_bot[1] = -0.27816f;
	ax4_mod->cm_bot[2] = -0.29322f;
	ax4_mod->cm_top[0] = 0.0022253f;
	ax4_mod->cm_top[1] = -0.63042f;
	ax4_mod->cm_top[2] = -0.33312f;*/
	// Identified inertia model for different speeds (Inertia not compensated while no intention detection)
	//imds->inertia_model[0] = 0.0045f;
	//imds->inertia_model[1] = -0.0064f;
	//imds->inertia_model[2] = 0.0041f;
}