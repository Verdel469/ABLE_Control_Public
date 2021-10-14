/***********************************************************************************************************************
* limb_identification.h -
*
* Author : Dorian Verdel
* Creation  date : 02/2019
*
* Description :
* File containing the functions achieving the identification of the human limb.
***********************************************************************************************************************/

#pragma once

#ifndef LIMB_IDENTIFICATION_H
#define LIMB_IDENTIFICATION_H

// Project includes
#include "control_struct.h"
#include "handle_communication.h"

// Functions declaration
// Main function of limb identification
void limbIdentification_Main(ThreadInformations* ableInfos);
// Compute currents induced by the robot during the procedure
void compute_ModelCurrents(ableDynamics* identDyn, ableMeasures* measValues, std::vector<float>* a_currents4_model);
// Compute currents induced by the presence of the human limb
void compute_HumanCurrents(ableMeasures* measValues, std::vector<float>* a_currents4_model,
	                       std::vector<float>* a_currents4_human);
// Estimate human limb mass based on human induced currents
void compute_HumanMass(ThreadInformations* ableInfos, std::vector<float>* a_currents4_human);
// Estimate human limb mass based on human induced interaction forces
void compute_HumanMass_FT(ThreadInformations* ableInfos);
// Store identified dynamic parameters in a text file for further use
void store_IdentifiedHDyn(ThreadInformations* ableInfos);
// Extract identified dynamic parameters of the human limb
void retrieve_IdentifiedDyn(AbleControlStruct* ctrl_ABLE);
#endif // !LIMB_IDENTIFICATION_H