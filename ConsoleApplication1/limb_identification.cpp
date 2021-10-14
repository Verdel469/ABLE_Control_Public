/***********************************************************************************************************************
* limb_identification.cpp -
*
* Author : Dorian Verdel
* Creation  date : 02/2020
*
* Description :
* File containing the identification of the human limb process.
***********************************************************************************************************************/

#include "limb_identification.h"

/*---------------------------------------------------------------------------------------------------------------------
| limbIdentification_Main - Start limb dynamic identification procedure
|
| Syntax --
|	void limbIdentification_Main(ThreadInformations* ableInfos)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
----------------------------------------------------------------------------------------------------------------------*/
void limbIdentification_Main(ThreadInformations* ableInfos)
{
	// Open storing file
	fopen_s(&ableInfos->ctrl_ABLE->hDynId.idDyn_File, "human_limb_identification.txt", "w");

	// Estimate human forearm 

	compute_HumanMass_FT(ableInfos);

	/* WILL BE USEFUL FOR FURTHER STEPS OF IDENTIFICATION */
	//// Initialize variables
	//std::vector<float> a_currents4_model;
	//std::vector<float> a_currents4_human;
	//// Extract measured values during the experiment and identified dynamics
	//ableMeasures* measValues = &ableInfos->ctrl_ABLE->aMeasures;
	//ableDynamics* identDyn = &ableInfos->ctrl_ABLE->aDynamics;
	//
	//// Compute theoretical currents without human limb
	//compute_ModelCurrents(identDyn, measValues, &a_currents4_model);
	//// Compute human induced currents
	//compute_HumanCurrents(measValues, &a_currents4_model, &a_currents4_human);

	// Write the identified dynamic parameters of the human limb in a file
	store_IdentifiedHDyn(ableInfos);
}

/*---------------------------------------------------------------------------------------------------------------------
| compute_ModelCurrents - Compute expected currents based on the robot static model
|
| Syntax --
|	void compute_ModelCurrents(ableDynamics* identDyn, std::vector<float>* a_currents4_model)
|
| Inputs --
|	ableDynamics* identDyn -> pointer towards the structure containing the model of the robot
|   std::vector<float>* a_currents4_model -> pointer towards the vector containing the computed currents
----------------------------------------------------------------------------------------------------------------------*/
void compute_ModelCurrents(ableDynamics* identDyn, ableMeasures* measValues, std::vector<float>* a_currents4_model)
{
	// Variables extraction
	//float* gm_stat = identDyn->g_models.gm_stat;
	//int nValues = measValues->able_speeds_4.size();

	// Compute model
	//for (int i(0); i < nValues; i++)
	//{
	//	a_currents4_model->push_back((gm_stat[0]* measValues->able_xs_slider.at(i) + 
	//		                          gm_stat[1])*cos(measValues->able_artpos_4.at(i)) +
	//	                              gm_stat[2]*sin(-measValues->able_artpos_4.at(i))+identDyn->f_models.adhfric);
	//}
}

/*---------------------------------------------------------------------------------------------------------------------
| compute_HumanCurrents - Compute human induced currents
|
| Syntax --
|	void compute_HumanCurrents(ableMeasures* measValues, std::vector<float>* a_currents4_model)
|
| Inputs --
|	ableMeasures* measValues -> pointer towards the substructure containing the measures taken on ABLE
|   std::vector<float>* a_currents4_model -> pointer towards the vector containing the computed currents
|   std::vector<float>* a_currents4_human -> pointer towards the vector containing the human induced currents
----------------------------------------------------------------------------------------------------------------------*/
void compute_HumanCurrents(ableMeasures* measValues, std::vector<float>* a_currents4_model,
	                       std::vector<float>* a_currents4_human)
{
	// Variables extraction
	int nValues = measValues->able_speeds_4.size();

	// Compute human induced currents (difference between observed and model)
	for (int i(0); i < nValues; i++)
	{
		a_currents4_human->push_back(measValues->able_currents_4.at(i) - a_currents4_model->at(i));
	}
}

/*---------------------------------------------------------------------------------------------------------------------
| compute_HumanMass - Estimate human limb mass based on human induced currents
|
| Syntax --
|	void compute_HumanMass(ThreadInformations* ableInfos, std::vector<float>* a_currents4_human)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
|   std::vector<float>* a_currents4_human -> pointer towards the vector containing the human induced currents
----------------------------------------------------------------------------------------------------------------------*/
void compute_HumanMass(ThreadInformations* ableInfos, std::vector<float>* a_currents4_human)
{
	// Extract measured values during the experiment
	ableMeasures* measValues = &ableInfos->ctrl_ABLE->aMeasures;
	ableDynamics* identDyn = &ableInfos->ctrl_ABLE->aDynamics;
	motorsParams* mVals = &ableInfos->ctrl_ABLE->mParams;
	// Variables extraction
	int nValues = measValues->able_speeds_4.size();
	float somme = 0;
	float mass_i = 0;

	for (int i(0); i < nValues; i++)
	{
		mass_i = measValues->able_currents_4.at(i)*mVals->kt_gain /
			     (G_VAL*measValues->able_xs_slider.at(i) *cos(measValues->able_artpos_4.at(i)));
		somme += mass_i;
	}
	ableInfos->ctrl_ABLE->hDynId.mass = mVals->able_AxisReductions[3] * somme / nValues;
}

/*---------------------------------------------------------------------------------------------------------------------
| compute_HumanMass_FT - Estimate human limb mass based on human induced interaction forces
|
| Syntax --
|	void compute_HumanMass_FT(ThreadInformations* ableInfos)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
----------------------------------------------------------------------------------------------------------------------*/
void compute_HumanMass_FT(ThreadInformations* ableInfos)
{
	// Extract measured values during the experiment
	ableMeasures* measValues = &ableInfos->ctrl_ABLE->aMeasures;
	// Variables extraction
	int nValues = measValues->fz_FTW_sensor.size();
	float somme = 0;
	float mass_i = 0;
	fprintf(ableInfos->identification_file, "Starting human forearm mass computation\n");
	if (measValues->fz_FTW_sensor.size() != measValues->able_artpos_4.size())
	{
		fprintf(ableInfos->identification_file, "Not the same number of measures of position and forces\n");
	}

	for (int i(1000); i < nValues; i++)
	{
		mass_i = - measValues->fz_FTW_sensor.at(i) / (G_VAL * cos(measValues->able_artpos_4.at(i)+0.53f));
		fprintf(ableInfos->identification_file, "At iteration %i : Angle : %f, Force : %f, Mass : %f\n",
			    i, measValues->able_artpos_4.at(i), measValues->fz_FTW_sensor.at(i),  mass_i);
		somme += mass_i;
	}
	ableInfos->ctrl_ABLE->hDynId.mass = somme / nValues;
}

/*---------------------------------------------------------------------------------------------------------------------
| store_IdentifiedHDyn - Store identified dynamic parameters in a text file for further use
|
| Syntax --
|	void store_IdentifiedHDyn(ThreadInformations* ableInfos)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
----------------------------------------------------------------------------------------------------------------------*/
void store_IdentifiedHDyn(ThreadInformations* ableInfos)
{
	FILE* h_File = ableInfos->ctrl_ABLE->hDynId.idDyn_File;
	fprintf(h_File, "%f ", ableInfos->ctrl_ABLE->hDynId.mass);
	fprintf(h_File, "0.5 0.0 ");
	fclose(h_File);
}

/*---------------------------------------------------------------------------------------------------------------------
| retrieve_IdentifiedDyn - Retrieve identified dynamic parameters
|
| Syntax --
|	void retrieve_IdentifiedDyn(ThreadInformations* ableInfos)
|
| Inputs --
|	ThreadInformations* ableInfos -> pointer towards the structure containing all the informations
----------------------------------------------------------------------------------------------------------------------*/
void retrieve_IdentifiedDyn(AbleControlStruct* ctrl_ABLE)
{
	FILE* h_File = ctrl_ABLE->hDynId.idDyn_File;
	if (fscanf(h_File, "%f %f %f", &ctrl_ABLE->hDynId.mass, &ctrl_ABLE->hDynId.delta_theta, &ctrl_ABLE->hDynId.farm_FE_inertia) == 3)
	{
		ctrl_ABLE->hDynId.check_ExtractData = 1;
	}else {
		ctrl_ABLE->hDynId.check_ExtractData = 0;
	}
	fclose(h_File);
}