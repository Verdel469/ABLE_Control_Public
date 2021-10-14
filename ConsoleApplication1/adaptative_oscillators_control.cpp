/***********************************************************************************************************************
* adaptative_oscillators_control.cpp -
*
* Authors : Dorian Verdel, Abdelwaheb Hafs
* Creation  date : 05/2021
*
* Description :
* Manages adaptative oscillators control of ABLE
***********************************************************************************************************************/

#include "adaptative_oscillators_control.h"
using namespace std;


/*---------------------------------------------------------------------------------------------------------------------
| able_Hopf_PositionAsserv - Control using Hopf oscillator
| Syntax --
|        void able_PositionAsserv(ThreadInformations* ableInfos)
|
| Inputs --
|        ThreadInformations* ableInfos -> pointer towards the structure
containing all the informations
----------------------------------------------------------------------------------------------------------------------*/
void able_Hopf_PositionAsserv(ThreadInformations* ableInfos)
{
    // Extract substructs
    realTimeParams* rtValues = &ableInfos->ctrl_ABLE->rtParams;
    ableOrders* oValues = &ableInfos->ctrl_ABLE->aOrders;
    motorsParams* mValues = &ableInfos->ctrl_ABLE->mParams;
    hopf_oscillator* hValues = &ableInfos->ctrl_ABLE->hopfParams;

    // Variables declaration
    float pos_dif;
    //static float omega_d, phi_d, alpha_d;
    static float omega = 0.0f;
    static float phi = 0.0f;
    static float alpha = 0.0f;
    static int old_counter;
    
    // Check if order has changed since previous iteration
    if (old_counter != rtValues->order_counter && oValues->ctrl_type != MINJERK_TRAJS)
    {
        // Update old counter to the current order
        old_counter = rtValues->order_counter;
        fprintf(ableInfos->out_file, "Order counter : %d ; Position order : %f\n", rtValues->order_counter,
                ableInfos->ctrl_ABLE->aOrders.positionOrder[3]);
    }
    // Asserv position
    for (int i(0); i < NB_MOTORS; i++)
    {
        if (i == NB_MOTORS - 1)
        {
            // Extract current data
            able_ExtractData(ableInfos, i);
            // Compute diference in coder position
            pos_dif = rtValues->currentPosition[i] - oValues->positionOrder[i];
            // Compute Hopf pulsation
            hValues->omega_dot = pos_dif * cos(phi) * HOPF_V;
            // integration
            omega += hValues->omega_dot * rtValues->sampling_frequency;
            // Compute arg
            hValues->phi_dot = omega - pos_dif * cos(phi) * HOPF_V;
            // integration
            phi += hValues->phi_dot * rtValues->sampling_frequency;
            // Compute module
            hValues->alpha_dot = pos_dif * sin(phi) * ETA;
            // integration
            alpha += hValues->alpha_dot * rtValues->sampling_frequency;
            // Target
            oValues->speedOrder[i] = (alpha * sin(phi) - oValues->positionOrder[i])
                                   / rtValues->sampling_frequency;
            oValues->positionOrder[i] = alpha * sin(phi);
        }
        // Regulate interaction force for CoT experimentation
    }
    fprintf(ableInfos->out_file, "Computed position/speed order value %f\n", oValues->speedOrder[3]);
    if (rtValues->order_counter > 0)
    {
        // Store current values
        storeValuesInVectors(ableInfos);
    }
    // Print current values
    if (oValues->ctrl_type == MINJERK_TRAJS)
    {
        recordCurrentValues(ableInfos);
    }
}