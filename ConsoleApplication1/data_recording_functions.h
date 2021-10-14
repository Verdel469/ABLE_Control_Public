/***********************************************************************************************************************
* data_recording_functions.h -
*
* Author : Dorian Verdel
* Creation  date : 03/2021
*
* Description :
* Manages the declarations of recording functions
***********************************************************************************************************************/

#pragma once

#ifndef DATA_RECORDING_FUNCTIONS_H
#define DATA_RECORDING_FUNCTIONS_H

#include "communication_struct_ABLE.h"
#include "utils_for_ABLE_Com.h"

// Temporary recording functions
void storeValuesInVectors(ThreadInformations* ableInfos);

// File recording functions
void recordCurrentValues(ThreadInformations* ableInfos);
void recordValuesInFile(ThreadInformations* ableInfos);

#endif // !DATA_RECORDING_FUNCTIONS_H