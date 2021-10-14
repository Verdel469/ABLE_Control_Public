/***********************************************************************************************************************
* able_Control_FTData.h -
*
* Author : Dorian Verdel
* Creation  date : 03/2021
*
* Description :
* Manages the declaration of functions establishing communication between ABLE thread and FT sensor thread.
***********************************************************************************************************************/

#pragma once

#ifndef ABLE_CONTROL_FTDATA_H
#define ABLE_CONTROL_FTDATA_H

#include "communication_struct_ABLE.h"

// Write function definition
void digitalFT_WriteData(AbleControlStruct* ctrl_ABLE);

// Read function definition
void digitalFT_ReadData(ThreadInformations* ableInfos);
#endif // !ABLE_CONTROL_FTDATA_H