/***********************************************************************************************************************
* able_OrdersManagement.h -
*
* Author : Dorian Verdel
* Creation  date : 03/2021
*
* Description :
* Manages the definition of the functions for orders management.
***********************************************************************************************************************/

#pragma once

#ifndef ABLE_ORDERSMANAGEMENT_H
#define ABLE_ORDERSMANAGEMENT_H

// Project includes
#include "communication_struct_ABLE.h"
#include "position_control.h"
#include "torque_control.h"
#include "adaptative_oscillators_control.h"

// Orders update functions
void able_UpdateOrders(ThreadInformations* ableInfos);
void able_UpdateOrderIdent(ThreadInformations* ableInfos);
void able_UpdateOrderMinJerk(ThreadInformations* ableInfos);
void able_SelectAdaptedControl(ThreadInformations* ableInfos);

// Check orders achievement
bool able_CheckOrderState(ThreadInformations* ableInfos);
void able_CheckOrderMinJerk(ThreadInformations* ableInfos);

// Handle movements end
int switch_OrderReached(ThreadInformations* ableInfos);
#endif // !ABLE_ORDERSMANAGEMENT_H