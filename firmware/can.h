#ifndef __CAN_H
#define __CAN_H

#include "stm32f10x_type.h"
#include "stm32f10x_can.h"
#include "protocol.h"

void CAN_Configuration(void);

void CAN_ConfigureFilters(enum hostIDs boardNr);

CanRxMsg *CAN_GetNextData();

void CAN_MarkNextDataAsRead();

#endif
