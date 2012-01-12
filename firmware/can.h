#ifndef __CAN_H
#define __CAN_H

#include "inc/stm32f10x_type.h"
#include "inc/stm32f10x_can.h"
#include "protocol.h"

enum CAN_REMAP {
    //PA11, PA12
    CAN_NO_REMAP,
    //PB8, PB9
    CAN_REMAP1,
    //PD0, PD1
    CAN_REMAP2
};

void CAN_Configuration(enum CAN_REMAP remap);
void CAN_CancelAllTransmits();

void CAN_ConfigureFilters(enum hostIDs boardNr);

CanRxMsg *CAN_GetNextData();

void CAN_MarkNextDataAsRead();

#endif
