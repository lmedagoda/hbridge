#ifndef __CAN_H
#define __CAN_H

#include "inc/stm32f10x_type.h"
#include "inc/stm32f10x_can.h"
#include "protocol.h"

#define CAN_SEND_RETRIES 5000
#define CAN_SEND_ARBITRATION_RETRIES 20 

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

/**
* Conveniance function, that will keep resending
* the message until the message was send, 
* or a timeout was hit
*/
int can_send_message_hard(CanTxMsg * message);


#endif
