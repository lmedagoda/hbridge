#ifndef __CAN_H
#define __CAN_H

#include "common/packets.h"
#include "common/protocol.h"
#include "inc/stm32f10x_can.h"

#define CAN_SEND_RETRIES 5000
#define CAN_SEND_ARBITRATION_RETRIES 20 

#ifdef __cplusplus
extern "C"{
#endif

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

/**
 * Returns a pointer to the next received message
 * in the receive queue or NULL if the queue is
 * empty
 * */
CanRxMsg *CAN_GetNextData();

/**
 * Removes the current message from the receive queue
 * */
void CAN_MarkNextDataAsRead();

/**
* Conveniance function, that will keep resending
* the message until the message was send, 
* or a timeout was hit
*/
int can_send_message_hard(CanTxMsg * message);

/**
 * Tries to send out the can message. Note this method is thread safe.
 * Return 0 on success 1 on failure.
 * */
unsigned char CAN_SendMessage(CanTxMsg* TxMessage);

#ifdef __cplusplus
}
#endif

#endif
