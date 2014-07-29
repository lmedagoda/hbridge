#include "../../hbridgeCommon/drivers/can.h"
#include <stdio.h>

void assert(int value){
    if(value){
        printf("Assertion\n");
    }
}


#include <canbus.hh>

canbus::Driver *m_driver;


extern "C" {

void CAN_Configuration(enum CAN_REMAP remap)
{
    m_driver = canbus::openCanDevice("can0");
}

void CAN_CancelAllTransmits()
{
}

void CAN_ConfigureFilters(enum hostIDs boardNr)
{
}


unsigned char CAN_SendMessage(CanTxMsg* TxMessage)
{
    canbus::Message msg;
    msg.can_id = TxMessage->StdId; //TODO check maybe ExtId
    msg.size = TxMessage->DLC;
    for(unsigned int i = 0; i< msg.size;i++){
        msg.data[i] = TxMessage->Data[i];
    }
    if(m_driver){
    m_driver->write(msg); 
    }else{
        printf("Theoretically write canMessage with ID 0x%02x\n",msg.can_id);
    }
    return 0;
}

CanRxMsg *CAN_GetNextData()
{
    return 0;
}

void CAN_MarkNextDataAsRead()
{
}

}

//
