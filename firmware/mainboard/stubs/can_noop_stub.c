#include "../../hbridgeCommon/drivers/can.h"

void CAN_Configuration(enum CAN_REMAP remap)
{
}

void CAN_CancelAllTransmits()
{
}

void CAN_ConfigureFilters(enum hostIDs boardNr)
{
}


unsigned char CAN_SendMessage(CanTxMsg* TxMessage)
{
    return 0;
}

CanRxMsg *CAN_GetNextData()
{
    return 0;
}

void CAN_MarkNextDataAsRead()
{
}
