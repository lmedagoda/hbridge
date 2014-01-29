#include "../../hbridgeCommon/protocol_can.h"
#include "asv_can.h"

#define DEPTH_READER_ID 131
#define TELNET_ID 0x141
#define CAN_ID_BATTERY_OUTGOING_DATA  

const int externalPressureRangeShift = 2048;

/*
 * This Function overrides the can_recvPacket function, to seperate ASV-CanPackages from the
 * hbridge Can Protocol
 */
int avaloncan_recvPacket(uint16_t *senderId, uint16_t *receiverId, uint16_t *packetId, unsigned char *data, const unsigned int dataSize){
    CanRxMsg *msg = CAN_GetNextData();

    if(!msg)
	return 0;
    if (msg->StdId >= 0x400){
        avaloncan_handlePacket(msg);
        CAN_MarkNextDataAsRead();
        return 0;
    } else {
        return can_recvPacket(senderId, receiverId, packetId, data, dataSize);
    }
}
/*
void avaloncan_handlePacket(CanRxMsg *msg){
    unsigned char data[8];
    int i;
    for(i = 0; i < msg->DLC; i++)
    {
        data[i] = msg->Data[i];
    }
    int dataSize = msg->DLC;
    printf("Incoming ASV-Can Protocol Package\n");
    switch(msg->StdId){
        case 0x501: //UW-Modem to ASV
            printf("Another UW-Modem in the System?\n");
            break;
        case 0x500: //ASV to UW-Modem
            printf("Message via CAN to Modem\n");
            uwmodem_sendData(data, dataSize);
            break;
        default:
            printf("Warning: Unhandled ASV-Can Packet\n");
            break;
    }

}*/

//TODO Adapt this function too for passthrought modem messages to the PC
//Don't forget to setup the Can message filter for match the needed ID's
void avaloncan_handlePacket(CanRXMsg *curMsg){
    if((curMsg = CAN_GetNextData()) != 0) {
    	if((curMsg->StdId) == 0x1E1){ //Modem Messages
		uwmodem_SendData(curMsg->Data,curMsg->DLC);
        }else if(curMsg->StdId == DEPTH_READER_ID){ //Depth Reader messages
                const struct canData *data = (const struct canData *)(curMsg->Data);
                //See depth_reader task for numeric explanation
                int32_t externalPress = (data->externalPressure - externalPressureRangeShift) *1600000 / 65536; //   / 65536.0 * 1600000;
                int32_t internalPress = data->internalPressure * 152167 / 4096 + 10556;   //  / 4096.0  * 152166.666666667 + 10555.555555556; 
                int32_t internalOffset = data->initialInternalPressure * 152167 / 4096 + 10556;  //  / 4096.0  * 152166.666666667 + 10555.555555556;
                int32_t depth = (externalPress - (internalOffset-internalPress)) * 655 / 10000; //Depth is positive here, sorry
                cur_depth = (depth + 6550);
        }else if(curMsg->StdId == CAN_ID_BATTERY_OUTGOING_DATA){ //????????????????
            uint8_t *pCombinedData = &combinedDataMessage[0];
            bool* pGotFirstMessagePart = &gotFirstDataMessagePart;
            bool* pGotCompleteMessage = &gotCompleteDataMessage;
            int packetCount = DATA_MEASSAGE_PACKET_COUNT;

            int i=0;
            for(i=0;i<curMsg->DLC;i++){
                (&pCombinedData[7*curMsg->Data[0]])[i] = (&curMsg->Data[1])[i];
            }
//            memcpy(&pCombinedData[7*curMsg->Data[0]], &curMsg->Data[1], curMsg->DLC);
            if (curMsg->Data[0] == 0) {
                *pGotFirstMessagePart = TRUE;
            }
            if (curMsg->Data[0] == packetCount-1 && *pGotFirstMessagePart) {
                *pGotCompleteMessage = TRUE;
            }
            if (gotCompleteDataMessage){
                gotFirstDataMessagePart = FALSE;
                gotCompleteDataMessage = FALSE;
                const BatteryMessage_t *batt = (BatteryMessage_t*)combinedDataMessage;
                taken_battery_capacity = batt->takenCapacity;
                voltage = batt->overallVoltage; //Does not work cucrently
                //voltage = batt->cellVoltage1 + batt->cellVoltage2 + batt->cellVoltage3 + batt->cellVoltage4 + batt->cellVoltage5 + batt->cellVoltage6 + batt->cellVoltage7 + batt->cellVoltage8;
                //voltage = batt->cellVoltage8;
                //See communication.h from battery for additional information

            }

            
        }else{
		printf("Got Unknown ID: %lu\n",curMsg->StdId);
		//TODO Add message for DEPTH Readings and other status messages
	}
    	CAN_MarkNextDataAsRead();

    }
}
