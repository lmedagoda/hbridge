#include "../../hbridgeCommon/protocol_can.h"
#include "avalon_can.h"
#include <protocol.h>

#define DEPTH_READER_FRONT 0x440
#define DEPTH_READER_REAR 0x441
#define TELNET_ID 0x141
#define CAN_ID_BATTERY_OUTGOING_DATA  
extern cur_depth;
typedef enum {FALSE = 0, TRUE = !FALSE} bool;
struct depthReader_canData {
    unsigned externalPressure:16;
    unsigned internalPressure:12;
    unsigned initialInternalPressure:12;
    unsigned temperature:12;
    unsigned waterIngress1:1;
    unsigned waterIngress2:1;
    unsigned waterIngress3:1;
    unsigned unused:5; //9
} __attribute__ ((packed)) __attribute__((__may_alias__));

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
void avaloncan_handlePacket(CanRxMsg *curMsg){
    if((curMsg->StdId) == 0x500){ //Modem Messages
        uwmodem_sendData(curMsg->Data,curMsg->DLC);
    }else if(curMsg->StdId == DEPTH_READER_FRONT){ //Depth Reader messages
        const struct depthReader_canData *data = (const struct canData *)(curMsg->Data);
        //See depth_reader task for numeric explanation
        int32_t externalPress = (data->externalPressure - externalPressureRangeShift) *1600000 / 65536; //   / 65536.0 * 1600000;
        int32_t internalPress = data->internalPressure * 152167 / 4096 + 10556;   //  / 4096.0  * 152166.666666667 + 10555.555555556; 
        int32_t internalOffset = data->initialInternalPressure * 152167 / 4096 + 10556;  //  / 4096.0  * 152166.666666667 + 10555.555555556;
        uint8_t water_ingress = 0;
        if (data->waterIngress1){
            water_ingress = 1;
        } 
        if (data->waterIngress2){
            water_ingress = 1;
        }
        if (data->waterIngress3){
            water_ingress = 1;
        }
        if (water_ingress){
            printf("WATER INGRESS\n");
        } else {
            //printf("no water ingress\n");
        }

        int32_t depth = (externalPress - (internalOffset-internalPress)) * 655 / 10000; //Depth is positive here, sorry
        cur_depth = (depth + 6550);
    } else if (curMsg->StdId == DEPTH_READER_REAR){
        const struct depthReader_canData *data = (const struct canData *)(curMsg->Data);
        //See depth_reader task for numeric explanation
        uint8_t water_ingress = 0;
        if (data->waterIngress1){
            water_ingress = 1;
        } 
        if (data->waterIngress2){
            water_ingress = 1;
        }
        if (data->waterIngress3){
            water_ingress = 1;
        }
        if (water_ingress){
            printf("WATER INGRESS\n");
        } else {
            //printf("no water ingress\n");
        }
    }
    else{
        printf("Got Unknown ID: %lu\n",curMsg->StdId);
        //TODO Add message for DEPTH Readings and other status messages
    }

}
