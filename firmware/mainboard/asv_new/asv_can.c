#include "../../hbridgeCommon/protocol_can.h"
#include "asv_can.h"
/*
 * This Function overrides the can_recvPacket function, to seperate ASV-CanPackages from the
 * hbridge Can Protocol
 */
int asvcan_recvPacket(uint16_t *senderId, uint16_t *receiverId, uint16_t *packetId, unsigned char *data, const unsigned int dataSize){
    CanRxMsg *msg = CAN_GetNextData();

    if(!msg)
	return 0;
    if (msg->StdId >= 0x400){
        asvcan_handlePacket(msg);
        CAN_MarkNextDataAsRead();
        return 0;
    } else {
        return can_recvPacket(senderId, receiverId, packetId, data, dataSize);
    }
}
void asvcan_handlePacket(CanRxMsg *msg){
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

}
