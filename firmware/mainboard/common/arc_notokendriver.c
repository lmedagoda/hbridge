#include "arc_driver.h"
ARC_SYSTEM_ID arcnotoken_system_id = 0;

int arcnotoken_channel_ids[5];
uint8_t arcnotoken_current_channel_num = 0;
int arcnotoken_current_channel;

int arcnotoken_init(arc_send_func_t sendFunc, arc_recv_func_t recvFunc, arc_seek_func_t seekFunc){
    arcnotoken_channel_ids[arcnotoken_current_channel_num] = arc_init(sendFunc, recvFunc, seekFunc);
    return arcnotoken_current_channel_num++;
}
int arcnotoken_add_serial_handler(arc_send_func_t sendFunc, arc_recv_func_t recvFunc, arc_seek_func_t seekFunc){
    int channel_id = arc_add_serial_handler(sendFunc, recvFunc, seekFunc);
    if (channel_id < 0){
        printf("Can not add a Channel");
        return channel_id;
    }
    arcnotoken_channel_ids[arcnotoken_current_channel_num] = channel_id;
    return arcnotoken_current_channel_num++;
}

int arcnotoken_readPacket(arc_packet_t* packet){
    int32_t result;
    int channel;
    for (channel=0; channel<arcnotoken_current_channel_num; channel++){
        while ((result = arc_readPacketChannel(packet, arcnotoken_channel_ids[channel])) != 0){
            //printf("Read a ARC PACKET\n");
            if (result<0){
                printf("Got an error by reading Packets\n");
                break;
            } else if (packet->system_id == arctoken_getOwnSystemID()){
                arcnotoken_current_channel = channel;
                return result;
            }
        }
    }
    return 0;
}

int arcnotoken_sendPacket(arc_packet_t* packet){
    int ret = arc_sendPacketChannel(packet, arcnotoken_channel_ids[arcnotoken_current_channel]); 
    if (ret < 0){
        printf("Got an error by sending Pending Packets\n");
        return;
    }
}
void arcnotoken_setOwnSystemID(ARC_SYSTEM_ID sys_id){
    arcnotoken_system_id = sys_id;
}
ARC_SYSTEM_ID arcnotoken_getOwnSystemID(){
    return arcnotoken_system_id;
}
