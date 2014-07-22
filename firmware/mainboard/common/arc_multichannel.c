#include "arc_tokendriver.h"
#include "arc_notokendriver.h"
int arc_multichannel_num_notoken;
int arc_multichannel_num_token;
uint8_t arc_multichannel_last_from_token;
void arc_multichannel_init(ARC_SYSTEM_ID sys_id){
    arctoken_setOwnSystemID(sys_id);
    arcnotoken_setOwnSystemID(sys_id);
    arc_multichannel_num_notoken = 0;
    arc_multichannel_num_token= 0;
}

int arc_multichannel_addSerialDriver(arc_send_func_t sendFunc, arc_recv_func_t recvFunc, arc_seek_func_t seekFunc){
    int ret;
    if (arc_multichannel_num_notoken == 0){
        ret = arcnotoken_init(sendFunc, recvFunc, seekFunc);
    } else {
        ret =  arcnotoken_add_serial_handler(sendFunc, recvFunc, seekFunc);
    }
    if (ret >= 0){
        arc_multichannel_num_notoken++;
    }
    return ret;
}
void arc_multichannel_setOwnSystemID(ARC_SYSTEM_ID sys_id){
    arcnotoken_setOwnSystemID(sys_id);
    arctoken_setOwnSystemID(sys_id);
}

int arc_multichannel_addTokenSerialDriver(arc_send_func_t sendFunc, arc_recv_func_t recvFunc, arc_seek_func_t seekFunc){
    int ret;
    if (arc_multichannel_num_token == 0){
        ret = arctoken_init(sendFunc, recvFunc, seekFunc);
    } else {
        ret = arctoken_add_serial_handler(sendFunc, recvFunc, seekFunc);
    }
    if (ret >= 0){
        arc_multichannel_num_token++;
    }
    return ret;
}
void arc_multichannel_processPackets(){
    if (arc_multichannel_num_token > 0){
        arctoken_processPackets();
    }
}
int arc_multichannel_readPacket(arc_packet_t * packet){
    int ret;
    if (arc_multichannel_num_token > 0){
        if (ret = arctoken_readPacket(packet)){
            arc_multichannel_last_from_token = 1;
            return ret; 
        }
    }
    if (arc_multichannel_num_notoken > 0){
        if (ret = arcnotoken_readPacket(packet)){
            arc_multichannel_last_from_token = 0;
            return ret;
        }
    }
    return 0;
}
int arc_multichannel_sendPacket(arc_packet_t * packet){
    if (arc_multichannel_num_token > 0 && arc_multichannel_last_from_token){
        return arctoken_sendPacket(packet);
    } else if (arc_multichannel_num_notoken > 0) {
        return arcnotoken_sendPacket(packet);
    }
    return 0;
}
