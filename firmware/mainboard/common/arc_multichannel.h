#ifndef ARC_MULTICHANNEL_H
#define ARC_MULTICHANNEL_H
#include "arc_packet.h"
#include "arc_driver.h"
void arc_multichannel_init();
int arc_multichannel_addSerialDriver(arc_send_func_t sendFunc, arc_recv_func_t recvFunc, arc_seek_func_t seekFunc);
int arc_multichannel_addTokenSerialDriver(arc_send_func_t sendFunc, arc_recv_func_t recvFunc, arc_seek_func_t seekFunc);
void arc_multichannel_setOwnSystemID(ARC_SYSTEM_ID sys_id);
int arc_multichannel_readPacket(arc_packet_t* packet);
int arc_multichannel_sendPacket(arc_packet_t* packet);
void arc_multichannel_processPackets();
void arc_multichannel_setOwnSystemID(ARC_SYSTEM_ID sys_id);
ARC_SYSTEM_ID arc_multichannel_getOwnSystem_ID();
#endif
