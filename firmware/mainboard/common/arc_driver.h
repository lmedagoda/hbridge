#ifndef ARC_DRIVER_H
#define ARC_DRIVER_H
#include "arc_packet.h"
#define MY_SYSTEM_ID 2
//extern volatile bool has_token;
//driver functions
int amber_getPacket(arc_packet_t* packet);
int amber_sendPacket(arc_packet_t* packet);
void amber_processPackets();
void initAmber();
//token functions

void sendProtocolPacket(ARC_PACKET_ID id);
#endif
