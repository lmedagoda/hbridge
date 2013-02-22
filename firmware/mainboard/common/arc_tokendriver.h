#ifndef ARC_TOKENDRIVER_H
#define ARC_TOKENDRIVER_H
#include "arc_packet.h"
#include "arc_driver.h"
void arctoken_init(arc_send_func_t sendFunc, arc_recv_func_t recvFunc, arc_seek_func_t seekFunc);
int arctoken_readPacket(arc_packet_t* packet);
int arctoken_sendPacket(arc_packet_t* packet);
void arctoken_processPackets();
void arctoken_sendProtocolPacket(ARC_PACKET_ID id);
#endif
