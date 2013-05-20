#ifndef ARC_DRIVER_H
#define ARC_DRIVER_H
#include "arc_packet.h"
#define MY_SYSTEM_ID 2
//extern volatile bool has_token;
//driver functions
int arc_getPacket(arc_packet_t* packet);
int arc_sendPacket(arc_packet_t* packet);
uint32_t arc_readPacket(arc_packet_t * packet); 
uint32_t arc_sendPacketDirect(arc_packet_t* packet); 
void arc_processPackets();
void arc_init();



typedef signed int (*arc_send_func_t)(const unsigned char *data, const unsigned int size);
typedef signed int (*arc_recv_func_t)(unsigned char *data, const unsigned int dataSize);

typedef signed int (*arc_seek_func_t)(unsigned char *data, const unsigned int dataSize);
//token functions
void arc_sendProtocolPacket(ARC_PACKET_ID id);
#endif
