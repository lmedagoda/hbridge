#ifndef ARC_DRIVER_H
#define ARC_DRIVER_H
#include "arc_packet.h"
void arc_init();
uint32_t arc_readPacket(arc_packet_t * packet); 
int arc_sendPacket(arc_packet_t *packet);
int arc_send(uint8_t *tmp_send_buffer, int size);

typedef signed int (*arc_send_func_t)(const unsigned char *data, const unsigned int size);
typedef signed int (*arc_recv_func_t)(unsigned char *data, const unsigned int dataSize);
typedef signed int (*arc_seek_func_t)(unsigned char *data, const unsigned int dataSize);
#endif
