#ifndef PACKETHANDLING_H
#define PACKETHANDLING_H
#include <stdint.h>

typedef void (*packet_callback_t)(int senderId, int receiverId, int id, unsigned char *data, unsigned short size);

void packet_init();

void packet_registerHandler(int id, packet_callback_t callback);
void packet_handlePacket(int senderId, int receiverId, int id, unsigned char *data, unsigned short size);

#endif
