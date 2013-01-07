#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>

enum hostIDs {
    RECEIVER_ID_ALL = 0,
    RECEIVER_ID_H_BRIDGE_1 = 1,
    RECEIVER_ID_H_BRIDGE_2,
    RECEIVER_ID_H_BRIDGE_3,
    RECEIVER_ID_H_BRIDGE_4,
    RECEIVER_ID_H_BRIDGE_5,
    RECEIVER_ID_H_BRIDGE_6,
    RECEIVER_ID_H_BRIDGE_7,
    RECEIVER_ID_H_BRIDGE_8,
};


/**
 * Initializes the internal data structures of the protocol handler.
 * */
void protocol_init();

/**
 * Returns the host id of this device
 * */
enum hostIDs protocol_getOwnHostId();

typedef void (*protocol_callback_t)(int senderId, int receiverId, int id, unsigned char *data, unsigned short size);

/**
 * Register a callback handler for the given id. 
 * If the handler is called, it is assured, that the 
 * packet was received correctly and that there is no 
 * data corruption.
 * */
void protocol_registerHandler(int id, protocol_callback_t handler);

/**
 * Sends out the given data using the given packet id.
 * @return Will return 0 on sucess and 1 on error
 * */
uint8_t protocol_sendData(int receiverId, int id, const unsigned char* data, short unsigned int size);

/**
 * Internal function. This function processes so calles low priority
 * data packets. This packets have an own internal header an allow
 * to send packets that are actually bigger as maxPacketSize.
 * 
 * */
// void protocol_processLowPrio(uint8_t *data, uint8_t size);

uint8_t protocol_getMaxPacketSize();

typedef signed int (*send_func_t)(uint16_t senderId, uint16_t receiverId, uint16_t packetId, const unsigned char *data, const unsigned int size);
typedef signed int (*recv_func_t)(uint16_t *senderId, uint16_t *receiverId, uint16_t *packetId, unsigned char *data, const unsigned int dataSize);


void protocol_setSendFunc(send_func_t func);
void protocol_setRecvFunc(recv_func_t func);

/**
 * @arg maxPacketSize Maximum packet size, any packet bigger than maxPacketSize will be send segmented
 * */
void protocol_setMaxPacketSize(uint8_t maxPacketSize);

/**
 * @arg ownHostId host id used for sending packets
 * */
void protocol_setOwnHostId(enum hostIDs id);

void protocol_ackPacket(int id, int receiverId);

void protocol_processPackage();



#endif
