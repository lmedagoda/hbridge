#ifndef ARC_DRIVER_H
#define ARC_DRIVER_H
#include "arc_packet.h"
#include "arc_driver.h"
typedef signed int (*arc_send_func_t)(const unsigned char *data, const unsigned int size);
typedef signed int (*arc_recv_func_t)(unsigned char *data, const unsigned int dataSize);
typedef signed int (*arc_seek_func_t)(unsigned char *data, const unsigned int dataSize);
#define MY_SYSTEM_ID 2
//extern volatile bool has_token;
//driver functions

uint32_t arc_sendPacketDirect(arc_packet_t* packet); 

/**
 * Initialise the arc driver with functionpointer for receiving, sending and seeking.
 **/
int arc_init(arc_send_func_t sendFunc, arc_recv_func_t recvFunc, arc_seek_func_t seekFunc);
/**
 * Reads every Packet, which can read with the receive function.
 * The function returns 0 if no packet or a bad packet found.
 * Otherwise the number of bytes in the packet.
 */
uint32_t arc_readPacket(arc_packet_t * packet); 
uint32_t arc_readPacketChannel(arc_packet_t * packet, int channel); 

/**
 * Sends a arc_packet with the send function.
 * Returns 0 if the packet was send, a negative value otherwise.
 */
int arc_sendPacket(arc_packet_t *packet);
int arc_sendPacketiChannel(arc_packet_t *packet, int channel);

int arc_add_serial_handler(arc_send_func_t sendFunc, arc_recv_func_t recvFunc, arc_seek_func_t seekFunc);

    

/**
 * Sends any byte buffer with the send function.
 * This is normally not necessary.
 */ 
int arc_send(uint8_t *tmp_send_buffer, int size);
int arc_sendChannel(uint8_t *tmp_send_buffer, int size, int channel);


#endif
