#include <stddef.h>
#include "arc_driver.h"
#include "printf.h"
#define SYSTEM_ID 2
int stat_bad_packets = 0;

unsigned int arc_num_serial_handler = 0;
unsigned int arc_current_serial_handler = 0;
arc_send_func_t arc_sendFunc[5];
arc_recv_func_t arc_recvFunc[5];
arc_seek_func_t arc_seekFunc[5];

//privats
//void arc_receivePackets();
//void arc_sendPendingPackets();
//void arc_receivePackets();
//void arc_giveTokenBack();

int arc_add_serial_handler(arc_send_func_t sendFunc, arc_recv_func_t recvFunc, arc_seek_func_t seekFunc){
    if (arc_num_serial_handler >= 5){
        return -1;
    }
    arc_sendFunc[arc_num_serial_handler] = sendFunc;
    arc_recvFunc[arc_num_serial_handler] = recvFunc;
    arc_seekFunc[arc_num_serial_handler] = seekFunc;
    return arc_num_serial_handler++;
}


//public
int arc_init(arc_send_func_t sendFunc, arc_recv_func_t recvFunc, arc_seek_func_t seekFunc)
{
    return arc_add_serial_handler(sendFunc, recvFunc, seekFunc);
}

uint32_t arc_readPacketChannel(arc_packet_t * packet, int channel){
    if (channel <  0 || channel >= arc_num_serial_handler){
        printf("Warning: This Channel %i number does not exist there are %i \n",channel, arc_num_serial_handler);
        return 0;
    }
    int seek, i;
    uint8_t packet_buffer[ARC_MAX_FRAME_LENGTH];
    while( (seek = arc_seekFunc[channel](packet_buffer, ARC_MAX_FRAME_LENGTH)) >= ARC_MIN_FRAME_LENGTH ) {

        //use parsePacket from arc_packet.c
        int result = parsePacket(packet_buffer, seek, packet);

        // if result is less than 0, this is the number of bytes that can be skipped
        if (result < 0) {
            // debug: print out content of buffer   
            if (seek >= ARC_MIN_FRAME_LENGTH) {
                //printf("bad packet: ");
                for (i = 0; i < seek; i++) {
                    int x = packet_buffer[i];
                    //printf("%d ", x);
                }
                //printf(" skipping:%d \n", -result);
            } else {
                //printf("got bad packet for seek %d. This shouldn't happen.", seek);
            }

            if( arc_recvFunc[channel](packet_buffer, -result) != -result )
                printf("skipping bytes didn't work\n");

        } else if (result == 0) {
            // not received enough data
            // return and wait for new data
            return 0;
        } else if (result > 0) {
            // found a packet, return it and skip the number of bytes from the buffer
            arc_recvFunc[channel](packet_buffer, result);
            arc_current_serial_handler=channel;
            return result;
        }
    }
    return 0;
}
uint32_t arc_readPacket(arc_packet_t * packet) {
    // function will return 0 if no packet has been found
    // the number of bytes in the packet otherwise
    int channel, ret;
    uint8_t packet_buffer[ARC_MAX_FRAME_LENGTH];
    for(channel=0;channel<arc_num_serial_handler;channel++){
        if (ret = arc_readPacketChannel(packet, channel)){
            return ret;
        }
    }
    return 0; // no packet found
}

int arc_send(uint8_t *tmp_send_buffer, int size){
    return arc_sendChannel(tmp_send_buffer, size, arc_current_serial_handler);
} 
int arc_sendChannel(uint8_t *tmp_send_buffer, int size, int channel){
    if (channel < 0 || channel >= arc_num_serial_handler){
        printf("Warning: This Channel number does not exist");
        return -1;
    }
    int sent = 0;
    while(sent < size)
    {
        int ret = arc_sendFunc[channel](tmp_send_buffer, size);
        if(ret < 0)
        {
            printf("Got an error by sending\n");
            return ret;
        }
        sent += ret;
    }
    return 0;
}
int arc_sendPacket(arc_packet_t *packet){
    return arc_sendPacketChannel(packet, arc_current_serial_handler);
}
int arc_sendPacketChannel(arc_packet_t *packet, int channel){
    uint8_t tmp_send_buffer[ARC_MAX_FRAME_LENGTH];
    int len = createPacket(packet, tmp_send_buffer);
    int ret = arc_sendChannel(tmp_send_buffer, len, channel);
    if (ret < 0){
        printf("Got an error by sending packet\n");
    }
    return ret;
}

uint32_t arc_sendPacketDirect(arc_packet_t* packet) 
{
    int len = 0;
    uint8_t tmp_send_buffer[ARC_MAX_FRAME_LENGTH];
    len = createPacket(packet, tmp_send_buffer); 
    int sent = 0;
    while(sent < len)
    {
	int ret = arc_sendFunc[arc_current_serial_handler](tmp_send_buffer, len);
	if(ret < 0)
	{
	    printf("Got an error by sending Packet");
	    break;
	}
	sent += ret;
    }
    return len;
}
