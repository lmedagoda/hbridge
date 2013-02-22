#include <stddef.h>
#include "arc_driver.h"
#include "printf.h"
#define SYSTEM_ID 2
int stat_bad_packets = 0;

arc_send_func_t arc_sendFunc;
arc_recv_func_t arc_recvFunc;
arc_seek_func_t arc_seekFunc;

//privats
void arc_receivePackets();
void arc_sendPendingPackets();
void arc_receivePackets();
void arc_giveTokenBack();

//public
void arc_init(arc_send_func_t sendFunc, arc_recv_func_t recvFunc, arc_seek_func_t seekFunc)
{
    arc_sendFunc = sendFunc;
    arc_recvFunc = recvFunc;
    arc_seekFunc = seekFunc;
}

/*
 * This Function is public only for NO token usage.
 * Do not use this funktion, if you want to use token.
 */
uint32_t arc_readPacket(arc_packet_t * packet) {
    // function will return 0 if no packet has been found
    // the number of bytes in the packet otherwise
    int seek, i;
    uint8_t packet_buffer[ARC_MAX_FRAME_LENGTH];

    // look for a valid packet as long as enough bytes are left
    while( (seek = arc_seekFunc(packet_buffer, ARC_MAX_FRAME_LENGTH)) >= ARC_MIN_FRAME_LENGTH ) {

        //use parsePacket from arc_packet.c
        int result = parsePacket(packet_buffer, seek, packet);

        // if result is less than 0, this is the number of bytes that can be skipped
        if (result < 0) {
            // debug: print out content of buffer   
            if (seek >= ARC_MIN_FRAME_LENGTH) {
                printf("bad packet: ");
                for (i = 0; i < seek; i++) {
                    int x = packet_buffer[i];
                    printf("%d ", x);
                }
                printf(" skipping:%d \n", -result);
            } else {
                printf("got bad packet for seek %d. This shouldn't happen.", seek);
            }

            if( arc_recvFunc(packet_buffer, -result) != -result )
                printf("skipping bytes didn't work\n");

        } else if (result == 0) {
            // not received enough data
            // return and wait for new data
            return 0;
        } else if (result > 0) {
            // found a packet, return it and skip the number of bytes from the buffer
            arc_recvFunc(packet_buffer, result);
            return result;
        }
    }

    return 0; // no packet found
}

int arc_send(uint8_t *tmp_send_buffer, int size){
    int sent = 0;
    while(sent < size)
    {
        int ret = arc_sendFunc(tmp_send_buffer, size);
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
    uint8_t tmp_send_buffer[ARC_MAX_FRAME_LENGTH];
    int len = createPacket(packet, tmp_send_buffer);
    int ret = arc_send(tmp_send_buffer, len);
    if (ret < 0){
        printf("Got an error by sending packet\n");
    }
    return ret;
}
