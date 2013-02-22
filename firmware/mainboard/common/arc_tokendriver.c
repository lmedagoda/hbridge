#include "arc_driver.h"
#include "tokenhandling.h"
#include "arc_ringbuffer.h"
#include "printf.h"
#define MY_SYSTEM_ID 2
#define MAX_BYTES 30


RING_BUFFER read_buffer;
RING_BUFFER send_buffer;

volatile uint8_t has_token = 0;
volatile AMBER_STATE amber_state = AMBER_UNREGISTERED;
int send_bytes = 0;

void arctoken_giveTokenBack();
void arctoken_receivePackets();
void arctoken_sendPendingPackets();
//public
void arctoken_init(arc_send_func_t sendFunc, arc_recv_func_t recvFunc, arc_seek_func_t seekFunc){
    //Init arc from here, because the user should not mix arc and arctoken
    arc_init(sendFunc, recvFunc, seekFunc);
    initTokenhandling();
}

int arctoken_readPacket(arc_packet_t* packet){
    int ret = pop_front(&read_buffer, packet);
    return ret;
}
int arctoken_sendPacket(arc_packet_t* packet){
    return push_back(*packet, &send_buffer); 
}
void arctoken_processPackets(){
    arctoken_receivePackets();
    arctoken_sendPendingPackets();
    return;
}

void arctoken_sendProtocolPacket(ARC_PACKET_ID id){
    arc_packet_t packet;
    packet.originator = SLAVE;
    packet.system_id = (ARC_SYSTEM_ID) MY_SYSTEM_ID; 
    packet.packet_id = id; 
    packet.length = 0;
    int ret = arc_sendPacket(&packet);
    if (ret < 0){
        printf("Got an error by send protocolpacket: %i\n",id );
    }
}
//privats
void arctoken_giveTokenBack(){
    has_token = 0;
    arctoken_sendProtocolPacket(MB_GIVE_BACK);
}

void arctoken_receivePackets(){
    //printf("RECEIVE PACKETS\n");
    arc_packet_t packet;
    int32_t result;
    while ((result = arc_readPacket(&packet)) != 0){
        if (result<0){
            printf("Got an error by reading Packets\n");
            break;
        } else if (!handleTokenPacket(&packet)){
            //for a hot fix not token packets can handled here
            //packet for me?
                if (packet.system_id == (ARC_SYSTEM_ID) MY_SYSTEM_ID){
                    if (!push_back(packet, &read_buffer)){
                        printf("receive buffer overflowed some pakets may not received\n");
                    }
                } else {
                    printf("packet to other system %i\n", packet.system_id);
                }
                //handlePacket(&packet);
        }
    }
}

void arctoken_sendPendingPackets(){
    arc_packet_t* packet;
    int len = 0;
    uint8_t tmp_send_buffer[ARC_MAX_FRAME_LENGTH];
    if (has_token){
        while ((packet = first(&send_buffer))!=0){
           len = createPacket(packet, tmp_send_buffer); 
           if (MAX_BYTES-send_bytes > len){
               pop_front(&send_buffer, packet);
	       int ret = arc_sendPacket(packet); 
               if (ret < 0){
                   printf("Got an error by sending Pending Packets\n");
                   break;
               }
           } else {
               break;
           }
        }
        arctoken_giveTokenBack();
    }
}
