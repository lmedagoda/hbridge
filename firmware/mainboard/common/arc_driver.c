#include <stddef.h>
#include "arc_driver.h"
#include "tokenhandling.h"
#include "arc_ringbuffer.h"
#include "printf.h"
#define MAX_BYTES 30
#define SYSTEM_ID 2
RING_BUFFER read_buffer;
RING_BUFFER send_buffer;
volatile uint8_t has_token = 0;
volatile AMBER_STATE amber_state = AMBER_UNREGISTERED;
int stat_bad_packets = 0;
int send_bytes = 0;

arc_send_func_t arc_sendFunc;
arc_recv_func_t arc_recvFunc;

//privats
int arc_newreadPacket(arc_packet_t* packet);
void arc_receivePackets();
void arc_sendPendingPackets();
int arc_readPacket(arc_packet_t* packet); 
void arc_receivePackets();
void arc_giveTokenBack();


//public
void arc_init(arc_send_func_t sendFunc, arc_recv_func_t recvFunc)
{
    arc_sendFunc = sendFunc;
    arc_recvFunc = recvFunc;
    initTokenhandling();
}

int arc_getPacket(arc_packet_t* packet){
    int ret = pop_front(&read_buffer, packet);
    //printf("AN DIESER STEKLLE DIE ID? %i \n", packet->packet_id);
    return ret;
}

int arc_sendPacket(arc_packet_t* packet){
    return push_back(*packet, &send_buffer); 
}

void arc_processPackets(){
    arc_receivePackets();
    arc_sendPendingPackets();
    return;
}

//privat
int arc_readPacket(arc_packet_t* packet) {
    int len, ret;
    unsigned char packet_buffer[ARC_MAX_FRAME_LENGTH];
    while((ret = arc_recvFunc(packet_buffer, 1))>0){
        int idx = 1;
        if (packet_buffer[0] == SYNC_MARKER){
            while (idx < ARC_MAX_FRAME_LENGTH){
                len=arc_recvFunc(packet_buffer+idx, 1);
                if (len < 0) {return -1;}
                int ret2 = parsePacket(packet_buffer, idx+1, packet); // ;-) nice variablenames need time
                if (ret2 > 0){
                    return ret2;
                } else if (len < 0){
                    printf("Wir haben haben zu viel gelesen, so ein mist\n");
                } else {
                    //printf("Wir haben noch nicht genug gelesen\n");
                }
                idx += len;
            }
            printf("A BAD PACKET\n");
            return -1;
        }
    }
    return 0;
}

unsigned char packet_buffer[ARC_MAX_FRAME_LENGTH];
int idx = 0;
int arc_newreadPacket(arc_packet_t* packet){
    int ret;
    while((ret = arc_recvFunc(packet_buffer+idx, 1))>0){
        if (idx == 0){
            if (packet_buffer[0] == SYNC_MARKER){
                idx++;
            }
        } else {
            idx++;
            ret = parsePacket(packet_buffer, idx ,packet);
            if (ret > 0) {
                idx = 0;
                return ret;
            } else if (ret == 0){
                //printf("noch nicht genug gelesen");
            } else {
                printf("Wir haben haben zu viel gelesen, so ein mist\n");
            }
            if (idx >= ARC_MAX_FRAME_LENGTH){
                printf("Bad package");
                idx = 0;
                packet_buffer[0] = 0x07;
                return -1;
            }
        } 
    }
    return 0;
}

void arc_sendPendingPackets(){
    arc_packet_t* packet;
    int len = 0;
    uint8_t tmp_send_buffer[ARC_MAX_FRAME_LENGTH];
    if (has_token){
        while ((packet = first(&send_buffer))!=0){
           len = createPacket(packet, tmp_send_buffer); 
           if (MAX_BYTES-send_bytes > len){
               pop_front(&send_buffer, packet);
	       int sent = 0;
	       while(sent < len)
	       {
		    int ret = arc_sendFunc(tmp_send_buffer, len);
		    {
			printf("Got an error by sending Packet");
			break;
		    }
		    sent += ret;
	       }
               //sendProtocolPacket(GIVE_BACK);
           } else {
               break;
           }
        }
        arc_giveTokenBack();
    }
}

void arc_sendProtocolPacket(ARC_PACKET_ID id){
    uint8_t tmp_send_buffer[ARC_MAX_FRAME_LENGTH];
    arc_packet_t packet;
    packet.originator = SLAVE;
    packet.system_id = (ARC_SYSTEM_ID) MY_SYSTEM_ID; 
    packet.packet_id = id; 
    packet.length = 0;
    int len = createPacket(&packet, tmp_send_buffer);
    int sent = 0;
    while(sent < len)
    {
	int ret = arc_sendFunc(tmp_send_buffer, len);
	if(ret < 0)
	{
	    printf("Got an error by sending Packet");
	    break;
	}
	sent += ret;
    }
}

void arc_giveTokenBack(){
    has_token = 0;
    arc_sendProtocolPacket(MB_GIVE_BACK);
}

void arc_receivePackets(){
    //printf("RECEIVE PACKETS\n");
    arc_packet_t packet;
    int32_t result;
    while ((result = arc_newreadPacket(&packet)) != 0){
        if (result<0){
            printf("Got an error by reading Packets");
            break;
        } else if (!handleTokenPacket(&packet)){
            //for a hot fix not token packets can handled here
            //packet for me?
                if (packet.system_id == (ARC_SYSTEM_ID) SYSTEM_ID){
                    if (!push_back(packet, &read_buffer)){
                        //Buffer overflow
                        //TODO asse
                    }
                } else {
                    printf("packet to other system %i\n", packet.system_id);
                }
                //handlePacket(&packet);
        }
    }
}
