#include "arc_driver.h"
#include "tokenhandling.h"
#include "arc_ringbuffer.h"
#include "bool.h"
#include "usart.h"
#include "printf.h"
#define MAX_BYTES 30

RING_BUFFER read_buffer;
RING_BUFFER* send_buffer;
volatile bool has_token = FALSE;
volatile AMBER_STATE amber_state = AMBER_UNREGISTERED;
int stat_bad_packets = 0;
int send_bytes = 0;

//privats
int amber_newreadPacket(arc_packet_t* packet);
void receivePackets();
void sendPendingPackets();
int amber_readPacket(arc_packet_t* packet); 
void receivePackets();
void giveTokenBack();
//public
void initAmber(){
    initTokenhandling();
}
int amber_getPacket(arc_packet_t* packet){
    packet = pop_front(&read_buffer);
    if (!packet){
        return 0;
    }
    return 1;
}

int amber_sendPacket(arc_packet_t* packet){
    return push_back(*packet, &read_buffer); 
}

void amber_processPackets(){
    receivePackets();
    sendPendingPackets();
    return;
}

//privat
int amber_readPacket(arc_packet_t* packet) {
    int len, ret;
    unsigned char packet_buffer[ARC_MAX_FRAME_LENGTH];
    while((ret = USART1_GetData(packet_buffer, 1))>0){
        int idx = 1;
        if (packet_buffer[0] == SYNC_MARKER){
            while (idx < ARC_MAX_FRAME_LENGTH){
                len=USART1_GetData(packet_buffer+idx, 1);
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
            printf("IDX: %i \n", idx);
            printf("A BAD PACKET\n");
            return -1;
        }
    }
    return 0;
}

unsigned char packet_buffer[ARC_MAX_FRAME_LENGTH];
int idx = 0;
int amber_newreadPacket(arc_packet_t* packet){
    int ret;
    printf("IDX: %i \n",idx);
    while((ret = USART1_GetData(packet_buffer+idx, 1))>0){
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
                printf("noch nicht genug gelesen");
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

void sendPendingPackets(){
    arc_packet_t packet;
    int len = 0;
    uint8_t tmp_send_buffer[ARC_MAX_FRAME_LENGTH];
    if (has_token){
        /*while (first(&packet, send_buffer)){
           len = createPacket(&packet, tmp_send_buffer); 
           if (MAX_BYTES-send_bytes > len){
               pop_front(&packet, send_buffer);
               USART1_SendData(tmp_send_buffer, len);
           } else {
               break;
           }
        }*/
        giveTokenBack();
    }
}

void sendProtocolPacket(ARC_PACKET_ID id){
    uint8_t send_buffer[ARC_MAX_FRAME_LENGTH];
    arc_packet_t packet;
    packet.originator = SLAVE;
    packet.system_id = (ARC_SYSTEM_ID) MY_SYSTEM_ID; 
    packet.packet_id = id; 
    packet.length = 0;
    
    int len = createPacket(&packet, send_buffer);
    USART1_SendData(send_buffer, len);
}

void giveTokenBack(){
    has_token = FALSE;
    sendProtocolPacket(GIVE_BACK);
}

void receivePackets(){
    //printf("RECEIVE PACKETS\n");
    arc_packet_t packet;
    int32_t result;
    while ((result = amber_newreadPacket(&packet)) != 0){
        printf("RESULT: %i \n" , result);
        if (result<0){
            printf("Got an error by reading Packets");
            break;
        } else if (handleTokenPacket(&packet)== FALSE){
            printf("Das war kein Token packet\n");
            handlePacket(&packet); 
            //for a hot fix not token packets can handled here
            /*if (!push_back(&packet, &read_buffer)){
                //Buffer overflow
                //TODO assert
            }*/
//            handlePacket(&packet);
        }
        printf ("WEDER IF NOCH ELSE \n");
    }

   // printf("%i\n", result);
}
