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

unsigned int arc_num_serial_hanlder = 0;
unsigned int arc_current_serial_handler = 0;
arc_send_func_t arc_sendFunc[5];
arc_recv_func_t arc_recvFunc[5];
arc_seek_func_t arc_seekFunc[5];

//privats
void arc_receivePackets();
void arc_sendPendingPackets();
void arc_receivePackets();
void arc_giveTokenBack();

void arc_add_serial_handler(arc_send_func_t sendFunc, arc_recv_func_t recvFunc, arc_seek_func_t seekFunc){
    arc_sendFunc[arc_num_serial_hanlder] = sendFunc;
    arc_recvFunc[arc_num_serial_hanlder] = recvFunc;
    arc_seekFunc[arc_num_serial_hanlder] = seekFunc;
    arc_num_serial_hanlder++;
}


//public
void arc_init(arc_send_func_t sendFunc, arc_recv_func_t recvFunc, arc_seek_func_t seekFunc)
{
    arc_sendFunc[0] = sendFunc;
    arc_recvFunc[0] = recvFunc;
    arc_seekFunc[0] = seekFunc;
    arc_current_serial_handler = 0;
    arc_num_serial_hanlder = 1;
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
/*
 * This Function is public only for NO token usage.
 * Do not use this funktion, if you want to use token.
 */
uint32_t arc_readPacket(arc_packet_t * packet) {
  // function will return 0 if no packet has been found
  // the number of bytes in the packet otherwise
  int seek, i,channel;
  uint8_t packet_buffer[ARC_MAX_FRAME_LENGTH];

  for(channel=0;channel<arc_num_serial_hanlder;channel++){
      // look for a valid packet as long as enough bytes are left
      while( (seek = arc_seekFunc[channel](packet_buffer, ARC_MAX_FRAME_LENGTH)) >= ARC_MIN_FRAME_LENGTH ) {

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
  }

  return 0; // no packet found
}

//privat
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
		    int ret = arc_sendFunc[arc_current_serial_handler](tmp_send_buffer, len);
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
	int ret = arc_sendFunc[arc_current_serial_handler](tmp_send_buffer, len);
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
    while ((result = arc_readPacket(&packet)) != 0){
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
