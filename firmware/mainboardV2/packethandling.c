#define MAX_HANDLERS 20
#include "bool.h"
#include "packethandling.h"
#include "arc_packet.h"
#include "printf.h"
#include "state.h"
#include "arc_driver.h"
bool (*packetHandlers[MAX_HANDLERS])(arc_packet_t* packet);
int handlers = 0;
bool pingHandler(arc_packet_t* packet);
bool arc_statusHandler(arc_packet_t* packet);
bool setStateHandler(arc_packet_t* packet);
bool controlHandler(arc_packet_t* packet);
bool canHandler(arc_packet_t* packet);
bool canAckHandler(arc_packet_t* packet);
void initPackethandling(){
    registerHandler(arc_statusHandler);
    registerHandler(pingHandler);
    registerHandler(setStateHandler);
    registerHandler(controlHandler);
    registerHandler(canHandler);
    registerHandler(canAckHandler);
}
volatile arc_asv_control_packet_t motor_command;
bool handlePacket(arc_packet_t* packet){
    int i = 0;
    for (i=0; i < handlers; i++){
        if (packetHandlers[i](packet) == TRUE){
            //printf("handlePacket: %i\n", i);
            return TRUE;
        }
    }
    printf("WARNING: One received Packet was not handled");
    return FALSE;
}

void registerHandler(bool (*handler)(arc_packet_t* packet)){
    if (handlers >= MAX_HANDLERS){
        //TODO assert
        return;
    }
    packetHandlers[handlers++] = handler;
}
bool pingHandler(arc_packet_t* packet){
    if (packet->packet_id == PING){
        //TODO Ping behandeln
        printf("PING\n");
        packet->originator = SLAVE;
        amber_sendPacket(packet);
        return TRUE;
    } else {
        return FALSE;
    }
}

bool arc_statusHandler(arc_packet_t* packet){
    if (packet->packet_id == STATUS){
        printf("Mainboard has receive a Statuspacket, this should not happen...\n");
        return TRUE;
    } else {
        return FALSE;
    }
}

bool setStateHandler(arc_packet_t* packet){
    if (packet->packet_id == SET_STATE){
        ARC_SYSTEM_STATE state = (ARC_SYSTEM_STATE)packet->data[0];
        printf("State: %i\n", state);
        wantedState.mainboardstate = state;
        return TRUE;
    } else {
        return FALSE;
    }
}

bool controlHandler(arc_packet_t* packet){
    
    if (packet->packet_id == CONTROL){
        //printf("CONTROL PACKET\n");
        int i;
        //printf("EIN WERT1 %i \n", packet->data[0]-127);
        //printf("EIN WERT2 %i \n", packet->data[1]-127);
        //printf("EIN WERT3 %i \n", packet->data[2]-127);
        //printf("EIN WERT4 %i \n", packet->data[3]-127);

        for (i = 0; i <sizeof(arc_asv_control_packet_t); i++){
           ((char*) &motor_command)[i] = packet->data[i];
        }
        
        return TRUE;
    } else {
        return FALSE;
    }
}




bool canHandler(arc_packet_t* packet){
    if (packet->packet_id == ID_CAN){
        printf("CAN PACKET BEKOMMEN\n");
        return TRUE;
    } else {
        return FALSE;
    }
}

bool canAckHandler(arc_packet_t* packet){
    if (packet->packet_id == ID_CAN_ACKED){
        //TODO Can resend and ackowledge can packet 
        return TRUE;
    } else {
        return FALSE;
    }
}

