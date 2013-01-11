#include "run.h"
#include "state.h"
#include "arc_driver.h"
#include "../common/hbridge_cmd.h"
#include "../hbridgeCommon/drivers/printf.h"

#define SYSTEM_ID 2 //ASV
#define STATUS_PACKET_PERIOD 100
volatile extern arc_asv_control_packet_t motor_command;
void sendStatusPacket();
void runningMotors();
int status_loops = 0;
void mainboard_run(){
     //If I'm not in the state i want to. it's better to do nothing
     if (currentState.mainboardstate == wantedState.mainboardstate){
         switch (currentState.mainboardstate){
             case RUNNING:
                 runningMotors();
                 break;
             case OFF:
                 break;
             case AUTONOMOUS:
                 printf("AUTONOMOUS\n");
                 break;
             case FULL_AUTONOMOUS:
                 printf("FULL_AUTONOMOUS\n");
                 break;
             case HALT:
                 printf("The State HALT ist deprecated\n");
                 break;
             case ERROR_HBRIDGE:
                 printf("ERROR\n");
                 //TODO check states of hbridges, is there still an error
                 break;
             case ERROR_CAN:
                 printf("ERROR\n");
                 //TODO check Can-Bus, is there still an errr
                 break;
             case ERROR_WRONG_STATE:
                 printf("ERROR\n");
                 break;
             default:
                 printf("Warning: The Mainboard is in a unhandled state\n");
         }
     } 
     //Sending a Status packet
     if (status_loops >= STATUS_PACKET_PERIOD){
         sendStatusPacket();
         status_loops = 0;
     } else {
         status_loops++;
     }
}
void sendStatusPacket(){
    arc_status_packet_t status_information;
    status_information.current_state = currentState.mainboardstate;
    status_information.wanted_state = wantedState.mainboardstate;
    arc_packet_t packet;
    packet.originator = SLAVE;
    packet.system_id = (ARC_SYSTEM_ID) SYSTEM_ID;
    packet.packet_id = STATUS;
    packet.length = sizeof(arc_status_packet_t);
    int i;
    for (i=0; i<sizeof(arc_status_packet_t); i++){
        packet.data[i] = ((char*)&status_information)[i]; 
    }
    amber_sendPacket(&packet);
}
void runningMotors(){
   //This will be not nessesary, if the hbridges are really tested             
        
        /*printf("MOTORENWERTE: %i, %i, %i, %i \n", 
                (motor_command.quer_vorne-127)/5,
                (motor_command.quer_hinten-127)/5,
                (motor_command.motor_rechts-127)/5,
                (motor_command.motor_links-127)/5);*/
        
        hbridge_setValue(
                //printf("MOTORENWERTE: %i, %i, %i, %i \n", 
                (motor_command.motor_rechts-127),
                (motor_command.motor_links-127),
                (motor_command.quer_hinten-127),
                (motor_command.quer_vorne-127));

}
