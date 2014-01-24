#include "uwmodem.h"
#include "avalon_can.h"
#include "../common/mainboardstate.h"
#include "../common/packethandling.h"
#include "../common/arc_packet.h"
#include "../common/time.h"
#include "../common/timeout.h"
#include "../common/arc_driver.h"
#include "../../common/hbridge_cmd2.h"
#include "../../common/hbridge_cmd.h"
#include "../../common/protocol.h"
#include "../../interfaces/thread.h"
#include "../../hbridgeCommon/protocol_can.h"
#include "../../hbridgeCommon/drivers/can.h"
#include "../../hbridgeCommon/drivers/usart.h"
#include "../../hbridgeCommon/drivers/assert.h"
#include "../../hbridgeCommon/drivers/printf.h"
#include <stddef.h>

#define STATUS_PACKET_PERIOD 200000 


//------- Importtand Defines ----------//
#define SYSTEM_ID AVALON
#define NUM_MOTORS 6
#define SURFACE_SIGN 0xFF
#define SURFACE_SIGN_COUNT 6 

//USART2 = AMBER
//USART3 = Ethernet/Serial
//USART5 = Underwater Modem
//TODO Add CAN IDs to keep a easy overview
//TODO Where co configure motor commands and set them to the motors


struct arc_avalon_control_packet{
    uint8_t dive;
    uint8_t strave;
    uint8_t left;
    uint8_t right;	
    uint8_t pitch;	
    uint8_t yaw;	
} __attribute__ ((packed)) __attribute__((__may_alias__));

void sendStatusPacket(){
    arc_status_packet_t status_information;
    status_information.current_state = (ARC_SYSTEM_STATE)mbstate_getCurrentState();
    status_information.wanted_state = (ARC_SYSTEM_STATE)mbstate_getCurrentState();
    arc_packet_t packet;
    packet.originator = SLAVE;
    packet.system_id = SYSTEM_ID;
    packet.packet_id = MB_STATUS;
    packet.length = sizeof(arc_status_packet_t);
    int i;
    for (i=0; i<sizeof(arc_status_packet_t); i++){
        packet.data[i] = ((char*)&status_information)[i]; 
    }
    //printf("send status packet\n");
    arc_sendPacket(&packet);
}


int status_loops = 0;
uint8_t surface_sign_counter= 0;
struct arc_avalon_control_packet curCmd;
uint8_t cmdValid;
void avalon_controlHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size){
    struct arc_avalon_control_packet *cmd  = (struct arc_avalon_control_packet *) data;
    curCmd = *cmd;
    cmdValid = 1;
}

void avalon_runningState(void){
    if(timeout_hasTimeout())
    {
	printf("Timout, switching to off\n");
	mbstate_changeState(MAINBOARD_OFF);
    }
    
    //check actuators
    if(hbridge_hasActuatorError())
    {
	printf("Actuator error, switching to off\n");
	mbstate_changeState(MAINBOARD_OFF);
        return;
    }

    if (!cmdValid)
        return;

    //TODO mapping korrigieren
    hbridge_setValues(
            (curCmd.strave-127)*4, 
            (curCmd.dive-127)*4, 
            (curCmd.left-127)*4, 
            (curCmd.right-127)*4,
            PACKET_ID_SET_VALUE14);

    hbridge_setValues(
            (curCmd.pitch-127)*4, 
            (curCmd.yaw-127)*4,
            0,
            0,
            PACKET_ID_SET_VALUE58);
};


//TODO Add this function to be executed and adapt it to compile ;)
//Don't forget to initialize the CANid and the USART
uint8_t handle_underwater_modem() {
  int seek, i;
  u8 packet_buffer[ARC_MAX_FRAME_LENGTH];

  // look for a valid packet as long as enough bytes are left
  while( (seek = UART5_SeekData(packet_buffer, ARC_MAX_FRAME_LENGTH)) >= 1 ) {

    //seek =  UART5_SeekData(packet_buffer, ARC_MAX_FRAME_LENGTH);
    int size = seek;
    while( size > 0 ) {
	int fragment_size = 8;

	if(size < 8)
	  fragment_size = size;

	printf("Readed %hi bytes\r\n",seek);
	CanTxMsg inputMessage;
	inputMessage.StdId=0x1E0; //TODO Hardcoded Modem value
	inputMessage.RTR=CAN_RTR_DATA;
	inputMessage.IDE=CAN_ID_STD;
	inputMessage.DLC= fragment_size;
	
	size -= fragment_size;

	for(i=0;i<fragment_size;i++) {
	    inputMessage.Data[i] = packet_buffer[i];
	    if(packet_buffer[i] == SURFACE_SIGN){ //TODO Extend packed
                if(++surface_sign_counter == SURFACE_SIGN_COUNT){
        	    	//print("diving up\n");
                        //TODO IMPORTAND CHANGE STATE TO SURFACE
        	    	//wanted_system_state = SURFACE;
                        surface_sign_counter=0;
                }
	    }else{
                surface_sign_counter=0;
            }
	}
	int counter=0;
	

        //TODO What's this:?
        /*
	while(CAN_Transmit(&inputMessage) == CAN_NO_MB) {
	    counter++;
	    if (counter > CAN_SEND_RETRIES) {
    		UART5_GetData(packet_buffer, size);
	    	printf("Giving up %hi\r\n",counter);
		return 1; //flase
	   }
	}
        */
    }
    UART5_GetData(packet_buffer, seek);
  }
  return 0; //true
}


int main()
{
    Assert_Init(GPIOA, GPIO_Pin_12, USE_USART1);

    USART1_Init(USART_POLL);
    printf_setSendFunction(USART1_SendData);

    //ARC's
    USART2_Init(USART_USE_INTERRUPTS);
    USART3_Init(USART_USE_INTERRUPTS);

    //Modem
    USART5_Init(USART_USE_INTERRUPTS);
    uwmodem_init(&USART5_SendData, &USART5_GetData, &USART5_SeekData);
    
    printf("The Maiboard is up with the version: ");
    
    timeout_init(300000);

    protocol_setOwnHostId(SENDER_ID_MAINBOARD);
    
    CAN_Configuration(CAN_REMAP1);
    CAN_ConfigureFilters(0);

    startHardPeriodicThread(1000, time_msPassed);
    uint32_t lastStateTime = time_getTimeInMs();

    can_protocolInit();

    //wait till rest got up
    while(time_getTimeInMs() - lastStateTime < 30);
    protocol_init(1);
    protocol_registerHandler(PACKET_ID_STATUS, &hbridgeStatusHandler);
    protocol_registerHandler(PACKET_ID_EXTENDED_STATUS, &hbridgeExtendedStatusHandler);
    protocol_setRecvFunc(avaloncan_recvPacket);
    hbridge_init(NUM_MOTORS);
    unsigned int i; 
    for(i = 0;i<NUM_MOTORS; i++){
        hbridge_setControllerWithData(i, CONTROLLER_MODE_PWM, 0, NULL, 0);
    }
   
    arc_init(&USART2_SendData, &USART2_GetData, &USART2_SeekData);
    arc_add_serial_handler(&USART3_SendData, &USART3_GetData, &USART3_SeekData);

    mbstate_init();
    packet_init();

    packet_registerHandler(MB_CONTROL, avalon_controlHandler);

    ///Overload the state handler for running
    struct MainboardState *state=mbstate_getState(MAINBOARD_RUNNING);
    //Functionpointer of the running_state
    state->stateHandler=avalon_runningState;
    while(1)
    {
      
	uwmodem_process();
	unsigned int curTime = time_getTimeInMs();
        //only call state processing every ms
	if(curTime != lastStateTime)
	{
	    //process state handlers
	    mbstate_processState();
	    
	    lastStateTime = curTime;
	    
// 	    printf(".");
	}
	arc_packet_t packet;	
	
	while(arctoken_readPacket(&packet)){
	  //Process packets
	  printf("incoming packets");
	  packet_handlePacket(packet.originator, packet.system_id, packet.packet_id, packet.data, packet.length);
	}
	arctoken_processPacket();	  

        hbridge_process();
        
	uwmodem_process();
    //Sending a Status packet
    if (status_loops >= STATUS_PACKET_PERIOD){
         sendStatusPacket();
         status_loops = 0;
     } else {
         status_loops++;
     }
    }
    return 0;
}

