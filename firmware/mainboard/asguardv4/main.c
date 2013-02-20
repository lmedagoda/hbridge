#include "../common/mainboardstate.h"
#include "../common/packethandling.h"
#include "../common/arc_packet.h"
#include "../common/time.h"
#include "../common/timeout.h"
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



struct AsguardControlData {
  int8_t speed; 
  int8_t direction;
  int8_t turn;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct AsguardControlData curCmd;
uint8_t cmdValid;

/**
 * Handler for COMMAND packages
 * */
void asguard_cmdHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size)
{
    struct AsguardControlData *cmd = (struct AsguardControlData *) data;
    curCmd = *cmd;
    cmdValid = 1;
}

#define SPEED_FACTOR 314/100
#define DIRECTION_RADIUS 6/1000
#define TURN_SPEED 20/100  

/**
 * Handler that gets called if in running state
 * */
void asguard_runningState(void)
{
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
    }

    //do nothing until we got a command packet
    if(!cmdValid)
	return;

    
    //set actuator cmd
    int left = curCmd.speed * SPEED_FACTOR *
	(1000+1000*curCmd.direction*DIRECTION_RADIUS)/1000 + curCmd.turn *TURN_SPEED;

    int right = curCmd.speed * SPEED_FACTOR * 
	(1000-1000*curCmd.direction*DIRECTION_RADIUS)/1000 - curCmd.turn *TURN_SPEED;
	
    hbridge_setValue(-left, right, right, -left);
}

int main()
{
        //setup assert correctly
    Assert_Init(GPIOA, GPIO_Pin_12, USE_USART3);

    USART3_Init(USART_POLL);

    printf_setSendFunction(USART3_SendData);
    
    USART1_Init(USART_USE_INTERRUPTS);
    
    printf("START: Up and running\n");
    
    timeout_init(3000);
    
    protocol_init(1);
    protocol_setOwnHostId(SENDER_ID_MAINBOARD);
    
    CAN_Configuration(CAN_REMAP1);
    CAN_ConfigureFilters(0);
    
    startHardPeriodicThread(1000, time_msPassed);
    
    can_protocolInit();

    uint32_t lastStateTime = time_getTimeInMs();
    
    //wait till rest got up
    while(time_getTimeInMs() - lastStateTime < 30)
	;
    
    hbridge_init(4);
    
    hbridge_setControllerWithData(0, CONTROLLER_MODE_PWM, 0, NULL, 0);
    hbridge_setControllerWithData(1, CONTROLLER_MODE_PWM, 0, NULL, 0);
    hbridge_setControllerWithData(2, CONTROLLER_MODE_PWM, 0, NULL, 0);
    hbridge_setControllerWithData(3, CONTROLLER_MODE_PWM, 0, NULL, 0);
    
    
//     arc_init(USART1_SendData, USART1_GetData);
    
    arc_init(&USART1_SendData, &USART1_GetData, &USART1_SeekData);
    mbstate_init();
    packet_init();

    packet_registerHandler(MB_CONTROL, asguard_cmdHandler);
 
    ///Overload the state handler for running
    struct MainboardState *state=mbstate_getState(MAINBOARD_RUNNING);
    state->stateHandler=asguard_runningState;
    
    while(1)
    {
	unsigned int curTime = time_getTimeInMs();
	
	//only call state processing every ms
	if(curTime != lastStateTime)
	{
	    //process state handlers
	    mbstate_processState();
	    
	    lastStateTime = curTime;
	    
// 	    printf(".");
	}

	//process arc packets
// 	arc_processPackets();
	
	arc_packet_t packet;	
	
	while(arc_readPacket(&packet))
	{
	    //process incomming packet
	    packet_handlePacket(packet.originator, packet.system_id, packet.packet_id, packet.data, packet.length);	
	}
	
	//process hbridge driver
	hbridge_process();
    }
    
    return 0;
}
