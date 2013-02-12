#include "../common/mainboardstate.h"
#include "../common/packethandling.h"
#include "../common/arc_driver.h"
#include "../common/time.h"
#include "../common/timeout.h"
#include "../../common/hbridge_cmd2.h"
#include "../../hbridgeCommon/drivers/usart.h"



struct MotionCommand curCmd;
uint8_t cmdValid;

/**
 * Handler for COMMAND packages
 * */
void asguard_cmdHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size)
{
    struct MotionCommand *cmd = (struct MotionCommand *) data;
    curCmd = *cmd;
    cmdValid = 1;
}

/**
 * Handler that gets called if in running state
 * */
void asguard_runningState(void)
{
    if(timeout_hasTimeout())
    {
	mbstate_changeState(MAINBOARD_OFF);
    }

    //check actuators

    //do nothing until we got a command packet
    if(!cmdValid)
	return;

    
    //set actuator cmd
//     int left = control_data->speed * SPEED_FACTOR *
// 	(1000+1000*control_data->direction*DIRECTION_RADIUS)/1000 + control_data->turn *TURN_SPEED;
// 
//     int right = control_data->speed * SPEED_FACTOR * 
// 	(1000-1000*control_data->direction*DIRECTION_RADIUS)/1000 - control_data->turn *TURN_SPEED;
//     
//     //hinten links, hinten rechts, vorne rechts, vorne links
//     set_value_to_hbridge(-(left), right, right, -(left));


}



int main()
{
    USART1_Init(USART_USE_INTERRUPTS);
    
    arc_init(USART1_SendData, USART1_GetData);
    
    mbstate_init();
    packet_init();

    packet_registerHandler(MB_CONTROL, asguard_cmdHandler);
 
    ///Overload the state handler for running
    struct MainboardState *state=mbstate_getState(MAINBOARD_RUNNING);
    state->stateHandler=asguard_runningState;
    
    unsigned int lastStateTime = -1;
    
    while(1)
    {
	unsigned int curTime = time_getTimeInMs();
	
	//only call state processing every ms
	if(curTime != lastStateTime)
	{
	    //process state handlers
	    mbstate_processState();
	    
	    lastStateTime = curTime;
	}

	//process incomming packets
	packet_handlePacket();	
    }
    
    return 0;
}