#include "mainboardstate.h"
#include "printf.h"
#include "timeout.h"
#include "time.h"
#include "../../common/hbridge_cmd2.h"
#include "../../common/hbridge_cmd.h"
#include "../../common/protocol.h"

enum MAINBOARDSTATE currentState;

struct MainboardState mbstate_states[MAINBOARD_NUM_STATES];

uint8_t mbstate_defaultToHandler()
{
    //allways fail
    return 1;
}

void mbstate_defaultHandler()
{
    //do nothing
}



uint8_t mbstate_toOffHandler()
{
    printf("Switching to Off\n");
    
    enum STATES lowestState = hbridge_getLowestHBState();
    
    if(lowestState >= STATE_SENSORS_CONFIGURED)
    {
	hbridge_resetActuators();
	printf("ACT OFF\n");
    }
    else
    {
	hbridge_resetSensors();
	printf("ALL OFF\n");
    }
    
    hbridge_sendAllowedSenderConfiguration(RECEIVER_ID_ALL, 0);
    
    return 0;
}

uint8_t mbstate_toRunningHandler()
{
    printf("Switching to Running\n");
    
    //be shure the pc can not interrupt
    hbridge_sendAllowedSenderConfiguration(RECEIVER_ID_ALL, 0);

    if(!hbridge_configureControllers())
    {
	return 1;
    }
    return 0;
}

uint8_t mbstate_toAutonomous()
{
    enum STATES lowestState = hbridge_getLowestHBState();
    
    if(lowestState >= STATE_SENSORS_CONFIGURED)
    {
	hbridge_resetActuators();
    }
    else
    {
	//go save, switch off everything
	hbridge_resetSensors();
    }

    //allow pc to take over control
    hbridge_sendAllowedSenderConfiguration(RECEIVER_ID_ALL, 1);
    
    return 0;
}

void mbstate_autonomousHandler()
{
    //switch off if we have a timeout
    if(timeout_hasTimeout())
    {
	mbstate_changeState(MAINBOARD_OFF);
    }
}

void mbstate_init()
{
    int i;
    for(i = 0; i < MAINBOARD_NUM_STATES; i++)
    {
	mbstate_states[i].enterStateHandler = mbstate_defaultToHandler;
	mbstate_states[i].stateHandler = mbstate_defaultHandler;
    }
    
    //register standard states
    mbstate_states[MAINBOARD_OFF].enterStateHandler = mbstate_toOffHandler;
    mbstate_states[MAINBOARD_RUNNING].enterStateHandler = mbstate_toRunningHandler;
    mbstate_states[MAINBOARD_AUTONOMOUS].enterStateHandler = mbstate_toAutonomous;
    mbstate_states[MAINBOARD_AUTONOMOUS].stateHandler = mbstate_autonomousHandler;
}



uint8_t mbstate_changeState(enum MAINBOARDSTATE newState)
{
    if(currentState != newState)
    {
	if(mbstate_states[newState].enterStateHandler())
	{
	    printf("Warning state switch failed");
	    return 1;
	}
	
	currentState = newState;
    }
    return 0;
}

enum MAINBOARDSTATE mbstate_getCurrentState()
{
    return currentState;
}

struct MainboardState* mbstate_getState(enum MAINBOARDSTATE state)
{
    return &(mbstate_states[state]);
}

void mbstate_processState()
{
    //call state handler
    mbstate_states[currentState].stateHandler();
}