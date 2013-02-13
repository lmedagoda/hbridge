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
    enum STATES lowestState = hbridge_getLowestHBState();
    
    if(lowestState >= STATE_SENSORS_CONFIGURED)
    {
	hbridge_resetActuators();
    }
    else
    {
	hbridge_resetSensors();
    }
    
    hbridge_sendAllowedSenderConfiguration(RECEIVER_ID_ALL, 0);
    
    return 0;
}

uint8_t mbstate_toRunningHandler()
{
    //be shure the pc can not interrupt
    hbridge_sendAllowedSenderConfiguration(RECEIVER_ID_ALL, 0);
    
    enum STATES lowestState = hbridge_getLowestHBState();
    
    if(lowestState < STATE_SENSORS_CONFIGURED)
    {
	if(!hbridge_configureSensors())
	{
	    return 1;
	}
    }

    if(!hbridge_configureActuators())
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
    }
    return 0;
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