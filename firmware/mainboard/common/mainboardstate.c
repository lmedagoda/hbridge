#include "mainboardstate.h"
#include "printf.h"
#include "timeout.h"
#include "time.h"
#include "../../common/hbridge_cmd2.h"
#include "../../common/hbridge_cmd.h"
#include "../../common/protocol.h"
#include "stm32f10x_gpio.h"

enum MAINBOARDSTATE currentState;

struct MainboardState mbstate_states[MAINBOARD_NUM_STATES];

uint8_t mb_blinkLedSet = 0;
GPIO_TypeDef* mbstate_BlinkLedBank;
uint16_t mbstate_BlinkLedPin;

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

void mbstate_setBlinkLed(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    mb_blinkLedSet = 1;
    
    mbstate_BlinkLedBank = GPIOx;
    mbstate_BlinkLedPin = GPIO_Pin;
    
    GPIO_InitTypeDef GPIO_InitStructure;

    //get default GPIO config
    GPIO_StructInit(&GPIO_InitStructure);

    if(mbstate_BlinkLedBank == GPIOC)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    }
    else
    {
        printf("Error, Activation GPIO bank is not implemented for given bank\n");
        assert_param(0);
    }
    
    GPIO_WriteBit(mbstate_BlinkLedBank,mbstate_BlinkLedPin,Bit_SET);
    GPIO_InitStructure.GPIO_Pin = mbstate_BlinkLedPin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(mbstate_BlinkLedBank, &GPIO_InitStructure);

    GPIO_SetBits(mbstate_BlinkLedBank, mbstate_BlinkLedPin);
}

void mbstate_generateBlinkCode()
{
    if(!mb_blinkLedSet)
        return;
    
    static unsigned int lastBlinkTime = 0;
    unsigned int curTime = time_getTimeInMs();
    
    static uint8_t blinksLeft = 0;
    static BitAction lastLedState = Bit_SET;
    
    if(curTime - lastBlinkTime > 500)
    {
        if(blinksLeft > 0)
        {
	    if(lastLedState == Bit_RESET)
		lastLedState = Bit_SET;
	    else
		lastLedState = Bit_RESET;
		
            GPIO_WriteBit(mbstate_BlinkLedBank, mbstate_BlinkLedPin, lastLedState);
	    blinksLeft--;
        } 
	else 
        {
            //recompute needed blinks from state
            blinksLeft = currentState * 2;
        }
        
        lastBlinkTime = curTime;
    }
}

void mbstate_processState()
{
    //give blink codes
    mbstate_generateBlinkCode();
    
    
    //call state handler
    mbstate_states[currentState].stateHandler();
}
