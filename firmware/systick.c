#include "inc/stm32f10x_can.h"
#include "inc/stm32f10x_gpio.h"
#include "inc/stm32f10x_systick.h"
#include "inc/stm32f10x_nvic.h"
#include "spi.h"
#include "i2c.h"
#include "pid.h"
#include "usart.h"
#include "protocol.h"
#include "can.h"
#include "state.h"
#include "controllers.h"
#include "hbridge.h"
#include "encoder.h"
#include "current_measurement.h"
#include "lm73cimk.h"
#include "printf.h"
#include <stdlib.h>

#define MIN_S16 (-(1<<15))
#define MAX_S16 ((1<<15) -1)

volatile enum hostIDs ownHostId;

unsigned int systemTick;
static u32 motorTemperature = 0;
volatile struct ControllerState cs1;
volatile struct ControllerState cs2;

void baseNvicInit()
{
    /* Set the Vector Table base location at 0x08000000 */ 
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   

    /* 2 bit for pre-emption priority, 2 bits for subpriority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* Systick with Preemption Priority 2 and Sub Priority as 0 */ 
    NVIC_SystemHandlerPriorityConfig(SystemHandler_SysTick, 3, 0);    
}

void baseInit()
{
    systemTick = 0;
    motorTemperature = 0;

    //read out dip switches
    ownHostId = getOwnHostId();

    hbridgeInit();

    currentMeasurementInit();

    encodersInit();
    
    activeCState = &(cs1);
    lastActiveCState = &(cs2);

    initControllers();

    //init cotroller state with sane values
    initStateStruct(activeCState);
    initStateStruct(lastActiveCState);

    //clear all errors, btw initialize error state struct
    clearErrors();
}

void pollCanMessages() 
{
    static u16 cnt = 0;

    CanRxMsg *curMsg;
    
    //check if we got a new message
    if((curMsg = CAN_GetNextData()) != 0) 
    {
	//print("Got a Msg \n");
	print("P");

	u8 forceSynchronisation = updateStateFromMsg(curMsg, lastActiveCState, ownHostId);
	
	//mark current message als processed
	CAN_MarkNextDataAsRead();

	//this is concurrency proof, as this code can not run, while
	//systick Handler is active !
	volatile struct ControllerState *tempstate = activeCState;

	//we go an new packet, so reset the timeout
	lastActiveCState->resetTimeoutCounter = 1;
	
	//this is atomar as only the write is relevant!
	activeCState = lastActiveCState;

	lastActiveCState = tempstate;
	
	*lastActiveCState = *activeCState;      
	
	if(forceSynchronisation) {
	    while(activeCState->resetTimeoutCounter)
		;
	}

	u16 errorDbg = inErrorState();
	if((curMsg->StdId != PACKET_ID_SET_VALUE14 && curMsg->StdId != PACKET_ID_SET_VALUE58)  || cnt == 50) {
	    cnt = 0;  
	    printf("Error is %hu \n", errorDbg);
	    print("ActiveCstate: ");
	    printStateDebug(activeCState);
	    printf("P:%hi I:%hi D:%hi \n", activeCState->speedPIDValues.kp,  activeCState->speedPIDValues.ki,  activeCState->speedPIDValues.kd );
	    print(" LastActiveCstate: ");
	    printStateDebug(lastActiveCState);
	    printf("P:%hi I:%hi D:%hi \n", lastActiveCState->speedPIDValues.kp,  lastActiveCState->speedPIDValues.ki, lastActiveCState->speedPIDValues.kd );
	} else {
	    cnt++;
	}
    } 
}

void SysTickHandler(void) {  
    ++systemTick;
    //request switch of adc value struct
    requestNewADCValues();

    static s32 currentPwmValue = 0;
    static u16 index = 0;
    static u16 overCurrentCounter = 0;
    static u16 timeoutCounter = 0;
    static u32 temperature = 0;

    s32 pwmValue = 0;
    s32 wheelPos = 0;
    u32 ticksPerTurn = 0;
    u8 tickDivider = 0;
    
    switch(activeCState->controllerInputEncoder) {
        case INTERNAL:
            wheelPos = getTicks(activeCState->internalEncoder);
            ticksPerTurn = getTicksPerTurn(activeCState->internalEncoder);
            tickDivider = getTickDivider(activeCState->internalEncoder);
            break;
        case EXTERNAL:
            wheelPos = getTicks(activeCState->externalEncoder);
            ticksPerTurn = getTicksPerTurn(activeCState->externalEncoder);
            tickDivider = getTickDivider(activeCState->externalEncoder);
            break;
    }


    index++;
    if(index >= (1<<10))
	index = 0;

    setNewSpeedPIDValues(activeCState->speedPIDValues.kp, activeCState->speedPIDValues.ki, activeCState->speedPIDValues.kd, activeCState->speedPIDValues.minMaxPidOutput);
    setNewPosPIDValues(activeCState->positionPIDValues.kp, activeCState->positionPIDValues.ki, activeCState->positionPIDValues.kd, activeCState->positionPIDValues.minMaxPidOutput);
  
    //wait for adc struct to be switched
    waitForNewADCValues();

    u32 currentValue = calculateCurrent(currentPwmValue);

    // get temperature
    lm73cimk_getTemperature(LM73_SENSOR1,&temperature);

    if(activeCState->useExternalTempSensor)
    {
	lm73cimk_getTemperature(LM73_SENSOR2,&motorTemperature);
    }
    
    //reset timeout, if "userspace" requested it
    if(activeCState->resetTimeoutCounter) {
	activeCState->resetTimeoutCounter = 0;
	timeoutCounter = 0;
    }

    //change state to error if error is set
    if(inErrorState() && (activeCState->internalState == STATE_CONFIGURED || activeCState->internalState == STATE_GOT_TARGET_VAL)) {
	activeCState->internalState = STATE_ERROR;
    }

    //switch into error stat if GPIO is pulled low
    //this should securly switch off the Hbridge
    if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)) {
	activeCState->internalState = STATE_ERROR;
	getErrorState()->hardwareShutdown = 1;
    }

    //only check for overcurrent if configured
    if(activeCState->internalState == STATE_CONFIGURED || activeCState->internalState == STATE_GOT_TARGET_VAL) {
	//check for overcurrent
	if(currentValue > activeCState->maxCurrent) {
	    overCurrentCounter++;

	    if(overCurrentCounter > activeCState->maxCurrentCount) {
		activeCState->internalState = STATE_ERROR;
		getErrorState()->overCurrent = 1;
	    }
	} else {
	    overCurrentCounter = 0;
	}
	
	//check for timeout and go into error state
	if(activeCState->timeout && (timeoutCounter > activeCState->timeout)) {
	    activeCState->internalState = STATE_ERROR;
	    getErrorState()->timeout = 1;
	}
    
	//only run controllers if we got an target value
	if(activeCState->internalState == STATE_GOT_TARGET_VAL) {
	    //calculate pwm value
	    switch(activeCState->controllMode) {
		case CONTROLLER_MODE_PWM:
		    pwmValue = activeCState->targetValue;
		    break;
		
		case CONTROLLER_MODE_POSITION: {
                    //the target value needs to be multiplied by the tickDivider
                    //in case of position mode, as the "external" and "internal" ticks 
                    //differ by the factor of tickDivider
                    //TargetValue is divided by TickDivider
                    //wheelPos is not divided
                    u32 targetPositionValue = activeCState->targetValue * tickDivider;
                    if(activeCState->cascadedPositionController) {
			pwmValue = cascadedPositionController(targetPositionValue, wheelPos, ticksPerTurn, activeCState->enablePIDDebug);
		    } else {
			controllers[CONTROLLER_MODE_POSITION].setDebugActive(activeCState->enablePIDDebug);
			pwmValue =  controllers[CONTROLLER_MODE_POSITION].step(targetPositionValue, wheelPos, ticksPerTurn);
		    }
		    break;
		}   
		case CONTROLLER_MODE_SPEED:
		    pwmValue = speedController(activeCState->targetValue, wheelPos, ticksPerTurn, activeCState->enablePIDDebug);
		    break; 
		default:
		    pwmValue = 0;
		    break;
	    }

	    //trunkcate to s16
	    if(pwmValue > MAX_S16) 
		pwmValue = MAX_S16;
	    if(pwmValue < MIN_S16)
		pwmValue = MIN_S16;

	    if(abs(currentPwmValue - pwmValue) < activeCState->pwmStepPerMillisecond) {
		currentPwmValue = pwmValue;
	    } else {
		if(currentPwmValue - pwmValue < 0) {
		    currentPwmValue += activeCState->pwmStepPerMillisecond;
		} else {
		    currentPwmValue -= activeCState->pwmStepPerMillisecond;      
		}
	    }
	} else {
	    currentPwmValue = 0;
	}
  
	//send status message over CAN
	CanTxMsg statusMessage;
	statusMessage.StdId= PACKET_ID_STATUS + ownHostId;
	statusMessage.RTR=CAN_RTR_DATA;
	statusMessage.IDE=CAN_ID_STD;
	statusMessage.DLC= sizeof(struct statusData);
	
	struct statusData *sdata = (struct statusData *) statusMessage.Data;
	
	sdata->pwm = currentPwmValue;
	sdata->externalPosition = getDividedTicks(activeCState->externalEncoder);
	sdata->position = getDividedTicks(activeCState->internalEncoder);
	sdata->currentValue = currentValue;
	sdata->index = index;
	
	//cancel out old messages
	CAN_CancelAllTransmits();
	
	if(CAN_Transmit(&statusMessage) == CAN_NO_MB) {
	    print("Error Tranmitting status Message : No free TxMailbox \n");
	} else {
	    //print("Tranmitting status Message : OK \n");  
	}
	
	//increase timeout
	timeoutCounter++;

	//set pwm
	setNewPWM(currentPwmValue, activeCState->useOpenLoop);
    } else {
	//send error message
	if(activeCState->internalState == STATE_ERROR) {  
	    //send status message over CAN
	    CanTxMsg errorMessage;
	    errorMessage.StdId= PACKET_ID_ERROR + ownHostId;
	    errorMessage.RTR=CAN_RTR_DATA;
	    errorMessage.IDE=CAN_ID_STD;
	    errorMessage.DLC= sizeof(struct errorData);
	    
	    struct errorData *edata = (struct errorData *) errorMessage.Data;

	    volatile struct ErrorState *es = getErrorState();

	    edata->temperature = temperature;
	    edata->position = getDividedTicks(activeCState->internalEncoder);
	    edata->index = index;
	    edata->externalPosition = getDividedTicks(activeCState->externalEncoder);
	    edata->motorOverheated = es->motorOverheated;
	    edata->boardOverheated = es->boardOverheated;
	    edata->overCurrent = es->overCurrent;
	    edata->timeout = es->timeout;
	    edata->badConfig = es->badConfig;
	    edata->encodersNotInitalized = es->encodersNotInitalized;

	    //cancel out old messages
	    CAN_CancelAllTransmits();

	    if(CAN_Transmit(&errorMessage) == CAN_NO_MB) {
		print("Error Tranmitting status Message : No free TxMailbox \n");
	    } else {
	    //print("Tranmitting status Message : OK \n");  
	    }
	}
    
	//reset to zero values
	overCurrentCounter = 0;
	timeoutCounter = 0;
	setNewPWM(0, activeCState->useOpenLoop);

	//reset PID struct, to avoid bad controller 
	//behavior an reactivation due to big I part
	resetControllers(wheelPos);
    }
}

void SysTick_Configuration(void) {
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    
    // SysTick end of count event each 1ms with input clock equal to 9MHz (HCLK/8)
    SysTick_SetReload(9000);

    // Enable SysTick interrupt
    SysTick_ITConfig(ENABLE);

    // Enable the SysTick Counter
    SysTick_CounterCmd(SysTick_Counter_Enable);
}
