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
static s32 currentPwmValue = 0;
static u16 index = 0;
static u16 overCurrentCounter = 0;
static u16 timeoutCounter = 0;
static s32 temperature = 0;
static u32 overTempCounter = 0;
static s32 motorTemperature = 0;
static u32 overMotorTempCounter = 0;

volatile struct ControllerState cs1;
volatile struct ControllerState cs2;

void checkMotorTemperature();
void checkTimeout();
void checkTemperature();
void checkOverCurrent();

void sendStatusMessage(u32 pwmValue, u32 currentValue, s32 temperature, s32 motorTemperature, u32 index);
void sendErrorMessage(u32 pwmValue, u32 currentValue, s32 temperature, s32 motorTemperature, u32 index);

void resetCounters()
{
    overCurrentCounter = 0;
    timeoutCounter = 0;
    overTempCounter = 0;
    overMotorTempCounter = 0;
}

void checkTimeout()
{	
    //check for timeout and go into error state
    if(activeCState->timeout && (timeoutCounter > activeCState->timeout)) {
	getErrorState()->timeout = 1;
    }
}

void checkOverCurrent(u32 currentValue)
{
    //check for overcurrent
    if(currentValue > activeCState->maxCurrent) {
	overCurrentCounter++;

	if(overCurrentCounter > activeCState->maxCurrentCount) {
	    getErrorState()->overCurrent = 1;
	}
    } else {
	overCurrentCounter = 0;
    }
}

void checkMotorTemperature(u32 temperature)
{
    if(!activeCState->useExternalTempSensor)
	return;
	
    //check for overcurrent
    if(motorTemperature > activeCState->maxMotorTemp) {
	overMotorTempCounter++;

	if(overMotorTempCounter > activeCState->maxMotorTempCount) {
	    getErrorState()->motorOverheated = 1;
	}
    } else {
	overMotorTempCounter = 0;
    }
}

void checkTemperature(u32 temperature)
{
    //check for overcurrent
    if(temperature > activeCState->maxBoardTemp) {
	overTempCounter++;

	if(overTempCounter > activeCState->maxBoardTempCount) {
	    getErrorState()->boardOverheated = 1;
	}
    } else {
	overTempCounter = 0;
    }
}

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
    currentPwmValue = 0;
    index = 0;
    overCurrentCounter = 0;
    timeoutCounter = 0;
    temperature = 0;
    overTempCounter = 0;
    motorTemperature = 0;
    overCurrentCounter = 0;
    

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

	checkTimeout();
	checkTemperature(temperature);
	checkMotorTemperature(motorTemperature);
	checkOverCurrent(currentValue);

	if(inErrorState())
	{
	    activeCState->internalState = STATE_ERROR;
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
		    controllers[CONTROLLER_MODE_SPEED].setDebugActive(activeCState->enablePIDDebug);
		    pwmValue = controllers[CONTROLLER_MODE_SPEED].step(activeCState->targetValue, wheelPos, ticksPerTurn);
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

	sendStatusMessage(currentPwmValue, currentValue, temperature, motorTemperature, index);
	
	//increase timeout
	timeoutCounter++;

	//set pwm
	setNewPWM(currentPwmValue, activeCState->useOpenLoop);
    } else {
	currentPwmValue = 0;
	
	//send error message
	if(activeCState->internalState == STATE_ERROR) {
	    sendErrorMessage(currentPwmValue, currentValue, temperature, motorTemperature, index);
	}
    
	//reset to zero values
	resetCounters();
	setNewPWM(currentPwmValue, activeCState->useOpenLoop);

	//reset PID struct, to avoid bad controller 
	//behavior an reactivation due to big I part
	resetControllers(wheelPos);
    }
}

void sendErrorMessage(u32 pwmValue, u32 currentValue, s32 temperature, s32 motorTemperature, u32 index)
{
    //send status message over CAN
    CanTxMsg errorMessage;
    errorMessage.StdId= PACKET_ID_ERROR + ownHostId;
    errorMessage.RTR=CAN_RTR_DATA;
    errorMessage.IDE=CAN_ID_STD;
    errorMessage.DLC= sizeof(struct errorData);
    
    struct errorData *edata = (struct errorData *) errorMessage.Data;

    volatile struct ErrorState *es = getErrorState();

    if(temperature < 0)
	temperature = 0;

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
//     CAN_CancelAllTransmits();

    if(CAN_SendMessage(&errorMessage)) {
	print("Error Tranmitting status Message : No free TxMailbox \n");
    } else {
    //print("Tranmitting status Message : OK \n");  
    }
}

void sendStatusMessage(u32 pwmValue, u32 currentValue, s32 temperature, s32 motorTemperature, u32 index)
{
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
//     CAN_CancelAllTransmits();
    
    if(CAN_SendMessage(&statusMessage)) {
	print("Error Tranmitting status Message : No free TxMailbox \n");
    } else {
	//print("Tranmitting status Message : OK \n");  
    }
    
    //not this is not exact every 100 ms, but good enough
    if(index % 100 == 0)
    {
	CanTxMsg extendedStatusMessage;
	extendedStatusMessage.StdId= PACKET_ID_EXTENDED_STATUS + ownHostId;
	extendedStatusMessage.RTR=CAN_RTR_DATA;
	extendedStatusMessage.IDE=CAN_ID_STD;
	extendedStatusMessage.DLC= sizeof(struct extendedStatusData);
	
	struct extendedStatusData *esdata = (struct extendedStatusData *) extendedStatusMessage.Data;
	if(temperature < 0)
	    temperature = 0;
	esdata->temperature = temperature;
	    
	if(motorTemperature < 0)
	    motorTemperature = 0;
	esdata->motorTemperature = motorTemperature;
	if(CAN_SendMessage(&extendedStatusMessage)) {
	    print("Error Tranmitting status Message : No free TxMailbox \n");
	} else {
	    //print("Tranmitting status Message : OK \n");  
	}
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
