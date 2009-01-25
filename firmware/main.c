/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : main.c
* Author             : MCD Application Team
* Version            : V2.0
* Date               : 05/23/2008
* Description        : Main program body
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED 
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.***************************/


/* Includes ---------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_it.h"
#include "stm32f10x_can.h"
#include "stdio.h"
#include "spi.h"
#include "i2c.h"
#include "adc.h"
#include "pid.h"
#include "usart.h"
#include "printf.h"
#include "protocol.h"
#include "can.h"

#include <stdlib.h>

/* Private typedef --------------------------------------------------------*/
/* Private define ---------------------------------------------------------*/
/* Private macro ----------------------------------------------------------*/  

/*
#define  USARTx                     USART1
#define  GPIOx                      GPIOA
#define  RCC_APB2Periph_GPIOx       RCC_APB2Periph_GPIOA
#define  GPIO_RxPin                 GPIO_Pin_10
#define  GPIO_TxPin                 GPIO_Pin_9
*/
/* Private variables ------------------------------------------------------*/
/* Private function prototypes --------------------------------------------*/
/* Private functions ------------------------------------------------------*/
void NVIC_Configuration(void);

void GPIO_Configuration(void);

void TIM_Configuration(void);

void SPI_Configuration(void);

void ADC_Configuration(void);

void SysTick_Configuration(void);

#define MIN_S16 (-(1<<15))
#define MAX_S16 ((1<<15) -1)

//motorticks * gear reduction
//512 * 729 / 16
#define HALF_WHEEL_TURN_TICKS (23328 * 2)

enum internalState {
  STATE_UNCONFIGURED,
  STATE_CONFIG1_RECEIVED,
  STATE_CONFIG2_RECEIVED,
  STATE_CONFIGURED,
  STATE_ERROR,
};

vu8 desieredDirection = 2;
vu8 actualDirection = 0;
volatile u8 newPWM = 0;


void setNewPWM(const s16 value);

vu8 wasinit = 0;
vu8 wasinif = 0;
vu32 wasinawdit = 0;
vu32 wasineocit = 0;
vu32 wasinadcit = 0;
vu32 wasinhtit = 0;

vu8 systickItIsRunning = 0;

vu32 forcedHighSideOn = 0;

u8 ownReceiverID = 0;

vu8 testcount = 0;


vu32 currentValue = 0;
struct adcValues{
  u32 currentValues[32];
  u32 currentValueCount;
  u32 batValueSum;
  u32 batValueCount;
};

vu8 switchAdcValues = 0;

volatile struct adcValues *activeAdcValues;
volatile struct adcValues *inActiveAdcValues;

#define dbgSize 100
vu16 dbgValue[dbgSize];
vu16 dbgCount = 0;
vu8 resetdbgCount = 0;
vu32 batValue = 0;
vu8 writeNewDebugValues = 0;

vu16 adcValueCount = 0;

vu32 acs712BaseVoltage = 0;
vu8 awdWasTriggered = 0;
volatile u16 timeoutCounter = 0;

volatile enum hostIDs ownHostId;

volatile struct pid_data speedPidData;
volatile struct pid_data posPidData;

volatile enum errorCodes error = ERROR_CODE_NONE;

struct ControllerState {
  enum controllerModes controllMode;
  enum internalState internalState;
  s16 kp;
  s16 ki;
  s16 kd;
  u16 minMaxPidOutput;
  enum controllerModes newPidDataType;
  u8 newPIDData;
  u8 useBackInduction;
  u8 useOpenLoop;
  u8 enablePIDDebug;
  u8 cascadedPositionController;
  u16 pwmStepPerMillisecond;
  u16 maxCurrent;
  u8 maxCurrentCount;
  u8 maxMotorTemp;
  u8 maxMotorTempCount;
  u8 maxBoardTemp;
  u8 maxBoardTempCount;
  u16 timeout;
  s32 targetValue;
  u8 resetTimeoutCounter;
};

volatile struct ControllerState *activeCState;
volatile struct ControllerState *lastActiveCState;


/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{
  int delay;
  int i;
  struct adcValues avs1;
  struct adcValues avs2;
  activeAdcValues = &avs1;
  inActiveAdcValues = &avs2;


  //Enable peripheral clock for GPIO
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

  /* Enable peripheral clocks ---------------------------------------*/
  /* enable I2C2 clock */
  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

  /* Enable USART1 clock */
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);


  NVIC_Configuration();

  GPIO_Configuration();
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);

  USART_Configuration();
  //setupI2C();

  //turn of red led
  GPIO_SetBits(GPIOA, GPIO_Pin_8);

  u16 gpioData = 0;
  gpioData |= GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);
  gpioData |= (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14) << 1);
  gpioData |= (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15) << 2);
  gpioData |= (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) << 3);

  //get correct host id from gpio pins
  switch(gpioData) {
  case 1 :
    print("Configured as H_BRIDGE_1\n");
    ownHostId = RECEIVER_ID_H_BRIDGE_1;
    break;
  case 2 :
    print("Configured as H_BRIDGE_2\n");
    ownHostId = RECEIVER_ID_H_BRIDGE_2;
    break;
  case 4 :
    print("Configured as H_BRIDGE_3\n");
    ownHostId = RECEIVER_ID_H_BRIDGE_3;
    break;
  case 8 :
    print("Configured as H_BRIDGE_4\n");
    ownHostId = RECEIVER_ID_H_BRIDGE_4;
    break;
  default:
    printf("Wrong host ide configured %hu\n", gpioData);
    //blink and do nothing
    assert_failed((u8 *)__FILE__, __LINE__);
    break;
  }

  CAN_Configuration();
  CAN_ConfigureFilters(ownHostId);

  TIM_Configuration();

  //SPI_Configuration();
  
  ADC_Configuration();

  print("Loop start 1\n");

  delay = 5000000;
  while(delay)
    delay--;

  volatile struct ControllerState cs1;
  volatile struct ControllerState cs2;
  
  activeCState = &(cs1);
  lastActiveCState = &(cs2);

  //init pid structs with sane values
  initPIDStruct(&(posPidData));
  initPIDStruct(&(speedPidData));

  //init cotroller state with sane values
  activeCState->controllMode = CONTROLLER_MODE_PWM;
  activeCState->controllMode = CONTROLLER_MODE_POSITION;
  activeCState->internalState = STATE_UNCONFIGURED;
  activeCState->useBackInduction = 0;
  activeCState->useOpenLoop = 0;
  activeCState->cascadedPositionController = 0;
  activeCState->pwmStepPerMillisecond = 0;
  activeCState->maxCurrent = 0;
  activeCState->maxMotorTemp = 0;
  activeCState->maxMotorTempCount = 0;
  activeCState->maxBoardTemp = 0;
  activeCState->maxBoardTempCount = 0;
  activeCState->timeout = 1;
  activeCState->targetValue = 0;
  activeCState->kp = 0;
  activeCState->ki = 0;
  activeCState->kd = 0;
  activeCState->minMaxPidOutput = 0;
  activeCState->newPIDData = 0;
  activeCState->enablePIDDebug = 0;
  *lastActiveCState = *activeCState;


  int k;
  u32 meanacs712base = 0;
  meanacs712base = 0;
  for(k = 0; k < 40; k++) {
    acs712BaseVoltage = 0;
    
    //disable pwm output, so that ADC is not triggered any more
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
    
    //configure ADC for calibration acs712 base voltage
    configureCurrentMeasurement(1);
  
    //reset adc values
    for( i = 0; i < USED_REGULAR_ADC_CHANNELS*2; i++) {
      activeAdcValues->currentValues[i] = 0;
    }
    activeAdcValues->currentValueCount = 0;
    
    //trigger adc
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    
    //wait for adc conversation to start
    while(!(activeAdcValues->currentValues[0])) {
      ;
    }
    
    //disable further triggering
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
    
    //Wait for conversation to be finished
    while(!activeAdcValues->currentValueCount) {
      ;
    }

    //sum up values
    for( i = 0; i < (USED_REGULAR_ADC_CHANNELS-1)*2; i++) {
      //printf("Calibration value %d  was %d \n",i, activeAdcValues->currentValues[i]);
      acs712BaseVoltage += activeAdcValues->currentValues[i];
    }
    
    //average filter, to reduce noise
    acs712BaseVoltage /= (USED_REGULAR_ADC_CHANNELS-1)*2;
    printf("ACS712 base Voltage is %lu \n", acs712BaseVoltage);
    meanacs712base += acs712BaseVoltage;
    
  }
  meanacs712base /= 40;
  acs712BaseVoltage = meanacs712base;
  printf("\nMENAN ACS712 base Voltage is %lu \n\n", meanacs712base);
  

  //reenable pwm output again for normal usage
  TIM_CtrlPWMOutputs(TIM1, ENABLE);

  //activate systick interrupt, at this point
  //activeCState1 hast to be initalized completely sane !
  SysTick_Configuration();


  
  print("Loop start 2\n");
 
  /** DEBUG**/
  u16 lastTime = 0;
  u16 time;
  u16 counter = 0;  
  /*  activeCState->targetValue = 880;
  activeCState->controllMode = CONTROLLER_MODE_PWM;
  activeCState->internalState = STATE_CONFIGURED;
  activeCState->pwmStepPerMillisecond = 3;
  activeCState->useBackInduction = 0;
  activeCState->useOpenLoop = 1;
  *lastActiveCState = *activeCState;
  */

  /* Enable AWD interupt */
  //ADC_ITConfig(ADC2, ADC_IT_AWD, ENABLE);
    
  print("Loop start 3\n");

  /** END DEBUG **/
  int j = 0;
 
  while(1) {

    /** START DEBUG **/

    if(counter > 10000) {
      
      
      /*for(; j < dbgCount; j++) {
	printf("ADC value %d  was %d \n",j, dbgValue[j]);
	dbgValue[j] = 123;
	}*/
      
      if(dbgCount > dbgSize - 20) {
	j = 0;
	resetdbgCount = 1;
      }
      /*
      
      for(i = 0; i < USED_REGULAR_ADC_CHANNELS*2; i++) {
	printf( "%d ", dbgValue[i]);
      }
      printf("%d\n", acs712BaseVoltage);
      
      /*
      for(i = 0; i < USED_REGULAR_ADC_CHANNELS*2 - 2; i++) {
	printf("ADC value %d  was %d \n",i, dbgValue[i]);
      }
      printf("RAW Mean current Value is %lu \n", dbgValue[30]);
      printf("Nr of adc Readings was %lu \n", dbgValue[31]);
      printf("ACS712 base Voltage is %lu \n", acs712BaseVoltage);
      */
      
      //printf("Mean Bat Value is %lu \n", (batValue *16 * 33 * 100) / 4096);
      //printf("RAW Mean Bat Value is %lu \n", batValue);
      //printf("ACS712 base Voltage is %lu \n", acs712BaseVoltage);

      writeNewDebugValues = 1;

      //voltage divider is 33/60
      //100 mV is 1A
      //1680 is adc value without load
      //adc sample time is 722.22 nsecs
      u32 convCurrent = currentValue;
      //printf("RAW Mean current Value is %lu \n", convCurrent);

      
      
      
      //multiply by pwm lenght time
      //u32 measurementToPWMLenght = (25000 * 880 / 1800) / (722 * 14) ;
      //convCurrent = (convCurrent * 880) / (1800 * 14);
      //convCurrent = (convCurrent / 14);
      
      /*
      printf("Mean current Value is %lu \n",	convCurrent);
	

	//printf("Mean current Value is %lu \n", ((((currentValue * 3300) / (4096 * 14)) - 1350) * (activeCState->targetValue) * 60) / (180 * 33));
	//printf("Mean current Value is %lu \n", ((((currentValue * 3300) / 4096) - 1350) * (activeCState->targetValue) * 60) / (180 * 33));
      
      
      testcount = 0;
    
      //testprintf();
    
      printf("Forced high Side on %d \n", forcedHighSideOn);
      forcedHighSideOn = 0;
      
      printf("wasinif is %d \n", wasinif);
      printf("Was in Update IT %d \n", wasinit);
      printf("Was in AWD it %d \n", wasinawdit);
      wasinawdit = 0;
      printf("Was in EOC it %d \n", wasineocit);
      wasineocit = 0;
      printf("Was in HT it %d \n", wasinhtit);
      wasinhtit = 0;
      printf("Was in ADC it %d \n", wasinadcit);
      wasinadcit = 0;
      
      RCC_ClocksTypeDef RCC_ClocksStatus;
      RCC_GetClocksFreq(&RCC_ClocksStatus);
      
      printf("PCLK2 %lu, PCLK1 %lu\n" ,RCC_ClocksStatus.PCLK2_Frequency ,RCC_ClocksStatus.PCLK1_Frequency);

      printf("ActiveCstate: targetVal : %l , openloop:%h , backIndo %h , pwmstep %hu \n", activeCState->targetValue, activeCState->useOpenLoop, activeCState->useBackInduction, activeCState->pwmStepPerMillisecond);
      printf("LastActiveCstate: targetVal : %l , openloop:%h , backIndo %h , pwmstep %hu \n", lastActiveCState->targetValue, lastActiveCState->useOpenLoop, lastActiveCState->useBackInduction, lastActiveCState->pwmStepPerMillisecond);
      */

      /*if(encoderValue < 500)
	GPIO_SetBits(GPIOB, GPIO_Pin_6);
	else 
	GPIO_ResetBits(GPIOB, GPIO_Pin_6);
      */
      
      /*      static int bla = 0;
      if(bla) {
	bla = 0;
	GPIO_SetBits(GPIOB, GPIO_Pin_10);
      } else {
	bla = 1;
	GPIO_ResetBits(GPIOB, GPIO_Pin_10);
	}*/
      
      
      //extern u16 wasincanit;

      //printf("Was in CAN it %d \n", wasincanit);
  
      counter = 0;

      print(".");
      
    }
    
    time = TIM_GetCounter(TIM1);
    if(lastTime > time) {
      counter++;
    }
    lastTime = time;

    /* END DEBUG */

    CanRxMsg *curMsg;
    
    //check if we got a new message
    if((curMsg = CAN_GetNextData()) != 0) {
      //print("Got a Msg \n");
      print("P");

      //clear device specific adress bits
      curMsg->StdId &= ~ownHostId;
    
      switch(curMsg->StdId) {
        case PACKET_ID_EMERGENCY_STOP:
	  //print("Got PACKET_ID_EMERGENCY_STOP Msg \n");
	  lastActiveCState->internalState = STATE_UNCONFIGURED;
	  lastActiveCState->targetValue = 0;
	  break;
        case PACKET_ID_SET_VALUE: {
	  //print("Got PACKET_ID_SET_VALUE Msg \n");
	  if(activeCState->internalState == STATE_CONFIGURED) {
	    struct setValueData *data = (struct setValueData *) curMsg->Data;
	    switch(ownHostId) {
	    case RECEIVER_ID_H_BRIDGE_1:
	      lastActiveCState->targetValue = data->board1Value;
	      break;
	    case RECEIVER_ID_H_BRIDGE_2:
	      lastActiveCState->targetValue = data->board2Value;
	      break;
	    case RECEIVER_ID_H_BRIDGE_3:
	      lastActiveCState->targetValue = data->board3Value;
	      break;
	    case RECEIVER_ID_H_BRIDGE_4:
	      lastActiveCState->targetValue = data->board4Value;
	      break;
	    }
	  } else {
	    print("Error, not configured \n");
	    lastActiveCState->internalState = STATE_ERROR;
	    error = ERROR_CODE_BAD_CONFIG;
	  }
	  break;
	}

        case PACKET_ID_SET_MODE: {
	  if(activeCState->internalState == STATE_CONFIGURED) {
	    struct setModeData *data = (struct setModeData *) curMsg->Data;
	    switch(ownHostId) {
	    case RECEIVER_ID_H_BRIDGE_1:
	      
	      lastActiveCState->controllMode = data->board1Mode;
	      break;
	    case RECEIVER_ID_H_BRIDGE_2:
	      lastActiveCState->controllMode = data->board2Mode;
	      break;
	    case RECEIVER_ID_H_BRIDGE_3:
	      lastActiveCState->controllMode = data->board3Mode;
	      break;
	    case RECEIVER_ID_H_BRIDGE_4:
	      lastActiveCState->controllMode = data->board4Mode;
	      break;
	    }
	  } else {
	    print("Error, not configured \n");
	    lastActiveCState->internalState = STATE_ERROR;
	    error = ERROR_CODE_BAD_CONFIG;
	  }
	  break;
	}
	  
        case PACKET_ID_SET_PID_POS: {
	  if(activeCState->internalState == STATE_CONFIGURED) {
	    struct setPidData *data = (struct setPidData *) curMsg->Data;
	    lastActiveCState->newPidDataType = CONTROLLER_MODE_POSITION;
	    lastActiveCState->kp = data->kp;
	    lastActiveCState->ki = data->ki;
	    lastActiveCState->kd = data->kd;
	    lastActiveCState->minMaxPidOutput = data->minMaxPidOutput;
	    lastActiveCState->newPIDData = 1;
	    //printf("Got PACKET_ID_SET_PI_POS Msg %h %h %h %hu\n", data->kp, data->ki, data->kd, data->minMaxPidOutput);
	  
	  } else {
	    print("Error, not configured \n");
	    lastActiveCState->internalState = STATE_ERROR;
	    error = ERROR_CODE_BAD_CONFIG;
	  }
	  break;
	}
        case PACKET_ID_SET_PID_SPEED: {
	  if(activeCState->internalState == STATE_CONFIGURED) {
	    struct setPidData *data = (struct setPidData *) curMsg->Data;
	    lastActiveCState->newPidDataType = CONTROLLER_MODE_SPEED;
	    lastActiveCState->kp = data->kp;
	    lastActiveCState->ki = data->ki;
	    lastActiveCState->kd = data->kd;
	    lastActiveCState->minMaxPidOutput = data->minMaxPidOutput;
	    lastActiveCState->newPIDData = 1;
	    //printf("Got PACKET_ID_SET_PID_SET Msg %h %h %h %hu\n", data->kp, data->ki, data->kd, data->minMaxPidOutput);
	  } else {
	    print("Error, not configured \n");
	    lastActiveCState->internalState = STATE_ERROR;
	    error = ERROR_CODE_BAD_CONFIG;
	  }
	  break;
	}
        case PACKET_ID_SET_CONFIGURE: {
	  //print("Got PACKET_ID_SET_CONFIGURE Msg \n");
	  struct configure1Data *data = (struct configure1Data *) curMsg->Data;
	  lastActiveCState->useBackInduction = data->activeFieldCollapse;
	  lastActiveCState->useOpenLoop = data->openCircuit;
	  lastActiveCState->cascadedPositionController = data->cascadedPositionController;
	  lastActiveCState->enablePIDDebug = data->enablePIDDebug;

	  lastActiveCState->maxMotorTemp = data->maxMotorTemp;
	  lastActiveCState->maxMotorTempCount = data->maxMotorTempCount;
	  lastActiveCState->maxBoardTemp = data->maxBoardTemp;
	  lastActiveCState->maxBoardTempCount = data->maxBoardTempCount;
	  lastActiveCState->timeout = data->timeout;

	  if(activeCState->internalState == STATE_CONFIG2_RECEIVED) {
	    lastActiveCState->internalState = STATE_CONFIGURED;
	    error = ERROR_CODE_NONE;
	  } else {
	    lastActiveCState->internalState = STATE_CONFIG1_RECEIVED;
	  }
	  
	  break;
	}
	  
        case PACKET_ID_SET_CONFIGURE2: {
	  //print("Got PACKET_ID_SET_CONFIGURE2 Msg \n");
	  struct configure2Data *data = (struct configure2Data *) curMsg->Data;
	  lastActiveCState->maxCurrent = data->maxCurrent;
	  lastActiveCState->maxCurrentCount = data->maxCurrentCount;
	  lastActiveCState->pwmStepPerMillisecond = data->pwmStepPerMs;

	  if(activeCState->internalState == STATE_CONFIG1_RECEIVED) {
	    lastActiveCState->internalState = STATE_CONFIGURED;
	    error = ERROR_CODE_NONE;
	  } else {
	    lastActiveCState->internalState = STATE_CONFIG2_RECEIVED;
	  }
	  break;
	}

      default: 
	{
	  u16 id = curMsg->StdId;
	  
	  printf("Got unknown packet id : %hu ! \n", id);
	  break;
	}
	
      }
    
      //mark current message als processed
      CAN_MarkNextDataAsRead();

      //this is concurrency proff, as this code can not run, while
      //systick Handler is active !
      volatile struct ControllerState *tempstate = activeCState;

      //wait till pid data is taken up by controll loop,
      if(activeCState->newPIDData) {
	;
      }

      //we go an new packet, so reset the timeout
      lastActiveCState->resetTimeoutCounter = 1;
      
      //this is atomar as only the write is relevant!
      activeCState = lastActiveCState;

      lastActiveCState = tempstate;
      
      *lastActiveCState = *activeCState;
      
      //pid values are written to controll loop,
      //so the are not new any more
      lastActiveCState->newPIDData = 0;
      
      printf("Error is %h \n", error);
      printf("ActiveCstate: ControllMode : %l , internal State : %l ,targetVal : %l , openloop:%h , backIndo %h , pwmstep %hu \n", activeCState->controllMode, activeCState->internalState, activeCState->targetValue, activeCState->useOpenLoop, activeCState->useBackInduction, activeCState->pwmStepPerMillisecond);
      printf("LastActiveCstate: ControllMode : %l , internal State : %l ,targetVal : %l , openloop:%h , backIndo %h , pwmstep %hu \n", lastActiveCState->controllMode, lastActiveCState->internalState, lastActiveCState->targetValue, lastActiveCState->useOpenLoop, lastActiveCState->useBackInduction, lastActiveCState->pwmStepPerMillisecond);

    } 
  }
}

void detectTouchdown(const int current) {
  static int highTimeCounter = 0;
  enum touchdownStates {
    TOUCHDOWN_DETECTED,
    LIFTOFF,
  };
  
  static enum touchdownStates state = LIFTOFF;

  switch (state) {
    case TOUCHDOWN_DETECTED:
      highTimeCounter++;
      if(highTimeCounter > 20) {
	state = LIFTOFF;
	highTimeCounter = 0;
	GPIO_SetBits(GPIOB, GPIO_Pin_10);
      }
      break;
      
    case LIFTOFF:
      if(current > 200) {
	state = TOUCHDOWN_DETECTED;
	GPIO_ResetBits(GPIOB, GPIO_Pin_10);
      }
      break;
  }
}


void SysTickHandler(void) {
  
  //TODO FIXME I am a hack
  //activeCState->enablePIDDebug = 1;

  //request switch of adc value struct
  switchAdcValues = 1;
  
  //GPIOA->BSRR |= GPIO_Pin_8;

  u16 encoderValue = 0;
  static u16 lastEncoderValue = 0;
  static s32 currentPwmValue = 0;
  static u16 index = 0;
  static u8 wheelHalfTurned = 0;
  static u16 overCurrentCounter = 0;


  s32 curSpeed = 0;
  s32 pwmValue = 0;

  //get encoder
  encoderValue = TIM_GetCounter(TIM4);

  int i, usableCurrentValues = 0;
  //length of 13Cycle Sample time is 722nsecs 
  //during 21660nsecs samples are taken, this 
  //is equal to a pwm value of 1559.52
  if(abs(currentPwmValue) < 1559) {
    usableCurrentValues = abs(currentPwmValue) / 52;
  } else {
    usableCurrentValues = 30;
  }

  //wait for adc struct to be switched
  while(switchAdcValues) {
    ;
  }

  //sum up all currents measured
  currentValue = 0;

  u32 adcValueCount =  inActiveAdcValues->currentValueCount;
  u32 *currentValues = inActiveAdcValues->currentValues;

  /**DEBUG**/
  if(writeNewDebugValues) {
    
    for( i = 0; i < USED_REGULAR_ADC_CHANNELS*2; i++) {
      dbgValue[i] = currentValues[i] / adcValueCount;
    }
    
    dbgValue[31] = adcValueCount;
  }
  
  
  /**END DEBUG**/


  //batValueSum and currentValueSum are increased at the 
  //same time therefore adcValueCount is valid for both
  //TODO FIXME bat value is only valid if PWM is on
  batValue = inActiveAdcValues->batValueSum / (2 * inActiveAdcValues->batValueCount);
  inActiveAdcValues->batValueSum = 0;
  inActiveAdcValues->batValueCount = 0;
  
  //base voltage of ACS712 this is measured at startup, as
  //it is related to 5V rail. We asume the 5V rail ist stable
  //since startup
  u32 baseVoltage = acs712BaseVoltage*adcValueCount;

  for(i = 0; i < usableCurrentValues; i++) {
    if(currentValues[i] > baseVoltage) {
      currentValues[i] -= baseVoltage;
      if(i == 0 || i == 15) {
	//multiply with 722nsec + 388.88 nsec (battery value conversation) to get integral
	currentValue += (currentValues[i] * 1111) / (adcValueCount);
      } else {     
	//multiply with 722nsec to get integral
	currentValue += (currentValues[i] * 722) / (adcValueCount);
      }
      //(adcValue[30])++;
    }
    currentValues[i] = 0;
  }
  

  //divide by complete time, to get aritmetic middle value
  currentValue /= 2500;

  //as 1000mA is 100mV multiply by 10
  //is included in the divide by 25000 step 
  //currentValue *= 10;
  
  //multiply by voltage divider, to get back to voltage at the
  //measuaring chip
  currentValue = (currentValue * 51) / 33;
  
  //convert from adc to volts
  currentValue = (currentValue * 3300) / 4096;

  /**DEBUG**/
  if(writeNewDebugValues) {
    dbgValue[30] = currentValue;
    writeNewDebugValues = 0;
  }
  /** END DEBUG **/

  //set rest to zero
  for(i = usableCurrentValues; i < (USED_REGULAR_ADC_CHANNELS -1) *2; i++) {
    currentValues[i] = 0;
  }

  inActiveAdcValues->currentValueCount = 0;

  //try to detect touchdown and switch on led
  detectTouchdown(currentValue);

  //reset timeout, if "userspace" requested it
  if(activeCState->resetTimeoutCounter) {
    activeCState->resetTimeoutCounter = 0;
    timeoutCounter = 0;
  }

  //change state to unconfigured if error is set
  if(error != ERROR_CODE_NONE && activeCState->internalState == STATE_CONFIGURED) {
    activeCState->internalState = STATE_ERROR;
  } else {
    //check for overcurrent
    if(currentValue > activeCState->maxCurrent) {
      overCurrentCounter++;

      if(overCurrentCounter > activeCState->maxCurrentCount) {
	activeCState->internalState = STATE_ERROR;
	error = ERROR_CODE_OVERCURRENT;
      }
    } else {
      overCurrentCounter = 0;
    }
    
    //check for timeout and go into error state
    if(activeCState->timeout && (timeoutCounter > activeCState->timeout)) {
      activeCState->internalState = STATE_ERROR;
      error = ERROR_CODE_TIMEOUT;      
    }
  }

  //calculate correct half wheel position
  if(abs(lastEncoderValue - encoderValue) > HALF_WHEEL_TURN_TICKS / 2) {
      if(wheelHalfTurned)
	wheelHalfTurned = 0;
      else
	wheelHalfTurned = 1;
  }

  s32 wheelPos = encoderValue;
  wheelPos += wheelHalfTurned * HALF_WHEEL_TURN_TICKS;
  
  if(activeCState->internalState == STATE_CONFIGURED ||
     activeCState->internalState == STATE_ERROR) {

    index++;
    
    if(index >= (1<<10))
      index = 0;
  
    //send status message over CAN
    CanTxMsg statusMessage;
    statusMessage.StdId= PACKET_ID_STATUS + ownHostId;
    statusMessage.RTR=CAN_RTR_DATA;
    statusMessage.IDE=CAN_ID_STD;
    statusMessage.DLC= sizeof(struct statusData);
    
    struct statusData *sdata = (struct statusData *) statusMessage.Data;
    
    sdata->currentValue = currentValue;
    sdata->index = index;
    sdata->position = wheelPos / 4;
    //TODO calculate temp
    sdata->tempHBrigde = 0;
    sdata->tempMotor = 0;
    sdata->error = error;
    
    if(CAN_Transmit(&statusMessage) == CAN_NO_MB) {
      print("Error Tranmitting status Message : No free TxMailbox \n");
    } else {
      //print("Tranmitting status Message : OK \n");  
    }
  }

  if(activeCState->newPIDData) {
    switch(activeCState->newPidDataType) {
      case CONTROLLER_MODE_SPEED:
	setKp((struct pid_data *) &(speedPidData), activeCState->kp);
	setKi((struct pid_data *) &(speedPidData), activeCState->ki);
	setKd((struct pid_data *) &(speedPidData), activeCState->kd);
	setMinMaxCommandVal((struct pid_data *) &(speedPidData), -activeCState->minMaxPidOutput, activeCState->minMaxPidOutput);
	break;
      case CONTROLLER_MODE_POSITION:
	setKp((struct pid_data *) &(posPidData), activeCState->kp);
	setKi((struct pid_data *) &(posPidData), activeCState->ki);
	setKd((struct pid_data *) &(posPidData), activeCState->kd);
	setMinMaxCommandVal((struct pid_data *) &(posPidData), -activeCState->minMaxPidOutput, activeCState->minMaxPidOutput);
	break;
      default:
	print("Error got PID data for nonPID driving mode\n");
	break;
    }
    activeCState->newPIDData = 0;
  }
  

  CanTxMsg pidMessagePos;
  CanTxMsg posDbgMessage;
  CanTxMsg pidMessageSpeed;
  CanTxMsg speedDbgMessage;

  switch(activeCState->controllMode) {
    case CONTROLLER_MODE_PWM:
      pwmValue = activeCState->targetValue;
      break;
    
    case CONTROLLER_MODE_POSITION: {
      s32 curVal = wheelPos;

      //correct wraparounds
      if(abs((activeCState->targetValue *4) - curVal) > HALF_WHEEL_TURN_TICKS) {
	if(curVal < HALF_WHEEL_TURN_TICKS)
	  curVal += HALF_WHEEL_TURN_TICKS * 2;
	else 
	  curVal -= HALF_WHEEL_TURN_TICKS * 2;
      }
      
      //calculate PID value
      setTargetValue((struct pid_data *) &(posPidData), activeCState->targetValue * 4);
      pwmValue = pid((struct pid_data *) &(posPidData), curVal);

      if(activeCState->enablePIDDebug) {
	//send status message over CAN
	pidMessagePos.StdId= PACKET_ID_PID_DEBUG_POS + ownHostId;
	pidMessagePos.RTR=CAN_RTR_DATA;
	pidMessagePos.IDE=CAN_ID_STD;
	pidMessagePos.DLC= sizeof(struct pidDebugData);
	
	struct pidDebugData *sdata = (struct pidDebugData *) pidMessagePos.Data;
	getInternalPIDValues(&(sdata->pPart), &(sdata->iPart), &(sdata->dPart));
	posDbgMessage.StdId= PACKET_ID_POS_DEBUG + ownHostId;
	posDbgMessage.RTR=CAN_RTR_DATA;
	posDbgMessage.IDE=CAN_ID_STD;
	posDbgMessage.DLC= sizeof(struct posDebugData);
	
	struct posDebugData *pdbgdata = (struct posDebugData *) posDbgMessage.Data;
	pdbgdata->targetVal = activeCState->targetValue;
	pdbgdata->pwmVal = pwmValue;
	pdbgdata->encoderVal = encoderValue;
	pdbgdata->posVal = (curVal / 4);
      }
    }

      if(!activeCState->cascadedPositionController) {
	break;
      } else {
	pwmValue /= 10;
      }
      
      
    case CONTROLLER_MODE_SPEED:
      curSpeed = encoderValue - lastEncoderValue;
      
      //this assumes, that the motor will never turn faster than
      //a quarter wheel turn (or 12.5 motor turns) in a ms 
      if(abs(lastEncoderValue - encoderValue) > HALF_WHEEL_TURN_TICKS / 2) {
	//wheel ist turning backward
	if(lastEncoderValue < encoderValue) {
	  curSpeed -= HALF_WHEEL_TURN_TICKS;
	}
	
	//wheel is turning forward
	if(lastEncoderValue > encoderValue) {
	  curSpeed += HALF_WHEEL_TURN_TICKS;
	}
      }

      //TODO FIXME convert to a real value like m/s
      
      if(activeCState->enablePIDDebug) {
	speedDbgMessage.StdId= PACKET_ID_SPEED_DEBUG + ownHostId;
	speedDbgMessage.RTR=CAN_RTR_DATA;
	speedDbgMessage.IDE=CAN_ID_STD;
	speedDbgMessage.DLC= sizeof(struct speedDebugData);
      }

      struct speedDebugData *sdbgdata = (struct speedDebugData *) speedDbgMessage.Data;
    
      //calculate PID value
      if(activeCState->cascadedPositionController && activeCState->controllMode == CONTROLLER_MODE_POSITION) {
	//use output of position controller as input
	setTargetValue((struct pid_data *) &(speedPidData), pwmValue);

	sdbgdata->targetVal = pwmValue;
	
      
	pwmValue = pid((struct pid_data *) &(speedPidData), curSpeed);
	
      } else {
	//use input from PC-Side
	setTargetValue((struct pid_data *) &(speedPidData), activeCState->targetValue);
      
	pwmValue = pid((struct pid_data *) &(speedPidData), curSpeed);

	sdbgdata->targetVal = activeCState->targetValue;
      }
      
      if(activeCState->enablePIDDebug) {
	sdbgdata->pwmVal = pwmValue;
	sdbgdata->encoderVal = encoderValue;
	sdbgdata->speedVal = curSpeed;

	//send status message over CAN
	pidMessageSpeed.StdId= PACKET_ID_PID_DEBUG_SPEED + ownHostId;
	pidMessageSpeed.RTR=CAN_RTR_DATA;
	pidMessageSpeed.IDE=CAN_ID_STD;
	pidMessageSpeed.DLC= sizeof(struct pidDebugData);
	
	struct pidDebugData *sdata = (struct pidDebugData *) pidMessageSpeed.Data;
	getInternalPIDValues(&(sdata->pPart), &(sdata->iPart), &(sdata->dPart));
      }
      
      break; 
  }

  //trunkcate to s16
  if(pwmValue > MAX_S16) 
    pwmValue = MAX_S16;
  if(pwmValue < MIN_S16)
    pwmValue = MIN_S16;

  if(abs(currentPwmValue - pwmValue) < activeCState-> pwmStepPerMillisecond) {
    currentPwmValue = pwmValue;
  } else {
    if(currentPwmValue - pwmValue < 0) {
      currentPwmValue += activeCState->pwmStepPerMillisecond;
    } else {
      currentPwmValue -= activeCState->pwmStepPerMillisecond;      
    }
  }

  if(activeCState->internalState == STATE_CONFIGURED) {
    //increase timeout
    timeoutCounter++;

    //set pwm
    setNewPWM(currentPwmValue);

    if(activeCState->enablePIDDebug) {

      struct speedDebugData *sdbgdata = (struct speedDebugData *) speedDbgMessage.Data;
      sdbgdata->pwmVal = currentPwmValue;

      //send out all debug messages in right order
      while(CAN_Transmit(&pidMessagePos) == CAN_NO_MB){
	;
      }
    
      while(CAN_Transmit(&posDbgMessage) == CAN_NO_MB) {
	;
      }
      
      while(CAN_Transmit(&pidMessageSpeed) == CAN_NO_MB){
	;
      }
    
      //send speed status message
      while(CAN_Transmit(&speedDbgMessage) == CAN_NO_MB) {
	;
      }
    }
  } else {
    //reset timeoutcounter
    timeoutCounter = 0;
    
    setNewPWM(0);   
  }
 
  lastEncoderValue = encoderValue;

  //  GPIOA->BRR |= GPIO_Pin_8;

}


void setNewPWM(const s16 value2) {
  //TODO add magic formular for conversation
  s16 value = value2;

  
  //trunkate to min of 3.0% PWM, as Current Measurement needs 763.8 us
  //388.8us for vbat and 375us for first current sample
  if((value > 0 && value < 55) || (value < 0 && value > -55))
    value = 0;
  

  //TODO exact value
  //trunkcate to max of 95% PWM
  if(value < -1600)
    value = -1600;
  
  if(value > 1600)
    value = 1600;

  u16 dutyTime;
  
  //remove signess
  if(value < 0) {
    dutyTime = value * -1;
  } else {
    dutyTime = value;
  }
  
  
  //disable transfer of values from config register
  //to timer intern shadow register 
  TIM_UpdateDisableConfig(TIM1, ENABLE);
  TIM_UpdateDisableConfig(TIM2, ENABLE);
  TIM_UpdateDisableConfig(TIM3, ENABLE);
  
  desieredDirection = value >= 0;

  /* Modified active field-collapse drive
   *
   *       Voltage Raise
   *          |
   *         \/
   * AIN __|-------
   * BIN --|__|----
   * ASD --|_______
   * BSD ----------
   *
   */
  if(desieredDirection) {  
    //AIN 
    TIM2->CCR1 = 1801;

    // BIN
    TIM2->CCR2 = dutyTime;

    if(activeCState->useOpenLoop) {
    
      if(activeCState->useBackInduction) {
	// ASD
	TIM3->CCR1 = dutyTime;

	// BSD
	//B Lo is switched against ground and Vb can load up
	//note this drive mode has problems with boost voltage if
	//high side variation is used
	//Set bsd to max_pulse+1, so it is on all the time
	TIM3->CCR2 = 1801;
      } else {
	// ASD
	TIM3->CCR1 = 1801;

	//BSD
	//switch on B-low for a short time, so boost voltage
	//can recharge
	TIM3->CCR2 = dutyTime + 100;
      }
    } else {
      //closed loop
      if(activeCState->useBackInduction) {  
	//TODO implement me
      } else {
	//ASD high, all the time, do not shutdown on switch
	TIM3->CCR1 = 1801;

	//BSD high, all the time, do not shutdown on switch
	TIM3->CCR2 = 1801;
      }
    }
    
  } else {
    // AIN
    TIM2->CCR1 = dutyTime;
    
    // BIN
    TIM2->CCR2 = 1801;
    
    if(activeCState->useOpenLoop) {
    
      if(activeCState->useBackInduction) {
	// BSD
	TIM3->CCR2 = dutyTime;

	// ASD
	//A Lo is switched against ground and Vb can load up
	//note this drive mode has problems with boost voltage if
	//high side variation is used
	//Set bsd to max_pulse+1, so it is on all the time
	TIM3->CCR1 = 1801;
      } else {
	// BSD
	TIM3->CCR2 = 1801;
	
	//ASD
	//switch on A-low for a short time, so boost voltage
	//can recharge
	TIM3->CCR1 = dutyTime + 100;
      }
    } else {
      //closed loop
      if(activeCState->useBackInduction) {  
	//TODO implement me
      } else {
	//ASD high, all the time, do not shutdown on switch
	TIM3->CCR1 = 1801;

	//BSD high, all the time, do not shutdown on switch
	TIM3->CCR2 = 1801;
      }
    }
  }

  //Current Measurement timer
  //take current at 80% of PWM high phase
  TIM1->CCR2 = 1;//(((u32) dutyTime) * 4 )/5;

  if(activeCState->useBackInduction && dutyTime < 1600) {
    //Watchdog enable timer
    TIM1->CCR3 = dutyTime + 10;
    
    //TODO add x to sec timer
    
    //security timer
    TIM2->CCR3 = dutyTime + 550;

    //TODO enable output of TIM1 CC3, and TIM2 CC3 it
    //reactivate security timer interrupt (TIM2 CC3)
    TIM2->DIER |= TIM_IT_CC3;
    
  } else {
    //disable the whole watchdog thing, as it won't 
    //trigger anyway on high pwms and only create 
    //race conditions

    //disable security interrupt
    TIM2->DIER &= ~TIM_IT_CC3;
  }
  
  //we had an direction change, disable H-Bridge
  //and reconfigure polarity
  if(actualDirection != desieredDirection) {

    //Disable PWM by pulling ADSD and BDSD low 
    u16 tmpccer = TIM3->CCER;

    tmpccer &= (~TIM_OutputState_Enable) | (u16)(~(TIM_OutputState_Enable << 4));
 
    TIM3->CCER = tmpccer;



    //programm correct polarity

    //disable OC1 and OC2 before changing polarity
    TIM2->CCER &= (u16) (~TIM_OutputState_Enable) | (u16) ~(TIM_OutputState_Enable << 4);

      //((u16)0xFFFE) & ((u16)0xFFEF);

    // Get the TIM2 CCER register value 
    tmpccer = TIM2->CCER;
    
    // Reset the Output Polarity level 
    tmpccer &= ((u16) (~TIM_OCPolarity_Low)) & (u16) ~(TIM_OCPolarity_Low << 4) & ((u16) (~(TIM_OutputState_Enable << 4)));
  
    if(desieredDirection) {  
      //OC1 low OC2 high
      tmpccer |= TIM_OCPolarity_Low;
    } else {
      //OC1 high OC2 low
      tmpccer |= (u16)(TIM_OCPolarity_Low << 4);
    }

    //reenable OC1 and OC2
    tmpccer |= TIM_OutputState_Enable | (u16)(TIM_OutputState_Enable << 4);

    // Write to TIMx CCER 
    TIM2->CCER = tmpccer;
  }

  newPWM = 1;
}

/**
 * This interrupt Handler handels the Capture 
 * Compare interrupts
 * In case of an CC4 Event Ch3 and Ch4 are 
 * reinitalized to Timer mode. 
 * If also a new PWM was set, the Update of the
 * internal shadow register is enabled again, so
 * that config is updated atomar by hardware on
 * the next update event.
 */
void TIM1_CC_IRQHandler(void) {
  //GPIOA->BSRR |= GPIO_Pin_8;

  static vu8 directionChangeLastTime = 0;

  if(TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET) {

    //Clear TIM1 update interrupt pending bit
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);
    
    //ReInit Tim2 OC1 and OC2 as timer mode (remove Forced high) 
    u16 tmpccmr1 = 0;
    
    //get CCMR1 content
    tmpccmr1 = TIM2->CCMR1;
    
    //Reset the OC1M OC2M Bits 
    tmpccmr1 &= 0xFF8F & 0x8FFF;
    
    //configure output mode
    tmpccmr1 |= TIM_OCMode_PWM1 | (TIM_OCMode_PWM1 << 8);
    
    //Write to TIM2 CCMR1 register
    TIM2->CCMR1 = tmpccmr1;

    //reset binary flag, for awd trigger indication
    awdWasTriggered = 0;

    //set High Value of ADC threshold to half battery Voltage
    //batValue is in ADC scale (0 - 4096)
    ADC2->LTR = batValue *2 /3;
    //ADC2->HTR = batValue / 2;

    //timers where updated atomar, so now we reenable
    //the H-Bridge and reconfigure the watchdog and 
    //current measurement and also update the direction value
    if(directionChangeLastTime) {
      directionChangeLastTime = 0;

      actualDirection = desieredDirection;
      configureWatchdog(actualDirection);

      //ReEnable ASD and BSD
      u16 tmpccer = TIM3->CCER;
      tmpccer |= TIM_OutputState_Enable | (u16)(TIM_OutputState_Enable << 4);      
      TIM3->CCER = tmpccer;

    }
      
    if(newPWM != 0) {
      wasinif = 1;
      newPWM = 0;
      //Enable update, all previous configured values
      //are now transfered atomic to the timer in the
      //moment the timer wraps around the next time
      TIM_UpdateDisableConfig(TIM1, DISABLE);
      TIM_UpdateDisableConfig(TIM2, DISABLE);
      TIM_UpdateDisableConfig(TIM3, DISABLE);

      
      //configureCurrentMeasurement(actualDirection);

      if(actualDirection != desieredDirection) {
	directionChangeLastTime = 1;
      }
    }
  }

  //GPIOA->BRR |= GPIO_Pin_8;
}


/**
 * This function deactivates either q1 or q3 in respect
 * to the turning direction of the motor.
 * It also disables the ADC_IT_AWD interrupt, and
 * stops the conversation, so that it can be enabled
 * by the TIM1CC3 again.
 */
inline void ForceHighSideOffAndDisableWatchdog(void) {
  u16 tmpccmr = 0;
  
  /*
  //Force high side gate on
  if(actualDirection) {
        //bdir low
    tmpccmr = TIM2->CCMR1;
    //clear mode bits
    tmpccmr &= ((u16)0x8FFF);
    //Configure The Forced output Mode
    tmpccmr |= (u16)(TIM_ForcedAction_Active << 8);
    //write to register
    TIM2->CCMR1 = tmpccmr;
  } else {
    //adir low
    tmpccmr = TIM2->CCMR1;
    //clear mode bits
    tmpccmr &= ((u16)0xFF8F);
    //Configure The Forced output Mode
    tmpccmr |= TIM_ForcedAction_Active;
    //write to register
    TIM2->CCMR1 = tmpccmr;
    }
  */
  //Stop conversation, and set ADC up for next trigger
  ADC2->CR2 &= ~0x01;
  ADC2->CR2 |= 0x01;
 
}


/**
 * Interrupt function used for security interrupt
 * This function forces the high side gate off, for
 * the case that the adc watchdog didn't trigger
 * also the Watchdog is disabled. 
 */
void TIM2_IRQHandler(void) {
  //GPIOA->BSRR |= GPIO_Pin_8;
    if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET) {
    //Clear TIM2 Capture compare interrupt pending bit 
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
    
    //only call function if AWD didn't trigger
    if(!awdWasTriggered) {
      forcedHighSideOn++;
      ForceHighSideOffAndDisableWatchdog();
    }

    ADC_ClearFlag(ADC2, ADC_FLAG_STRT);

  }

    //GPIOA->BRR |= GPIO_Pin_8;

}

/**
 * This is the ADC interrupt function
 * In case the source was the analog watchdog
 * the motor voltage just reverted, because of 
 * reinduction and we need to turn on the 
 * high side gate of the H-Bridge
 *
 * In case the source of the interrupt was an 
 * End of Conversation this means we just took
 * the Current values. 
 */
void ADC1_2_IRQHandler(void) {
  //wasinadcit = 1;

  //test if Analog Watchdog Flag is set.
  //this means the souce of the IT was AWD
  if(ADC2->SR & ADC_FLAG_AWD) {
    //GPIOA->BSRR |= GPIO_Pin_8;
    ForceHighSideOffAndDisableWatchdog();
    awdWasTriggered = 1;

    //clear interrupt source
    ADC2->SR = ~(u32) ADC_FLAG_AWD;

    wasinawdit++;
    //GPIOA->BRR |= GPIO_Pin_8;
  }
  

  //End of Conversion
  /*if(ADC1->SR & ADC_FLAG_EOC) {
    
    currentValueSum += ADC1->DR;
    if(adcValueCount < 50) {
      adcValue[adcValueCount] = ADC1->DR;
      batValues[adcValueCount] = adcValue[adcValueCount];
    }
    adcValueCount++;
    
    //clear interrupt source
    ADC1->SR = ~(u32) ADC_FLAG_EOC;   
    

    wasineocit++;
    //ADC_ITConfig(ADC1, ADC_IT_EOC, DISABLE);
    }*/


}

void DMA1_Channel1_IRQHandler(void) {
  //GPIOA->BSRR |= GPIO_Pin_8;
  static u8 secondInterrupt = 0;
  int i;

  vu32 *cvp = activeAdcValues->currentValues;
  vu16 *avp = adc_values;

  /**DEBUG**/
  //u16 *dbgp = dbgValue + dbgCount;

  /**END DEBUG**/


  if(DMA1->ISR & DMA1_IT_HT1) {
    activeAdcValues->batValueSum += adc_values[0];
    (activeAdcValues->batValueCount)++;
    cvp += (secondInterrupt * (USED_REGULAR_ADC_CHANNELS - 1));
    avp++;


    /**DEBUG**/
    wasinhtit++;
    /*if(dbgCount < dbgSize -9) {
      for(i = 1; i < (USED_REGULAR_ADC_CHANNELS / 2); i++) {
	*dbgp = *avp;
	avp++;
	dbgp++;
      }
      avp -= (USED_REGULAR_ADC_CHANNELS / 2)-1;
      dbgCount += (USED_REGULAR_ADC_CHANNELS / 2)-1;
      }*/
    
    /** END DEBUG **/

    for(i = 1; i < (USED_REGULAR_ADC_CHANNELS / 2); i++) {
      *cvp += *avp;
      cvp++;
      avp++;
    }

    /*for( i = 1; i < USED_REGULAR_ADC_CHANNELS / 2; i++) {
      currentValues[i-1 + (secondInterrupt * (USED_REGULAR_ADC_CHANNELS - 1))] += adc_values[i];
      
      }*/
    //clear DMA interrupt pending bit
    DMA1->IFCR = DMA1_FLAG_HT1;
  }


  if(DMA1->ISR & DMA1_IT_TC1) {
    cvp = activeAdcValues->currentValues;
    avp = adc_values;

    wasinadcit++;

    if(secondInterrupt) {
      cvp += (USED_REGULAR_ADC_CHANNELS - 1);

      //GPIOA->BRR |= GPIO_Pin_8;
        //disable continous mode
      configureCurrentMeasurement(actualDirection);
      //clear half transfer finished interrupt pending bit
      //DMA1->IFCR = DMA1_FLAG_HT1;
      //DMA1->IFCR = DMA1_FLAG_GL1;
      //GPIOA->BSRR |= GPIO_Pin_8;
    
      secondInterrupt = 0;
    
      //only increase every second iteration
      (activeAdcValues->currentValueCount)++;
    } else {
      secondInterrupt= 1;
    }

    cvp += (USED_REGULAR_ADC_CHANNELS / 2) -1;
    avp += (USED_REGULAR_ADC_CHANNELS / 2);
    /**DEBUG**/
    /*
    if(dbgCount < dbgSize - 9) {
      for(i = 0; i < (USED_REGULAR_ADC_CHANNELS / 2); i++) {
	*dbgp = *avp;
	avp++;
	dbgp++;
      }
      avp -= (USED_REGULAR_ADC_CHANNELS / 2);
      dbgCount += (USED_REGULAR_ADC_CHANNELS / 2);
      }*/
    /** END DEBUG **/
    
    
    for(i = 0; i < (USED_REGULAR_ADC_CHANNELS / 2); i++) {
      *cvp += *avp;
      cvp++;
      avp++;
    }

    /*for( i = USED_REGULAR_ADC_CHANNELS / 2; i < USED_REGULAR_ADC_CHANNELS; i++) {
      currentValues[i-1 + (secondInterrupt * (USED_REGULAR_ADC_CHANNELS - 1))] += adc_values[i];
      }*/

    //there was a request to switch the adc values
    //this has to be done here, to garantie, the
    //valid state of the values
    if(switchAdcValues && !secondInterrupt) {
      volatile struct adcValues *tempAdcValues;

      tempAdcValues = activeAdcValues;
      activeAdcValues = inActiveAdcValues;
      inActiveAdcValues = tempAdcValues;
      switchAdcValues = 0;
    }
    

    /**DEBUG**/
    /*if(resetdbgCount && !secondInterrupt) {
	dbgCount = 40;
	resetdbgCount = 0;
	}*/
    /** END DEBUG**/
    
    //DMA interrupt pending bit
    DMA1->IFCR = DMA1_FLAG_TC1;
  }

  wasineocit++;  

  //GPIOA->BRR |= GPIO_Pin_8;
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


/*******************************************************************************
* Function Name  : SPI_Configuration
* Description    : Configures the SPI interface.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_Configuration(void)
{

  //TODO is SOFT_NSS suficient

  /* SPI1 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  SPI_InitTypeDef SPI_InitStructure;

  /* Disable SPI1 for configuration */
  SPI_Cmd(SPI2, DISABLE);

  /* SPI1 Master */
  /* SPI2 Config -----------------------------------------------------------*/
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);


  /* Enable SPI1 */
  SPI_Cmd(SPI2, ENABLE);
}

/**
 * pwmvalue in ranges from 0 to 1800
 *
 */
void InitTimerStructAsPWM(volatile TIM_OCInitTypeDef *ocstruct, u16 pwmvalue) {
  assert_param(pwmvalue <= 1800);
  TIM_OCStructInit((TIM_OCInitTypeDef *) ocstruct);
  ocstruct->TIM_OCMode = TIM_OCMode_PWM1;
  ocstruct->TIM_OutputState = TIM_OutputState_Enable;
  ocstruct->TIM_OCPolarity = TIM_OCPolarity_High;
  ocstruct->TIM_Pulse = pwmvalue;
}

void InitTimerStructAsInternalTimer(volatile TIM_OCInitTypeDef *ocstruct, u16 value) {
  TIM_OCStructInit((TIM_OCInitTypeDef *) ocstruct);
  ocstruct->TIM_OCMode = TIM_OCMode_Timing;
  ocstruct->TIM_Pulse = value;
}

volatile TIM_OCInitTypeDef TIM1_OC2InitStructure;
volatile TIM_OCInitTypeDef TIM1_OC3InitStructure;
volatile TIM_OCInitTypeDef TIM1_OC4InitStructure;

volatile TIM_OCInitTypeDef TIM2_OC1InitStructure;
volatile TIM_OCInitTypeDef TIM2_OC2InitStructure;
volatile TIM_OCInitTypeDef TIM2_OC3InitStructure;

volatile TIM_OCInitTypeDef TIM3_OC1InitStructure;
volatile TIM_OCInitTypeDef TIM3_OC2InitStructure;


void InitTimerStructs() {

  //Current Measurement
  InitTimerStructAsPWM(&TIM1_OC2InitStructure, 750);
  //TIM1_OC2InitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM1_OC2InitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;  

  //start adc watchdog
  InitTimerStructAsPWM(&TIM1_OC3InitStructure, 1500);
  //TIM1_OC3InitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM1_OC3InitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;

  //atomar config timer / Resetup AIN/BIN
  InitTimerStructAsInternalTimer(&TIM1_OC4InitStructure, 0);
  

  //ADIR and BDIR
  InitTimerStructAsPWM(&TIM2_OC1InitStructure, 1500);
  InitTimerStructAsPWM(&TIM2_OC2InitStructure, 1500);

  //Security Timer
  InitTimerStructAsInternalTimer(&TIM2_OC3InitStructure, 1550);
  
  //ASD and BSD (OFF !)
  InitTimerStructAsPWM(&TIM3_OC1InitStructure, 00);
  InitTimerStructAsPWM(&TIM3_OC2InitStructure, 00);
}


/*******************************************************************************
* Function Name  : TIM_Configuration
* Description    : Configures the different Timers.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_Configuration(void)
{

  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

  //init OC struct for timers
  InitTimerStructs();

  //turn on timer hardware
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  // Time base configuration 
  TIM_TimeBaseStructure.TIM_Period = 65000;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  //configure TIM4 as encoder interface
  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,
			     TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  
  //configure value to reload after wrap around
  TIM_SetAutoreload(TIM4, HALF_WHEEL_TURN_TICKS);

  TIM_ARRPreloadConfig(TIM4, ENABLE);

  //signal needs to be 8 clock cycles stable
  TIM4->CCMR1 |= (3<<4) | (3<<12);
  TIM4->CCMR2 |= (3<<4) | (3<<12);

  // TIM enable counter
  TIM_Cmd(TIM4, ENABLE);

  //timer used for PWM generation
  //turn on timer hardware
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  
  //TIM2CLK = 72 MHz, Period = 1800, TIM2 
  //counter clock = 40 kHz 
  // Time base configuration 
  TIM_TimeBaseStructure.TIM_Period = 1800;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  // Prescaler configuration 
  TIM_PrescalerConfig(TIM2, 0, TIM_PSCReloadMode_Immediate); 

  //All channels use buffered registers
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM2, ENABLE);

  //timer used for shutdown generation
  //turn on timer hardware
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  
  //TIM3CLK = 72 MHz, Period = 1800, TIM3 
  //counter clock = 40 kHz 
  // Time base configuration 
  TIM_TimeBaseStructure.TIM_Period = 1800;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  // Prescaler configuration  
  TIM_PrescalerConfig(TIM3, 0, TIM_PSCReloadMode_Immediate); 

  //All channels use buffered registers
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM3, ENABLE);


  //timer used for shutdown generation
  //turn on timer hardware
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  
  //TIM1CLK = 72 MHz, Period = 1800, TIM1 
  //counter clock = 40 kHz 
  // Time base configuration 
  TIM_TimeBaseStructure.TIM_Period = 1800;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  // Prescaler configuration  
  TIM_PrescalerConfig(TIM1, 0, TIM_PSCReloadMode_Immediate); 

  //All channels use buffered registers
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM1, ENABLE);

  TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
  TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);
  TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);

  //TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Reset);
  //use Update, as reset dosen't work for synchronising (reason unknown)
  TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);


  //Sync Timers
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
  TIM_SelectInputTrigger(TIM2, TIM_TS_ITR0);
  TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
  TIM_SelectInputTrigger(TIM3, TIM_TS_ITR0);

  //enable interrupts
  //TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
  TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);

  //interrupt for atomar config change
  TIM_ITConfig(TIM1, TIM_IT_CC4, ENABLE);
  TIM_OC4Init(TIM1, (TIM_OCInitTypeDef *) (&TIM1_OC4InitStructure));

  TIM_OC2Init(TIM1, (TIM_OCInitTypeDef *) (&TIM1_OC2InitStructure));
  TIM_OC3Init(TIM1, (TIM_OCInitTypeDef *) (&TIM1_OC3InitStructure));
  TIM_OC1Init(TIM2, (TIM_OCInitTypeDef *) (&TIM2_OC1InitStructure));
  TIM_OC2Init(TIM2, (TIM_OCInitTypeDef *) (&TIM2_OC2InitStructure));
  TIM_OC3Init(TIM2, (TIM_OCInitTypeDef *) (&TIM2_OC3InitStructure));
  TIM_OC1Init(TIM3, (TIM_OCInitTypeDef *) (&TIM3_OC1InitStructure));
  TIM_OC2Init(TIM3, (TIM_OCInitTypeDef *) (&TIM3_OC2InitStructure));


  // TIM3 enable counter 
  TIM_Cmd(TIM1, ENABLE);

  // TIM3 enable counter 
  TIM_Cmd(TIM3, ENABLE);

  // TIM2 enable counter 
  TIM_Cmd(TIM2, ENABLE);
}


/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
******************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  //get default GPIO config
  GPIO_StructInit(&GPIO_InitStructure);

  /* Enable GPIOA, GPIOD, USB_DISCONNECT(GPIOC) and USART1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB 
			 | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD
                         | RCC_APB2Periph_USART1, ENABLE);

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  //configure TIM2 channel 1 as Push Pull (for AIN)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //configure TIM2 channel 2 as Push Pull (for BIN)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //configure PA2 (ADC Channel2) as analog input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //configure PA3 (ADC Channel3) as analog input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //configure PA4 (ADC Channel4) as analog input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //configure PA5 (ADC Channel5) as analog input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //TIM3 channel 1 pin (PA6)
  //configure TIM3 channel 1 as Push Pull (for ASD)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  //TIM3 channel 2 pin (PA7)
  //configure TIM3 channel 2 as Push Pull (for BSD)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //LED (PA8)
  //configure as Push Pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Configure USART1 Tx (PA09) as alternate function push-pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Configure USART1 Rx (PA10) as input floating
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //Configure CAN pin: RX 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //Configure CAN pin: TX 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  
  //configure PB0 (ADC Channel8) as analog input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //configure pb 1 input pull up (maeuseklavier)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //TODO perhaps OD is wrong for SMBA !!
  // Configure SMBA
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //configure Timer4 ch1 (PB6) as encoder input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  //configure Timer4 ch2 (PB7) as encoder input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //Configure I2C1 Pins, SDA and SCL
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  // Configure I2C2 pins: SCL and SDA 
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  //Configure SPI2 pins: SCK, MISO and MOSI
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Configure PC.12 as output push-pull (LED)
  GPIO_WriteBit(GPIOC,GPIO_Pin_12,Bit_SET);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //configure pc 13/14/15 as input pull up (maeuseklavier)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /** DEBUG**/
  // Configure I2C2 pins: SCL and SDA as push pull for testLED
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  /** END DEBUG**/
  
  
}


/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures NVIC and Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_StructInit(&NVIC_InitStructure);

  /* Set the Vector Table base location at 0x08000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   

  /* 2 bit for pre-emption priority, 2 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
   
  /* Configure and enable I2C1 interrupt ------------------------------------*/
  /*NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQChannel;
  NVIC_Init(&NVIC_InitStructure);*/

  /* Configure and enable I2C2 interrupt ------------------------------------*/
  /*NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQChannel;
  NVIC_Init(&NVIC_InitStructure);*/

  //Configure and enable TIM1 Update interrupt
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  //Configure and enable TIM1 Update interrupt
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  //Configure and enable TIM2 interrupt
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable ADC interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable ADC interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable USB interrupt -------------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable CAN RX0 interrupt IRQ channel */
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN_RX0_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable CAN RX0 interrupt IRQ channel */
  NVIC_InitStructure.NVIC_IRQChannel = CAN_RX1_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Systick with Preemption Priority 2 and Sub Priority as 0 */ 
  NVIC_SystemHandlerPriorityConfig(SystemHandler_SysTick, 3, 0);
}

/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  printf("Wrong parameters value: file %s on line %d\n", file, (int) line);

  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA, ENABLE);

  // Configure PA.8 as output push-pull (LED)
  GPIO_WriteBit(GPIOA,GPIO_Pin_8,Bit_SET);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  volatile int delay;
  int waittime = 500000;
  


  while(1)
    {    
      GPIO_SetBits(GPIOA, GPIO_Pin_8);
      delay = waittime;
      while(delay) {
	delay--;
      }
      
      GPIO_ResetBits(GPIOA, GPIO_Pin_8);
      delay = waittime;
      while(delay) {
	delay--;
      }
    }

  /* Infinite loop */
  while (1)
  {
  }
}

/*
void *memcpy(void *dest, const void *src, size_t n) {
  size_t i;
  u8 *d = (u8 *) dest;
  u8 *s = (u8 *) src;
  

  for(i = 0; i < n; i++) {
    *d = *s;
    d++;
    s++;
  }

  return dest;
  }*/
