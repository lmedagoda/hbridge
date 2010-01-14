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
#include "inc/stm32f10x_lib.h"
#include "inc/stm32f10x_gpio.h"
#include "inc/stm32f10x_tim.h"
#include "stm32f10x_it.h"
#include "inc/stm32f10x_can.h"
#include "stdio.h"
#include "spi.h"
#include "i2c.h"
#include "pid.h"
#include "usart.h"
#include "printf.h"
#include "protocol.h"
#include "can.h"
#include "state.h"
#include "controllers.h"
#include "hbridge.h"
#include "encoder.h"
#include "current_measurement.h"
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

void SysTick_Configuration(void);

#define MIN_S16 (-(1<<15))
#define MAX_S16 ((1<<15) -1)

volatile enum hostIDs ownHostId;


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


  //Enable peripheral clock for GPIO
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

  NVIC_Configuration();

  GPIO_Configuration();
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);

  USART_Configuration();
  //setupI2C();

  //turn of red led
  GPIO_SetBits(GPIOA, GPIO_Pin_8);

  //read out dip switches
  ownHostId = getOwnHostId();
  
  CAN_Configuration();
  CAN_ConfigureFilters(ownHostId);

  currentMeasurementInit();

  encoderInit();
  
  print("Loop start 1\n");

  delay = 5000000;
  while(delay)
    delay--;

  volatile struct ControllerState cs1;
  volatile struct ControllerState cs2;
  
  activeCState = &(cs1);
  lastActiveCState = &(cs2);

  initControllers();

  //init cotroller state with sane values
  initStateStruct(activeCState);
  initStateStruct(lastActiveCState);

  hbridgeInit();
  
  measureACS712BaseVoltage(); 

  //activate systick interrupt, at this point
  //activeCState1 hast to be initalized completely sane !
  SysTick_Configuration();


  
  print("Loop start 2\n");
 
  /** DEBUG**/
  u16 lastTime = 0;
  u16 time;
  u16 counter = 0;  

  /* Enable AWD interupt */
  //ADC_ITConfig(ADC2, ADC_IT_AWD, ENABLE);
    
  print("Loop start 3\n");

  /** END DEBUG **/
 
  while(1) {

    /** START DEBUG **/

    if(counter > 10000) {  
      counter = 0;
      print(".");
      printf("Error is %h \n", error);
      print("ActiveCstate: ");
      printStateDebug(activeCState);
      print("LastActiveCstate: ");
      printStateDebug(lastActiveCState);
    }
    /** END DEBUG **/
    
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

      updateStateFromMsg(curMsg, lastActiveCState, ownHostId);
    
      //mark current message als processed
      CAN_MarkNextDataAsRead();

      //this is concurrency proff, as this code can not run, while
      //systick Handler is active !
      volatile struct ControllerState *tempstate = activeCState;

      //we go an new packet, so reset the timeout
      lastActiveCState->resetTimeoutCounter = 1;
      
      //this is atomar as only the write is relevant!
      activeCState = lastActiveCState;

      lastActiveCState = tempstate;
      
      *lastActiveCState = *activeCState;      
      
      printf("Error is %h \n", error);
      print("ActiveCstate: ");
      printStateDebug(activeCState);
      print("LastActiveCstate: ");
      printStateDebug(lastActiveCState);
    } 
  }
}

void SysTickHandler(void) {  
  //request switch of adc value struct
  requestNewADCValues();
  
  //GPIOA->BSRR |= GPIO_Pin_8;

  static s32 currentPwmValue = 0;
  static u16 index = 0;
  static u16 overCurrentCounter = 0;
  static u16 timeoutCounter = 0;

  s32 pwmValue = 0;
  
  //wait for adc struct to be switched
  waitForNewADCValues();

  u32 currentValue = calculateCurrent(currentPwmValue);

  //reset timeout, if "userspace" requested it
  if(activeCState->resetTimeoutCounter) {
    activeCState->resetTimeoutCounter = 0;
    timeoutCounter = 0;
  }

  //change state to unconfigured if error is set
  if(error != ERROR_CODE_NONE && activeCState->internalState == STATE_CONFIGURED) {
    activeCState->internalState = STATE_ERROR;
  }

  //only check for overcurrent if configured
  if(activeCState->internalState == STATE_CONFIGURED) {
    //check for overcurrent
    if(currentValue > activeCState->maxCurrent) {
      overCurrentCounter++;

      if(overCurrentCounter > activeCState->maxCurrentCount) {
	activeCState->internalState = STATE_ERROR;
	error |= ERROR_CODE_OVERCURRENT;
      }
    } else {
      overCurrentCounter = 0;
    }
    
    //check for timeout and go into error state
    if(activeCState->timeout && (timeoutCounter > activeCState->timeout)) {
      activeCState->internalState = STATE_ERROR;
      error |= ERROR_CODE_TIMEOUT;      
    }
  } else {
    //reset to zero values
    overCurrentCounter = 0;
    timeoutCounter = 0;
  }

  s32 wheelPos = getTicks();
  
    setNewSpeedPIDValues(activeCState->speedPIDValues.kp, activeCState->speedPIDValues.ki, activeCState->speedPIDValues.kd, activeCState->speedPIDValues.minMaxPidOutput);
    setNewPosPIDValues(activeCState->positionPIDValues.kp, activeCState->positionPIDValues.ki, activeCState->positionPIDValues.kd, activeCState->positionPIDValues.minMaxPidOutput);
  
  //calculate pwm value
  switch(activeCState->controllMode) {
    case CONTROLLER_MODE_PWM:
      pwmValue = activeCState->targetValue;
      break;
    
    case CONTROLLER_MODE_POSITION: {
      if(activeCState->cascadedPositionController) {
	    pwmValue = cascadedPositionController(activeCState->targetValue, wheelPos, activeCState->ticksPerTurn, activeCState->enablePIDDebug);
      } else {
	    pwmValue = positionController(activeCState->targetValue, wheelPos, activeCState->ticksPerTurn, activeCState->enablePIDDebug);
      }
      
    case CONTROLLER_MODE_SPEED:
	    pwmValue = speedController(activeCState->targetValue, wheelPos, activeCState->ticksPerTurn, activeCState->enablePIDDebug);
      break; 
    }
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

  //send out status message
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
    sdata->pwm = currentPwmValue;
    //TODO calculate temp
    //sdata->tempHBrigde = 0;
    //sdata->tempMotor = 0;
    sdata->error = error;
    
    if(CAN_Transmit(&statusMessage) == CAN_NO_MB) {
      print("Error Tranmitting status Message : No free TxMailbox \n");
    } else {
      //print("Tranmitting status Message : OK \n");  
    }
  }

  if(activeCState->internalState == STATE_CONFIGURED) {
    //increase timeout
    timeoutCounter++;

    //set pwm
    setNewPWM(currentPwmValue, activeCState->useOpenLoop, activeCState->useBackInduction);
  } else {
    //reset timeoutcounter
    timeoutCounter = 0;
    setNewPWM(0, activeCState->useOpenLoop, activeCState->useBackInduction);

    //reset PID struct, to avoid bad controller 
    //behavior an reactivation due to big I part
    resetControllers();
  }
 

  //  GPIOA->BRR |= GPIO_Pin_8;

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
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
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
