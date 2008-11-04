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
#include "stdio.h"
#include "spi.h"
#include "i2c.h"
#include "adc.h"
#include "pid.h"
#include "usart.h"
#include "printf.h"
#include "packet.h"
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

#define dbgSize 400
vu16 dbgValue[dbgSize];
vu16 dbgCount = 0;
vu8 resetdbgCount = 0;
vu32 batValue = 0;

vu16 adcValueCount = 0;

vu32 acs712BaseVoltage = 0;


struct ControllerState {
  enum controllerModes controllMode;
  struct pid_data pid_data;
  u16 pwmStepPerMillisecond;
  s32 targetValue;
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

  TIM_Configuration();

  //SPI_Configuration();
  
  ADC_Configuration();

  print("Loop start \n");

  delay = 5000000;
  while(delay)
    delay--;

  volatile struct ControllerState cs1;
  volatile struct ControllerState cs2;
  
  activeCState = &(cs1);
  lastActiveCState = &(cs2);

  setKp((struct pid_data *) &(activeCState->pid_data), 100);
  setKi((struct pid_data *) &(activeCState->pid_data), 0);
  setKd((struct pid_data *) &(activeCState->pid_data), 0);

  activeCState->controllMode = CONTROLLER_MODE_HALT;
  *lastActiveCState = *activeCState;

  u8 rxBuffer[100];
  u32 rxCount = 0;

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


  
  print("Loop start \n");
 
  /** DEBUG**/
  u16 lastTime = 0;
  u16 time;
  u16 counter = 0;  
  activeCState->targetValue = 880;
  activeCState->controllMode = CONTROLLER_MODE_PWM;
  activeCState->pwmStepPerMillisecond = 3;
  *lastActiveCState = *activeCState;

  /* Enable AWD interupt */
  ADC_ITConfig(ADC2, ADC_IT_AWD, ENABLE);
  
  TIM_CtrlPWMOutputs(TIM1, ENABLE);  
    
  print("Loop start 1\n");
  
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
      

      for(i = 0; i < USED_REGULAR_ADC_CHANNELS*2; i++) {
	printf("ADC value %d  was %d \n",i, dbgValue[i]);
      }

      printf("Mean Bat Value is %lu \n", (batValue *16 * 33 * 100) / 4096);
      printf("ACS712 base Voltage is %lu \n", acs712BaseVoltage);

      //voltage divider is 33/60
      //100 mV is 1A
      //1680 is adc value without load
      //adc sample time is 722.22 nsecs
      u32 convCurrent = currentValue;
      printf("RAW Mean current Value is %lu \n", convCurrent);

      
      
      
      //multiply by pwm lenght time
      //u32 measurementToPWMLenght = (25000 * 880 / 1800) / (722 * 14) ;
      //convCurrent = (convCurrent * 880) / (1800 * 14);
      //convCurrent = (convCurrent / 14);
      
	
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
      
      /*if(encoderValue < 500)
	GPIO_SetBits(GPIOB, GPIO_Pin_6);
	else 
	GPIO_ResetBits(GPIOB, GPIO_Pin_6);
      */

      counter = 0;
    }
    
    time = TIM_GetCounter(TIM1);
    if(lastTime > time) {
      counter++;
    }
    lastTime = time;
    

    /* END DEBUG */


    //receive and process data
    u32 len = USART1_GetData(rxBuffer + rxCount, 100 - rxCount);
    rxCount += len;
    
    if(len != 0) {
      
      printf("Got data len is %d", len);
    }

    if(USART1_Data.RxBufferFullError) {
      print("Error buffer is full \n");
    }
    
    
    //process data
    while(rxCount > sizeof(struct packetHeader)) {
      print("Got a Packet !\n");

      struct packetHeader *ph = (struct packetHeader *) rxBuffer;

      u16 packetSize = sizeof(struct packetHeader) + ph->length;

      //check if whole packet was received
      if(rxCount < packetSize) {
	break;
      }

      printf("receiver ID is %hu !\n", ph->receiverID);
      
      if(ph->receiverID == ownReceiverID || ph->receiverID == RECEIVER_ID_H_BRIDGE_ALL) {
	printf("ID is %hu !\n", ph->id);
      
	switch(ph->id) {
	case PACKET_ID_SET_PWM:
	  lastActiveCState->controllMode = CONTROLLER_MODE_PWM;
	  signed short *value = (signed short *) (rxBuffer + sizeof(struct packetHeader));
	  lastActiveCState->targetValue = *value;
	  printf("Got PWM Value %d\n", *value);
	  
	  break;
	  
	case PACKET_ID_EMERGENCY_STOP:
	  break;
	default:
	  break;
	}
      }

      //TODO, this is slow
      //copy bytes down
      int i;
      for(i = 0; i < rxCount - packetSize; i++) {
	rxBuffer[i] = rxBuffer[packetSize + i];
      }
      //decrease rxCount by size of processed packet
      rxCount -= packetSize;
    

      //this is concurrency proff, as this code can not run, while
      //systick Handler is active !
      volatile struct ControllerState *tempstate = activeCState;
      
      //this is atomar as only the write is relevant!
      activeCState = lastActiveCState;

      *lastActiveCState = *activeCState;
 
      lastActiveCState = tempstate;
    }
  }
}

void SysTickHandler(void) {

  //request switch of adc value struct
  switchAdcValues = 1;
  

  //GPIOA->BSRR |= GPIO_Pin_8;

  u16 encoderValue = 0;
  static u16 lastEncoderValue = 0;
  static s32 currentPwmValue = 0;

  s32 curSpeed = 0;
  s32 pwmValue = 0;

  //get encoder
  encoderValue = TIM_GetCounter(TIM4);

  int i, usableCurrentValues = 0;
  //length of 13Cycle Sample time is 722nsecs 
  //during 21660nsecs samples are taken, this 
  //is equal to a pwm value of 1559.52
  if(currentPwmValue < 1559) {
    usableCurrentValues = currentPwmValue / 52;
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
  for( i = 0; i < USED_REGULAR_ADC_CHANNELS*2; i++) {
    dbgValue[i] = currentValues[i] / adcValueCount;
  }

  dbgValue[31] = adcValueCount;

  dbgValue[30] = 0;
  
  /**END DEBUG**/


  //batValueSum and currentValueSum are increased at the 
  //same time therefore adcValueCount is valid for both
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
  currentValue = (currentValue * 60) / 33;
  
  //convert from adc to volts
  currentValue = (currentValue * 3300) / 4096;


  //set rest to zero
  for(i = usableCurrentValues; i < (USED_REGULAR_ADC_CHANNELS -1) *2; i++) {
    currentValues[i] = 0;
  }

  inActiveAdcValues->currentValueCount = 0;
  
  switch(activeCState->controllMode) {
    case CONTROLLER_MODE_HALT:
      pwmValue = 0;
      break;
      
    case CONTROLLER_MODE_PWM:
      pwmValue = activeCState->targetValue;
      
      break;
    
    case CONTROLLER_MODE_SPEED:
      //TODO FIXME convert to a real value like m/s
      //FIXME Wrap around
      curSpeed = encoderValue - lastEncoderValue;
      
      //calculate PID value
      setTargetValue((struct pid_data *) &(activeCState->pid_data), activeCState->targetValue);
      pwmValue = pid((struct pid_data *) &(activeCState->pid_data), curSpeed);
      break;
    
    case CONTROLLER_MODE_POSITION:
      break;
  }
  //todo truncate pwmValue s32 -> s16

  if(abs(currentPwmValue - pwmValue) < activeCState-> pwmStepPerMillisecond) {
    currentPwmValue = pwmValue;
  } else {
    if(currentPwmValue - pwmValue < 0) {
      currentPwmValue += activeCState-> pwmStepPerMillisecond;
    } else {
      currentPwmValue -= activeCState-> pwmStepPerMillisecond;      
    }
  }
  
  
  //set pwm
  setNewPWM( currentPwmValue);
  
  lastEncoderValue = encoderValue;

  //  GPIOA->BRR |= GPIO_Pin_8;

}


void setNewPWM(const s16 value2) {
  //TODO add magic formular for conversation
  s16 value = value2;

  
  //trunkate to min of 8% PWM, as Current Measurement needs 1.777 us
  if((value > 0 && value < 144) || (value < 0 && value > -144))
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
   * AIN __|--|____
   * BIN --|_______
   * ASD ----------
   * BSD --|_______
   *
   */
  if(desieredDirection) {  
    // AIN
    TIM2->CCR1 = dutyTime;
    
    // BIN
    TIM2->CCR2 = dutyTime;
    
    // ASD
    //Set asd to max_pulse+1, so it is on all the time
    TIM3->CCR1 = 1801;

    // BSD
    //the plus 100 is, so that B Lo is switched against
    //ground and Vb can load up
    TIM3->CCR2 = dutyTime+100;
  } else {
    // AIN
    TIM2->CCR1 = dutyTime;
    
    // BIN
    TIM2->CCR2 = dutyTime;
    
    // ASD
    //the plus 100 is, so that A Lo is switched against
    //ground and Vb can load up
    TIM3->CCR1 = dutyTime+100;

    // BSD
    //Set bsd to max_pulse+1, so it is on all the time
    TIM3->CCR2 = 1801;
  }

  //Current Measurement timer
  //take current at 80% of PWM high phase
  TIM1->CCR2 = 1;//(((u32) dutyTime) * 4 )/5;

  if(dutyTime < 1600) {
    //Watchdog enable timer
    TIM1->CCR3 = dutyTime + 50;
    
    //TODO add x to sec timer
    
    //security timer
    TIM2->CCR3 = dutyTime + 150;
  } else {
    //disable the whole watchdog thing, as it won't 
    //trigger anyway on high pwms and only create 
    //race conditions
    
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

    //set High Value of ADC threshold to half battery Voltage
    //batValue is in ADC scale (0 - 4096)
    ADC2->HTR = batValue / 2;

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

  //Force high side gate off
  if(actualDirection) {
    tmpccmr = TIM2->CCMR1;
    //clear mode bits
    tmpccmr &= ((u16)0xFF8F);
    //Configure The Forced output Mode
    tmpccmr |= TIM_ForcedAction_Active;
    //write to register
    TIM2->CCMR1 = tmpccmr;
  } else {
    tmpccmr = TIM2->CCMR1;
    //clear mode bits
    tmpccmr &= ((u16)0x8FFF);
    //Configure The Forced output Mode
    tmpccmr |= (u16)(TIM_ForcedAction_Active << 8);
    //write to register
    TIM2->CCMR1 = tmpccmr;
  }

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
    if(!ADC_GetFlagStatus(ADC2, ADC_FLAG_AWD)) {
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
  //GPIOA->BSRR |= GPIO_Pin_8;
  //wasinadcit = 1;

  //test if Analog Watchdog Flag is set.
  //this means the souce of the IT was AWD
  if(ADC2->SR & ADC_FLAG_AWD) {
    ForceHighSideOffAndDisableWatchdog();
    //clear interrupt source
    ADC2->SR = ~(u32) ADC_FLAG_AWD;   

    wasinawdit++;
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

  //GPIOA->BRR |= GPIO_Pin_8;

}

void DMA1_Channel1_IRQHandler(void) {
  GPIOA->BSRR |= GPIO_Pin_8;
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

      GPIOA->BRR |= GPIO_Pin_8;
        //disable continous mode
      configureCurrentMeasurement(actualDirection);
      //clear half transfer finished interrupt pending bit
      //DMA1->IFCR = DMA1_FLAG_HT1;
      //DMA1->IFCR = DMA1_FLAG_GL1;
      GPIOA->BSRR |= GPIO_Pin_8;
    
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

  GPIOA->BRR |= GPIO_Pin_8;
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
  TIM_TimeBaseStructure.TIM_Prescaler = 2;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  //configure TIM4 as encoder interface
  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,
			     TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  
  //configure value to reload after wrap around
  TIM_SetAutoreload(TIM4, 1<<15);

  TIM_ARRPreloadConfig(TIM4, ENABLE);

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
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
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
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  //Configure and enable TIM1 Update interrupt
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  //Configure and enable TIM2 interrupt
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
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
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable USB interrupt -------------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Systick with Preemption Priority 2 and Sub Priority as 0 */ 
  NVIC_SystemHandlerPriorityConfig(SystemHandler_SysTick, 2, 0);
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

  // Configure PC.12 as output push-pull (LED)
  GPIO_WriteBit(GPIOC,GPIO_Pin_12,Bit_SET);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  int delay;

  while(1)
    {    
      GPIOC->BRR |= 0x00001000;
      delay = 500000;
      while(delay) 
	delay--;
      GPIOC->BSRR |= 0x00001000;
      delay = 500000;
      while(delay)
	delay--; 
    }

  /* Infinite loop */
  while (1)
  {
  }
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
