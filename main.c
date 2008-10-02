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

volatile TIM_OCInitTypeDef TIM1_OC2InitStructure;
volatile TIM_OCInitTypeDef TIM1_OC3InitStructure;
volatile TIM_OCInitTypeDef TIM1_OC4InitStructure;

volatile TIM_OCInitTypeDef TIM2_OC1InitStructure;
volatile TIM_OCInitTypeDef TIM2_OC2InitStructure;
volatile TIM_OCInitTypeDef TIM2_OC3InitStructure;

volatile TIM_OCInitTypeDef TIM3_OC1InitStructure;
volatile TIM_OCInitTypeDef TIM3_OC2InitStructure;

vu8 desieredDirection = 0;
vu8 actualDirection = 0;
volatile u8 newPWM = 0;

vu8 pwmBiggerXPercent;


vu16 wantedPWM = 0;
vu16 currentPWM = 0;

vu8 pidEnabled = 0;

void setNewPWM(const s16 value);

vu8 wasinit = 0;
vu8 wasinif = 0;

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
  s32 wantedSpeed = 0;
  struct pid_data pid_data;
  

  //Enable peripheral clock for GPIO
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

  /* Enable peripheral clocks ---------------------------------------*/
  /* Enable I2C1 and I2C2 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

  /* Enable USART1 clock */
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);


  NVIC_Configuration();

  GPIO_Configuration();

  USART_Configuration();

  //setupI2C();

  TIM_Configuration();

  //SPI_Configuration();
  
  //ADC_Configuration();

  delay = 5000000;
  while(delay)
    delay--;
  
  /** DEBUG **/
  
  setNewPWM(500);
  //TIM_UpdateDisableConfig(TIM1, DISABLE);

  //TIM_GenerateEvent(TIM1, TIM_EventSource_Update);

  u16 lastTime = 0;
  u16 time;
  u16 counter = 0;
    
  while(1) {
    u16 tim1Value = TIM_GetCounter(TIM1);
    u16 tim1Value2 = TIM_GetCounter(TIM1);
    u16 tim2Value = TIM_GetCounter(TIM2);
    u16 tim2Value2 = TIM_GetCounter(TIM2);
    u16 tim3Value = TIM_GetCounter(TIM3);
    u16 tim3Value2 = TIM_GetCounter(TIM3);

    u8 wrapnotfound = 1;

    while(wrapnotfound) { 
      tim1Value = TIM_GetCounter(TIM1);
      tim2Value = TIM_GetCounter(TIM2);
      tim3Value = TIM_GetCounter(TIM3);
      if(tim1Value2 > tim1Value) {
	wrapnotfound = 0;
      } else {
	tim1Value2 = tim1Value;
	tim2Value2 = tim2Value;
	tim3Value2 = tim3Value;
      }
    }
    
    u16 debugvalues[128*3];

    u16 *debugprt = debugvalues;

    int i;

    for(i = 0; i < 128; i++) {
      *debugprt = TIM_GetCounter(TIM1);
      debugprt++;
      *debugprt = TIM_GetCounter(TIM2);
      debugprt++;
      *debugprt = TIM_GetCounter(TIM3);
      debugprt++;      
    }
    
    for(i = 0; i < 128*3; i+=3) {
      printf("i is %d TIM1 is %d TIM2 is %d, TIM3 is %d\n", i, debugvalues[i + 0], debugvalues[i + 1], debugvalues[i + 2]);
    }
    
        
    printf("TIM1 is %d \n", tim1Value);
    printf("TIM2 is %d \n", tim2Value);
    printf("TIM3 is %d \n", tim3Value);

    printf("last TIM1 is %d \n", tim1Value2);
    printf("last TIM2 is %d \n", tim2Value2);
    printf("last TIM3 is %d \n", tim3Value2);

    printf("Update flag set %d \n", TIM_GetFlagStatus(TIM1, TIM_FLAG_Update));
    printf("Update Interrupt enabled %d \n", TIM_GetITStatus(TIM1, TIM_IT_Update));
    
    printf("NewPWM is %d \n", newPWM);
    printf("wasinif is %d \n", wasinif);
    printf("Was in Update IT %d \n", wasinit);
  
    print("Loop start \n");

    RCC_ClocksTypeDef RCC_ClocksStatus;
    RCC_GetClocksFreq(&RCC_ClocksStatus);
    
    printf("PCLK2 %lu, PCLK1 %lu\n" ,RCC_ClocksStatus.PCLK2_Frequency ,RCC_ClocksStatus.PCLK1_Frequency);

    /*if(encoderValue < 500)
      GPIO_SetBits(GPIOB, GPIO_Pin_6);
    else 
      GPIO_ResetBits(GPIOB, GPIO_Pin_6);
    */

    lastTime = 0;
    time;
    counter = 0;
    while(counter < 10000) {

      time = TIM_GetCounter(TIM1);
      if(lastTime > time) {
	counter++;
      }
      lastTime = time;
    }
  }

  /* END DEBUG */
  
  print("Loop start \n");
 
  delay = 5000000;
  while(delay)
    delay--;

  setKp(&pid_data, 100);
  setKi(&pid_data, 0);
  setKd(&pid_data, 0);

  u16 encoderValue = 0;
  u16 lastEncoderValue = 0;
  
  s32 curSpeed = 0;
  s32 pwmValue = 0;

  while (1) {
    delay = 10000000;
    while(delay)
      delay--;
    
    //get encoder
    encoderValue = TIM_GetCounter(TIM4);
        
    printf("Encoder is %d \n", encoderValue);
    
    curSpeed = encoderValue - lastEncoderValue;
    
    //calculate PID value
    setTargetValue(&pid_data, wantedSpeed);
    pwmValue = pid(&pid_data, curSpeed);

    //todo truncate pwmValue s32 -> s16

    //set pwm
    setNewPWM( pwmValue);

    lastEncoderValue = encoderValue;
  }
}

void setNewPWM(const s16 value) {
  //TODO add magic formular for conversation
  
  //remove signess
  u16 dutyTime = value & ((1<<16)-1);
  
  //disable transfer of values from config register
  //to timer intern shadow register 
  TIM_UpdateDisableConfig(TIM1, ENABLE);
  TIM_UpdateDisableConfig(TIM2, ENABLE);
  TIM_UpdateDisableConfig(TIM3, ENABLE);
  
  desieredDirection = value > 0;

  /* Modified active field-collapse drive
   *
   *       Voltage Raise
   *          |
   *         \/
   * AIN --|__|----
   * BIN --|_______
   * ASD ----------
   * BSD --|_______
   *
   */
  if(desieredDirection) {  
    TIM2_OC1InitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM2_OC2InitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    
    // AIN
    TIM2_OC1InitStructure.TIM_Pulse = dutyTime;
    
    // BIN
    TIM2_OC2InitStructure.TIM_Pulse = dutyTime;
    
    // ASD
    TIM3_OC1InitStructure.TIM_Pulse = 1800;

    // BSD
    TIM3_OC2InitStructure.TIM_Pulse = dutyTime;
  } else {
    TIM2_OC1InitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM2_OC2InitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;

    // AIN
    TIM2_OC1InitStructure.TIM_Pulse = dutyTime;
    
    // BIN
    TIM2_OC2InitStructure.TIM_Pulse = dutyTime;
    
    // ASD
    TIM3_OC1InitStructure.TIM_Pulse = dutyTime;

    // BSD
    TIM3_OC2InitStructure.TIM_Pulse = 1800;
  }

  //Current Measurement timer
  TIM1_OC2InitStructure.TIM_Pulse = dutyTime/2;

  //Watchdog enable timer
  TIM1_OC3InitStructure.TIM_Pulse = dutyTime;
  
  //TODO add x to sec timer

  //security timer
  TIM2_OC3InitStructure.TIM_Pulse = dutyTime + 100;


  //TODO FIXME, do not use OC init, as it resets the timers during reconfigure
  TIM_OC2Init(TIM1, (TIM_OCInitTypeDef *) (&TIM1_OC2InitStructure));
  TIM_OC3Init(TIM1, (TIM_OCInitTypeDef *) (&TIM1_OC3InitStructure));
  TIM_OC1Init(TIM2, (TIM_OCInitTypeDef *) (&TIM2_OC1InitStructure));
  TIM_OC2Init(TIM2, (TIM_OCInitTypeDef *) (&TIM2_OC2InitStructure));
  TIM_OC3Init(TIM2, (TIM_OCInitTypeDef *) (&TIM2_OC3InitStructure));
  TIM_OC1Init(TIM3, (TIM_OCInitTypeDef *) (&TIM3_OC1InitStructure));
  TIM_OC2Init(TIM3, (TIM_OCInitTypeDef *) (&TIM3_OC2InitStructure));
 
  newPWM = 1;
}

void TIM1_CC_IRQHandler(void) {
  static vu8 NewPWMLastTime = 0;
  
  if(TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET) {
    //Clear TIM1 update interrupt pending bit
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);
  
    //ReInit Output as timer mode (remove Forced high) 
    u16 tmpccmr1 = 0;
    u16 tmpccmr2 = 0;

    //get CCMR1 and CCMR2 content
    tmpccmr1 = TIM2->CCMR1;
    tmpccmr2 = TIM2->CCMR2;
    
    //Reset the OC1M Bits 
    tmpccmr1 &= 0xFF8F;
    tmpccmr2 &= 0xFF8F;
    
    //configure output mode
    tmpccmr1 |= TIM2_OC1InitStructure.TIM_OCMode;
    tmpccmr2 |= TIM2_OC2InitStructure.TIM_OCMode;
    
    //Write to TIM2 CCMR1 & CCMR2 register
    TIM2->CCMR1 = tmpccmr1;
    TIM2->CCMR2 = tmpccmr2;
    

    if(newPWM != 0) {
      wasinif = 1;
      newPWM = 0;
      //Enable update, all previous configured values
      //are now transfered atomic to the timer in the
      //moment the timer wraps around the next time
      TIM_UpdateDisableConfig(TIM1, DISABLE);
      TIM_UpdateDisableConfig(TIM2, DISABLE);
      TIM_UpdateDisableConfig(TIM3, DISABLE);
      NewPWMLastTime = 1;
    }
    
    //timers where updated atomar, so now we reconfigure 
    //the watchdog and current measurement and also update
    //the direction value
    if(NewPWMLastTime) {
      NewPWMLastTime = 0;
      actualDirection = desieredDirection;
      /**DISABLED FOR DEBUG*/
      //configureCurrentMeasurement(actualDirection);
      //configureWatchdog(actualDirection);      
    }
  }
  
}


/**
 * This interrupt Handler handels the update Event
 * In case of an Update Event Ch3 and Ch4 are 
 * reinitalized to Timer mode. 
 * If also a new PWM was set, the Update of the
 * internal shadow register is enabled again, so
 * that config is updated atomar by hardware on
 * the next update event.
 */
void TIM1_UP_IRQHandler(void) {
  static vu8 NewPWMLastTime = 0;

  if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
    

    //Clear TIM1 update interrupt pending bit
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
  
    wasinit = 1;
 
    //ReInit Output as timer mode (remove Forced high) 
    TIM_OC1Init(TIM2, (TIM_OCInitTypeDef *) (&TIM2_OC1InitStructure));
    TIM_OC2Init(TIM2, (TIM_OCInitTypeDef *) (&TIM2_OC2InitStructure));
    
    if(newPWM != 0) {
      wasinif = 1;
      newPWM = 0;
      //Enable update, all previous configured values
      //are now transfered atomic to the timer in the
      //moment the timer wraps around the next time
      TIM_UpdateDisableConfig(TIM1, DISABLE);
      TIM_UpdateDisableConfig(TIM2, DISABLE);
      TIM_UpdateDisableConfig(TIM3, DISABLE);
      NewPWMLastTime = 1;
    }
    
    //timers where updated atomar, so now we reconfigure 
    //the watchdog and current measurement and also update
    //the direction value
    if(NewPWMLastTime) {
      NewPWMLastTime = 0;
      actualDirection = desieredDirection;
      /**DISABLED FOR DEBUG*/
      //configureCurrentMeasurement(actualDirection);
      //configureWatchdog(actualDirection);      
    }
  }

}

/**
 * Interrupt function used for security interrupt
 * This function forces the high side gate on, for
 * the case that the adc watchdog didn't trigger
 * also the Watchdog is disabled and set up for
 * the next trigger.
 */
void TIM2_IRQHandler(void) {
  if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET) {
    /* Clear TIM2 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
    
    //Force high side gate on
    if(actualDirection) {
      TIM_ForcedOC1Config(TIM2, TIM_ForcedAction_Active);
    } else {
      TIM_ForcedOC2Config(TIM2, TIM_ForcedAction_Active);
    }

    /**DISABLED FOR DEBUG*/
    //disable adc watchdog and set it up again for next trigger
    //configureWatchdog(actualDirection);      
  }
}

/**
 * This is the ADC interrupt function
 * In case the source of the interrupt was an 
 * End of Conversation this means we just took
 * the Current values. 
 */
void ADC1_Interrupt() {
  //End of Conversion
  if(ADC_GetITStatus(ADC1, ADC_IT_EOC)) {
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    //TODO do something with the value
  }
}

/**
 * This is the ADC interrupt function
 * In case the source was the analog watchdog
 * the motor voltage just reverted, because of 
 * reinduction and we need to turn on the 
 * high side gate of the H-Bridge
 */
void ADC2_Interrupt() {
  //Analog Watchdog
  if(ADC_GetITStatus(ADC2, ADC_IT_AWD)) {
    ADC_ClearFlag(ADC2, ADC_FLAG_AWD);
    //Force high side gate on
    if(actualDirection) {
      TIM_ForcedOC3Config(TIM2, TIM_ForcedAction_Active);
    } else {
      TIM_ForcedOC4Config(TIM2, TIM_ForcedAction_Active);
    }

    //Disable Watchdog interupt
    ADC_ITConfig(ADC2, ADC_IT_AWD, DISABLE);
  }
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


void InitTimerStructs() {

  //Current Measurement
  InitTimerStructAsInternalTimer(&TIM1_OC2InitStructure, 750);

  //start adc watchdog
  InitTimerStructAsInternalTimer(&TIM1_OC3InitStructure, 1500);

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

  /* Configure and enable USB interrupt -------------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);


  /* Configure and enable SPI1 interrupt ------------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
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
