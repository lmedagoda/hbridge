#include "inc/stm32f10x_tim.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_adc.h"
#include "inc/stm32f10x_gpio.h"
#include "hbridge.h"
#include "current_measurement.h"
#include <stdlib.h>

/// direction, that was given to setNewPWM
vu8 desiredDirection = 2;

/// direction in which the hbridge is configured atm
vu8 actualDirection = 0;

///indicates, if a new pwm-value was set since the last pwm-cycle
volatile u8 newPWM = 0;

volatile TIM_OCInitTypeDef TIM1_OC2InitStructure;
volatile TIM_OCInitTypeDef TIM1_OC3InitStructure;
volatile TIM_OCInitTypeDef TIM1_OC4InitStructure;

volatile TIM_OCInitTypeDef TIM2_OC1InitStructure;
volatile TIM_OCInitTypeDef TIM2_OC2InitStructure;
volatile TIM_OCInitTypeDef TIM2_OC3InitStructure;

volatile TIM_OCInitTypeDef TIM3_OC1InitStructure;
volatile TIM_OCInitTypeDef TIM3_OC2InitStructure;

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
  InitTimerStructAsPWM(&TIM1_OC2InitStructure, 750);
  TIM1_OC2InitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;  

  //start adc watchdog
  InitTimerStructAsPWM(&TIM1_OC3InitStructure, 1500);
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

void initHbridgeTimers()
{

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    //TIM2CLK = 72 MHz, Period = 1800
    //counter clock = 40 kHz 
    // Time base configuration 
    TIM_TimeBaseStructure.TIM_Period = 1800;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  
    //init OC struct for timers
    InitTimerStructs();

    //timer used for PWM generation
    //turn on timer hardware
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    //init timer to 40khz
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
    
    //init timer to 40khz
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // Prescaler configuration  
    TIM_PrescalerConfig(TIM3, 0, TIM_PSCReloadMode_Immediate); 

    //All channels use buffered registers
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM3, ENABLE);


    //turn on timer hardware
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    
    //init timer to 40khz
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

    //use Update, as reset doesn't work for synchronising (reason unknown)
    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);

    //Sync Timers
    TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
    TIM_SelectInputTrigger(TIM2, TIM_TS_ITR0);
    TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
    TIM_SelectInputTrigger(TIM3, TIM_TS_ITR0);

    //enable interrupts
    TIM_ITConfig(TIM2, TIM_IT_CC3, DISABLE);

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


    // TIM1 enable counter
    TIM_Cmd(TIM1, ENABLE);

    // TIM3 enable counter 
    TIM_Cmd(TIM3, ENABLE);

    // TIM2 enable counter 
    TIM_Cmd(TIM2, ENABLE);
}

void hbridgeGPIOConfig() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    //get default GPIO config
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

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

    //configure TIM2 channel 1 as Push Pull (for AIN)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //configure TIM2 channel 2 as Push Pull (for BIN)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
}

void hbridgeInit() {
    //init gpios
    hbridgeGPIOConfig();
    
    //init all timers
    initHbridgeTimers();

    //set pwm to zero
    setNewPWM(0, 0);
    
}

void setNewPWM(const s16 value2, u8 useOpenLoop) {
    
  s16 value = value2;

  //truncate to max of 95% PWM, needed to charge boost circuit
  if(value < -1710)
  {
    value = -1710;
  }
  else
  {
    if(value > 1710)
    {
        value = 1710;
    }
  }  

  u16 dutyTime = abs(value);
  
  //disable transfer of values from config register
  //to timer intern shadow register 
  TIM_UpdateDisableConfig(TIM1, ENABLE);
  TIM_UpdateDisableConfig(TIM2, ENABLE);
  TIM_UpdateDisableConfig(TIM3, ENABLE);

  if(value != 0) {
    desiredDirection = value > 0;
  }
  
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
  if(desiredDirection) {
    //AIN 
    TIM2->CCR1 = 1801;

    // BIN
    TIM2->CCR2 = dutyTime;

    if(useOpenLoop) {
	// ASD
	TIM3->CCR1 = 1801;

	//BSD
	//switch on B-low for a short time, so boost voltage
	//can recharge
	TIM3->CCR2 = dutyTime + 100;
    } else {
        //closed loop
	//ASD high, all the time, do not shutdown on switch
	TIM3->CCR1 = 1801;

	//BSD high, all the time, do not shutdown on switch
	TIM3->CCR2 = 1801;
    }    
  } else {
    // AIN
    TIM2->CCR1 = dutyTime;
    
    // BIN
    TIM2->CCR2 = 1801;
    
    if(useOpenLoop) {
	// BSD
	TIM3->CCR2 = 1801;
	
	//ASD
	//switch on A-low for a short time, so boost voltage
	//can recharge
	TIM3->CCR1 = dutyTime + 100;
    } else {
        //closed loop
	//ASD high, all the time, do not shutdown on switch
	TIM3->CCR1 = 1801;

	//BSD high, all the time, do not shutdown on switch
	TIM3->CCR2 = 1801;
    }
  }

  //Current Measurement timer
  //triger adc conversion at start of PWM interval
  TIM1->CCR2 = 1;
  
  //we had a direction change, disable H-Bridge
  //and reconfigure polarity
  if(actualDirection != desiredDirection) {

    //Disable PWM by pulling ADSD and BDSD low 
    u16 tmpccer = TIM3->CCER;

    tmpccer &= (~TIM_OutputState_Enable) | (u16)(~(TIM_OutputState_Enable << 4));
 
    TIM3->CCER = tmpccer;

    //programm correct polarity

    //disable OC1 and OC2 before changing polarity
    TIM2->CCER &= (u16) (~TIM_OutputState_Enable) | (u16) ~(TIM_OutputState_Enable << 4);

    // Get the TIM2 CCER register value 
    tmpccer = TIM2->CCER;
    
    // Reset the Output Polarity level 
    tmpccer &= ((u16) (~TIM_OCPolarity_Low)) & (u16) ~(TIM_OCPolarity_Low << 4) & ((u16) (~(TIM_OutputState_Enable << 4)));
  
    if(desiredDirection) {
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
  static vu8 directionChangeLastTime = 0;

  if(TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET) {

    //Clear TIM1 update interrupt pending bit
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);

    //timers where updated atomar, so now we reenable
    //the H-Bridge and reconfigure the watchdog and 
    //current measurement and also update the direction value
    if(directionChangeLastTime) {
      directionChangeLastTime = 0;

      actualDirection = desiredDirection;

      //ReEnable ASD and BSD
      u16 tmpccer = TIM3->CCER;
      tmpccer |= TIM_OutputState_Enable | (u16)(TIM_OutputState_Enable << 4);      
      TIM3->CCER = tmpccer;

    }
      
    if(newPWM != 0) {
      newPWM = 0;
      //Enable update, all previous configured values
      //are now transfered atomic to the timer in the
      //moment the timer wraps around the next time
      TIM_UpdateDisableConfig(TIM1, DISABLE);
      TIM_UpdateDisableConfig(TIM2, DISABLE);
      TIM_UpdateDisableConfig(TIM3, DISABLE);

      if(actualDirection != desiredDirection) {
	directionChangeLastTime = 1;
      }
    }
  }
}
