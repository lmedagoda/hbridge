#include "inc/stm32f10x_tim.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_adc.h"
#include "inc/stm32f10x_gpio.h"
#include "hbridge.h"
#include "current_measurement.h"

/// direction, that was given to setNewPWM
vu8 desieredDirection = 2;

/// direction in which the hbridge is configured atm
vu8 actualDirection = 0;

///indicates, if a new pwm-value was set since the last pwm-cycle
volatile u8 newPWM = 0;

///indicates, if the analog watchdog was triggered
vu8 awdWasTriggered = 0;

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


    //timer used for shutdown generation
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

void initAnalogeWatchdog() {
    ADC_InitTypeDef ADC_InitWatchdog;

    // Enable ADC2 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

    //ADC2 configuration
    ADC_InitWatchdog.ADC_Mode = ADC_Mode_Independent;
    ADC_InitWatchdog.ADC_ScanConvMode = DISABLE;
    ADC_InitWatchdog.ADC_ContinuousConvMode = ENABLE;
    ADC_InitWatchdog.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC3;
    //ADC_InitWatchdog.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitWatchdog.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitWatchdog.ADC_NbrOfChannel = 1;
    ADC_Init(ADC2, &ADC_InitWatchdog);

    // ADC2 regular channel14 configuration 
    ADC_RegularChannelConfig(ADC2, ADC_Channel_4, 1, ADC_SampleTime_1Cycles5);
    //ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 2, ADC_SampleTime_1Cycles5);

    // Configure high and low analog watchdog thresholds
    ADC_AnalogWatchdogThresholdsConfig(ADC2, 4095, 0);
    //ADC_AnalogWatchdogThresholdsConfig(ADC2, 470, 0);

    // Configure channel4 as the single analog watchdog guarded channel 
    ADC_AnalogWatchdogSingleChannelConfig(ADC2, ADC_Channel_4);

    // Enable analog watchdog on one regular channel 
    ADC_AnalogWatchdogCmd(ADC2, ADC_AnalogWatchdog_SingleRegEnable);

    // Disable automatic injected conversion start after regular one 
    ADC_AutoInjectedConvCmd(ADC2, DISABLE);

    // Disable EOC interupt
    ADC_ITConfig(ADC2, ADC_IT_EOC, DISABLE);

    // Enable ADC1 external trigger
    ADC_ExternalTrigConvCmd(ADC2, ENABLE);

    // Enable ADC2
    ADC_Cmd(ADC2, ENABLE);

    // Enable ADC2 reset calibaration register  
    ADC_ResetCalibration(ADC2);
    // Check the end of ADC2 reset calibration register
    while(ADC_GetResetCalibrationStatus(ADC2));

    // Start ADC2 calibaration
    ADC_StartCalibration(ADC2);
    // Check the end of ADC2 calibration
    while(ADC_GetCalibrationStatus(ADC2));
    
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

    initAnalogeWatchdog();
    
    //set pwm to zero
    setNewPWM(0, 0, 0);
    
}

void setNewPWM(const s16 value2, u8 useOpenLoop, u8 useBackInduction) {
    
  //TODO add magic formular for conversation
  s16 value = value2;
  
  //trunkate to min of 3.0% PWM, as Current Measurement needs 763.8 us
  //388.8us for vbat and 375us for first current sample
  if((value > 0 && value < 55) || (value < 0 && value > -55))
    value = 0;
  

  //TODO exact value
  //trunkcate to max of 95% PWM
  if(value < -1710)
    value = -1710;
  
  if(value > 1710)
    value = 1710;

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

  static u8 lastDesiredDirection = 2;
  if(value == 0) {
    desieredDirection = lastDesiredDirection;
  } else {
    desieredDirection = value >= 0;
  }
  
  lastDesiredDirection = desieredDirection;

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

    if(useOpenLoop) {
    
      if(useBackInduction) {
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
      if(useBackInduction) {  
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
    
    if(useOpenLoop) {
    
      if(useBackInduction) {
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
      if(useBackInduction) {  
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

  if(useBackInduction && dutyTime < 1600) {
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
 * This function programms the watchdog.
 * It selects the  channel to be guarded in respect 
 * to the direction the motor turns.
 */
void configureWatchdog(vu8 dir) {
  // Disable ADC for Configuration
  ADC_Cmd(ADC2, DISABLE);

  if(dir) {
    //in forward case we want to detect voltage drop on the A-Side
    //so we use VUB
    ADC_RegularChannelConfig(ADC2, ADC_Channel_4, 1, ADC_SampleTime_1Cycles5);
    ADC_AnalogWatchdogSingleChannelConfig(ADC2, ADC_Channel_4);
  } else {
    //in reverse case we want to detect voltage drop on the B-Side
    //so we use VUA
    // Configure channel4 as the single analog watchdog guarded channel 
    ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 1, ADC_SampleTime_1Cycles5);
    ADC_AnalogWatchdogSingleChannelConfig(ADC2, ADC_Channel_5);
  }

  // Enable analog watchdog on one regular channel 
  ADC_AnalogWatchdogCmd(ADC2, ADC_AnalogWatchdog_SingleRegEnable);

  // Enable ADC2
  ADC_Cmd(ADC2, ENABLE);

  //Enable Watchdog interupt
  ADC_ITConfig(ADC2, ADC_IT_AWD, ENABLE);
}

/**
 * This function deactivates either q1 or q3 in respect
 * to the turning direction of the motor.
 * It also disables the ADC_IT_AWD interrupt, and
 * stops the conversation, so that it can be enabled
 * by the TIM1CC3 again.
 */
inline void ForceHighSideOffAndDisableWatchdog(void) {  
  /* disabled as it dosen't work atm
  u16 tmpccmr = 0;
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
    ADC2->LTR = getBatteryVoltage() *2 /3;
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
  //test if Analog Watchdog Flag is set.
  //this means the souce of the IT was AWD
  if(ADC2->SR & ADC_FLAG_AWD) {
    ForceHighSideOffAndDisableWatchdog();
    //awdWasTriggered = 1;

    //clear interrupt source
    ADC2->SR = ~(u32) ADC_FLAG_AWD;
  }  
}

/**
 * Interrupt function used for security interrupt
 * This function forces the high side gate off, for
 * the case that the adc watchdog didn't trigger
 * also the Watchdog is disabled. 
 *
 * This function is only used if backInduction mode is active
 */
void TIM2_IRQHandler(void) {
  //GPIOA->BSRR |= GPIO_Pin_8;
    if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET) {
    //Clear TIM2 Capture compare interrupt pending bit 
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
    
    //only call function if AWD didn't trigger
    if(!awdWasTriggered) {
      ForceHighSideOffAndDisableWatchdog();
    }

    ADC_ClearFlag(ADC2, ADC_FLAG_STRT);

  }

    //GPIOA->BRR |= GPIO_Pin_8;

}
