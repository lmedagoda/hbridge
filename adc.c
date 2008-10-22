#include "stm32f10x_type.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "adc.h"

#define ADC1_DR_Address    ((u32)0x4001244C)

vu16 adc_values[USED_REGULAR_ADC_CHANNELS];

static ADC_InitTypeDef ADC_InitWatchdog;
static ADC_InitTypeDef ADC_InitSingleShot;
static DMA_InitTypeDef DMA_InitStructure;

u32 SQR1Forward = 0;
u32 SQR2Forward = 0;
u32 SQR3Forward = 0;
u32 SQR1Reverse = 0;
u32 SQR2Reverse = 0;
u32 SQR3Reverse = 0;


/**
 * This function configures the ADCs
 * ADC1 is configured to be Triggered by Timer1 Ch2
 * and to convert ADC_Channel2 and 3 (VIA and VIB)
 * once. 
 *
 * ADC2 is configured to be Triggered by Timer Ch3
 * and to start an ADC Watchdog in response. 
 */
void ADC_Configuration(void)
{
  // Enable ADC1 clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  
  // Enable DMA1 clock 
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  //DMA1 channel1 configuration
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32) adc_values;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = USED_REGULAR_ADC_CHANNELS;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  //enable interrupt when dma is finished
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

  //Enable DMA1 channel1
  DMA_Cmd(DMA1_Channel1, ENABLE);
  
  ADC_InitSingleShot.ADC_Mode = ADC_Mode_Independent;
  ADC_InitSingleShot.ADC_ScanConvMode = ENABLE;
  ADC_InitSingleShot.ADC_ContinuousConvMode = DISABLE;
  ADC_InitSingleShot.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC2;
  //ADC_InitSingleShot.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitSingleShot.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitSingleShot.ADC_NbrOfChannel = USED_REGULAR_ADC_CHANNELS;
  ADC_Init(ADC1, &ADC_InitSingleShot);

  //measure VBat on high side for watchdog trigger calibration 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_1Cycles5);
  //in forward case the current flows through q3 and q2,
  //so we need to use VIA for measurement
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 4, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 5, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 6, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 7, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 8, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 9, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 10, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 11, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 12, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 13, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 14, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 15, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 16, ADC_SampleTime_13Cycles5);

  //save register values, so that we can do a fast switch in the ISR
  SQR1Forward = ADC1->SQR1;
  SQR2Forward = ADC1->SQR2;
  SQR3Forward = ADC1->SQR3;


  //measure VBat on high side for watchdog trigger calibration 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_1Cycles5);
  //in reverse case the current flows through q1 and q4,
  //so we need to use VIB for measurement
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 5, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 6, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 7, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 8, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 9, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 10, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 11, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 12, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 13, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 14, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 15, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 16, ADC_SampleTime_13Cycles5);

  //save register values, so that we can do a fast switch in the ISR
  SQR1Reverse = ADC1->SQR1;
  SQR2Reverse = ADC1->SQR2;
  SQR3Reverse = ADC1->SQR3;

  
  // Set injected sequencer length
  ADC_InjectedSequencerLengthConfig(ADC1, 1);

  // ADC1 injected channel Configuration  
  ADC_InjectedChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_1Cycles5);

  // ADC1 injected external trigger configuration 
  ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);

  // Disable automatic injected conversion start after regular one 
  ADC_AutoInjectedConvCmd(ADC1, DISABLE);

  /* Enable ADC1 external trigger */ 
  ADC_ExternalTrigConvCmd(ADC1, ENABLE);

  /* Enable EOC interupt */
  //ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);

  // Enable ADC1
  ADC_Cmd(ADC1, ENABLE);

  // Enable ADC1 reset calibaration register  
  ADC_ResetCalibration(ADC1);
  // Check the end of ADC1 reset calibration register
  while(ADC_GetResetCalibrationStatus(ADC1));

  // Start ADC1 calibaration
  ADC_StartCalibration(ADC1);
  // Check the end of ADC1 calibration
  while(ADC_GetCalibrationStatus(ADC1));


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
  ADC_AnalogWatchdogThresholdsConfig(ADC2, 1000, 0);
  //ADC_AnalogWatchdogThresholdsConfig(ADC2, 470, 0);

  // Configure channel4 as the single analog watchdog guarded channel 
  ADC_AnalogWatchdogSingleChannelConfig(ADC2, ADC_Channel_4);

  // Enable analog watchdog on one regular channel 
  ADC_AnalogWatchdogCmd(ADC2, ADC_AnalogWatchdog_SingleRegEnable);

  /*
  // Set injected sequencer length
  ADC_InjectedSequencerLengthConfig(ADC2, 1);

  // ADC2 injected channel Configuration  
  ADC_InjectedChannelConfig(ADC2, ADC_Channel_8, 1, ADC_SampleTime_1Cycles5);

  // ADC2 injected external trigger configuration 
  ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_None);
  */
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

/**
 * This function programms the watchdog.
 * It selects the  channel to be guarded in respect 
 * to the direction the motor turns.
 */
void configureWatchdog(vu8 dir) {
  // Disable for Configuration
  ADC_Cmd(ADC2, DISABLE);

  if(dir) {
    //in forward case we want to detect voltage raise on the B-Side
    //so we use VUB
    ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 1, ADC_SampleTime_1Cycles5);
    ADC_AnalogWatchdogSingleChannelConfig(ADC2, ADC_Channel_5);
  } else {
    //in reverse case we want to detect voltage raise on the A-Side
    //so we use VUA
    // Configure channel4 as the single analog watchdog guarded channel 
    ADC_RegularChannelConfig(ADC2, ADC_Channel_4, 1, ADC_SampleTime_1Cycles5);
    ADC_AnalogWatchdogSingleChannelConfig(ADC2, ADC_Channel_4);
  }

  // Enable analog watchdog on one regular channel 
  ADC_AnalogWatchdogCmd(ADC2, ADC_AnalogWatchdog_SingleRegEnable);

  // Enable ADC2
  ADC_Cmd(ADC2, ENABLE);

  //Enable Watchdog interupt
  ADC_ITConfig(ADC2, ADC_IT_AWD, ENABLE);
}

/**
 * This function configures the channel from which
 * the current measurement is taken, in respect of
 * the direction in which the motor turns.
 */
void configureCurrentMeasurement(vu8 dir, u16 highPhaseLength) {
  // Disable for Configuration
  ADC_Cmd(ADC1, DISABLE);
  DMA_Cmd(DMA1_Channel1, DISABLE);

  //takes 630 nsecs
  u8 curSampleTime = ADC_SampleTime_7Cycles5;
  u8 nrOfSamples = (((u32) highPhaseLength) * 250 / 18) / 630;

  //max length of 7Cycle Sample time is 630nsecs * 15 = 8.750 usecs
  //11.250 usec == 630 pwmlenght
  if(highPhaseLength > 630) {
    //takes 750 nsecs
    curSampleTime = ADC_SampleTime_13Cycles5;
    nrOfSamples = (((u32) highPhaseLength) * 250 / 18) / 750;
  }

  //max length of 13Cycle Sample time is 750nsecs * 15 = 11.250 usecs
  //11.250 usec == 810 pwmlenght
  if(highPhaseLength > 810) {
    //takes 1166.6 nsecs
    curSampleTime = ADC_SampleTime_28Cycles5;
    nrOfSamples = (((u32) highPhaseLength) * 2500 / 18) / 11666;;
    
  }
  
  //max length of 28Cycle Sample time is 1166.6 nsecs * 15 = 17.500 usecs
  //17.500 usec == 1260 pwmlenght
  if(highPhaseLength > 1260) {
    //takes 1527.7 nsecs
    curSampleTime = ADC_SampleTime_41Cycles5;
    nrOfSamples = (((u32) highPhaseLength) * 2500 / 18) / 1527.7;
  }

  if(nrOfSamples < 1)
    nrOfSamples = 1;
     
  if(nrOfSamples > 15)
    nrOfSamples = 15;
    
  //increase nrOfSamples by one, as we measure vBat in the beginning
  nrOfSamples++;

  DMA1_Channel1->CNDTR = nrOfSamples;//USED_REGULAR_ADC_CHANNELS;

  u32 tmpreg1 = 0;
  u32 tmpreg2 = 0;

  /* Get the ADCx SQR1 value */
  tmpreg1 = ADC1->SQR1;
  /* Clear L bits */
  tmpreg1 &= ((u32)0xFF0FFFFF);

  /* Configure ADCx: regular channel sequence length */
  /* Set L bits according to ADC_NbrOfChannel value */
  tmpreg2 |= (nrOfSamples - 1);
  tmpreg1 |= ((u32)tmpreg2 << 20);
  /* Write to ADCx SQR1 */
  ADC1->SQR1 = tmpreg1;


  /* Get the old register value */
  tmpreg1 = ADC1->SMPR2;
  /* Calculate the mask to clear */
  tmpreg2 = ((u32)0x00000007) << (3 * ADC_Channel_2) & ((u32)0x00000007) << (3 * ADC_Channel_3);
  /* Clear the old discontinuous mode channel count */
  tmpreg1 &= ~tmpreg2;
  /* Calculate the mask to set */
  tmpreg2 = (u32)curSampleTime << (3 * ADC_Channel_2) | (u32)curSampleTime << (3 * ADC_Channel_3);
  /* Set the discontinuous mode channel count */
  tmpreg1 |= tmpreg2;
  /* Store the new register value */
  ADC1->SMPR2 = tmpreg1;


  if(dir) {
    ADC1->SQR1 = SQR1Forward;
    ADC1->SQR2 = SQR2Forward;
    ADC1->SQR3 = SQR3Forward;
  } else {
    ADC1->SQR1 = SQR1Reverse;
    ADC1->SQR2 = SQR2Reverse;
    ADC1->SQR3 = SQR3Reverse;
  }
  
  //Enable End of Conversion interupt
  ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

  DMA_Cmd(DMA1_Channel1, ENABLE);

  // Enable ADC2
  ADC_Cmd(ADC1, ENABLE);
}


