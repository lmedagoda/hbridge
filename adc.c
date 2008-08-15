#include "stm32f10x_type.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_rcc.h"

#define ADC1_DR_Address    ((u32)0x4001244C)

#define USED_REGULAR_ADC_CHANNELS 2

u16 adc_values[USED_REGULAR_ADC_CHANNELS];

static ADC_InitTypeDef ADC_InitWatchdog;
static ADC_InitTypeDef ADC_InitSingleShot;
static DMA_InitTypeDef DMA_InitStructure;

void ADC_Configuration(void)
{
  // Enable ADC1 clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  // Enable DMA1 clock 
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  //DMA1 channel1 configuration
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)adc_values;
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
  
  //Enable DMA1 channel1
  DMA_Cmd(DMA1_Channel1, ENABLE);

  //ADC1 configuration
  ADC_InitWatchdog.ADC_Mode = ADC_Mode_Independent;
  ADC_InitWatchdog.ADC_ScanConvMode = ENABLE;
  ADC_InitWatchdog.ADC_ContinuousConvMode = ENABLE;
  ADC_InitWatchdog.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
  ADC_InitWatchdog.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitWatchdog.ADC_NbrOfChannel = USED_REGULAR_ADC_CHANNELS;

  ADC_InitSingleShot.ADC_Mode = ADC_Mode_Independent;
  ADC_InitSingleShot.ADC_ScanConvMode = ENABLE;
  ADC_InitSingleShot.ADC_ContinuousConvMode = DISABLE;
  ADC_InitSingleShot.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
  ADC_InitSingleShot.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitSingleShot.ADC_NbrOfChannel = USED_REGULAR_ADC_CHANNELS;
  ADC_Init(ADC1, &ADC_InitSingleShot);

  // ADC1 regular channel14 configuration 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_1Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_1Cycles5);

  // Set injected sequencer length
  ADC_InjectedSequencerLengthConfig(ADC1, 1);

  // ADC1 injected channel Configuration  
  ADC_InjectedChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_1Cycles5);

  // ADC1 injected external trigger configuration 
  ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);

  // Disable automatic injected conversion start after regular one 
  ADC_AutoInjectedConvCmd(ADC1, DISABLE);

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

}

extern vu8 pwmBiggerXPercent;


void ADC_StartWatchdog() {
  // Disable for Configuration
  ADC_Cmd(ADC1, DISABLE);

  //Disable DMA1 channel1
  DMA_Cmd(DMA1_Channel1, DISABLE);

  ADC_Init(ADC1, &ADC_InitWatchdog);

  // Disable ADC1 DMA 
  ADC_DMACmd(ADC1, DISABLE);

  // Enable ADC1 external trigger  
  ADC_ExternalTrigConvCmd(ADC1, ENABLE);

  // Disable EOC interupt
  ADC_ITConfig(ADC1, ADC_IT_EOC, DISABLE);

  // Enable ADC1
  ADC_Cmd(ADC1, ENABLE);
}

void ADC_ProgramSingleShot() {
  // Disable for Configuration
  ADC_Cmd(ADC1, DISABLE);

  //Reinit DMA
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  //Enable DMA1 channel1
  DMA_Cmd(DMA1_Channel1, ENABLE);

  ADC_Init(ADC1, &ADC_InitSingleShot);

  // Enable ADC1 DMA 
  ADC_DMACmd(ADC1, ENABLE);

  // Enable ADC1
  ADC_Cmd(ADC1, ENABLE);

  // Enable ADC1 external trigger  
  ADC_ExternalTrigConvCmd(ADC1, ENABLE);

  // Enable EOC interupt
  ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

  // Enable ADC1
  ADC_Cmd(ADC1, ENABLE);
}

void ADC_Interrupt() {
  if(ADC_GetITStatus(ADC1, ADC_IT_EOC)) {
    if(pwmBiggerXPercent) {  
      ADC_StartWatchdog();

      //TODO reprogram timer

    }
  }
  
  if(ADC_GetITStatus(ADC1, ADC_IT_AWD)) {
    //TODO switch dead pin on
    if(!pwmBiggerXPercent) {
      //get a current measurement
      ADC_ProgramSingleShot();

      //TODO reprogram timer
    }


  }
  
}

