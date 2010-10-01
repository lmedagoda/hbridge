#include "inc/stm32f10x_type.h"
#include "inc/stm32f10x_adc.h"
#include "inc/stm32f10x_dma.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_tim.h"
#include "printf.h"
#include "current_measurement.h"
#include <stdlib.h>

#define ADC1_DR_Address    ((u32)0x4001244C)
#define AVERAGE_THRESHOLD 300 // [mA]

vu16 adc_values[USED_REGULAR_ADC_CHANNELS];

static ADC_InitTypeDef ADC_InitSingleShot;
static DMA_InitTypeDef DMA_InitStructure;


vu32 acs712BaseVoltage = 0;
vu32 currentValue = 0;

struct adcValues{
  u32 currentValues[USED_REGULAR_ADC_CHANNELS];
  u32 currentValueCount;
};

vu8 switchAdcValues = 0;

//need to be static, so that they 
//are initalized with zero
static struct adcValues avs1;
static struct adcValues avs2;

volatile struct adcValues *activeAdcValues = &avs1;
volatile struct adcValues *inActiveAdcValues = &avs2;

extern vu8 actualDirection;
vu8 oldDirection;

vu32 hall1[USED_REGULAR_ADC_CHANNELS/2];
vu32 hall2[USED_REGULAR_ADC_CHANNELS/2];

void requestNewADCValues() {
    switchAdcValues = 1;
};

void waitForNewADCValues() {
    while(switchAdcValues) {
	;
    }
};

void measureACS712BaseVoltage()
{
  int k, i;
  u32 meanacs712base = 0;

  //disable pwm output, so that ADC is not triggered any more
  TIM_CtrlPWMOutputs(TIM1, DISABLE);

  for(k = 0; k < 40; ++k) {
    acs712BaseVoltage = 0;

    //reset adc values
    for( i = 0; i < USED_REGULAR_ADC_CHANNELS; ++i) {
      activeAdcValues->currentValues[i] = 0;
    }
    activeAdcValues->currentValueCount = 0;
    
    //trigger adc
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    
    //wait for adc conversion to start
    while(!(activeAdcValues->currentValues[0])) {;}
    
    //disable further triggering
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
    
    //Wait for conversion to be finished
    while(!activeAdcValues->currentValueCount) {;}

    //sum up values
    for( i = 0; i < USED_REGULAR_ADC_CHANNELS; ++i) {
      //printf("Calibration value[%lu]= %lu \n",i, activeAdcValues->currentValues[i]);
      acs712BaseVoltage += activeAdcValues->currentValues[i];
    }
    
    //average filter, to reduce noise
    acs712BaseVoltage /= USED_REGULAR_ADC_CHANNELS;

    //printf("ACS712 baseVoltage = %lu \n", acs712BaseVoltage);
    meanacs712base += acs712BaseVoltage;
  }

  meanacs712base /= 40;
  acs712BaseVoltage = meanacs712base;
  printf("\nMEAN ACS712 baseVoltage = %lu \n\n", meanacs712base);
  
  //reenable pwm output again for normal usage
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}


u32 calculateCurrent(s32 currentPwmValue) {

    /*
	PWM period: 25000 ns
	adc sample time:	1666.67 ns/sample
	total sample time:	12*1666.67 ns = 20000ns
    */

    // pointer to active value struct
    u32 adcValueCount =  inActiveAdcValues->currentValueCount;
    vu32 *currentValues = inActiveAdcValues->currentValues;

    // compute base voltage over all samples
    u32 baseVoltage = acs712BaseVoltage*adcValueCount;

    u32 rawCurrentValueHall1 = 0;
    u32 rawCurrentValueHall2 = 0;
    u32 currentValueHall1 = 0;
    u32 currentValueHall2 = 0;
    u32 currentValue = 0;

    int i,k;

    // sum up all values for hall-sensor 1 and 2
    for(i = 0, k = 0; k < USED_REGULAR_ADC_CHANNELS / 2; i=i+2, ++k) {
	// copy values
	hall1[k] = currentValues[i];
	hall2[k] = currentValues[i+1];

	// sum up values
	rawCurrentValueHall1 += hall1[k];
	rawCurrentValueHall2 += hall2[k];
    }

    // compute average of hall1 and hall2
    rawCurrentValueHall1 = rawCurrentValueHall1 / (USED_REGULAR_ADC_CHANNELS / 2);
    rawCurrentValueHall2 = rawCurrentValueHall2 / (USED_REGULAR_ADC_CHANNELS / 2);

    //  set thresholds
    u32 threshold = AVERAGE_THRESHOLD * adcValueCount;
    u32 hall1_threshold_min = rawCurrentValueHall1 - threshold;
    u32 hall1_threshold_max = rawCurrentValueHall1 + threshold;
    u32 hall2_threshold_min = rawCurrentValueHall2 - threshold;
    u32 hall2_threshold_max = rawCurrentValueHall2 + threshold;

    u8 hall1_cnt = 0;
    u8 hall2_cnt = 0;

    // search for corrupted values
    for(k = 0; k < USED_REGULAR_ADC_CHANNELS / 2; ++k) {
	// sum up only usable values
	if (hall1[k] > hall1_threshold_min && hall1[k] < hall1_threshold_max) {
	    currentValueHall1 += hall1[k];
	    ++hall1_cnt;
	}
	if (hall2[k] > hall2_threshold_min && hall2[k] < hall2_threshold_max) {
	    currentValueHall2 += hall2[k];
	    ++hall2_cnt;
	}
    }

    // compute average
    if (hall1_cnt > 0)
	currentValueHall1 = currentValueHall1 / hall1_cnt;
    else 
	currentValueHall1 = 0;

    if (hall2_cnt > 0)
	currentValueHall2 = currentValueHall2 / hall2_cnt;
    else 
	currentValueHall2 = 0;

    // compute delta current
    if ( (currentValueHall1 + currentValueHall2) < (baseVoltage*2) )
	currentValue = 0; // negative current, something is wrong
    else
	currentValue = (currentValueHall1 + currentValueHall2) - (baseVoltage*2);

    // compute average over all adc samples
    currentValue = currentValue / adcValueCount;

    // multiply by 10 to get mV -> mA
    currentValue = currentValue*10;

    // voltage divider
    currentValue = (currentValue * 51) / 33;
    
    // adc voltage to real value
    currentValue = (currentValue * 3300) / 4096;

    //set all values to zero
    for(i = 0; i < USED_REGULAR_ADC_CHANNELS; ++i) {
	currentValues[i] = 0;
    }

    // reset value count
    inActiveAdcValues->currentValueCount = 0;
    
    return currentValue;
}


/**
 * This function configures the ADCs
 * ADC1 is configured to be Triggered by Timer1 Ch2
 * and to convert ADC_Channel2 and 3 (VIA and VIB)
 * once. 
 */
void currentMeasurementInit()
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

  //disable interrupt when dma half finished
  DMA_ITConfig(DMA1_Channel1, DMA_IT_HT, ENABLE);

  //Enable DMA1 channel1
  DMA_Cmd(DMA1_Channel1, ENABLE);
  
  ADC_InitSingleShot.ADC_Mode = ADC_Mode_Independent;
  ADC_InitSingleShot.ADC_ScanConvMode = ENABLE;
  ADC_InitSingleShot.ADC_ContinuousConvMode = DISABLE;
  ADC_InitSingleShot.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC2;
  ADC_InitSingleShot.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitSingleShot.ADC_NbrOfChannel = USED_REGULAR_ADC_CHANNELS;
  ADC_Init(ADC1, &ADC_InitSingleShot);

  // channel 2 -> hallsensor1 ; channel 3 -> hallsensor2
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_7Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_7Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_7Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_7Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 5, ADC_SampleTime_7Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 6, ADC_SampleTime_7Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 7, ADC_SampleTime_7Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 8, ADC_SampleTime_7Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 9, ADC_SampleTime_7Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 10, ADC_SampleTime_7Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 11, ADC_SampleTime_7Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 12, ADC_SampleTime_7Cycles5);

  /* Enable ADC1 external trigger */ 
  ADC_ExternalTrigConvCmd(ADC1, ENABLE);

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

}

void DMA1_Channel1_IRQHandler(void) {

  int i;

  // set pointer to active value struct
  vu32 *cvp = activeAdcValues->currentValues;

  // set pointer to converted value struct
  vu16 *avp = adc_values;

  if(DMA1->ISR & DMA1_IT_HT1) {
  
    // copy the first values to active struct
    for(i = 0; i < (USED_REGULAR_ADC_CHANNELS/2); ++i) {
      *cvp += *avp;
      ++cvp;
      ++avp;
    }

    //clear DMA interrupt pending bit
    DMA1->IFCR = DMA1_FLAG_HT1;
  };

  // all channels are sampled
  if(DMA1->ISR & DMA1_IT_TC1) {

    cvp += (USED_REGULAR_ADC_CHANNELS/2);
    avp += (USED_REGULAR_ADC_CHANNELS/2);
  
    // copy adc values to active struct
    for(i = (USED_REGULAR_ADC_CHANNELS/2); i < USED_REGULAR_ADC_CHANNELS; ++i) {
      *cvp += *avp;
      ++cvp;
      ++avp;
    }

    // increase value count
    ++(activeAdcValues->currentValueCount);

    // check switch value request
    if(switchAdcValues) {
      volatile struct adcValues *tempAdcValues;

      tempAdcValues = activeAdcValues;
      activeAdcValues = inActiveAdcValues;
      inActiveAdcValues = tempAdcValues;
      switchAdcValues = 0;
      
      //set rest to zero
      for(i = 0; i < USED_REGULAR_ADC_CHANNELS; ++i) {
        activeAdcValues->currentValues[i] = 0;
      }
      activeAdcValues->currentValueCount = 0;
    }
  
    //clear DMA interrupt pending bit
    DMA1->IFCR = DMA1_FLAG_TC1;
  }
}


