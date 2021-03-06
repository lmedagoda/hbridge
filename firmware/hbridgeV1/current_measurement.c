#include <stdint.h>
#include "inc/stm32f10x_adc.h"
#include "inc/stm32f10x_dma.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_tim.h"
#include "printf.h"
#include "current_measurement.h"
#include <stdlib.h>

#define ADC1_DR_Address    ((uint32_t)0x4001244C)
#define AVERAGE_THRESHOLD 10
#define SLOPE_1 1388
#define SLOPE_2 1275
#define USED_REGULAR_ADC_CHANNELS 12

volatile uint16_t adc_values[USED_REGULAR_ADC_CHANNELS];

static ADC_InitTypeDef ADC_InitSingleShot;
static DMA_InitTypeDef DMA_InitStructure;


volatile uint32_t acs712BaseVoltage = 0;
volatile uint32_t currentValue = 0;

struct adcValues{
  uint32_t currentValues[USED_REGULAR_ADC_CHANNELS];
  uint32_t currentValueCount;
};

volatile uint8_t switchAdcValues = 0;

//need to be static, so that they 
//are initalized with zero
static struct adcValues avs1;
static struct adcValues avs2;

volatile struct adcValues *activeAdcValues = &avs1;
volatile struct adcValues *inActiveAdcValues = &avs2;

extern volatile uint8_t actualDirection;
volatile uint8_t oldDirection;

// hall sensor values
volatile uint32_t h1[USED_REGULAR_ADC_CHANNELS/2];
volatile uint32_t h2[USED_REGULAR_ADC_CHANNELS/2];

void measureACS712BaseVoltage()
{
  int k, i;
  uint32_t meanacs712base = 0;

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
    
    // add factor to increase accuracy
    acs712BaseVoltage = acs712BaseVoltage*100;

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

uint32_t currentMeasurement_getValue()
{
    switchAdcValues = 1;

    while(switchAdcValues) {
	;
    }

    /*
	PWM period: 25000 ns
	adc sample time:	1666.67 ns/sample
	total sample time:	12*1666.67 ns = 20000ns
    */

    // pointer to active value struct
    uint32_t adcValueCount =  inActiveAdcValues->currentValueCount;
    volatile uint32_t *currentValues = inActiveAdcValues->currentValues;

    // init all used values
    uint32_t rawCurrentValueH1 = 0;
    uint32_t rawCurrentValueH2 = 0;
    uint32_t currentValueH1 = 0;
    uint32_t currentValueH2 = 0;
    uint32_t currentValue = 0;
    uint8_t i = 0;
    uint8_t k = 0;
    uint8_t cntH1 = 0;
    uint8_t cntH2 = 0;

    // sum up all values for hall-sensor 1 and 2
    for(k = 0; k < USED_REGULAR_ADC_CHANNELS / 2; k++) {
	// copy values
	h1[k] = currentValues[i++];
	h2[k] = currentValues[i++];

	// sum up values for raw average
	rawCurrentValueH1 += h1[k];
	rawCurrentValueH2 += h2[k];
    }

    // compute average of hall1 and hall2
    rawCurrentValueH1 = rawCurrentValueH1 / (USED_REGULAR_ADC_CHANNELS / 2);
    rawCurrentValueH2 = rawCurrentValueH2 / (USED_REGULAR_ADC_CHANNELS / 2);

    //  set thresholds with respect to adcValueCount
    uint32_t threshold = AVERAGE_THRESHOLD * adcValueCount;
    uint32_t thresholdMinH1 = rawCurrentValueH1 - threshold;
    uint32_t thresholdMaxH1 = rawCurrentValueH1 + threshold;
    uint32_t thresholdMinH2 = rawCurrentValueH2 - threshold;
    uint32_t thresholdMaxH2 = rawCurrentValueH2 + threshold;

    // search for corrupted values and sum up only usable values
    for(k = 0; k < (USED_REGULAR_ADC_CHANNELS / 2); k++) {
	// hall sensor 1
	if (h1[k] > thresholdMinH1 && h1[k] < thresholdMaxH1) {
	    currentValueH1 += h1[k];
	    cntH1++;
	}
	// hall sensor 2
	if (h2[k] > thresholdMinH2 && h2[k] < thresholdMaxH2) {
	    currentValueH2 += h2[k];
	    cntH2++;
	}
    }

    // compute average with accuracy factor
    if (cntH1 > 0)
	currentValueH1 = (currentValueH1*100) / cntH1;
    else 
	currentValueH1 = 0;

    if (cntH2 > 0)
	currentValueH2 = (currentValueH2*100) / cntH2;
    else
	currentValueH2 = 0;

    currentValue = currentValueH1 + currentValueH2;

    // compute average over all adc samples
    currentValue = currentValue / adcValueCount;

    // debug: uncomment for calibration 
    //return (currentValue/100);

    // calculate offsets with respect to base voltage
    uint32_t offset1 = SLOPE_1 * 2 * acs712BaseVoltage;
    uint32_t offset2 = SLOPE_2 * 2 * acs712BaseVoltage;

    if (currentValueH1 < currentValueH2) {
	if ((currentValue * SLOPE_1) > offset1)
	    currentValue = (currentValue * SLOPE_1) - offset1; 
	else 
	    currentValue = 0;
    } else {
	if ((currentValue * SLOPE_2) >  offset2)
	    currentValue = (currentValue * SLOPE_2) - offset2;
	else 
	    currentValue = 0;
    }

    // divide by accuracy factor 100*100
    currentValue = currentValue / 10000;

    //set all values to zero
    for(i = 0; i < USED_REGULAR_ADC_CHANNELS; ++i) {
	currentValues[i] = 0;
    }

    // reset value count
    inActiveAdcValues->currentValueCount = 0; 

    // truncate value if maximum size is reached (14Bit in CAN-package)
    if (currentValue > 16384)
	currentValue = 16384;

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
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) adc_values;
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
  
  NVIC_InitTypeDef NVIC_InitStructure;
  
  // Configure and enable ADC interrupt
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //enable interrupt when dma is finished
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

  //disable interrupt when dma half finished
  DMA_ITConfig(DMA1_Channel1, DMA_IT_HT, DISABLE);

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
  volatile uint32_t *cvp = activeAdcValues->currentValues;

  // set pointer to converted value struct
  volatile uint16_t *avp = adc_values;

  // all channels are sampled
  if(DMA1->ISR & DMA1_IT_TC1) {

    for(i = 0; i < USED_REGULAR_ADC_CHANNELS; i++) {
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
      
      //set rest to zero
      for(i = 0; i < USED_REGULAR_ADC_CHANNELS; ++i) {
        activeAdcValues->currentValues[i] = 0;
      }
      activeAdcValues->currentValueCount = 0;

      switchAdcValues = 0;
    }
  
    //clear DMA interrupt pending bit
    DMA1->IFCR = DMA1_FLAG_TC1;
  }
}

void currentMeasurement_init()
{
    activeAdcValues = &avs1;
    inActiveAdcValues = &avs2;
    
    int i;
    //set all values to zero
    for(i = 0; i < USED_REGULAR_ADC_CHANNELS; ++i) {
	activeAdcValues->currentValues[i] = 0;
	inActiveAdcValues->currentValues[i] = 0;
    }

    // reset value count
    activeAdcValues->currentValueCount = 0; 
    inActiveAdcValues->currentValueCount = 0; 


    //wait until 5V rail get's stable
    volatile uint32_t delay = 20000000;
    while(delay)
	delay--;

    currentMeasurementInit();

    measureACS712BaseVoltage(); 
    
}


    



