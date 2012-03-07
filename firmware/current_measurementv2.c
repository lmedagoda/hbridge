#include "inc/stm32f10x_type.h"
#include "inc/stm32f10x_adc.h"
#include "inc/stm32f10x_dma.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_tim.h"
#include "inc/stm32f10x_nvic.h"
#include "printf.h"
#include "current_measurement.h"
#include <stdlib.h>
#include <stm32f10x_gpio.h>

#define ADC1_DR_Address    ((u32)0x4001244C)
#define AVERAGE_THRESHOLD 10

vu16 adc_values[USED_REGULAR_ADC_CHANNELS];

struct adcValues{
  u32 currentValues[USED_REGULAR_ADC_CHANNELS];
  u32 currentValueCount;
};

vu8 switchAdcValues = 0;

//need to be static, so that they 
//are initalized with zero
static struct adcValues avs1;
static struct adcValues avs2;

volatile struct adcValues *activeAdcValues;
volatile struct adcValues *inActiveAdcValues;

extern vu8 actualDirection;
vu8 oldDirection;

// hall sensor values
vu32 h1[USED_REGULAR_ADC_CHANNELS/2];
vu32 h2[USED_REGULAR_ADC_CHANNELS/2];

void requestNewADCValues() {
    switchAdcValues = 1;
};

void waitForNewADCValues() {
    while(switchAdcValues) {
	;
    }
};

u32 calculateCurrent() {
    /*
	PWM period: 25000 ns
	adc sample time:	1666.67 ns/sample
	total sample time:	12*1666.67 ns = 20000ns
    */

    // pointer to active value struct
    u32 adcValueCount =  inActiveAdcValues->currentValueCount;
    vu32 *currentValues = inActiveAdcValues->currentValues;

    // init all used values
    u32 rawCurrentValueH1 = 0;
    u32 rawCurrentValueH2 = 0;
    u32 currentValueH1 = 0;
    u32 currentValueH2 = 0;
    u32 currentValue = 0;
    u8 i = 0;
    u8 k = 0;
    u8 cntH1 = 0;
    u8 cntH2 = 0;

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
    u32 threshold = AVERAGE_THRESHOLD * adcValueCount;
    u32 thresholdMinH1 = rawCurrentValueH1 - threshold;
    u32 thresholdMaxH1 = rawCurrentValueH1 + threshold;
    u32 thresholdMinH2 = rawCurrentValueH2 - threshold;
    u32 thresholdMaxH2 = rawCurrentValueH2 + threshold;

    // search for corrupted values and sum up only usable values
    for(k = 0; k < (USED_REGULAR_ADC_CHANNELS / 2); k++) {
	// hall sensor 1
// 	if (h1[k] > thresholdMinH1 && h1[k] < thresholdMaxH1) {
	    currentValueH1 += h1[k];
	    cntH1++;
// 	}
	// hall sensor 2
// 	if (h2[k] > thresholdMinH2 && h2[k] < thresholdMaxH2) {
	    currentValueH2 += h2[k];
	    cntH2++;
// 	}
    }

    // compute average with accuracy factor
    if (cntH1 > 0)
	currentValueH1 = (currentValueH1*16) / cntH1;
    else 
	currentValueH1 = 0;

    if (cntH2 > 0)
	currentValueH2 = (currentValueH2*16) / cntH2;
    else
	currentValueH2 = 0;

    u32 curVal[12];
    
    for(i = 0; i< 12; i++)
    {
	u32 tmp = currentValues[i];
	tmp = tmp * 100 / 17;
	tmp = tmp * 3300 / 4096;
	tmp = tmp / adcValueCount;
	curVal[i] = tmp;
    }
    

    currentValue = currentValueH1 + currentValueH2;

    //values are still raw adc values
    //convert to V
    currentValue = (currentValue * 3300) / 4096;

    
    //convert from voltage to Amps
    currentValue = currentValue * 100 / 17; 

    // divide by accuracy factor 128
    currentValue = currentValue / 16;

    //divide by adcValueCount as every value until now is
    //the sum of values over adcValueCount pwm periods
    currentValue = currentValue / adcValueCount;

    //set all values to zero
    for(i = 0; i < USED_REGULAR_ADC_CHANNELS; ++i) {
	currentValues[i] = 0;
    }

    // reset value count
    inActiveAdcValues->currentValueCount = 0; 

    // truncate value if maximum size is reached (14Bit in CAN-package)
    if (currentValue > (1<<14))
	currentValue = (1<<14);

    return currentValue;
}

/**
 * This function sets up timer 4 at 40 khz and
 * synchronises it to timer 1. This is needed
 * as the adc can only be triggerd (in our configuration)
 * by timer4 CC 4
 * */
void triggerTimerInit()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    //remap TIM to PD (not on chip)
    GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);
    
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    //(TIM2CLK = 72 MHz / 40 kHz = Period = 1800)
    TIM_TimeBaseStructure.TIM_Period = 1799;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    //init timer to 40khz
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // Prescaler configuration 
    TIM_PrescalerConfig(TIM4, 0, TIM_PSCReloadMode_Immediate); 

    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM4, ENABLE);

    TIM_OCInitTypeDef ocstruct;
    TIM_OCStructInit(&ocstruct);

    //setup channel 4 for triggering of adc
    ocstruct.TIM_OCMode = TIM_OCMode_PWM1;
    ocstruct.TIM_OutputState = TIM_OutputState_Enable;
    ocstruct.TIM_Pulse = 1;    
    ocstruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC4Init(TIM4, &ocstruct);

    TIM_UpdateDisableConfig(TIM4, DISABLE);
    //sync timer on timer 1
    TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);
    TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);
    TIM_SelectInputTrigger(TIM4, TIM_TS_ITR0);

    // TIM1 enable counter
    TIM_Cmd(TIM4, ENABLE);    
}

void adcInit() 
{
    // Enable ADC1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    // Enable DMA1 clock 
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitTypeDef DMA_InitStructure;
    DMA_StructInit(&DMA_InitStructure);
    
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

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_StructInit(&NVIC_InitStructure);

    // Configure and enable ADC interrupt
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQChannel;
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

    ADC_InitTypeDef ADC_InitSingleShot;
    ADC_StructInit(&ADC_InitSingleShot);
    
    ADC_InitSingleShot.ADC_Mode = ADC_Mode_Independent;
    ADC_InitSingleShot.ADC_ScanConvMode = ENABLE;
    ADC_InitSingleShot.ADC_ContinuousConvMode = DISABLE;
    ADC_InitSingleShot.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T4_CC4;
    ADC_InitSingleShot.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitSingleShot.ADC_NbrOfChannel = USED_REGULAR_ADC_CHANNELS;
    ADC_Init(ADC1, &ADC_InitSingleShot);

    //Channel 8 and 9 are connected to the shunts
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 3, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 4, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 5, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 6, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 7, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 8, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 9, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 10, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 11, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 12, ADC_SampleTime_7Cycles5);

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

void currentMeasurementInit()
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

    adcInit();
    
    triggerTimerInit();
}

void DMA1_Channel1_IRQHandler(void) 
{
    GPIO_SetBits(GPIOA, GPIO_Pin_12);

    int i;

    // set pointer to active value struct
    vu32 *cvp = activeAdcValues->currentValues;

    // set pointer to converted value struct
    vu16 *avp = adc_values;

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
  
    GPIO_ResetBits(GPIOA, GPIO_Pin_12);
}


