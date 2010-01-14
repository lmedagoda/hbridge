#include "inc/stm32f10x_type.h"
#include "inc/stm32f10x_adc.h"
#include "inc/stm32f10x_dma.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_tim.h"
#include "printf.h"
#include "current_measurement.h"
#include <stdlib.h>

#define ADC1_DR_Address    ((u32)0x4001244C)

vu16 adc_values[USED_REGULAR_ADC_CHANNELS];

static ADC_InitTypeDef ADC_InitSingleShot;
static DMA_InitTypeDef DMA_InitStructure;

u32 SQR1Forward = 0;
u32 SQR2Forward = 0;
u32 SQR3Forward = 0;
u32 SQR1Reverse = 0;
u32 SQR2Reverse = 0;
u32 SQR3Reverse = 0;

vu32 acs712BaseVoltage = 0;
vu32 currentValue = 0;
vu32 batValue = 0;

struct adcValues{
  u32 currentValues[32];
  u32 currentValueCount;
  u32 batValueSum;
  u32 batValueCount;
};

vu8 switchAdcValues = 0;

struct adcValues avs1;
struct adcValues avs2;

volatile struct adcValues *activeAdcValues = &avs1;
volatile struct adcValues *inActiveAdcValues = &avs2;
extern vu8 actualDirection;

void requestNewADCValues() {
    switchAdcValues = 1;
};

void waitForNewADCValues() {
    while(switchAdcValues) {
	;
    }
};

u32 getBatteryVoltage()
{
    return batValue;
}


void measureACS712BaseVoltage()
{
  int k, i;
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

}


u32 calculateCurrent(s32 currentPwmValue) {
    //sum up all currents measured
    u32 currentValue = 0;

    u32 adcValueCount =  inActiveAdcValues->currentValueCount;
    vu32 *currentValues = inActiveAdcValues->currentValues;

    int i, usableCurrentValues = 0;
    //length of 13Cycle Sample time is 722nsecs 
    //during 21660nsecs samples are taken, this 
    //is equal to a pwm value of 1559.52
    if(abs(currentPwmValue) < 1559) {
	usableCurrentValues = abs(currentPwmValue) / 52;
    } else {
	usableCurrentValues = 30;
    }
    
    //batValueSum and currentValueSum are increased at the 
    //same time therefore adcValueCount is valid for both
    //TODO FIXME bat value is only valid if PWM is on
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
    currentValue = (currentValue * 51) / 33;
    
    //convert from adc to volts
    currentValue = (currentValue * 3300) / 4096;

    //set rest to zero
    for(i = usableCurrentValues; i < (USED_REGULAR_ADC_CHANNELS -1) *2; i++) {
	currentValues[i] = 0;
    }

    inActiveAdcValues->currentValueCount = 0;  
    
    return currentValue;
}


/**
 * This function configures the ADCs
 * ADC1 is configured to be Triggered by Timer1 Ch2
 * and to convert ADC_Channel2 and 3 (VIA and VIB)
 * once. 
 *
 * ADC2 is configured to be Triggered by Timer Ch3
 * and to start an ADC Watchdog in response. 
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

  //enable interrupt when dma half finished
  DMA_ITConfig(DMA1_Channel1, DMA_IT_HT, ENABLE);

  //Enable DMA1 channel1
  DMA_Cmd(DMA1_Channel1, ENABLE);
  
  ADC_InitSingleShot.ADC_Mode = ADC_Mode_Independent;
  ADC_InitSingleShot.ADC_ScanConvMode = ENABLE;
  ADC_InitSingleShot.ADC_ContinuousConvMode = ENABLE;
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

}



void DMA1_Channel1_IRQHandler(void) {
  //GPIOA->BSRR |= GPIO_Pin_8;
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
//    wasinhtit++;
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

  //  wasinadcit++;

    if(secondInterrupt) {
      cvp += (USED_REGULAR_ADC_CHANNELS - 1);

      //GPIOA->BRR |= GPIO_Pin_8;
        //disable continous mode
      configureCurrentMeasurement(actualDirection);
      //clear half transfer finished interrupt pending bit
      //DMA1->IFCR = DMA1_FLAG_HT1;
      //DMA1->IFCR = DMA1_FLAG_GL1;
      //GPIOA->BSRR |= GPIO_Pin_8;
    
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

  //wasineocit++;  

  //GPIOA->BRR |= GPIO_Pin_8;
}

/**
 * This function configures the channel from which
 * the current measurement is taken, in respect of
 * the direction in which the motor turns.
 */
/*void configureCurrentMeasurement(vu8 dir) {
  // Disable for Configuration
  ADC1->CR2 &= ~0x01;
  DMA1_Channel1->CCR &= ~0x01;

  DMA1_Channel1->CNDTR = USED_REGULAR_ADC_CHANNELS;

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
  //ADC1->CR1 |= ADC_IT_EOC

  //enable dma
  DMA1_Channel1->CCR |= 0x01;

  // Enable ADC1
  ADC1->CR2 |= 0x01;
  }*/

/*
void temp() {
  
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


  u32 tmpreg1 = 0;
  u32 tmpreg2 = 0;

  // Get the ADCx SQR1 value 
  tmpreg1 = ADC1->SQR1;
  // Clear L bits 
  tmpreg1 &= ((u32)0xFF0FFFFF);

  // Configure ADCx: regular channel sequence length
  // Set L bits according to ADC_NbrOfChannel value
  tmpreg2 |= (nrOfSamples - 1);
  tmpreg1 |= ((u32)tmpreg2 << 20);
  // Write to ADCx SQR1 
  ADC1->SQR1 = tmpreg1;


  // Get the old register value
  tmpreg1 = ADC1->SMPR2;
  // Calculate the mask to clear 
  tmpreg2 = ((u32)0x00000007) << (3 * ADC_Channel_2) & ((u32)0x00000007) << (3 * ADC_Channel_3);
  // Clear the old discontinuous mode channel count 
  tmpreg1 &= ~tmpreg2;
  // Calculate the mask to set 
  tmpreg2 = (u32)curSampleTime << (3 * ADC_Channel_2) | (u32)curSampleTime << (3 * ADC_Channel_3);
  // Set the discontinuous mode channel count 
  tmpreg1 |= tmpreg2;
  // Store the new register value 
  ADC1->SMPR2 = tmpreg1;

}
*/
