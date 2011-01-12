#include "encoder.h"
#include "inc/stm32f10x_adc.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_nvic.h"
#include "inc/stm32f10x_gpio.h"

#define ADC_VALUES_PER_MS 17

u32 adcEncoderValue = 0;
u32 adcTicksPerTurn;
u8 adcTickDivider;
vu8 adcEncDone = 0; 
vu16 adcValues[ADC_VALUES_PER_MS];

void encoderInitADC()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2 | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);

    //setup PB0 as adc in
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);


    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure and enable ADC interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    ADC_InitTypeDef ADC_InitStructure;

    //disable for config
    ADC_Cmd(ADC2, DISABLE);

    /* ADC2 configuration ------------------------------------------------------*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC2, &ADC_InitStructure);
    
    /* ADC2 regular channels configuration */ 
    //55,5 usecs 
    ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5);
    
    /* Enable ADC2 EOC interupt */
    ADC_ITConfig(ADC2, ADC_IT_EOC, ENABLE);

    //Enable ADC2
    ADC_Cmd(ADC2, ENABLE);

    /* Enable ADC2 reset calibaration register */   
    ADC_ResetCalibration(ADC2);
    /* Check the end of ADC2 reset calibration register */
    while(ADC_GetResetCalibrationStatus(ADC2));

    /* Start ADC2 calibaration */
    ADC_StartCalibration(ADC2);
    /* Check the end of ADC2 calibration */
    while(ADC_GetCalibrationStatus(ADC2));

    /* Start ADC2 Software Conversion */ 
    ADC_SoftwareStartConvCmd(ADC2, ENABLE);
}

void ADC1_2_IRQHandler(void)
{
    static int cnt = 0;
    
    adcValues[cnt] = ADC_GetConversionValue(ADC2);
    
    if(cnt < ADC_VALUES_PER_MS) {
	//trigger next round
	cnt++;
	ADC_SoftwareStartConvCmd(ADC2, ENABLE);
    } else {
        cnt = 0;
	adcEncDone = 1;
    }
}

u32 getTicksPerTurnADC()
{
    return adcTicksPerTurn;
}

void setTicksPerTurnADC(u32 ticks, u8 tickDivider)
{
    adcTicksPerTurn = ticks;
    adcTickDivider = tickDivider;
}

u32 getTicksADC(void)
{
    if(adcEncDone)
    {
	adcEncoderValue = 0;
	int i;
	for(i = 0; i < ADC_VALUES_PER_MS; i++)
	{
	    adcEncoderValue += adcValues[i];
	}
	adcEncoderValue = adcEncoderValue / ADC_VALUES_PER_MS;
        adcEncDone = 0;

        //trigger next round
        ADC_SoftwareStartConvCmd(ADC2, ENABLE);
    }
    return adcEncoderValue * adcTicksPerTurn / (1<<12); 
}

u16 getDividedTicksADC(void)
{
    return getTicksADC() / adcTickDivider;
}

void encoderDeInitADC(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure and enable ADC interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);

    ADC_DeInit(ADC2);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, DISABLE);
}

