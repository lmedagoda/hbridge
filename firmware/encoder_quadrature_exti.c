#include "encoder.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_rcc.h"

volatile int32_t externalEncoderValue = 0;

static volatile uint8_t configured = 0;
static volatile uint8_t foundZero = 0;

struct encoderData externalEncoderConfig;

enum encoderTypes externalEncoderType;
enum encoderStates encoderState;
enum encoderInputs encoderInput;

void EXTI15_10_IRQHandler(void)
{
    uint16_t ain = GPIOB->IDR & GPIO_Pin_13; //GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13);
    uint16_t bin = GPIOB->IDR & GPIO_Pin_14; //GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14);

    // set binary encoder input
    if(ain) {
	if(bin)
	    encoderInput = INPUT_11;
	else
	    encoderInput = INPUT_10;
    } else {
	if(bin)
	    encoderInput = INPUT_01;
	else
	    encoderInput = INPUT_00;
    }

    /*
	encoder state machine with 'State_ab'
	---> 00 <-> 01 <-> 11 <-> 10 <---
    */
    switch(encoderState) {
	case STATE_00: 
	    // check if we got a valid transition
	    switch(encoderInput) {
		case INPUT_01:
		    externalEncoderValue++;
		    encoderState = STATE_01;
		    break;
		case INPUT_10:
		    externalEncoderValue--;
		    encoderState = STATE_10;
		    break;
		default:
		    //printf("e00 a: %du b: %du \n", ain,bin);
		    break;
		}
	    break;
	case STATE_01:
	    // check if we got a valid transition
	    switch(encoderInput) {
		case INPUT_11:
		    externalEncoderValue++;
		    encoderState = STATE_11;
		    break;
		case INPUT_00:
		    externalEncoderValue--;
		    encoderState = STATE_00;
		    break;
		default:
		    //printf("e01 a: %du b: %du \n", ain,bin);
		    break;
		}
	    break;
	case STATE_10:
	    // check if we got a valid transition
	    switch(encoderInput) {
		case INPUT_11:
		    externalEncoderValue--;
		    encoderState = STATE_11;
		    break;
		case INPUT_00:
		    externalEncoderValue++;
		    encoderState = STATE_00;
		    break;
		default:
		    //printf("e10 a: %du b: %du \n", ain,bin);
		    break;
		}
	    break;
	case STATE_11:
	    // check if we got a valid transition
	    switch(encoderInput) {
		case INPUT_10:
		    externalEncoderValue++;
		    encoderState = STATE_10;
		    break;
		case INPUT_01:
		    externalEncoderValue--;
		    encoderState = STATE_01;
		    break;
		default:
		    //printf("e11 a: %du b: %du \n", ain,bin);
		    break;
		}
	    break;
	}

    // clear interrupts
    if(EXTI_GetITStatus(EXTI_Line13) != RESET) {
	EXTI_ClearITPendingBit(EXTI_Line13);
    }
    if(EXTI_GetITStatus(EXTI_Line14) != RESET) {
	EXTI_ClearITPendingBit(EXTI_Line14);
    }

    static int32_t zeroStart = -1;

    if(externalEncoderValue < 0)
	externalEncoderValue += externalEncoderConfig.ticksPerTurn;

    if(externalEncoderValue > externalEncoderConfig.ticksPerTurn)
	externalEncoderValue -= externalEncoderConfig.ticksPerTurn;

    if(EXTI_GetITStatus(EXTI_Line12) != RESET) {
	uint16_t zero = GPIOB->IDR & GPIO_Pin_12; //GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);

	//raising edge
        //a valid zero mark starts with ain and bin low
	if(zero && !ain && !bin) 
	{
	    zeroStart = externalEncoderValue;
	} else {
	    //falling edge
	    
	    //caluclate tick diff
	    int32_t diff = zeroStart - externalEncoderValue;
            
            //handle wrap arounds
            if(abs(diff) > externalEncoderConfig.ticksPerTurn / 5 * 4)
            {
                if(diff < 0)
                    diff += externalEncoderConfig.ticksPerTurn;
                
                if(diff > 0)
                    diff = externalEncoderConfig.ticksPerTurn - diff;
            }
            
	    uint8_t valid = 1;
	    
            //if the zero mark was not long enough for detecting
            //the start, it was noise, or we have a serious problem anyway
	    if(zeroStart == -1)
            {
		valid = 0;
            }
            
	    //a valid end mark has one encoder line high
            if(!((!ain && bin) || (ain && !bin)))
            {
                valid = 0;
            }
            
            //a valid zero mark is exactly one tick long
	    if(abs(diff) != 1)
            {
		valid = 0;
            }
            
	    if(valid) 
	    {                
		externalEncoderValue = 0;
	    
		if(configured)
		    foundZero = 1;	  
	    }    

	    zeroStart = -1; 
	}
	EXTI_ClearITPendingBit(EXTI_Line12);
    }
}

// incremental encoder with zero signal
void encoderInitQuadratureWithZero() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;

    //get default GPIO config
    GPIO_StructInit(&GPIO_InitStructure);

    //Configure GPIO pins: AIN BIN and Zero
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);    

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource14);

    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line12 | EXTI_Line13 | EXTI_Line14;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line13;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line14;
    EXTI_Init(&EXTI_InitStructure);

    //programm encoder interrutps to highest priority
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    externalEncoderValue = 0;
    externalEncoderConfig.ticksPerTurn = 0;
    externalEncoderConfig.tickDivider = 1;

    // set zero encoder state
    uint16_t ain = GPIOB->IDR & GPIO_Pin_13;
    uint16_t bin = GPIOB->IDR & GPIO_Pin_14;
    //printf("init a: %du b: %du \n", ain,bin);
    if(ain) {
        if(bin)
            encoderState = STATE_11;
        else
            encoderState = STATE_10;
    } else {
        if(bin)
            encoderState = STATE_01;
        else
            encoderState = STATE_00;
    }
}

void setTicksPerTurnQuadratureWithZero(uint32_t ticks, uint8_t tickDivider) {
    if(configured && externalEncoderConfig.ticksPerTurn == ticks && externalEncoderConfig.tickDivider == tickDivider)
        return;
    
    externalEncoderConfig.tickDivider = tickDivider;
    externalEncoderConfig.ticksPerTurn = ticks;
    configured = 1;
    foundZero = 0;
}

uint32_t getTicksQuadratureWithZero() {
    if(!foundZero)
	return 0;

    return externalEncoderValue;
}
