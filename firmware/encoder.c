#include "inc/stm32f10x_tim.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_gpio.h"
#include "inc/stm32f10x_exti.h"
#include "inc/stm32f10x_nvic.h"
#include <stdlib.h>
#include "printf.h"
#include "encoder.h"
#include "protocol.h"
#include "inc/stm32f10x_spi.h"
#include "encoder_adc.h"
#include "encoder_quadrature.h"

struct EncoderInterface encoders[NUM_ENCODERS];

enum encoderTypes externalEncoderType;
enum encoderStates encoderState;
enum encoderInputs encoderInput;

extern unsigned int systemTick;

vs32 externalEncoderValue = 0;

static vu8 configured = 0;
static vu8 foundZero = 0;

struct encoderData externalEncoderConfig;

void defaultEncoderInit(void) {}
void defaultSetTicksPerTurn(u32 ticks, u8 tickDivider) {}
u32 defaultGetTicks(void) {return 0;}
void defaultEncoderDeInit(void) {}

void defaultInitEncoder(struct EncoderInterface *encoder)
{
    encoder->encoderConfig.configured = 0;
    encoder->encoderConfig.tickDivider = 1;
    encoder->encoderConfig.ticksPerTurn = 0;
    encoder->encoderInit = defaultEncoderInit;
    encoder->getTicks = defaultGetTicks;
    encoder->setTicksPerTurn = defaultSetTicksPerTurn;
    encoder->encoderDeInit = defaultEncoderDeInit;
}

u32 getTicksQuadratureWithZero() {
    if(!foundZero)
	return 0;

    return externalEncoderValue;
}

// Baumer multiturn absolute encoder
void encoderInitBMMV()
{
    print("encoderInitBMMV\n");
    /* SPI2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    SPI_InitTypeDef SPI_InitStructure;

    /* Disable SPI2 for configuration */
    SPI_Cmd(SPI2, DISABLE);

    /* SPI2 Master */
    /* SPI2 Config -----------------------------------------------------------*/
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; // maybe change to 64 for stability
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI2, &SPI_InitStructure);


    /* Enable SPI2 */
    SPI_Cmd(SPI2, ENABLE);

     // TODO: Does something need be changed here?
    externalEncoderConfig.tickDivider = 1;
    externalEncoderConfig.ticksPerTurn = 0;
  
}

void setTicksPerTurnBMMV(u32 ticks, u8 tickDivider)
{
    if(configured && externalEncoderConfig.ticksPerTurn == ticks && externalEncoderConfig.tickDivider == tickDivider)
	return;

    externalEncoderConfig.tickDivider = tickDivider;
    externalEncoderConfig.ticksPerTurn = ticks;
    configured = 1;
}

u32 getTicksBMMV()
{
    static unsigned int lastTick = 0;
    static u32 lastValue = 0;
    
    if(lastTick == systemTick) // if sensor was already polled in this system tick
    {
        return lastValue;
    }
    
    /* Wait for SPI1 Tx buffer empty */
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
    {
  //      print("*");
    }

    GPIO_ResetBits(GPIOB, GPIO_Pin_12);

    u16 dataArray[2] = {0,0};
    SPI_I2S_SendData(SPI2, 0);

    /* Wait for SPI2 data reception */
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
    {
  //      print(".");
    }

    //read current data from data register
    dataArray[1] = SPI_I2S_ReceiveData(SPI2) & ~0x8000; // first bit is not from encoder, discard it

    SPI_I2S_SendData(SPI2, 0);

    /* Wait for SPI2 data reception */
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
    {
 //       print("_");
    }

    //read current data from data register
    dataArray[0] = SPI_I2S_ReceiveData(SPI2);

    u32 value = (dataArray[1] << 16) | dataArray[0];
    value = value >> 6; // align the 25 data bits

    lastTick = systemTick;
    return lastValue = value;
}

u16 getDividedTicksBMMV()
{
     // TODO: Implement more useful algorithm?
    u32 ticks = getTicksBMMV() / externalEncoderConfig.tickDivider;
    return ticks & 4095; // limit to 12 bit
}

void EXTI15_10_IRQHandler(void)
{
    u16 ain = GPIOB->IDR & GPIO_Pin_13; //GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13);
    u16 bin = GPIOB->IDR & GPIO_Pin_14; //GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14);

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

    static s32 zeroStart = -1;

    if(externalEncoderValue < 0)
	externalEncoderValue += externalEncoderConfig.ticksPerTurn;

    if(externalEncoderValue > externalEncoderConfig.ticksPerTurn)
	externalEncoderValue -= externalEncoderConfig.ticksPerTurn;

    if(EXTI_GetITStatus(EXTI_Line12) != RESET) {
	u16 zero = GPIOB->IDR & GPIO_Pin_12; //GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);

	//raising edge
        //a valid zero mark starts with ain and bin low
	if(zero && !ain && !bin) 
	{
	    zeroStart = externalEncoderValue;
	} else {
	    //falling edge
	    
	    //caluclate tick diff
	    s32 diff = zeroStart - externalEncoderValue;
            
            //handle wrap arounds
            if(abs(diff) > externalEncoderConfig.ticksPerTurn / 5 * 4)
            {
                if(diff < 0)
                    diff += externalEncoderConfig.ticksPerTurn;
                
                if(diff > 0)
                    diff = externalEncoderConfig.ticksPerTurn - diff;
            }
            
	    u8 valid = 1;
	    
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
    NVIC_StructInit(&NVIC_InitStructure);    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    externalEncoderValue = 0;
    externalEncoderConfig.ticksPerTurn = 0;
    externalEncoderConfig.tickDivider = 1;

    // set zero encoder state
    u16 ain = GPIOB->IDR & GPIO_Pin_13;
    u16 bin = GPIOB->IDR & GPIO_Pin_14;
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

void setTicksPerTurnQuadratureWithZero(u32 ticks, u8 tickDivider) {
    if(configured && externalEncoderConfig.ticksPerTurn == ticks && externalEncoderConfig.tickDivider == tickDivider)
        return;
    
    externalEncoderConfig.tickDivider = tickDivider;
    externalEncoderConfig.ticksPerTurn = ticks;
    configured = 1;
    foundZero = 0;
}

void encodersInit()
{
    int i;
    for(i = 0; i < NUM_ENCODERS; i++)
    {
    	defaultInitEncoder(encoders + i);
    }

    //the non existing encoder is allways configured
    encoders[NO_ENCODER].encoderConfig.configured = 1;

    encoders[QUADRATURE].encoderInit = encoderInitQuadrature;
    encoders[QUADRATURE].getTicks = getTicksQuadrature;
    encoders[QUADRATURE].setTicksPerTurn = setTicksPerTurnQuadrature;

    encoders[QUADRATURE_WITH_ZERO].encoderInit = encoderInitQuadratureWithZero;
    encoders[QUADRATURE_WITH_ZERO].getTicks = getTicksQuadratureWithZero;
    encoders[QUADRATURE_WITH_ZERO].setTicksPerTurn = setTicksPerTurnQuadratureWithZero;

    encoders[BMMV30_SSI].encoderInit = encoderInitBMMV;
    encoders[BMMV30_SSI].getTicks = getTicksBMMV;
    encoders[BMMV30_SSI].setTicksPerTurn = setTicksPerTurnBMMV;
    
    encoders[ANALOG_VOLTAGE].encoderInit = encoderInitADC;
    encoders[ANALOG_VOLTAGE].encoderDeInit = encoderDeInitADC;
    encoders[ANALOG_VOLTAGE].getTicks = getTicksADC;
    encoders[ANALOG_VOLTAGE].setTicksPerTurn = setTicksPerTurnADC;
    
    encoders[V2_QUADRATURE].encoderInit = encoderInitADC;
    encoders[V2_QUADRATURE].getTicks = getTicksADC;
    encoders[V2_QUADRATURE].setTicksPerTurn = setTicksPerTurnADC;

    encoders[V2_QUADRATURE_WITH_ZERO].encoderInit = encoderInitADC;
    encoders[V2_QUADRATURE_WITH_ZERO].getTicks = getTicksADC;
    encoders[V2_QUADRATURE_WITH_ZERO].setTicksPerTurn = setTicksPerTurnADC;

}


u32 getTicks(enum encoderTypes type)
{
    return encoders[type].getTicks();
}

u16 getDividedTicks(enum encoderTypes type)
{
    if(encoders[type].encoderConfig.tickDivider != 0)
        return encoders[type].getTicks() / encoders[type].encoderConfig.tickDivider;
    else
        return 0;
}

void setTicksPerTurn(enum encoderTypes type, u32 ticks, u8 tickDivider)
{
    encoders[type].encoderConfig.configured = 1;

    //do not bother encoder code with anything if the config didn't change
    if((encoders[type].encoderConfig.ticksPerTurn == ticks * tickDivider) && (encoders[type].encoderConfig.tickDivider == tickDivider))
        return;
 
    encoders[type].encoderConfig.tickDivider = tickDivider;
    encoders[type].encoderConfig.ticksPerTurn = ticks * tickDivider;
    encoders[type].setTicksPerTurn(ticks * tickDivider, tickDivider);
}

u32 getTicksPerTurn(enum encoderTypes type)
{
    return encoders[type].encoderConfig.ticksPerTurn;
}

u8 getTickDivider(enum encoderTypes type)
{
    if(encoders[type].encoderConfig.tickDivider == 0)
	return 1;
    return encoders[type].encoderConfig.tickDivider;
}

u8 encoderConfigured(enum encoderTypes type)
{
    return encoders[type].encoderConfig.configured;
}


void initEncoder(enum encoderTypes type) 
{
    encoders[type].encoderInit();
    if(type != NO_ENCODER)
        encoders[type].encoderConfig.configured = 0;
}

void deinitEncoder(enum encoderTypes type) 
{
    encoders[type].encoderDeInit();
}
