#include "inc/stm32f10x_gpio.h"
#include "inc/stm32f10x_tim.h"
#include "inc/stm32f10x_rcc.h"
#include "stm32f10x_it.h"
#include "printf.h"
#include "usart.h"
#include "systick.h"
#include "current_measurement.h"
#include "can.h"
#include "assert.h"
#include "i2c.h"
#include "lm73cimk.h"
#include "encoder.h"
#include "encoder_ichaus.h"
#include "encoder_adc.h"
#include "encoder_quadrature.h"
#include "encoder_endswitch.h"
#include "protocol_can.h"
#include <stdlib.h>

enum hostIDs getOwnHostId() {
    
    GPIO_InitTypeDef GPIO_InitStructure;

    //get default GPIO config
    GPIO_StructInit(&GPIO_InitStructure);

    //Enable GPIOB, GPIOC clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    //configure pb 1 input pull up (maeuseklavier)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //configure pc 13/14/15 as input pull up (maeuseklavier)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    enum hostIDs id = RECEIVER_ID_H_BRIDGE_1; 
    uint16_t gpioData = 0;
    gpioData |= GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);
    gpioData |= (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14) << 1);
    gpioData |= (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15) << 2);
    gpioData |= (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) << 3);

    //get correct host id from gpio pins
    id += gpioData;
    printf("Configured as H_BRIDGE_%hu\n", gpioData);

    if(id > 8)
    {
        printf("Wrong host ide configured %hu\n", gpioData);
	//blink and do nothing
	assert_failed((uint8_t *)__FILE__, __LINE__);
    }    
    return id;
}

void GPIO_Configuration(void);


/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{
    GPIO_Configuration();
    //Enable peripheral clock for GPIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    //setup assert correctly
    Assert_Init(GPIOA, GPIO_Pin_12, USE_USART3);

    USART3_Init(USART_POLL, 115200);

    printf_setSendFunction(USART3_SendData);
    
    //note, this mesage can only be send AFTER usart configuration
    printf("Entered main loop\n");

    //turn of red led
    GPIO_SetBits(GPIOA, GPIO_Pin_12);

    //init basic functionality
    //read address, turn on peripherals etc.
    baseInit();

//     printf("Setting up I2C\n");
//     //setup I2C bus for lm73cimk
//     setupI2Cx(0xA0, 100000, I2C1, DISABLE);
//     
// 
//     printf("LM73 init\n");
//     //init temperature sensor
//     lm73cimk_init(I2C1);
//     
//     //address of sensor one 148 // 1001110 + r/w bit
//     printf("LM73 Sensor1 setup\n");
//     lm73cimk_setup_sensor(LM73_SENSOR1, 148);
//     printf("LM73 Sensor2 setup\n");
//     lm73cimk_setup_sensor(LM73_SENSOR2, 144);

    
    printf("Peripheral configuration finished\n");

    enum hostIDs id = getOwnHostId();
    protocol_setOwnHostId(id);
    
    CAN_Configuration(CAN_REMAP1);
    CAN_ConfigureFilters(id);
    
    
    can_protocolInit();

    struct EncoderInterface encoder;
    encoder_defaultStructInit(&encoder);

    encoder.encoderInit = encoderInitQuadratureV2;
    encoder.encoderDeInit = encoderDeInitQuadratureV2;
    encoder.getTicks = getTicksQuadratureV2;
    encoder.setTicksPerTurn = setTicksPerTurnQuadratureV2;
    encoder_setImplementation(QUADRATURE, encoder);

    encoder.encoderInit = encoderInitQuadratureWithZeroV2;
    encoder.encoderDeInit = encoderDeInitQuadratureWithZeroV2;
    encoder.getTicks = getTicksQuadratureWithZeroV2;
    encoder.setTicksPerTurn = setTicksPerTurnQuadratureWithZeroV2;
    encoder_setImplementation(QUADRATURE_WITH_ZERO, encoder);

    encoder.encoderInit = encoderInitIcHaus;
    encoder.getTicks = getTicksIcHaus;
    encoder.setTicksPerTurn = setTicksPerTurnIcHaus;
    encoder_setImplementation(IC_HOUSE_MH_Y, encoder);
    
    encoder.encoderInit = encoderInitADC;
    encoder.encoderDeInit = encoderDeInitADC;
    encoder.getTicks = getTicksADC;
    encoder.setTicksPerTurn = setTicksPerTurnADC;
    encoder_setImplementation(ANALOG_VOLTAGE, encoder);

    encoder.encoderInit = endSwitchEncoder_Init;
    encoder.encoderDeInit = endSwitchEncoder_encoderDeInit;
    encoder.getTicks = endSwitchEncoder_getTicks;
    encoder.setTicksPerTurn = endSwitchEncoder_setTicksPerTurn;
    encoder_setImplementation(END_SWITCH, encoder);

    platformInit();
    
    run();
    
    while(1)
	;
}




/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
******************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  //get default GPIO config
  GPIO_StructInit(&GPIO_InitStructure);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  //LED (PA12)
  //configure as Push Pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}


