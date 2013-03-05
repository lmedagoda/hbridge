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
#include "protocol_can.h"
#include "encoder.h"
#include "encoder_ichaus.h"
#include "encoder_adc.h"
#include "encoder_quadrature.h"
#include "encoder_quadrature_exti.h"

void GPIO_Configuration(void);


enum hostIDs getOwnHostId() {
    
    GPIO_InitTypeDef GPIO_InitStructure;

    //get default GPIO config
    GPIO_StructInit(&GPIO_InitStructure);

    //Enable GPIOB, GPIOC clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    //configure pb 1 input pull up (maeuseklavier)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //configure pc 13/14/15 as input pull up (maeuseklavier)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    enum hostIDs id = RECEIVER_ID_H_BRIDGE_1; 
    uint16_t gpioData = 0;
    gpioData |= GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);
    gpioData |= (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14) << 1);
    gpioData |= (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15) << 2);
    gpioData |= (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) << 3);

    //get correct host id from gpio pins
    id = gpioData +1;
    printf("Configured as H_BRIDGE_%hu\n", id);

    if(id > 8)
    {
        printf("Wrong host ide configured %hu\n", gpioData);
	//blink and do nothing
	assert_failed((uint8_t *)__FILE__, __LINE__);
    }    
    return (id << 5);
}

/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{
    //setup assert correctly
    Assert_Init(GPIOA, GPIO_Pin_8, USE_USART1);
    
    //Enable peripheral clock for GPIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_Configuration();
    
    USART1_Init(USART_POLL);

    printf_setSendFunction(USART1_SendData);
    
    //note, this mesage can only be send AFTER usart configuration
    printf("Entered main loop\n");

    //turn of red led
    GPIO_SetBits(GPIOA, GPIO_Pin_8);

    //init basic functionality
    //read address, turn on peripherals etc.
    baseInit();

//     //setup I2C bus for lm73cimk
//     setupI2Cx(0xA0, 100000, I2C1, ENABLE);
//     
//     //init temperature sensor
//     lm73cimk_init(I2C1);
// 
//     //address of LM73_SENSOR_1 is 156 // 1001110 + r/w bit
//     lm73cimk_setup_sensor(LM73_SENSOR1, 156);
//     lm73cimk_setup_sensor(LM73_SENSOR2, 148);
    
    printf("Peripheral configuration finished\n");

    enum hostIDs id = getOwnHostId();
    protocol_setOwnHostId(id);
    
    CAN_Configuration(CAN_NO_REMAP);
    CAN_ConfigureFilters(id);
    
    
    can_protocolInit();

    struct EncoderInterface encoder;
    encoder_defaultStructInit(&encoder);

    encoder.encoderInit = encoderInitQuadrature;
    encoder.getTicks = getTicksQuadrature;
    encoder.setTicksPerTurn = setTicksPerTurnQuadrature;
    encoder_setImplementation(QUADRATURE, encoder);

    encoder.encoderInit = encoderInitQuadratureWithZero;
    encoder.getTicks = getTicksQuadratureWithZero;
    encoder.setTicksPerTurn = setTicksPerTurnQuadratureWithZero;
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

  /* Enable GPIOA, GPIOD, USB_DISCONNECT(GPIOC) and USART1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB 
			 | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD
                         | RCC_APB2Periph_USART1, ENABLE);

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;


  //configure PA2 (ADC Channel2) as analog input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //configure PA3 (ADC Channel3) as analog input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //configure PA4 (ADC Channel4) as analog input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //configure PA5 (ADC Channel5) as analog input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //LED (PA8)
  //configure as Push Pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //configure PB0 (ADC Channel8) as analog input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Configure SMBA
  /*
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  */
  //Configure GPIOB Pin 11 as input pull up for emergency switch off
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  //Configure SPI2 pins: SCK, MISO and MOSI
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Configure PC.12 as output push-pull (LED)
  GPIO_WriteBit(GPIOC,GPIO_Pin_12,Bit_SET);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

}
