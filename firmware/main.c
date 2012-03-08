#include "inc/stm32f10x_lib.h"
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
#include "lm73cimk.h"

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
    u16 gpioData = 0;
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
	assert_failed((u8 *)__FILE__, __LINE__);
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
    
    baseNvicInit();

    USART1_Init(ENABLE);

    //note, this mesage can only be send AFTER usart configuration
    print("Entered main loop\n");

    //turn of red led
    GPIO_SetBits(GPIOA, GPIO_Pin_8);

    //init basic functionality
    //read address, turn on peripherals etc.
    baseInit();
    
    //init temperature sensor
    lm73cimk_init(I2C1, ENABLE);

    //address of LM73_SENSOR_1 is 156 // 1001110 + r/w bit
    lm73cimk_setup_sensor(LM73_SENSOR1, I2C1, ENABLE, 156);
    lm73cimk_setup_sensor(LM73_SENSOR2, I2C1, ENABLE, 148);
    
    //wait until 5V rail get's stable
    vu32 delay = 20000000;
    while(delay)
	delay--;
    
    measureACS712BaseVoltage(); 
    
    print("Peripheral configuration finished\n");

    CAN_Configuration(CAN_NO_REMAP);
    CAN_ConfigureFilters(ownHostId);

    //activate systick interrupt
    //note base init has to be called before 
    SysTick_Configuration();

 
  /** DEBUG**/
/*
  u16 lastTime = 0;
  u16 time;
  u16 counter = 0;  
*/

  u16 cnt = 0;
    
//   u32 temp = 0;
//   u32 gotTmpCnt = 0;
//   u8 lmk72addr = (0x4E<<1);

  /** END DEBUG **/
 
  while(1) {

    /** START DEBUG **/
    /*if(!getTemperature(lmk72addr, &temp)) {
	gotTmpCnt++;
	    //printf("got temp %lu\n", temp);
    }

    printfI2CDbg();
    if(counter > 10000) {
      printf("cur temp is %lu got tmp %lu times\n", temp, gotTmpCnt);
      gotTmpCnt = 0;
      counter = 0;
      print(".");
      u32 eet = getTicksExtern();
      u32 iet = getTicks();
      printf("externalEncoderTicks are %lu internalTicks %lu \n", eet, iet);
      //printf("Error is %h \n", error);
      //print("ActiveCstate: ");
      //printStateDebug(activeCState);
      //print("LastActiveCstate: ");
      //printStateDebug(lastActiveCState);
    }

    time = TIM_GetCounter(TIM1);
    if(lastTime > time) {
      counter++;
    }
    lastTime = time;
*/

    /* END DEBUG */

    pollCanMessages();
  }
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
