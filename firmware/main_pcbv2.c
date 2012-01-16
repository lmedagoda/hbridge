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
    u16 gpioData = 0;
    gpioData |= GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);
    gpioData |= (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14) << 1);
    gpioData |= (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15) << 2);
    gpioData |= (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) << 3);

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
    //setup assert correctly
    Assert_Init(GPIOA, GPIO_Pin_12, USE_USART1);

    vu32 delay;
    //Enable peripheral clock for GPIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_Configuration();

    baseNvicInit();

    USART3_Init(DISABLE);

    //note, this mesage can only be send AFTER usart configuration
    print("Entered main loop\n");

    //turn of red led
    GPIO_SetBits(GPIOA, GPIO_Pin_12);

    //init basic functionality
    //read address, turn on peripherals etc.
    baseInit();
    
    //init temperature sensor
    lm73cimk_init(I2C1, DISABLE);
    
    print("Peripheral configuration finished\n");

    CAN_Configuration(CAN_REMAP1);
    CAN_ConfigureFilters(ownHostId);

    //activate systick interrupt, at this point
    //activeCState1 hast to be initalized completely sane !
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

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  //LED (PA12)
  //configure as Push Pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}


