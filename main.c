/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : main.c
* Author             : MCD Application Team
* Version            : V2.0
* Date               : 05/23/2008
* Description        : Main program body
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED 
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.***************************/


/* Includes ---------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "md03.h"
#include "stm32f10x_it.h"
#include "usb.h"
#include "usb_init.h"
#include "usb_desc.h"
#include "usb_lib.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "stdio.h"
#include "controller_interface.h"
#include "spi.h"
#include "i2c.h"

#include "usb/platform_config.h"
#include <stdarg.h>
#include "init.h"
#include "rc.h"

volatile controller_func controller = 0;


/* Private typedef --------------------------------------------------------*/
/* Private define ---------------------------------------------------------*/
/* Private macro ----------------------------------------------------------*/  

/*
#define  USARTx                     USART1
#define  GPIOx                      GPIOA
#define  RCC_APB2Periph_GPIOx       RCC_APB2Periph_GPIOA
#define  GPIO_RxPin                 GPIO_Pin_10
#define  GPIO_TxPin                 GPIO_Pin_9
*/
/* Private variables ------------------------------------------------------*/
/* Private function prototypes --------------------------------------------*/
/* Private functions ------------------------------------------------------*/
void NVIC_Configuration(void);

void USART_Configuration(void);

void GPIO_Configuration(void);

void TIM_Configuration(void);

void SPI_Configuration(void);

#undef printf

int print(const char *format) {
  const char *fmt = format;
  int len = 0;
  
  while(*fmt) {
    fmt++;
    len++;
  }
  
  USB_Send_Data(format, len);

  return len;
}

int printf(const char *format, ...) {
  unsigned char msg[128];
  const char *fmt = format;
  int pos = 0;
  va_list ap;
  int d;
  int i, j;
  char c, *s;

  va_start(ap, format);
  while (*fmt) {
  
    if(*fmt != '%') {
      msg[pos] = *fmt;
      pos++;
      fmt++;
      continue;
    } else {
      fmt++;
      assert_param(*fmt);
    }

    switch (*fmt) {
    case 's':              /* string */
      s = va_arg(ap, char *);
      while(*s) {
	msg[pos] = *s;
	pos++;
	s++;
      }
      break;
    case 'l':               /* unsigned long int */
      fmt++;
      if(*fmt == 'u') {
	d = va_arg(ap, unsigned long int);
      } else {
	d = va_arg(ap, long int);
      }
      
      
      if(d == 0) {
	msg[pos] = '0';
	pos++;
	break;
      }

      if(d < 0) {
	msg[pos] = '-';
	pos++;
	d *= -1;
      }

      i = 0;
      j = d;
      while(j) {
	j /= 10;
	i++;
      }

      j = i;
      while(i) {
	msg[pos + i - 1] = (char) (d % 10) + '0';
	d /= 10;
	i--;
      }
      pos += j;
      break;

    case 'd':              /* int */
      d = va_arg(ap, int);

      if(d == 0) {
	msg[pos] = '0';
	pos++;
	break;
      }

      if(d < 0) {
	msg[pos] = '-';
	pos++;
	d *= -1;
      }

      i = 0;
      j = d;
      while(j) {
	j /= 10;
	i++;
      }

      j = i;
      while(i) {
	msg[pos + i - 1] = (char) (d % 10) + '0';
	d /= 10;
	i--;
      }
      pos += j;
      break;

    case 'c':              /* char */
      /* need a cast here since va_arg only
	 takes fully promoted types */
      c = (char) va_arg(ap, int);
      msg[pos] = c;
      pos++;
      break;
    }
    fmt++;
  }
  
  va_end(ap);

  assert_param(pos < 128);

  USB_Send_Data(msg, pos);  

  return pos;
}

void sendDebugBuffer() {
  while(dbgWrite != dbgRead) {
    printf("Number is %lu ", dbgBuffer[dbgRead]);
    dbgRead = (dbgRead +1) % dbgBufferSize;

    if(dbgBuffer[dbgRead] == I2C_WRITE) { 
      print("Mode is I2C_WRITE ");
    } else {
      if(dbgBuffer[dbgRead] == I2C_READ) {
	print("Mode is I2C_READ "); 
      } else {
	print("Mode is I2C_WRITE_READ ");
      }
    }
    dbgRead = (dbgRead +1) % dbgBufferSize;

    if(((void *) dbgBuffer[dbgRead]) == I2C1) {  
      print("Channel is I2C1 ");
    } else {
      print("Channel is I2C2 ");      
    }
    
    dbgRead = (dbgRead +1) % dbgBufferSize;
    
    switch(dbgBuffer[dbgRead]) {
    case 2:
      dbgRead = (dbgRead +1) % dbgBufferSize;  
      printf("Got event 2 Rx Idx is %lu ", dbgBuffer[dbgRead]);
      dbgRead = (dbgRead +1) % dbgBufferSize;
      printf("Rx Size is %lu \n", dbgBuffer[dbgRead]);
      break;
    case 3:
      dbgRead = (dbgRead +1) % dbgBufferSize;  
      printf("Got event 3 Tx Idx is %lu ", dbgBuffer[dbgRead]);
      dbgRead = (dbgRead +1) % dbgBufferSize;
      printf("Tx Size is %lu \n", dbgBuffer[dbgRead]);
      break;
    case 4:
      dbgRead = (dbgRead +1) % dbgBufferSize;  
      printf("Got event 4 Rx Idx is %lu ", dbgBuffer[dbgRead]);
      dbgRead = (dbgRead +1) % dbgBufferSize;
      printf("Rx Size is %lu \n", dbgBuffer[dbgRead]);
      break;
    case 5:
      dbgRead = (dbgRead +1) % dbgBufferSize;  
      printf("Got event 5 Tx Idx is %lu ", dbgBuffer[dbgRead]);
      dbgRead = (dbgRead +1) % dbgBufferSize;
      printf("Tx Size is %lu \n", dbgBuffer[dbgRead]);
      break;
    case 6:
      dbgRead = (dbgRead +1) % dbgBufferSize;
      printf("Got unknown event %lu \n", dbgBuffer[dbgRead]);
      break;
    case 7:
      dbgRead = (dbgRead +1) % dbgBufferSize;  
      printf("Got event 7 Tx Idx is %lu ", dbgBuffer[dbgRead]);
      dbgRead = (dbgRead +1) % dbgBufferSize;
      printf("Tx Size is %lu \n", dbgBuffer[dbgRead]);
      break;
    case 200:
      dbgRead = (dbgRead +1) % dbgBufferSize;  
      printf("Got Fail Idx is %lu ", dbgBuffer[dbgRead]);
      dbgRead = (dbgRead +1) % dbgBufferSize;
      printf("Size is %lu \n", dbgBuffer[dbgRead]);
      break;
    case 300:
      dbgRead = (dbgRead +1) % dbgBufferSize;  
      printf("\nGlobal Md03 mode is is %lu ", dbgBuffer[dbgRead]);
      dbgRead = (dbgRead +1) % dbgBufferSize;  
      printf("single Md03 mode is is %lu ", dbgBuffer[dbgRead]);
      dbgRead = (dbgRead +1) % dbgBufferSize;  
      printf("device is is %lu \n", dbgBuffer[dbgRead]);
      break;
    default:
      printf("Got event %lu \n", dbgBuffer[dbgRead]);
      break;
    }
    
    dbgRead = (dbgRead +1) % dbgBufferSize;  
  }
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
  //assert_param(0);
  assert_param(1);

  int delay;

  /* GPIO_InitTypeDef GPIO_InitStructure;

   Configure all unused GPIO port pins in Analog Input mode (floating input
     trigger OFF), this will reduce the power consumption and increase the 
     device immunity against EMI/EMC *//*
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE, DISABLE);  

				       */



  
  //Enable peripheral clock for GPIO
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

  /* Enable peripheral clocks ---------------------------------------*/
  /* Enable I2C1 and I2C2 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 | RCC_APB1Periph_I2C2, ENABLE);

  /* Enable USART1 clock */
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);


  NVIC_Configuration();

  //USART_Configuration();

  GPIO_Configuration();
  
  setupI2C();

  Set_USBClock();

  USB_Init();

  controller_interface_init();
  
  TIM_Configuration();

  SPI_Configuration();
  
  delay = 5000000;
  while(delay)
    delay--;
  

  print("Loop start \n");

  delay = 5000000;
  while(delay)
    delay--;

  print("Loop start \n");
 
  delay = 5000000;
  while(delay)
    delay--;  

  //send stop command on startup
  md03_global_state = MD03_ALL_DATA_RECEIVED;
  
  md03_data[MD03_01].acceleration = 50;
  md03_data[MD03_02].acceleration = 50;
  md03_data[MD03_03].acceleration = 50;
  md03_data[MD03_04].acceleration = 50;
  
  md03_data[MD03_01].speed = 0;
  md03_data[MD03_02].speed = 0;
  md03_data[MD03_03].speed = 0;
  md03_data[MD03_04].speed = 0;
  
  md03_data[MD03_01].direction = MD03_FORWARD;
  md03_data[MD03_02].direction = MD03_FORWARD;    
  md03_data[MD03_03].direction = MD03_FORWARD;
  md03_data[MD03_04].direction = MD03_FORWARD;
  
  requestMD03DataWrite();
  
  while(!MD03DataWritten()) {
    print("Waiting for first data write \n");
    printf("MD03 Single State is %d, device num is %d, Global state is %d \n", single_md03_state, cur_md03_device, md03_global_state);
    //sendDebugBuffer();
    if(I2C1_Data.I2CError) {
      md03_global_state = MD03_ALL_DATA_RECEIVED;
      requestMD03DataWriteI2C1();
      I2C1_Data.I2CError = 0;
    }
  }
  
  //init Quadratur counters
  initLS7366();
  
  while(!initLS7366Finished()) {
    ;
  }
  
  //init 'userland' structures and let it install the controller
  init();
  
  if(!controller) {
    while(1) {    
      print("No controller was installed in init() !! \n");
      GPIOC->BRR |= 0x00001000;
      delay = 500000;
      while(delay) 
	delay--;
      GPIOC->BSRR |= 0x00001000;
      delay = 500000;
      while(delay)
	delay--; 
    }
  }
  

  u16 starttime = 0;
  u16 stoptime = 0;
  
  s32 timediff;


  /* Main data aqusition/controll/write loop */
  while (1)
  {
    starttime = TIM_GetCounter(TIM2);

    print("Main Loop start \n");

    requestMD03Data();

    requestLS7366Data();

    while(!MD03DataReady() || !LS7366DataReady()) {
      //print("Waiting for data \n");
      //printf("MD03 Single State is %d, device num is %d, Global state is %d \n", single_md03_state, cur_md03_device, md03_global_state);
      /*printf("MD03 state is %d \n", md03_I2C1_state);
      printf("I2C1_Tx_Ids is %d \n", I2C1_Data.I2C_Tx_Idx);
      printf("I2C1_Tx_Size is %d \n", I2C1_Data.I2C_Tx_Size);
      printf("I2C1_Rx_Idx is %d \n", I2C1_Data.I2C_Rx_Idx);
      printf("I2C1_Rx_Size is %d \n", I2C1_Data.I2C_Rx_Size);
      printf("I2C1Mode is %d \n", I2C1_Data.I2CMode);
      sendDebugBuffer();*/
      
      if(I2C1_Data.I2CError) {
	printf("Error Reason was %d \n", I2C1_Data.I2CErrorReason);
	md03_global_state = MD03_WROTE_ALL_DATA;
	requestMD03DataI2C1();
	I2C1_Data.I2CError = 0;
	printf("Error on I2C%d \n", 1);
      }
      //sendDebugBuffer();
    }

    updateRCValues();
    
    //call current controler
    if(controller) {
      controller();
    }

    
    requestMD03DataWrite();
    
    while(!MD03DataWritten()) {
      //print("Waiting for data write\n");
      //printf("MD03 Single State is %d, device num is %d, Global state is %d \n", single_md03_state, cur_md03_device, md03_global_state);
      
      if(I2C1_Data.I2CError) {
	print("ERROR WRITE\n");
	printf("Error Reason was %d \n", I2C1_Data.I2CErrorReason);
	md03_global_state = MD03_ALL_DATA_RECEIVED;
	requestMD03DataWriteI2C1();
	I2C1_Data.I2CError = 0;
	printf("Error on I2C%d \n", 1);
      }
      //sendDebugBuffer();
    }
    
    //reduce frequency to 100 herz
    do { 
      stoptime = TIM_GetCounter(TIM2);


      if(starttime <= stoptime) { 
	timediff = stoptime - starttime;
      } else {
	timediff = stoptime - starttime + (1<<16);
      }
      //printf("Timediff is %li \n\n", timediff);
    } while (timediff < 10000);
  }
}


/*******************************************************************************
* Function Name  : SPI_Configuration
* Description    : Configures the SPI interface.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_Configuration(void)
{
  /* SPI1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

  SPI_InitTypeDef SPI_InitStructure;

  /* Disable SPI1 for configuration */
  SPI_Cmd(SPI1, DISABLE);

  /* SPI1 Master */
  /* SPI1 Config -----------------------------------------------------------*/
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);

  //set all PINs high (slave not active)
  GPIO_SetBits(GPIOC, GPIO_Pin_6);
  GPIO_SetBits(GPIOC, GPIO_Pin_7);
  GPIO_SetBits(GPIOC, GPIO_Pin_8);
  GPIO_SetBits(GPIOC, GPIO_Pin_9);

  /* Enable SPI1 */
  SPI_Cmd(SPI1, ENABLE);
}


/*******************************************************************************
* Function Name  : TIM_Configuration
* Description    : Configures the different Timers.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_Configuration(void)
{

  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  
  //turn on timer hardware
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  
  //TIM2CLK = 36 MHz, Prescaler = 36, TIM2 
  //counter clock = 1 MHz 
  // Time base configuration 
  TIM_TimeBaseStructure.TIM_Period = 65000;
  TIM_TimeBaseStructure.TIM_Prescaler = 72;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* Prescaler configuration */ 
  TIM_PrescalerConfig(TIM2, 72, TIM_PSCReloadMode_Immediate); 

  /* TIM2 enable counter */ 
  TIM_Cmd(TIM2, ENABLE); 

  /*
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  //init with default values
  TIM_OCStructInit(&TIM_OCInitStructure);
  
  // Output Compare Toggle Mode configuration: Channel1 
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_Pulse = 2000;
  TIM_OC1Init(TIM2, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);
  */
  

  //turn on timer hardware
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  
  //TIM3CLK = 36 MHz, Prescaler = 36, TIM3 
  //counter clock = 1 MHz 
  // Time base configuration 
  TIM_TimeBaseStructure.TIM_Period = 65000;
  TIM_TimeBaseStructure.TIM_Prescaler = 36;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  /* TIM3 configuration: PWM Input mode ------------------------
     The external signal is connected to TIM3 CH3 pin (PB.00), 
     The Rising edge is used as active edge,
     The TIM3 CCR3 contains raising edge time 
     The TIM3 CCR4 contains falling edge time
  ------------------------------------------------------------ */
  TIM_ICInitTypeDef  TIM_ICInitStructure;

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 10;

  TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);

  /* TIM enable counter */
  TIM_Cmd(TIM3, ENABLE);

  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
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


  /* Configure I2C1 pins: SCL and SDA ---------------------------------------*/
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Configure I2C2 pins: SCL and SDA ---------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);


  // Configure PC.12 as output push-pull (LED)
  GPIO_WriteBit(GPIOC,GPIO_Pin_12,Bit_SET);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure USB pull-up pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // Configure USART1 Rx (PA.10) as input floating
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Configure USART1 Tx (PA.09) as alternate function push-pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //configure TIM3 channel 3 as input floating (for RC)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //Configure SPI1 pins: SCK, MISO and MOSI
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //configure PC6-9 as Output Push Pull for SPI
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}


/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures NVIC and Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_StructInit(&NVIC_InitStructure);

  /* Set the Vector Table base location at 0x08000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   

  /* 2 bit for pre-emption priority, 2 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
   
  /* Configure and enable I2C1 interrupt ------------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQChannel;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable I2C2 interrupt ------------------------------------*/
  /*NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQChannel;
  NVIC_Init(&NVIC_InitStructure);*/

  /* Configure and enable USB interrupt -------------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN_RX0_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable Timer interrupt -----------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable SPI1 interrupt ------------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  printf("Wrong parameters value: file %s on line %d\n", file, (int) line);

  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA, ENABLE);

  // Configure PC.12 as output push-pull (LED)
  GPIO_WriteBit(GPIOC,GPIO_Pin_12,Bit_SET);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  int delay;

  while(1)
    {    
      GPIOC->BRR |= 0x00001000;
      delay = 500000;
      while(delay) 
	delay--;
      GPIOC->BSRR |= 0x00001000;
      delay = 500000;
      while(delay)
	delay--; 
    }

  /* Infinite loop */
  while (1)
  {
  }
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
