#include "usart.h"
#include "inc/stm32f10x_usart.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_gpio.h"
#include "inc/stm32f10x_lib.h"
#include "stm32f10x_it.h"

#define USART_BUFFER_SIZE 256

//internal helper functions, 
//user space does not need to know 
//about these
struct USART_Data {
  u8 RxBuffer[USART_BUFFER_SIZE];
  u16 RxWritePointer;
  u16 RxReadPointer;
  
  u8 TxBuffer[USART_BUFFER_SIZE];
  u16 TxWritePointer;
  u16 TxReadPointer;

  u8 RxBufferFullError;

  FunctionalState usesInterrupts;
};

u8 USARTx_SendData(USART_TypeDef* USARTx, volatile struct USART_Data *usart_data,const u8 *data, const u32 size);
u32 USARTx_GetData(USART_TypeDef* USARTx, volatile struct USART_Data *usart_data, u8 *buffer, const u32 buffer_length);
void USART_IRQHandler(USART_TypeDef* USARTx, volatile struct USART_Data *data);

volatile struct USART_Data USART1_Data;
volatile struct USART_Data USART3_Data;

void USART1_Init(FunctionalState useInterrupts)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //get default GPIO config
    GPIO_StructInit(&GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    // Configure USART1 Tx (PA09) as alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure USART1 Rx (PA10) as input floating
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure); 

    USART1_Data.RxWritePointer = 0;
    USART1_Data.RxReadPointer = 0;
    
    USART1_Data.TxWritePointer = 0;
    USART1_Data.TxReadPointer = 0;
    USART1_Data.usesInterrupts = useInterrupts;

    if(useInterrupts)
    {
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_StructInit(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    }

    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    /* Configure USART1 */
    USART_Init(USART1, &USART_InitStructure);

    /* Enable USART1 Receive and Transmit interrupts */
    USART_ITConfig(USART1, USART_IT_RXNE, useInterrupts);
    USART_ITConfig(USART1, USART_IT_TXE, useInterrupts);

    /* Enable the USART1 */
    USART_Cmd(USART1, ENABLE);
}


void USART1_DeInit(void )
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_StructInit(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Disable USART1 Receive and Transmit interrupts
    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);

    // Enable the USART1
    USART_Cmd(USART1, DISABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, DISABLE);
}

void USART3_Init(FunctionalState useInterrupts)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //get default GPIO config
    GPIO_StructInit(&GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    // Configure USART1 Tx (PA09) as alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Configure USART1 Rx (PA10) as input floating
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure); 

    USART3_Data.RxWritePointer = 0;
    USART3_Data.RxReadPointer = 0;
    
    USART3_Data.TxWritePointer = 0;
    USART3_Data.TxReadPointer = 0;
    USART3_Data.usesInterrupts = useInterrupts;

    if(useInterrupts)
    {
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_StructInit(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    }

    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    /* Configure USART1 */
    USART_Init(USART3, &USART_InitStructure);

    /* Enable USART1 Receive and Transmit interrupts */
    USART_ITConfig(USART3, USART_IT_RXNE, useInterrupts);
    USART_ITConfig(USART3, USART_IT_TXE, useInterrupts);

    /* Enable the USART1 */
    USART_Cmd(USART3, ENABLE);
}

void USART3_DeInit(void )
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_StructInit(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Disable USART1 Receive and Transmit interrupts
    USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
    USART_ITConfig(USART3, USART_IT_TXE, DISABLE);

    // Enable the USART1
    USART_Cmd(USART3, DISABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, DISABLE);
}



u8 USART1_SendData(const u8 *data, const u32 size) {
  return USARTx_SendData(USART1, &USART1_Data, data, size);
}

u8 USART3_SendData(const u8* data, const u32 size)
{
  return USARTx_SendData(USART3, &USART3_Data, data, size);
}

u32 USART1_GetData (u8 *buffer, const u32 buffer_length) {
  return USARTx_GetData(USART1, &USART1_Data, buffer, buffer_length);
}

u32 USART3_GetData (u8 *buffer, const u32 buffer_length) {
  return USARTx_GetData(USART3, &USART3_Data, buffer, buffer_length);
}

void USART1_IRQHandler(void)
{ 
  USART_IRQHandler(USART1, &USART1_Data);
}

void USART3_IRQHandler(void)
{ 
  USART_IRQHandler(USART3, &USART3_Data);
}


/**
 * return 0 on sucess and 1 tx buffer full error
 *
 */
u8 USARTx_SendData(USART_TypeDef* USARTx, volatile struct USART_Data *usart_data,const u8 *buffer, const u32 count) {
    if(!usart_data->usesInterrupts)
    {
	u32 pos = 0;

	for(pos = 0; pos < count; pos++)
	{
	    USART_SendData(USARTx, buffer[pos]);
	
	    //Wait until data is send
	    while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
	    {
	    }
	}
	return 0;
    }
    
    volatile int free_space = 0;
    u32 size = 0;

    free_space = (USART_BUFFER_SIZE - (usart_data->TxWritePointer - usart_data->TxReadPointer) - 1) % USART_BUFFER_SIZE;
    if(free_space < count) {
      return 1;
    }
    
    //write to ringbuffer
    while(size < count) {
      usart_data->TxBuffer[usart_data->TxWritePointer] = buffer[size];
      usart_data->TxWritePointer = (usart_data->TxWritePointer + 1) % USART_BUFFER_SIZE;
      size++;
    }


    //    if(!USART_GetITStatus(USARTx, USART_IT_TXE)) {
      //Enable the USARTx Transmit interrupt
    USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);
      //}
    
    return 0;      
}

/**
 * Fetches Data from the receive queue.
 * returns bytes fetched from rx queue
 */
u32 USARTx_GetData(USART_TypeDef* USARTx, volatile struct USART_Data *usart_data, u8 *buffer, const u32 buffer_length) {
  u32 counter = 0;
  while(counter < buffer_length && usart_data->RxWritePointer != usart_data->RxReadPointer) {
    buffer[counter] = usart_data->RxBuffer[usart_data->RxReadPointer];
    counter++;
    usart_data->RxReadPointer = (usart_data->RxReadPointer + 1) % USART_BUFFER_SIZE;
  }
  return counter;
}


/*******************************************************************************
* Function Name  : USARTx_IRQHandler
* Description    : This function handles USARTx global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART_IRQHandler(USART_TypeDef* USARTx,volatile struct USART_Data *data)
{
  if(USART_GetITStatus(USARTx, USART_IT_RXNE) != RESET)
  {
    vu8 nextRxWritePointer = (data->RxWritePointer + 1) % USART_BUFFER_SIZE;

    //ringbuffer is full :-((
    if(nextRxWritePointer == data->RxReadPointer) {
      //set error flag and disable receiving
      data->RxBufferFullError = 1;
      //Disable the USARTx Receive interrupt
      USART_ITConfig(USARTx, USART_IT_RXNE, DISABLE);
    }

    // Read one byte from the receive data register
    data->RxBuffer[data->RxWritePointer] = USART_ReceiveData(USARTx);      
    data->RxWritePointer = nextRxWritePointer;
  }
  
  if(USART_GetITStatus(USARTx, USART_IT_TXE) != RESET)
  {
    //ringbuffer empty, nothing more to send
    if(data->TxReadPointer == data->TxWritePointer)
    {
      //Disable the USARTx Transmit interrupt
      USART_ITConfig(USARTx, USART_IT_TXE, DISABLE);
    } else {
      // Write one byte to the transmit data register
      USART_SendData(USARTx, data->TxBuffer[data->TxReadPointer]);                    
      data->TxReadPointer = (data->TxReadPointer + 1) % USART_BUFFER_SIZE;
    }
  }
}

