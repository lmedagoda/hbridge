#include "usart.h"
#include "inc/stm32f10x_usart.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_gpio.h"
#include "inc/stm32f10x_lib.h"


volatile struct USART_Data USART1_Data;


/*******************************************************************************
* Function Name  : USART_Configuration
* Description    : Configures USART 1.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART_Configuration(void) {
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

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_StructInit(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART1_Data.RxWritePointer = 0;
    USART1_Data.RxReadPointer = 0;
    
    USART1_Data.TxWritePointer = 0;
    USART1_Data.TxReadPointer = 0;

    USART_InitTypeDef USART_InitStructure;

    /* USART1 and USART2 configuration ---------------------------------------*/
    /* USART and USART2 configured as follow:
	    - BaudRate = 230400 baud  
	    - Word Length = 8 Bits
	    - One Stop Bit
	    - No parity
	    - Hardware flow control disabled (RTS and CTS signals)
	    - Receive and transmit enabled
    */

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    /* Configure USART1 */
    USART_Init(USART1, &USART_InitStructure);

    /* Enable USART1 Receive and Transmit interrupts */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

    /* Enable the USART1 */
    USART_Cmd(USART1, ENABLE);
}

u8 USART1_SendData(const u8 *data, const u32 size) {
  return USARTx_SendData(USART1, &USART1_Data, data, size);
}


/**
 * return 0 on sucess and 1 tx buffer full error
 *
 */
u8 USARTx_SendData(USART_TypeDef* USARTx, volatile struct USART_Data *usart_data,const u8 *buffer, const u32 count) {

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

u32 USART1_GetData (u8 *buffer, const u32 buffer_length) {
  return USARTx_GetData(USART1, &USART1_Data, buffer, buffer_length);
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
* Function Name  : USART1_IRQHandler
* Description    : This function handles USART1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART1_IRQHandler(void)
{ 
  USART_IRQHandler(USART1, &USART1_Data);
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

