#ifndef __USART_H
#define __USART_H

#include "stm32f10x_type.h"
#include "stm32f10x_usart.h"


#define USART_BUFFER_SIZE 128

struct USART_Data {
  u8 RxBuffer[USART_BUFFER_SIZE];
  u16 RxWritePointer;
  u16 RxReadPointer;
  
  u8 TxBuffer[USART_BUFFER_SIZE];
  u16 TxWritePointer;
  u16 TxReadPointer;

  u8 RxBufferFullError;

};

extern volatile struct USART_Data USART1_Data;

void USART_Configuration(void);

u8 USART1_SendData(const u8 *data, const u32 size);
u8 USARTx_SendData(USART_TypeDef* USARTx, volatile struct USART_Data *usart_data,const u8 *data, const u32 size);

void USART_IRQHandler(USART_TypeDef* USARTx, volatile struct USART_Data *data);



#endif
