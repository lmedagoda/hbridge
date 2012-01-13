#ifndef __USART_H
#define __USART_H

#include "stm32f10x_type.h"
#include "stm32f10x_usart.h"


void USART1_Init(FunctionalState useInterrupts);
void USART1_DeInit(void);
u8 USART1_SendData(const u8 *data, const u32 size);
u32 USART1_GetData (u8 *buffer, const u32 buffer_length);

void USART3_Init(FunctionalState useInterrupts);
void USART3_DeInit(void);
u8 USART3_SendData(const u8 *data, const u32 size);
u32 USART3_GetData (u8 *buffer, const u32 buffer_length);

#endif
