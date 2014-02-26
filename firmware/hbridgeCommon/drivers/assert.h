#ifndef CST_ASSERT_H
#define CST_ASSERT_H

#include "inc/stm32f10x_gpio.h"

enum ASSERT_UASRT {
  USE_USART1,
  USE_USART3,
};

void Assert_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, enum ASSERT_UASRT used_usart);

void Assert_assert(int expr);
#endif
