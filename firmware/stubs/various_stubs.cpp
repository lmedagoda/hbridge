#include <iostream>
#include <stdlib.h>
#include <stdint.h>
#include "stm32f10x_gpio.h"

enum ASSERT_UASRT {
  USE_USART1,
};

extern "C"
{
void Assert_Init(GPIO_TypeDef* GPIOx, unsigned short GPIO_Pin, enum ASSERT_UASRT used_usart)
{
}


void GPIO_Configuration(void)
{
}
    
void assert_failed(unsigned char* file, unsigned int line)
{
    std::cout << "Assertion failed in file " << file << " on line " << line << std::endl;
    exit(1);
}

uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    return 1;
}
}
