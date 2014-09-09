#include <inc/stm32f10x_gpio.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/select.h>
#include <sys/ioctl.h>




void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState){

}


void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct){

}

void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal){
}

void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct){
}

void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
}
