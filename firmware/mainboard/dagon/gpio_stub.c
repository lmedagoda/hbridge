#include <inc/stm32f10x_gpio.h>
#include "../common/arc_packet.h"
#include "../common/mainboardstate.h"

extern void dagon_offState(void);
extern void dagon_runningState(void);



void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState){

}

void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct){

    //TODO this is hanky and does not belong here but it is a easy point to initialize things
    struct MainboardState *state_off=mbstate_getState(MAINBOARD_RUNNING);
    state_off->stateHandler=dagon_offState;

    struct MainboardState *state_running=mbstate_getState(MAINBOARD_RUNNING);
    state_running->stateHandler=dagon_runningState;

}

void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal){
}

void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct){
}

void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
}
