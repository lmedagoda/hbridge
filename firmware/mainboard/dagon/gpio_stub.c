#include <inc/stm32f10x_gpio.h>
#include "../common/arc_packet.h"
#include "../common/mainboardstate.h"
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

extern void dagon_offState(void);
extern void dagon_runningState(void);
extern void dagon_autonomousState(void);

int socked_left;
struct sockaddr_in si_me_left, si_other_left;



void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState){

}


void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct){

    //TODO this is hanky and does not belong here but it is a easy point to initialize things
    struct MainboardState *state_off=mbstate_getState(MAINBOARD_OFF);
    state_off->stateHandler=dagon_offState;

    struct MainboardState *state_running=mbstate_getState(MAINBOARD_RUNNING);
    state_running->stateHandler=dagon_runningState;
    
    struct MainboardState *state_autonomous=mbstate_getState(MAINBOARD_AUTONOMOUS);
    state_autonomous->stateHandler=dagon_autonomousState;
    
    int i;
    int intslen=sizeof(si_other_left);
    char buf[1024];

    if ((socked_left=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1){
        fprintf(stderr,"Couldnot open socked\n");
        exit(-1);
    }
    memset((char *) &si_me_left, 0, sizeof(si_me_left));
    si_me_left.sin_family = AF_INET;
    si_me_left.sin_port = htons(4000);
    si_me_left.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(socked_left, &si_me_left, sizeof(si_me_left))==-1){
        fprintf(stderr,"Bind Failed\n");
        exit(-1);
    }
    printf("Bind Successfull\n");

}

void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal){
}

void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct){
}

void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
}
