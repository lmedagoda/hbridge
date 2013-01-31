#include "inc/stm32f10x_gpio.h"
#include "inc/stm32f10x_tim.h"
#include "inc/stm32f10x_rcc.h"
#include "stm32f10x_it.h"
#include "printf.h"
#include "usart.h"
#include "systick.h"
#include "current_measurement.h"
#include "can.h"
#include "assert.h"
#include "i2c.h"
#include "lm73cimk.h"
#include "protocol_can.h"
#include "packethandling.h"
#include <stdlib.h>
#include "arc_packet.h"
#include "../common/hbridge_cmd.h"
#include "arc_driver.h"
#include "mainboardstate.h"
#include "run.h"
#include "hbridgestate.h"

//For Amber PING Watchdog
#include "thread.h"
#include "watchdog.h"


#define SYSTEM_ID 2 //ASV
#define CAN_ID_STATUS 0x101
#define CAN_ID_MODE_CHANGED 0x102
#define CAN_ID_EXP_MARKER 0x1C0
#define MAX_BYTES 30
#define CONTROL_CHANNEL_TIMEOUT 50

volatile extern enum hostIDs ownHostId;
extern arc_asv_control_packet_t motor_command;
volatile int state;
volatile MainState currentState;
volatile MainState wantedState;

int lastPacket;
void GPIO_Configuration(void);


/**
 * Function that will registered to the watchdog system to shutdown
 * the hbridges when not receiving pings from the ocu anymore
 */
void shutdownHBridgesAfterTimeout() {
	wantedState.mainboardstate = MAINBOARD_OFF;
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
    GPIO_Configuration();
    //Enable peripheral clock for GPIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    //setup assert correctly
    Assert_Init(GPIOA, GPIO_Pin_12, USE_USART3);

    USART3_Init(USART_POLL);
    //USART2_Init(USART_POLL);
    USART1_Init(USART_USE_INTERRUPTS);
    printf_setSendFunction(USART3_SendData);
    
    
    //note, this mesage can only be send AFTER usart configuration
    printf("This is a firmware for the Maiboard V2\n");
    printf("This firmware is developed in Dec 2012 by AUV 12\n");
    //init basic functionality
    //read address, turn on peripherals etc.
    baseInit();
    printf("Peripheral configuration finished\n");

        
    CAN_Configuration(CAN_REMAP1);
    CAN_ConfigureFilters(-1);   //TODO using better ID -> understand CAN_IDs
     
    ownHostId = SENDER_ID_MAINBOARD;
    can_protocolInit();
    
    protocol_init();
    
    initAmber();

    //Init Watchdog listing for PING pakets on Amber
    //Setting frequeucy below 5 crashes the system for some reason :-/
    watchdog_init(20);
    watchdog_registerWatchDogFunc(shutdownHBridgesAfterTimeout);
    startHardPeriodicThread(5, watchdog_tick);
    
    //127 == 0 (range 0 - 254)
    motor_command.quer_vorne = 127;
    motor_command.quer_hinten = 127;
    motor_command.motor_rechts = 127;
    motor_command.motor_links = 127;
    
    initPackethandling();
    initHbridgeState();
    //state off

    
    currentState.mainboardstate = MAINBOARD_OFF;
    wantedState.mainboardstate = MAINBOARD_OFF;

    
    while(1){
        //handle CAN packets
        protocol_processPackage();
        volatile int time;
        for(time = 0; time < 1000; time++);
        //Statemachine HBridges
        processHbridgestate();
        //Read from Amber-Buffer
        arc_packet_t packet;
        while (amber_getPacket(&packet)){
//            printf("handle Packet, %i\n", packet.packet_id);
            handlePacket(&packet);
        } 
        //Statemachine Mainboard
        processMainboardstate();
        //Running
        mainboard_run();
        //Amber protocol
        amber_processPackets();
    }
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

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    //LED (PA12)
    //configure as Push Pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}


