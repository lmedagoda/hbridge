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
    
    //127 == 0 (range 0 - 254)
    motor_command.quer_vorne = 127;
    motor_command.quer_hinten = 127;
    motor_command.motor_rechts = 127;
    motor_command.motor_links = 127;
    
    initPackethandling();
    initHbridgeState();
    //state off

    
    currentState.mainboardstate = OFF;
    wantedState.mainboardstate = OFF;
    
    currentState.hbridges[0].state = STATE_UNCONFIGURED;
    currentState.hbridges[1].state = STATE_UNCONFIGURED;
    currentState.hbridges[2].state = STATE_UNCONFIGURED;
    currentState.hbridges[3].state = STATE_UNCONFIGURED;
    
    while(1){
        
        //printf("test");
      //  hbridge_setValue(2,0,0,2);
        //hbridge_requestState(1);
        protocol_processPackage();
        processHbridgestate();
        //Amber lauschen (Befehle erwarten)
        //packet_handling
        processMainboardstate();
        mainboard_run();
        /*
        if ((motor_command.quer_vorne-127)/5 < 30 && (motor_command.quer_vorne-127)/5 > -30 &&
        (motor_command.quer_hinten-127)/5 < 30 && (motor_command.quer_hinten-127)/5 > -30 &&
        (motor_command.motor_rechts-127)/5 < 30 && (motor_command.motor_rechts-127)/5 > -30 &&
        (motor_command.motor_links-127)/5 < 30 && (motor_command.motor_links-127)/5 > -30){
            
        
        printf("MOTORENWERTE: %i, %i, %i, %i \n", 
                (motor_command.quer_vorne-127)/5,
                (motor_command.quer_hinten-127)/5,
                (motor_command.motor_rechts-127)/5,
                (motor_command.motor_links-127)/5);
        
        hbridge_setValue(
                //printf("MOTORENWERTE: %i, %i, %i, %i \n", 
                (motor_command.motor_links-127)/5,
                (motor_command.motor_rechts-127)/5,
                (motor_command.quer_hinten-127)/5,
                (motor_command.quer_vorne-127)/5);
        }
        else printf("OVERCURRENT \n");//Amber Test
        //printf ("Test: %i\n", len);
       //TODO
        //Read packets and handle packets
        //Statemachine
        //Hbridge Command
        //process Amber
        arc_packet_t packet;
        while (amber_getPacket(&packet)){
            printf("handle Packet\n");
            handlePacket(&packet);
        } */
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


