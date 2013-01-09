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
#define SYSTEM_ID 2 //ASV
#define CAN_ID_STATUS 0x101
#define CAN_ID_MODE_CHANGED 0x102
#define CAN_ID_EXP_MARKER 0x1C0
#define MAX_BYTES 30
#define CONTROL_CHANNEL_TIMEOUT 50

extern arc_asv_control_packet_t motor_command;
volatile int state;
int lastPacket;
void GPIO_Configuration(void);
void stateHandler(int id, unsigned char *data, unsigned short size){
    printf("a");
    struct announceStateData *asd = (struct announceStateData*) data;
        
    if (state == STATE_SENSOR_ERROR){
        printf("try:");
        if(asd->curState == (uint8_t) STATE_UNCONFIGURED){
            state = STATE_UNCONFIGURED;
            printf("HBridge Ready\n");
        }else{
            printf("got wrong state: %i, not %i\n", asd->curState, STATE_UNCONFIGURED);
        }
    }
    if (state == STATE_UNCONFIGURED){
        if(asd->curState == (uint8_t) STATE_SENSORS_CONFIGURED){
            state = STATE_SENSORS_CONFIGURED;
            printf("sensors configured\n");
        }else{
            printf("got wrong state: %i, not %i\n", asd->curState, STATE_SENSORS_CONFIGURED);
        }
    }
    if(state == STATE_SENSORS_CONFIGURED){
        if(asd->curState == (uint8_t) STATE_ACTUATOR_CONFIGURED){
            state = STATE_ACTUATOR_CONFIGURED;
            printf("Actuators configured\n");
        }else{
            printf("got wrong state: %i, not %i\n", asd->curState, STATE_ACTUATOR_CONFIGURED);
        }
    }
    if(state == STATE_ACTUATOR_CONFIGURED){
        if(asd->curState == (uint8_t) STATE_CONTROLLER_CONFIGURED){
            state = STATE_CONTROLLER_CONFIGURED;
            printf("controllers configured");
        }else{
            printf("got wrong state: %i, not %i\n", asd->curState, STATE_CONTROLLER_CONFIGURED);
        }
    }
    if(state == STATE_CONTROLLER_CONFIGURED){
        if(asd->curState == (uint8_t) STATE_RUNNING){
            state = STATE_RUNNING;
            printf("running");
        }else{
            printf("got wrong state: %i, not %i\n", asd->curState, STATE_RUNNING);
        }
    }

}

void ackHandler(int id, unsigned char *data, unsigned short size){
    /*struct ackData *ack = (struct ackData*) &data;
    if(ack->packetId == lastPacket)
    {
        lastPacket = 0;
    }*/
    //printf("ack");
}

void statusHandler(int id, unsigned char *data, unsigned short size){
    //printf("status");
    //printf("status");
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
    CAN_ConfigureFilters(RECEIVER_ID_ALL);   //TODO using better ID -> understand CAN_IDs
     
    can_protocolInit();
    
    protocol_init();
    /*state = STATE_SENSOR_ERROR;
    state = STATE_SENSOR_ERROR;*/
    state = STATE_UNCONFIGURED;
    protocol_registerHandler(PACKET_ID_ANNOUNCE_STATE, &stateHandler);
    protocol_registerHandler(PACKET_ID_ACK, &ackHandler);
    protocol_registerHandler(PACKET_ID_STATUS, &statusHandler);
    protocol_registerHandler(PACKET_ID_EXTENDED_STATUS, &statusHandler);
    //printf("0\n");
 //   usleep(20);
    int i;
    for(i = 0; i < 1000; i++){
        printf("waiting...\n");
    }
    //hbridge_sendClearError();
    //while(state == STATE_SENSOR_ERROR){
    //    protocol_processPackage();
   // }

   //hbridge_sendEncoderConfiguration(1);
    //hbridge_sendEncoderConfiguration(2);
    hbridge_sendEncoderConfiguration(3);
    //hbridge_sendEncoderConfiguration(4);
    //lastPacket = PACKET_ID_SET_SENSOR_CONFIG;
    printf("1\n");
    while(state == STATE_UNCONFIGURED){
        protocol_processPackage();
    }
    printf("2\n");
    hbridge_sendActuatorConfiguration(1);
    hbridge_sendActuatorConfiguration(2);
    hbridge_sendActuatorConfiguration(3);
    hbridge_sendActuatorConfiguration(4);
    printf("3\n");
    while(state == STATE_SENSORS_CONFIGURED){
        protocol_processPackage();
    }
    printf("4\n");
    hbridge_sendControllerConfiguration(1);
    hbridge_sendControllerConfiguration(2);
    hbridge_sendControllerConfiguration(3);
    hbridge_sendControllerConfiguration(4);
    printf("5\n");
    while(state == STATE_ACTUATOR_CONFIGURED){
        protocol_processPackage();
    }
    printf("6\n");
    hbridge_setValue(0,0,0,2);
    printf("7\n");
    while(state == STATE_CONTROLLER_CONFIGURED){
        protocol_processPackage();
    }
    printf("test\n");

            //hbridge_setValue(2,1,1,1);
        
    //platformInit();
    //run();*/
    initAmber();
    motor_command.quer_vorne = 127;
    motor_command.quer_hinten = 127;
    motor_command.motor_rechts = 127;
    motor_command.motor_links = 127;
    initPackethandling();
    
    //state off
    
    while(1){
        printf("test");
      //  hbridge_setValue(2,0,0,2);
        
        //process_hbridges
        //Amber lauschen (Befehle erwarten)
        //packet_handling
        //process_state
        //run
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
        /*arc_packet_t packet;
        while (amber_getPacket(&packet)){
            printf("handle Packet\n");
            handlePacket(&packet);
        } */
        //amber_processPackets(); 
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


