#include "../common/mainboardstate.h"
#include "../common/packethandling.h"
#include "../common/arc_packet.h"
#include "../common/time.h"
#include "../common/timeout.h"
#include "../common/arc_tokendriver.h"
#include "../../common/hbridge_cmd2.h"
#include "../../common/hbridge_cmd.h"
#include "../../common/protocol.h"
#include "../../interfaces/thread.h"
#include "../../hbridgeCommon/protocol_can.h"
#include "../../hbridgeCommon/drivers/can.h"
#include "../../hbridgeCommon/drivers/usart.h"
#include <hbridgeCommon/lib/STM32F10x_StdPeriph_Driver/inc/stm32f10x_can.h>
#include "../../hbridgeCommon/drivers/assert.h"
#include "../../hbridgeCommon/drivers/printf.h"
#include <stddef.h>
#include "uwmodem.h"
#include "asv_can.h"
#define STATUS_PACKET_PERIOD 200000 
struct arc_asv_control_packet{
    uint8_t quer_vorne;
    uint8_t quer_hinten;
    uint8_t motor_rechts;
    uint8_t motor_links;	
} __attribute__ ((packed)) __attribute__((__may_alias__));

void hbridgeStatusHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size){
}
void hbridgeExtendedStatusHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size){
}

void sendStatusPacket(){
    arc_status_packet_t status_information;
    status_information.current_state = (ARC_SYSTEM_STATE)mbstate_getCurrentState();
    status_information.wanted_state = (ARC_SYSTEM_STATE)mbstate_getCurrentState();
    arc_packet_t packet;
    packet.originator = SLAVE;
    packet.system_id = ASV;//(ARC_SYSTEM_ID) SYSTEM_ID;
    packet.packet_id = MB_STATUS;
    packet.length = sizeof(arc_status_packet_t);
    int i;
    for (i=0; i<sizeof(arc_status_packet_t); i++){
        packet.data[i] = ((char*)&status_information)[i]; 
    }
    //printf("send status packet\n");
    arctoken_sendPacket(&packet);
}


int status_loops = 0;
struct arc_asv_control_packet curCmd;
uint8_t cmdValid;
void asv_controlHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size){
    printf("Control Packet\n");
    printf("Data: %i, %i, %i, %i\n");
    struct arc_asv_control_packet *cmd  = (struct arc_asv_control_packet *) data;
    curCmd = *cmd;
    cmdValid = 1;
}

void asv_packet_canHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size){
    timeout_reset();
    struct canMsg *inMsg = (struct canMsg*) data;
    const uint8_t payloadSize = size-2;
    CanTxMsg msg;
    msg.StdId = data[0];
    msg.StdId = (msg.StdId << 3) | (data[1]>>5);
    msg.RTR=CAN_RTR_DATA;
    msg.IDE=CAN_ID_STD;

    printf("Got A Can Passhrough and send it out with id %i\n", msg.StdId);
    printf("ARC letzten Byte %i", data[9]);
    msg.DLC = payloadSize;
    int i;
    //copy down data as first two are CAN id and index
    for(i=0; i < payloadSize; i++) {
	msg.Data[i] = inMsg->data[i];
    }
    printf("CAN letztes Byte %i, %i, %i", msg.Data[7], inMsg->data[7], msg.DLC);
    //Simulates receiving self send can messages
    asvcan_handlePacket(&msg);
    //Real sending of the Can Package
    CAN_SendMessage(&msg);

}

void asv_runningState(void){
    if(timeout_hasTimeout())
    {
        printf("Timout, switching to off\n");
        mbstate_changeState(MAINBOARD_OFF);
    }

    //check actuators
    if(hbridge_hasActuatorError())
    {
        printf("Actuator error, switching to off\n");
        mbstate_changeState(MAINBOARD_OFF);
        return;
    }

    if (!cmdValid)
        return;
    printf("Set Value: %i, %i, %i, %i\n", (curCmd.motor_links-127)*20, (curCmd.motor_rechts-127)*20,(curCmd.quer_hinten-127)*20,  (curCmd.quer_vorne-127)*20);
    hbridge_setValue((curCmd.motor_links-127)*4, (curCmd.motor_rechts-127)*4, (curCmd.quer_hinten-127)*4, (curCmd.quer_vorne-127)*4);

}

int main()
{
    Assert_Init(GPIOA, GPIO_Pin_12, USE_USART3);

    USART3_Init(USART_POLL);

    printf_setSendFunction(USART3_SendData);

    USART1_Init(USART_USE_INTERRUPTS);

    printf("The Maiboard is up with the version: ");

    timeout_init(3000);

    protocol_setOwnHostId(SENDER_ID_MAINBOARD);

    CAN_Configuration(CAN_REMAP1);
    CAN_ConfigureFilters(0);

    startHardPeriodicThread(1000, time_msPassed);
    uint32_t lastStateTime = time_getTimeInMs();

    can_protocolInit();
    //uwmodem_init(&USART3_SendData, &USART1_GetData, &USART1_SeekData);
    //wait till rest got up
    while(time_getTimeInMs() - lastStateTime < 30)
        ;
    protocol_init(1);
    protocol_registerHandler(PACKET_ID_STATUS, &hbridgeStatusHandler);
    protocol_registerHandler(PACKET_ID_EXTENDED_STATUS, &hbridgeExtendedStatusHandler);
    protocol_setRecvFunc(asvcan_recvPacket);
    hbridge_init(4);
    int i;
    struct actuatorConfig *ac;
    struct sensorConfig *sc;
    for (i=0; i<4; i++){
        hbridge_setControllerWithData(i, CONTROLLER_MODE_PWM, 0, NULL, 0);
        ac = getActuatorConfig(i);
        ac->maxCurrent = 1000;
        //ac->maxPWM = 5000;
        ac->maxCurrentCount = 200;
        ac->pwmStepPerMs = 2;
        sc = getSensorConfig(i);
        sc->statusEveryMs = 100;
        hbridge_configureSensors();

    }

    arctoken_init(&USART1_SendData, &USART1_GetData, &USART1_SeekData);
    arctoken_setOwnSystemID(ASV);
    mbstate_init();
    packet_init();

    packet_registerHandler(MB_CONTROL, asv_controlHandler);
    packet_registerHandler(MB_ID_CAN, asv_packet_canHandler);

    ///Overload the state handler for running
    struct MainboardState *state=mbstate_getState(MAINBOARD_RUNNING);
    state->stateHandler=asv_runningState;
    while(1)
    {
        //uwmodem_process();
        unsigned int curTime = time_getTimeInMs();
        //only call state processing every ms
        if(curTime != lastStateTime)
        {
            //process state handlers
            mbstate_processState();

            lastStateTime = curTime;

            // 	    printf(".");
        }
        arc_packet_t packet;	
        while(arctoken_readPacket(&packet))
        {
            //process incomming packet
            //printf("incoming packet");
            packet_handlePacket(packet.originator, packet.system_id, packet.packet_id, packet.data, packet.length);	
        }
        hbridge_process();
        arctoken_processPackets();
        //Sending a Status packet
        if (status_loops >= STATUS_PACKET_PERIOD){
            sendStatusPacket();
            status_loops = 0;
        } else {
            status_loops++;
        }
    }
    return 0;
}

