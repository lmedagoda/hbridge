#include "uwmodem.h"
#include "avalon_can.h"
#include "../common/mainboardstate.h"
#include "../common/packethandling.h"
#include "../common/arc_packet.h"
#include "../common/time.h"
#include "../common/timeout.h"
#include "../common/arc_driver.h"
#include "../../common/hbridge_cmd2.h"
#include "../../common/hbridge_cmd.h"
#include "../../common/protocol.h"
#include "../../interfaces/thread.h"
#include "../../hbridgeCommon/protocol_can.h"
#include "../../hbridgeCommon/drivers/can.h"
#include "../../hbridgeCommon/drivers/usart.h"
#include "../../hbridgeCommon/drivers/assert.h"
#include "../../hbridgeCommon/drivers/printf.h"
#include <stddef.h>
#include "avalon_types.h"
#define STATUS_PACKET_PERIOD_MS 500 
 

uint32_t lastStateTime;
uint8_t substate = 0;

//------- Importtand Defines ----------//
#define SYSTEM_ID AVALON
#ifndef NUM_MOTORS
    #define NUM_MOTORS 6
#endif
#define SURFACE_SIGN 0xFF
#define SURFACE_SIGN_COUNT 6 

#ifdef DAGON
extern void avalon_offState(void);
extern void avalon_runningState(void);
extern void avalon_autonomousState(void);
#endif

//USART? = AMBER
//USART3 = Underwater Modem
//USART? = Ethernet/Serial
//TODO Add CAN IDs to keep a easy overview
//TODO Where co configure motor commands and set them to the motors
avalon_status_t current_status;
uint8_t full_autonomous_minuetes;
uint8_t full_autonomous_timeout;
void hbridgeStatusHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size){
}
void hbridgeExtendedStatusHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size){
}
int surface(){
    mbstate_changeState(MAINBOARD_OFF);
    current_status.change_reason = CR_EMERGENCY;
    return 0;
}

void sendStatusPacket(){
    CanTxMsg msg;
    msg.StdId = 0x541;
    msg.RTR=CAN_RTR_DATA;
    msg.IDE=CAN_ID_STD;
    msg.DLC = 8;
    msg.Data[2] =(ARC_SYSTEM_STATE)mbstate_getCurrentState();
    msg.Data[7] = substate;
    CAN_SendMessage(&msg);

    avalon_status_t status_information;
    status_information.current_state = (ARC_SYSTEM_STATE)mbstate_getCurrentState();
    status_information.wanted_state = current_status.wanted_state;
    status_information.current_depth = current_status.current_depth;
    status_information.water_ingress_front = current_status.water_ingress_front;
    status_information.water_ingress_back =  current_status.water_ingress_back;
    status_information.change_reason = current_status.change_reason;
    arc_packet_t packet;
    packet.originator = SLAVE;
    packet.system_id = SYSTEM_ID;
    packet.packet_id = MB_STATUS;
    packet.length = sizeof(avalon_status_t);
    int i;
    for (i=0; i<sizeof(avalon_status_t); i++){
        packet.data[i] = ((char*)&status_information)[i]; 
    }
    //printf("send status packet\n");
    arc_multichannel_sendPacket(&packet);
}
void avalon_packet_setStateHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size){
    substate = data[1];
    if ((enum MAINBOARDSTATE) *data == MAINBOARD_FULL_AUTONOMOUS){
        if (size > 2){
            full_autonomous_timeout = data[2]; 
            full_autonomous_minuetes = 0;
            timeout_init(30000);
        } else {
            timeout_init(10000);
        }
    } else {
        timeout_init(10000);
    }
    timeout_reset();
    packet_setStateHandler(senderId, receiverId, id, data, size);
}

void avalon_packet_canHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size){
    timeout_reset();
    struct canMsg *inMsg = (struct canMsg*) data;
    const uint8_t payloadSize = size-2;
    CanTxMsg msg;
    msg.StdId = data[0];
    msg.StdId = (msg.StdId << 3) | (data[1]>>5);
    msg.RTR=CAN_RTR_DATA;
    msg.IDE=CAN_ID_STD;

    msg.DLC = payloadSize;
    int i;
    //copy down data as first two are CAN id and index
    for(i=0; i < payloadSize; i++) {
	msg.Data[i] = inMsg->data[i];
    }
    //Simulates receiving self send can messages
    //asvcan_handlePacket(&msg);
    //Real sending of the Can Package
    CAN_SendMessage(&msg);

}
int status_loops = 0;
struct arc_avalon_control_packet curCmd;
uint8_t cmdValid;
void avalon_controlHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size){
    timeout_reset();
    struct arc_avalon_control_packet *cmd  = (struct arc_avalon_control_packet *) data;

    curCmd = *cmd;
    cmdValid = 1;
}

void avalon_full_autonomousState(void){
    if (timeout_hasTimeout()){
        full_autonomous_minuetes++;
        timeout_reset();
    }
    if (full_autonomous_minuetes/2 >= full_autonomous_timeout){
        printf("Timout, switching to off\n");
        timeout_init(20000);
        timeout_reset();
        mbstate_changeState(MAINBOARD_EMERGENCY);
        current_status.change_reason = CR_MB_TIMEOUT;
    } 
}
void avalon_emergency(void){
    if (timeout_hasTimeout()){
        mbstate_changeState(MAINBOARD_OFF);
    } else {
        hbridge_setValues(
                0, 
                0,
                    270  *-1,
                    ((1*3)/10) * 270 *-1,
                PACKET_ID_SET_VALUE14);

        hbridge_setValues(
                0,
                0,
                0,
                0,
                PACKET_ID_SET_VALUE58);
        current_status.change_reason = CR_LEGAL;
    }

}

#ifndef DAGON
void avalon_autonomousState(void){
    if(timeout_hasTimeout())
    {
        printf("Timout, switching to off\n");
        mbstate_changeState(MAINBOARD_OFF);
        current_status.change_reason = CR_MB_TIMEOUT;
        timeout_reset();
        return;
    }
    current_status.change_reason = CR_LEGAL;
}
void avalon_offState(void){
    timeout_reset();
}
void avalon_runningState(void){
    //printf("runs avalon\n");

    if(timeout_hasTimeout())
    {
        printf("Timout, switching to off\n");
        mbstate_changeState(MAINBOARD_OFF);
        current_status.change_reason = CR_MB_TIMEOUT;
        timeout_reset();
        return;
    }

    //check actuators
    if(hbridge_hasActuatorError())
    {
        printf("Actuator error, switching to off\n");
        mbstate_changeState(MAINBOARD_OFF);
        current_status.change_reason = CR_HB_ERROR;
        return;
    }

    if (!cmdValid)
        return;

    //TODO mapping korrigieren

    /*hbridge_setValues(
            0, 
            0, 
            0, 
            0,
            PACKET_ID_SET_VALUE14);

    hbridge_setValues(
            0, 
            0,
            0,
            0,
            PACKET_ID_SET_VALUE58);*/
    int scaling=270;
    //printf("%i, %i, %i, %i, %i, %i\n", (curCmd.strave-127)*scaling, (curCmd.dive-127)*scaling, (curCmd.left-127)*scaling, (curCmd.right-127)*scaling, (curCmd.pitch-127)*scaling, (curCmd.yaw-127)*scaling);
    hbridge_setValues(
            (curCmd.right-127)      * scaling *-1, 
            (curCmd.left-127)       * scaling *-1,
            (curCmd.dive-127)       * scaling *-1,
            (curCmd.pitch-127) + (((curCmd.dive-127)*3)/10) * scaling *-1,
            PACKET_ID_SET_VALUE14);

    hbridge_setValues(
            (curCmd.strave-127)     * scaling,
            (curCmd.yaw-127)        * scaling,
            0,
            0,
            PACKET_ID_SET_VALUE58);
    current_status.change_reason = CR_LEGAL;
    
};
#endif

void init(){
    Assert_Init(GPIOA, GPIO_Pin_12, USE_USART1);

    USART1_Init(USART_POLL);

    //ARC's
    USART2_Init(USART_USE_INTERRUPTS);
    USART3_Init(USART_USE_INTERRUPTS, 57600);
    USART5_Init(USART_USE_INTERRUPTS);

    //Modem
    //USART5_Init(USART_POLL);
    printf_setSendFunction(USART1_SendData); //Debug irgendwas
    //printf_setSendFunction(USART2_SendData); //Debug amber 
    //printf_setSendFunction(USART5_SendData); //Debug ethernet

    uwmodem_init(&USART3_SendData, &USART3_GetData, &USART3_SeekData, &surface);
    printf("The Maiboard is up with the version: 1.3 \n");
    timeout_init(13000);
    //Set ARC System ID to filter the ARC Packets
    protocol_setOwnHostId(SENDER_ID_MAINBOARD);

    CAN_Configuration(CAN_NO_REMAP);
    CAN_ConfigureFilters(0);

    startHardPeriodicThread(1000, time_msPassed);
    lastStateTime = time_getTimeInMs();

    can_protocolInit();

    current_status.current_state = (ARC_SYSTEM_STATE)mbstate_getCurrentState();
    current_status.wanted_state = OFF;
    current_status.current_depth = 0;
    current_status.water_ingress_front = 0;
    current_status.water_ingress_back = 0;
    current_status.change_reason = CR_INITIAL;

    //wait till rest got up
    while(time_getTimeInMs() - lastStateTime < 30);
    //Hbridge Protocol
    protocol_init(1);
    protocol_setRecvFunc(avaloncan_recvPacket);
    protocol_registerHandler(PACKET_ID_STATUS, &hbridgeStatusHandler);
    protocol_registerHandler(PACKET_ID_EXTENDED_STATUS, &hbridgeExtendedStatusHandler);
    //Register a own can Receive function to filter No-Hbridge-Packets out

    hbridge_init(NUM_MOTORS);
    unsigned int i; 
    struct actuatorConfig *ac;
    struct sensorConfig *sc;
    for(i = 0;i<NUM_MOTORS;i++){
        hbridge_setControllerWithData(i, CONTROLLER_MODE_PWM, 0, NULL, 0);
        ac = getActuatorConfig(i);
        ac->openCircuit = 1;
        ac->maxCurrent = 3000;
        ac->maxCurrentCount = 200;
        ac->maxMotorTemp = 200;
        ac->maxMotorTempCount = 50;
        ac->maxBoardTemp = 60;
        ac->maxBoardTempCount = 50;
        ac->pwmStepPerMs = 40;
        ac->timeout = 32000;

        sc = getSensorConfig(i);
        sc->statusEveryMs = 100;
        hbridge_configureSensors();
    }
    //ARC Init
    //First ARC-Channel Amber 
    //arctoken_init(&USART1_SendData, &USART1_GetData, &USART1_SeekData); //Nix
    //arctoken_init(&USART5_SendData, &USART5_GetData, &USART5_SeekData); //Ethernet
    arc_multichannel_init(); //Ethernet
    arc_multichannel_addSerialDriver(&USART5_SendData, &USART5_GetData, &USART5_SeekData);//Ethernet
    arc_multichannel_addTokenSerialDriver(&USART2_SendData, &USART2_GetData, &USART2_SeekData); //Amber
    arc_multichannel_setOwnSystemID(AVALON);
    //Second ARC-Channel Ethernet
    //arctoken_add_serial_handler(&USART2_SendData, &USART2_GetData, &USART2_SeekData);


    mbstate_init();
    packet_init();
    //State handler wenn das MB Controlled 
    packet_registerHandler(MB_CONTROL, avalon_controlHandler);
    packet_registerHandler(MB_ID_CAN, avalon_packet_canHandler);
    packet_registerHandler(MB_SET_STATE, avalon_packet_setStateHandler);


    //struct MainboardState *state_off=mbstate_getState(MAINBOARD_RUNNING);
    //state_off->stateHandler=avalon_offState;

    struct MainboardState *state_running=mbstate_getState(MAINBOARD_RUNNING);
    state_running->stateHandler=avalon_runningState;

    struct MainboardState *state_autonomous=mbstate_getState(MAINBOARD_AUTONOMOUS);
    state_autonomous->stateHandler=avalon_autonomousState;

    struct MainboardState *state_full_autonomous=mbstate_getState(MAINBOARD_FULL_AUTONOMOUS);
    state_full_autonomous->stateHandler=avalon_full_autonomousState;


    struct MainboardState *state_emergency=mbstate_getState(MAINBOARD_EMERGENCY);
    state_emergency->stateHandler=avalon_emergency;
}


int main()
{
    init();
    while(1)
    {
        unsigned int curTime = time_getTimeInMs();
        //only call state processing every ms
        if(curTime != lastStateTime)
        {
            //process state handlers
            mbstate_processState();
            lastStateTime  = curTime;
            if (status_loops >= STATUS_PACKET_PERIOD_MS){
                sendStatusPacket();
                status_loops = 0;
            } else {
                status_loops++;
            }
        }
        arc_packet_t packet;	
        while(arc_multichannel_readPacket(&packet)){
            //Process packets
            packet_handlePacket(packet.originator, packet.system_id, packet.packet_id, packet.data, packet.length);
        }
        arc_multichannel_processPackets();
        hbridge_process();

        uwmodem_process();
        //printf(".");
        //Sending a Status packet
        //usleep(100);

    }
    return 0;
}
