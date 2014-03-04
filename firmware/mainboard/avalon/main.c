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

//------- Importtand Defines ----------//
#define SYSTEM_ID AVALON
#define NUM_MOTORS 6
#define SURFACE_SIGN 0xFF
#define SURFACE_SIGN_COUNT 6 

//USART? = AMBER
//USART3 = Underwater Modem
//USART? = Ethernet/Serial
//TODO Add CAN IDs to keep a easy overview
//TODO Where co configure motor commands and set them to the motors
avalon_status_t current_status;
void hbridgeStatusHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size){
}
void hbridgeExtendedStatusHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size){
}

void sendStatusPacket(){
    CanTxMsg msg;
    msg.StdId = 0x541;
    msg.RTR=CAN_RTR_DATA;
    msg.IDE=CAN_ID_STD;
    msg.DLC = 8;
    msg.Data[2] =(ARC_SYSTEM_STATE)mbstate_getCurrentState();
    CAN_SendMessage(&msg);

    avalon_status_t status_information;
    status_information.current_state = (ARC_SYSTEM_STATE)mbstate_getCurrentState();
    status_information.wanted_state = current_status.wanted_state;
    status_information.current_depth = current_status.current_depth;
    status_information.water_ingress_front = current_status.water_ingress_front;
    status_information.water_ingress_back =  current_status.water_ingress_back;
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
    arc_sendPacket(&packet);
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
    //asvcan_handlePacket(&msg);
    //Real sending of the Can Package
    CAN_SendMessage(&msg);

}
int status_loops = 0;
uint8_t surface_sign_counter= 0;
struct arc_avalon_control_packet curCmd;
uint8_t cmdValid;
void avalon_controlHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size){
    //printf("control packet\n");
    struct arc_avalon_control_packet *cmd  = (struct arc_avalon_control_packet *) data;

    curCmd = *cmd;
    cmdValid = 1;
}

void avalon_runningState(void){
    //printf("runs avalon\n");

    /*if(timeout_hasTimeout())
      {
      printf("Timout, switching to off\n");
      mbstate_changeState(MAINBOARD_OFF);
        curent_status.change_reason = CR_MB_TIMEOUT;
      }

    //check actuators
    */
    /*if(hbridge_hasActuatorError())
    {
        printf("Actuator error, switching to off\n");
        mbstate_changeState(MAINBOARD_OFF);
        current_status.change_reason = CR_HB_ERROR;
        return;
    }

    if (!cmdValid)
        return;
        */

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
    int scaling=320;
    //printf("%i, %i, %i, %i, %i, %i\n", (curCmd.strave-127)*scaling, (curCmd.dive-127)*scaling, (curCmd.left-127)*scaling, (curCmd.right-127)*scaling, (curCmd.pitch-127)*scaling, (curCmd.yaw-127)*scaling);
    hbridge_setValues(
            (curCmd.right-127)      * scaling, 
            (curCmd.left-127)       * scaling,
            (curCmd.dive-127)       * scaling,
            (curCmd.yaw-127)      * scaling,
            PACKET_ID_SET_VALUE14);

    hbridge_setValues(
            (curCmd.strave-127)     * scaling,
            (curCmd.pitch-127)        * scaling,
            0,
            0,
            PACKET_ID_SET_VALUE58);
};


//TODO Add this function to be executed and adapt it to compile ;)
//Don't forget to initialize the CANid and the USART
uint8_t handle_underwater_modem() {
    return;
    int seek, i;
    u8 packet_buffer[ARC_MAX_FRAME_LENGTH];

    // look for a valid packet as long as enough bytes are left
    while( (seek = USART5_SeekData(packet_buffer, ARC_MAX_FRAME_LENGTH)) >= 1 ) {

        //seek =  UART5_SeekData(packet_buffer, ARC_MAX_FRAME_LENGTH);
        int size = seek;
        while( size > 0 ) {
            int fragment_size = 8;

            if(size < 8)
                fragment_size = size;

            printf("Readed %hi bytes\r\n",seek);
            CanTxMsg inputMessage;
            inputMessage.StdId=0x1E0; //TODO Hardcoded Modem value
            inputMessage.RTR=CAN_RTR_DATA;
            inputMessage.IDE=CAN_ID_STD;
            inputMessage.DLC= fragment_size;

            size -= fragment_size;

            for(i=0;i<fragment_size;i++) {
                inputMessage.Data[i] = packet_buffer[i];
                if(packet_buffer[i] == SURFACE_SIGN){ //TODO Extend packed
                    if(++surface_sign_counter == SURFACE_SIGN_COUNT){
                        //print("diving up\n");
                        //TODO IMPORTAND CHANGE STATE TO SURFACE
                        //wanted_system_state = SURFACE;
                        surface_sign_counter=0;
                    }
                }else{
                    surface_sign_counter=0;
                }
            }
            int counter=0;


            //TODO What's this:?
            /*
               while(CAN_Transmit(&inputMessage) == CAN_NO_MB) {
               counter++;
               if (counter > CAN_SEND_RETRIES) {
               UART5_GetData(packet_buffer, size);
               printf("Giving up %hi\r\n",counter);
               return 1; //flase
               }
               }
               */
        }
        USART5_GetData(packet_buffer, seek);
    }
    return 0; //true
}

void init(){
    Assert_Init(GPIOA, GPIO_Pin_12, USE_USART1);

    USART1_Init(USART_POLL);
    //USART3_Init(USART_POLL);
    //USART1_Init(USART_USE_INTERRUPTS);

    //ARC's
    USART2_Init(USART_USE_INTERRUPTS);
    USART3_Init(USART_USE_INTERRUPTS);

    //Modem
    //USART5_Init(USART_POLL);
    //printf_setSendFunction(USART1_SendData); //Debug irgendwas
    printf_setSendFunction(USART2_SendData); //Debug amber 
    //printf_setSendFunction(USART5_SendData); //Debug ethernet

    //uwmodem_init(&USART3_SendData, &USART3_GetData, &USART3_SeekData);
    //while (1){
    //    printf(".");
    //}

    printf("The Maiboard is up with the version: 1.2 ");
    USART5_Init(USART_USE_INTERRUPTS);
    printf("OK ");

    timeout_init(300000);
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
    arctoken_init(&USART5_SendData, &USART5_GetData, &USART5_SeekData); //Ethernet
    //arctoken_init(&USART2_SendData, &USART2_GetData, &USART2_SeekData); //Amber
    arctoken_setOwnSystemID(AVALON);
    //Second ARC-Channel Ethernet
    //arctoken_add_serial_handler(&USART2_SendData, &USART2_GetData, &USART2_SeekData);


    mbstate_init();
    packet_init();
    //State handler wenn das MB Controlled 
    packet_registerHandler(MB_CONTROL, avalon_controlHandler);
    packet_registerHandler(MB_ID_CAN, avalon_packet_canHandler);

    //Overload the state handler for running
    struct MainboardState *state=mbstate_getState(MAINBOARD_RUNNING);
    state->stateHandler=avalon_runningState;

}


int main()
{
    init();
    //mbstate_changeState(MAINBOARD_RUNNING);
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

            static int blasel=0;
            if(blasel++==1000){
                printf(".");
                blasel=0;
            }

        }
        arc_packet_t packet;	

        while(arctoken_readPacket(&packet)){
            //Process packets
            packet_handlePacket(packet.originator, packet.system_id, packet.packet_id, packet.data, packet.length);
        }
        arctoken_processPackets();	  
        hbridge_process();

        //uwmodem_process();
        //Sending a Status packet
    }
    return 0;
}


