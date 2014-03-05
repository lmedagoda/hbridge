#include "../common/mainboardstate.h"
#include "../common/packethandling.h"
#include "../common/arc_driver.h"
#include "../common/time.h"
#include "../common/timeout.h"
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

int16_t voltage_reading = 0;
int16_t encoder_reading = 0;

void sendStatusPacket() {
    arc_packet_t packet;
    packet.originator = SLAVE;
    packet.system_id = 1; // ASGUARD
    packet.packet_id = MB_STATUS;
    packet.length = 7;
    packet.data[0] = voltage_reading >> 8;
    packet.data[1] = voltage_reading & 0xff;
    packet.data[2] = mbstate_getCurrentState();
    packet.data[3] = 0; //stat_packets_persec;
    packet.data[4] = 0; //stat_control_packets_persec;
    packet.data[5] = encoder_reading >> 8;
    packet.data[6] = encoder_reading & 0xff;
    arc_sendPacketDirect(&packet);
}

struct AsguardControlData {
  int8_t speed; 
  int8_t direction;
  int8_t turn;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct AsguardControlData curCmd;
uint8_t cmdValid;

/**
 * Handler for COMMAND packages
 * */
void asguard_cmdHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size)
{
    struct AsguardControlData *cmd = (struct AsguardControlData *) data;
    curCmd = *cmd;
    cmdValid = 1;
}

#define STATUS_PACKET_PERIOD 500 
#define SPEED_FACTOR (314)
#define DIRECTION_RADIUS (6)
#define TURN_SPEED (40)
#define MAX_PWM 32767

uint8_t asguard_enterRunningState(void)
{
    cmdValid = 0;
    return mbstate_toRunningHandler();
}

/**
 * Handler that gets called if in running state
 * */
void asguard_runningState(void)
{
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
    }

    //do nothing until we got a command packet
    if(!cmdValid)
	return;

    
    //set actuator cmd
    int left = curCmd.speed * SPEED_FACTOR + curCmd.turn *TURN_SPEED;

    int right = curCmd.speed * SPEED_FACTOR - curCmd.turn *TURN_SPEED;

    if(left > MAX_PWM)
	left = MAX_PWM;

    if(right > MAX_PWM)
	right = MAX_PWM;

    hbridge_setValue(-left, right, right, -left);
}

int main()
{
    //setup assert correctly
    Assert_Init(GPIOG, GPIO_Pin_8, USE_USART3);

    USART3_Init(USART_POLL);

    printf_setSendFunction(USART3_SendData);
    
    USART2_Init(USART_USE_INTERRUPTS);
    
    printf("START: Up and running\n");
    
    timeout_init(3000);
    
    protocol_init(1);
    protocol_setOwnHostId(SENDER_ID_MAINBOARD);
    
    CAN_Configuration(CAN_NO_REMAP);
    CAN_ConfigureFilters(0);
    
    startHardPeriodicThread(1000, time_msPassed);
    
    can_protocolInit();

    uint32_t lastStateTime = time_getTimeInMs();
    
    //wait till rest got up
    while(time_getTimeInMs() - lastStateTime < 30)
	;
    
    hbridge_init(4);
    
    hbridge_setControllerWithData(0, CONTROLLER_MODE_PWM, 0, NULL, 0);
    hbridge_setControllerWithData(1, CONTROLLER_MODE_PWM, 0, NULL, 0);
    hbridge_setControllerWithData(2, CONTROLLER_MODE_PWM, 0, NULL, 0);
    hbridge_setControllerWithData(3, CONTROLLER_MODE_PWM, 0, NULL, 0);
    int i = 0;
    for(i = 0; i < 4; i++)
    {
	struct actuatorConfig *conf = getActuatorConfig(i);
	conf->maxBoardTemp = 100;
	conf->maxBoardTempCount = 100;
	conf->maxCurrent = 4000;
	conf->maxCurrentCount = 200;
	conf->pwmStepPerMs = 1500;
	conf->timeout = 150;
    }
    
    
    arc_init(&USART2_SendData, &USART2_GetData, &USART2_SeekData);
    mbstate_init();
    packet_init();

    packet_registerHandler(MB_CONTROL, asguard_cmdHandler);
 
    ///Overload the state handler for running
    struct MainboardState *state=mbstate_getState(MAINBOARD_RUNNING);
    state->enterStateHandler = asguard_enterRunningState;
    state->stateHandler=asguard_runningState;
    
    while(1)
    {
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
	while(arc_readPacket(&packet))
	{
	    //process incomming packet
	    packet_handlePacket(packet.originator, packet.system_id, packet.packet_id, packet.data, packet.length);	

	    // respond with a status packet
	    sendStatusPacket();

	    // reset timeout 
	    // TODO this should happen in the ping handler, but doesn't seem to be
	    timeout_reset();
	}
	
	//process hbridge driver
	hbridge_process();
    }
    
    return 0;
}
