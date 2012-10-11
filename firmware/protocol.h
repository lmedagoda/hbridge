#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>

enum hostIDs {
  RECEIVER_ID_H_BRIDGE_1 = (1<<5),
  RECEIVER_ID_H_BRIDGE_2 = (2<<5),
  RECEIVER_ID_H_BRIDGE_3 = (3<<5),
  RECEIVER_ID_H_BRIDGE_4 = (4<<5),
  RECEIVER_ID_H_BRIDGE_5 = (5<<5),
  RECEIVER_ID_H_BRIDGE_6 = (6<<5),
  RECEIVER_ID_H_BRIDGE_7 = (7<<5),
  RECEIVER_ID_H_BRIDGE_8 = (8<<5),
};

enum controllerModes {
  CONTROLLER_MODE_NONE = 0,
  CONTROLLER_MODE_PWM = 1,
  CONTROLLER_MODE_SPEED = 2,
  CONTROLLER_MODE_POSITION = 3,
};

enum packetIDs {
  PACKET_ID_EMERGENCY_STOP = 0,
  PACKET_ID_ACK,
  PACKET_ID_ERROR,
  PACKET_ID_STATUS,
  PACKET_ID_EXTENDED_STATUS,   
  PACKET_ID_ENCODER_CONFIG_INTERN,
  PACKET_ID_ENCODER_CONFIG_EXTERN,
  PACKET_ID_SET_CONFIGURE,
  PACKET_ID_SET_CONFIGURE2,
  PACKET_ID_SET_MODE14,
  //10
  PACKET_ID_SET_MODE58,
  PACKET_ID_SET_VALUE14,
  PACKET_ID_SET_VALUE58,

  END_BASE_PACKETS,
  
  PACKET_ID_SET_PID_SPEED,
  PACKET_ID_SPEED_DEBUG,
  PACKET_ID_PID_DEBUG_SPEED,
  
  PACKET_ID_SET_PID_POS,
  PACKET_ID_PID_DEBUG_POS,
  PACKET_ID_POS_DEBUG,
  //20 (?)
  PACKET_ID_POS_CONTROLLER_DATA,
  NUM_PACKET_IDS,
};

#define NUM_ENCODERS 6
enum encoderTypes {
    NO_ENCODER = 0,
    QUADRATURE = 1,
    QUADRATURE_WITH_ZERO = 2,
    IC_HOUSE_MH_Y = 3,
    BMMV30_SSI = 4,
    ANALOG_VOLTAGE = 5,
};

enum controllerInputEncoder {
    INTERNAL = 0,
    EXTERNAL = 1,
};

struct ackData {
    unsigned short packetId;
};

struct speedDebugData {
  int16_t targetVal;
  int16_t pwmVal;
  unsigned short encoderVal;
  int16_t speedVal;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct posDebugData {
  uint16_t targetVal;
  int16_t pwmVal;
  uint16_t encoderVal;
  uint16_t posVal;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct pidDebugData {
  int16_t pPart;
  int16_t iPart;
  int16_t dPart;
  uint16_t minMaxPidOutput;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct posControllerData {
    uint16_t minHystDist;
    uint16_t maxHystDist;
    unsigned hysteresisActive:1;
    unsigned allowWrapAround:1;
    unsigned debugActive:1;
    unsigned unused:5;
    uint8_t overDistCount;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct errorData {
    uint8_t temperature;
    uint16_t position;
    unsigned index :10;
    unsigned externalPosition:12;
    unsigned motorOverheated:1;
    unsigned boardOverheated:1;
    unsigned overCurrent:1;
    unsigned timeout:1;
    unsigned badConfig:1;
    unsigned encodersNotInitalized:1;
    unsigned hardwareShutdown:1;
    unsigned unused:3;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct statusData {
  signed pwm :12;
  unsigned externalPosition:12;
  uint16_t position;
  unsigned currentValue :14;
  unsigned index :10;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct extendedStatusData {
    uint8_t temperature;
    uint8_t motorTemperature;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct setValueData {
  int16_t board1Value;
  int16_t board2Value;
  int16_t board3Value;
  int16_t board4Value;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct setModeData {
  enum controllerModes board1Mode:8;
  enum controllerModes board2Mode:8;
  enum controllerModes board3Mode:8;
  enum controllerModes board4Mode:8;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct setPidData {
  int16_t kp;
  int16_t ki;
  int16_t kd;
  uint16_t minMaxPidOutput;
} __attribute__ ((packed)) __attribute__((__may_alias__)) ;

struct encoderConfiguration {
    enum encoderTypes encoderType:8;
    uint32_t ticksPerTurn;
    uint8_t tickDivider;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct configure1Data {
  unsigned openCircuit :1;
  unsigned externalTempSensor :1;
  unsigned externalEncoder : 1;
  enum controllerInputEncoder controllerInputEncoder :1;
  unsigned unused :12;
  uint8_t maxMotorTemp;
  uint8_t maxMotorTempCount;
  uint8_t maxBoardTemp;
  uint8_t maxBoardTempCount;
  uint16_t timeout;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct configure2Data {
  uint16_t maxCurrent;
  uint8_t maxCurrentCount;
  uint16_t pwmStepPerMs;
  uint16_t statusEveryMs;
  //  unsigned unused :24;
} __attribute__ ((packed)) __attribute__((__may_alias__));

#ifndef __NO_STM32
#include "inc/stm32f10x_can.h"
struct ControllerState;
uint8_t updateStateFromMsg(CanRxMsg *curMsg, volatile struct ControllerState *state, enum hostIDs ownHostId);
enum hostIDs getOwnHostId();
void protocol_init(enum hostIDs ownHostId);
void protocol_registerHandler(int id, void (*handler)(int id, unsigned char *data, unsigned short size));
void protocol_ackPacket(int id);
#endif


#endif
