#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#ifndef __NO_STM32
#include "inc/stm32f10x_type.h"
#include "inc/stm32f10x_can.h"
#endif

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
  CONTROLLER_MODE_PWM = 0,
  CONTROLLER_MODE_SPEED = 1,
  CONTROLLER_MODE_POSITION = 2,
};

enum packetIDs {
  PACKET_ID_EMERGENCY_STOP = 0,
  PACKET_ID_ERROR = 1,
  PACKET_ID_STATUS = 2,
  PACKET_ID_SET_VALUE14 = 3,
  PACKET_ID_SET_VALUE58 = 4,
  PACKET_ID_SET_MODE14 = 5,
  PACKET_ID_SET_MODE58 = 6,
  PACKET_ID_SET_PID_POS = 7,
  PACKET_ID_SET_PID_SPEED = 8,
  PACKET_ID_SET_CONFIGURE = 9,
  PACKET_ID_SET_CONFIGURE2 = 10,
  PACKET_ID_ENCODER_CONFIG = 11,
  PACKET_ID_PID_DEBUG_POS = 12,
  PACKET_ID_POS_DEBUG = 13,
  PACKET_ID_PID_DEBUG_SPEED = 14,
  PACKET_ID_SPEED_DEBUG = 15,
  PACKET_ID_PIEZO = 16,
};

struct speedDebugData {
  s16 targetVal;
  s16 pwmVal;
  u16 encoderVal;
  s16 speedVal;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct posDebugData {
  u16 targetVal;
  s16 pwmVal;
  u16 encoderVal;
  u16 posVal;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct pidDebugData {
  s16 pPart;
  s16 iPart;
  s16 dPart;
  u16 minMaxPidOutput;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct piezoData {
  u16 value1;
  u16 value2;
  u16 value3;
  u16 value4;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct errorData {
    u8 temperature;
    u16 position;
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
  u16 position;
  unsigned currentValue :14;
  unsigned index :10;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct setValueData {
  s16 board1Value;
  s16 board2Value;
  s16 board3Value;
  s16 board4Value;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct setModeData {
  enum controllerModes board1Mode:8;
  enum controllerModes board2Mode:8;
  enum controllerModes board3Mode:8;
  enum controllerModes board4Mode:8;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct setPidData {
  s16 kp;
  s16 ki;
  s16 kd;
  u16 minMaxPidOutput;
} __attribute__ ((packed)) __attribute__((__may_alias__)) ;

struct encoderConfiguration {
    unsigned ticksPerTurnIntern:24;
    u8 tickDividerIntern;
    unsigned ticksPerTurnExtern:24;
    u8 tickDividerExtern;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct configure1Data {
  unsigned openCircuit :1;
  unsigned activeFieldCollapse :1;
  unsigned externalTempSensor :1;
  unsigned cascadedPositionController :1;
  unsigned enablePIDDebug :1;
  unsigned externalEncoder : 1;
  unsigned unused : 10;
  u8 maxMotorTemp;
  u8 maxMotorTempCount;
  u8 maxBoardTemp;
  u8 maxBoardTempCount;
  u16 timeout;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct configure2Data {
  u16 maxCurrent;
  u8 maxCurrentCount;
  u16 pwmStepPerMs;
  //  unsigned unused :24;
} __attribute__ ((packed)) __attribute__((__may_alias__));

#ifndef __NO_STM32
struct ControllerState;
u8 updateStateFromMsg(CanRxMsg *curMsg, volatile struct ControllerState *state, enum hostIDs ownHostId);
enum hostIDs getOwnHostId();
#endif


#endif
