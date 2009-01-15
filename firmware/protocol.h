#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#ifndef __NO_STM32
#include "stm32f10x_type.h"
#endif

enum controllerModes {
  CONTROLLER_MODE_PWM = 0,
  CONTROLLER_MODE_SPEED = 1,
  CONTROLLER_MODE_POSITION = 2,
};

enum errorCodes {
  ERROR_CODE_OVERHEATMOTOR = 0,
  ERROR_CODE_OVERHEATBOARD = 1,
  ERROR_CODE_OVERCURRENT = 2,
  ERROR_CODE_TIMEOUT = 3,
  ERROR_CODE_BAD_CONFIG = 4,
};

enum packetIDs {
  PACKET_ID_EMERGENCY_STOP = 0,
  PACKET_ID_STATUS = 1,
  PACKET_ID_SET_VALUE = 2,
  PACKET_ID_SET_MODE = 3,
  PACKET_ID_SET_PID_POS = 4,
  PACKET_ID_SET_PID_SPEED = 5,
  PACKET_ID_SET_CONFIGURE = 6,
  PACKET_ID_SET_CONFIGURE2 = 7,
  PACKET_ID_PID_DEBUG_POS = 8,
  PACKET_ID_POS_DEBUG = 9,
  PACKET_ID_PID_DEBUG_SPEED = 10,
  PACKET_ID_SPEED_DEBUG = 11,
};

enum hostIDs {
  RECEIVER_ID_H_BRIDGE_1 = (1<<5),
  RECEIVER_ID_H_BRIDGE_2 = (2<<5),
  RECEIVER_ID_H_BRIDGE_3 = (3<<5),
  RECEIVER_ID_H_BRIDGE_4 = (4<<5),
};

struct speedDebugData {
  s16 targetVal;
  s16 pwmVal;
  u16 encoderVal;
  s16 speedVal;
} __attribute__ ((packed));

struct posDebugData {
  u16 targetVal;
  s16 pwmVal;
  u16 encoderVal;
  u16 posVal;
} __attribute__ ((packed));

struct pidDebugData {
  s16 pPart;
  s16 iPart;
  s16 dPart;
  u16 minMaxPidOutput;
} __attribute__ ((packed));


struct statusData {
  unsigned currentValue :14;
  unsigned index :10;
  u16 position;
  u8 tempHBrigde;
  u8 tempMotor;
  enum errorCodes error:8;
} __attribute__ ((packed));

struct setValueData {
  s16 board1Value;
  s16 board2Value;
  s16 board3Value;
  s16 board4Value;
} __attribute__ ((packed));

struct setModeData {
  enum controllerModes board1Mode:8;
  enum controllerModes board2Mode:8;
  enum controllerModes board3Mode:8;
  enum controllerModes board4Mode:8;
} __attribute__ ((packed));

struct setPidData {
  s16 kp;
  s16 ki;
  s16 kd;
  u16 minMaxPidOutput;
} __attribute__ ((packed));

struct configure1Data {
  unsigned openCircuit :1;
  unsigned activeFieldCollapse :1;
  unsigned externalTempSensor :1;
  unsigned cascadedPositionController :1;
  unsigned enablePIDDebug :1;
  unsigned unused : 11;
  u8 maxMotorTemp;
  u8 maxMotorTempCount;
  u8 maxBoardTemp;
  u8 maxBoardTempCount;
  u16 timeout;
} __attribute__ ((packed));

struct configure2Data {
  u16 maxCurrent;
  u8 maxCurrentCount;
  u16 pwmStepPerMs;
  //  unsigned unused :24;
} __attribute__ ((packed));



#endif
