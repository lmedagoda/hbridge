#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "stm32f10x_type.h"

enum controllerModes {
  CONTROLLER_MODE_HALT = 0,
  CONTROLLER_MODE_PWM = 1,
  CONTROLLER_MODE_SPEED = 2,
  CONTROLLER_MODE_POSITION = 3,
};

enum errorCodes {
  ERROR_CODE_OVERHEAT = 0,
};

enum packetIDs {
  PACKET_ID_EMERGENCY_STOP = 0,
  PACKET_ID_STATUS = 1,
  PACKET_ID_SET_VALUE = 2,
  PACKET_ID_SET_MODE = 3,
  PACKET_ID_SET_CONFIGURE = 4,
  PACKET_ID_SET_CONFIGURE2 = 5,
};

enum hostIDs {
  RECEIVER_ID_H_BRIDGE_1 = (1<<5),
  RECEIVER_ID_H_BRIDGE_2 = (2<<5),
  RECEIVER_ID_H_BRIDGE_3 = (3<<5),
  RECEIVER_ID_H_BRIDGE_4 = (4<<5),
};

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
  u8 driveMode;
  u8 unused;
  u16 kp;
  u16 ki;
  u16 kd;
} __attribute__ ((packed));

struct configure1Data {
  unsigned openCircuit :1;
  unsigned activeFieldCollapse :1;
  unsigned externalTempSensor :1;
  unsigned unused : 13;
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
  unsigned unused :24;
} __attribute__ ((packed));



#endif
