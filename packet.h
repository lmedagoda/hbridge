#ifndef __PACKET_H
#define __PACKET_H

#include "stm32f10x_type.h"

struct packetHeader {
  u16 id;   //11 bit id
  u8 length; //4 bit lenght
  u8 receiverID; //4 bit receiverID
};

enum packetIDs {
  PACKET_ID_EMERGENCY_STOP = 0,
  PACKET_ID_ERROR = 1,
  PACKET_ID_STATUS = 2,
  PACKET_ID_SET_SPEED,
  PACKET_ID_SET_POSITION,
  PACKET_ID_SET_PWM,
  PACKET_ID_SET_PID,
  PACKET_ID_SET_MODE,
};

enum receiverIDs {
  RECEIVER_ID_H_BRIDGE_1 = 0,
  RECEIVER_ID_H_BRIDGE_2,
  RECEIVER_ID_H_BRIDGE_3,
  RECEIVER_ID_H_BRIDGE_4,
  RECEIVER_ID_H_BRIDGE_ALL,
};

struct statusData {
  u16 encoderValue;
  u16 currentValue;
};

struct statusPacket {
  struct packetHeader header;
  struct statusData data;
};

enum errorCodes {
  ERROR_CODE_OVERHEAT = 0,
};

struct errorData {
  enum errorCodes error;
};

struct setPWMData {
  u16 newValue;
};

struct setPositionData {
  u16 newPosition;
};

struct setSpeedData {
  u16 newSpeed;
};

struct emergencyStopData {
  u8 active;
};

enum controllerModes {
  CONTROLLER_MODE_HALT = 0,
  CONTROLLER_MODE_PWM = 1,
  CONTROLLER_MODE_SPEED = 2,
  CONTROLLER_MODE_POSITION = 3,
};

struct setControllerModeData {
  enum controllerModes mode;
};

#endif
