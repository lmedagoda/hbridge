
#ifndef __CONTROLER_INTERFACE_H
#define __CONTROLER_INTERFACE_H

#include "stm32f10x_type.h"

enum motor_position {
  FRONT_RIGHT = 1,
  FRONT_LEFT = 2,
  REAR_RIGHT = 0,
  REAR_LEFT = 3,
};

enum rc_channels {
  CHANNEL_1 = 0,
  CHANNEL_2 = 1,
  CHANNEL_3 = 2,
  CHANNEL_4 = 3,
  CHANNEL_5 = 4,
  CHANNEL_6 = 5,
  CHANNEL_7 = 6,
  CHANNEL_8 = 7,
  CHANNEL_9 = 8,
};

typedef void (*controller_func) (void);

//must be called befor fist usage of this interface
//initalizes internal data structures
void controller_interface_init();

//helper function, to avoid memory corruptions through
//stupid mistakes
void setNewSpeed(const int speed, const enum motor_position num_motor);
void setNewSpeedPID(const int speed, const enum motor_position num_motor);
void setNewAcceleration(const int acc, const enum motor_position num_motor);
u32 getTemperature(const enum motor_position num_motor);
u32 getCurrent(const enum motor_position num_motor);
u32 getCurTicks(const enum motor_position num_motor);
u32 getLastTicks(const enum motor_position num_motor);
s32 getCurSpeed(const enum motor_position num_motor);
s32 getValueForChannel(const enum rc_channels channel);

void installController(controller_func controller);

#undef printf

int printf(const char *format, ...);

#endif
