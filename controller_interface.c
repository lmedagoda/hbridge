
#include "md03.h"
#include "stm32f10x_lib.h"
#include "controller_interface.h"
#include "spi.h"
#include "pid.h"
#include "rc.h"
#include "main.h"

#define NR_MOTORS 4

struct motor_data {
  volatile struct md03_data *md03_data;
};

struct motor_data motor_data[NR_MOTORS];
struct pid_data pid_data[NR_MOTORS];

void controller_interface_init() {
  int i;
  
  for(i = 0; i < NR_MOTORS; i++) {
    motor_data[i].md03_data = &(md03_data[i]);  
  }
  
}

u32 getTemperature(const enum motor_position num_motor) {
  return motor_data[num_motor].md03_data->temperature;
}

u32 getCurrent(const enum motor_position num_motor) {
  return motor_data[num_motor].md03_data->motor_current;
}

u32 getCurTicks(const enum motor_position num_motor) {
  return LS7366Data[num_motor].curTicks;
}

u32 getLastTicks(const enum motor_position num_motor) {
  return LS7366Data[num_motor].lastTicks;
}

s32 getCurSpeed(const enum motor_position num_motor) {
  return LS7366Data[num_motor].curTicks - LS7366Data[num_motor].lastTicks;
}

void setNewSpeedPID(const int speed, const enum motor_position num_motor) {  
  setTargetValue(pid_data+num_motor, speed);
  s32 pidvalue = pid(pid_data+num_motor, getCurSpeed(num_motor));
  setNewSpeed(pidvalue, num_motor);
}

void setNewSpeed(const int speed, const enum motor_position num_motor) {

  if(speed > 0) {
    motor_data[num_motor].md03_data->direction = MD03_FORWARD;
    motor_data[num_motor].md03_data->speed = speed;
  } else {
    motor_data[num_motor].md03_data->direction = MD03_REVERSE;
    motor_data[num_motor].md03_data->speed = -speed;
  }
  
};

void setNewAcceleration(const int acc,const enum motor_position num_motor) {
  assert_param(num_motor <= NR_MOTORS);
  motor_data[num_motor].md03_data->acceleration = acc;
};

s32 getValueForChannel(const enum rc_channels channel) {
  return rc_values[channel];
}

void installController(controller_func controller1) {
  controller = controller1;
}

