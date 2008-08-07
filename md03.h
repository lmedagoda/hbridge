#ifndef __MD03_H
#define __MD03_H

#include "stm32f10x_type.h"


enum md03_direction {
  MD03_EMERGENCY_STOP = 0,
  MD03_FORWARD = 1,
  MD03_REVERSE = 2,
};

enum md03_global_state {
  MD03_READING_ALL_DATA,
  MD03_ALL_DATA_RECEIVED,
  MD03_WRITING_ALL_DATA,
  MD03_WROTE_ALL_DATA,
};


enum md03_state {
  MD03_IDLE = 0,
  MD03_READ_TEMP = 1,
  MD03_READ_CURRENT = 2,
  MD03_DATA_RECEIVED = 3,
  MD03_WRITE_ACC = 4,
  MD03_WRITE_SPEED = 5,
  MD03_WRITE_CMD = 6,
  MD03_WAIT_FOR_CMD_WRITE = 7,
  MD03_WRITE_FINISHED = 8,
};



enum md03_registers {
  MD03_COMMAND = 0,
  MD03_STATUS = 1,
  MD03_SPEED = 2,
  MD03_ACCELERATION = 3,
  MD03_TEMPERATURE = 4,
  MD03_MOTOR_CURRENT = 5,
  MD03_UNUSED = 6,
  MD03_REVISION = 7,
};

enum md03_names {
  MD03_01 = 0,
  MD03_02 = 1,
  MD03_03 = 2,
  MD03_04 = 3,
};

struct md03_data {
  u16 address;
  enum md03_direction direction;
  u8 speed;
  u8 acceleration;
  u32 temperature;
  u32 motor_current;
};

extern volatile struct md03_data md03_data[4];
extern volatile enum md03_global_state md03_global_state;
extern volatile enum md03_state single_md03_state;
extern volatile enum md03_names cur_md03_device;

void requestMD03Data();
void requestMD03DataI2C1();
void requestMD03DataI2C2();
int MD03DataReady();
void requestMD03DataWrite();
void requestMD03DataWriteI2C1();
void requestMD03DataWriteI2C2();
int MD03DataWritten();
void setupI2C();

#endif
