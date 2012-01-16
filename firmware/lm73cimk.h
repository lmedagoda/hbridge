#ifndef __LM73CIMK_H
#define __LM73CIMK_H

#include "inc/stm32f10x_type.h"
#include "inc/stm32f10x_i2c.h"

#define LM73_SENSOR_1 156 // 1001110 + r/w bit

enum lm73cimkStates {
  LM73_IDLE,
  LM73_TRIGGERED,
  LM73_TEMP_REQUESTED,
  LM73_ERROR,
  LM73_ACQUIRED_TEMP,
};

extern enum lm73cimkStates lm73cimkState;

void lm73cimk_init(I2C_TypeDef* I2C_Bus_l, FunctionalState remapped);

u8 lm73cimk_getTemperature(u8 addr, u32 *val);

//for internal use only
u8 lm73cimk_triggerTemeratureConversion(u8 addr);

u8 lm73cimk_requestTemperature(u8 addr);


#endif
