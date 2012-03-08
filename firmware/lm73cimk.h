#ifndef __LM73CIMK_H
#define __LM73CIMK_H

#include "inc/stm32f10x_type.h"
#include "inc/stm32f10x_i2c.h"

enum LM73_SENSORS {
  LM73_SENSOR1 = 0,
  LM73_SENSOR2 = 1,
};

void lm73cimk_init();

void lm73cimk_setup_sensor(enum LM73_SENSORS sensor, I2C_TypeDef* I2C_Bus, u8 i2c_addr);

u8 lm73cimk_getTemperature(enum LM73_SENSORS sensor, u32* val);

#endif
