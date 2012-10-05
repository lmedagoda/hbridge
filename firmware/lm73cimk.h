#ifndef __LM73CIMK_H
#define __LM73CIMK_H

#include "i2c.h"
#include <stdint.h>

enum LM73_SENSORS {
  LM73_SENSOR1 = 0,
  LM73_SENSOR2 = 1,
  LM73_NONE = 2,
};

void lm73cimk_init(I2C_TypeDef* I2C_Bus);

void lm73cimk_setup_sensor(enum LM73_SENSORS sensor, uint8_t i2c_addr);

uint8_t lm73cimk_getTemperature(enum LM73_SENSORS sensor, int32_t* val);

#endif
