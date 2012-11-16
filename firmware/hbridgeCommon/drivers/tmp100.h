#ifndef __TMP100_H__
#define __TMP100_H__

#include <inc/stm32f10x_i2c.h>

enum TMP100_SENSORS {
    TMP100_SENSOR1=0,
};

void tmp100_init(I2C_TypeDef* I2C_Bus);

void tmp100_setup_sensor(uint8_t i2c_addr);

uint8_t tmp100_getTemperature(enum TMP100_SENSORS sensor, int32_t* val);

#endif