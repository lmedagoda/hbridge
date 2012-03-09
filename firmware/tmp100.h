#ifndef __TMP100_H__
#define __TMP100_H__

#include <inc/stm32f10x_type.h>
#include <inc/stm32f10x_i2c.h>

enum TMP100_SENSORS {
    TMP100_SENSOR1=0,
};

void tmp100_init(I2C_TypeDef* I2C_Bus);

void tmp100_setup_sensor(u8 i2c_addr);

u8 tmp100_getTemperature(enum TMP100_SENSORS sensor, s32* val);

#endif