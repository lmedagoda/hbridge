extern "C" {
#include "drivers/lm73cimk.h"
}

void lm73cimk_init(I2C_TypeDef* I2C_Bus)
{
    
}

void lm73cimk_setup_sensor(enum LM73_SENSORS sensor, uint8_t i2c_addr)
{
}

uint8_t lm73cimk_getTemperature(enum LM73_SENSORS sensor, int32_t* val)
{
    *val = 0;
    return 0;
}