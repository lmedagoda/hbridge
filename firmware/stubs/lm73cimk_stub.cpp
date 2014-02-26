extern "C" {
#include "../interfaces/temperature_sensor.h"
}



void stub_tempInit()
{
    
}

uint8_t stub_getTemperature(int32_t* val)
{
    *val = 0;
    return 0;
}


void tempSensorInit()
{
    struct TemperatureInterface interface;
    interface.getTemperature = stub_getTemperature;
    interface.sensorInit = stub_tempInit;
    interface.sensorDeInit = stub_tempInit;
    
    temperatureSensors_setImpl(POSITION_MOTOR, interface);
    temperatureSensors_setImpl(POSITION_PCB, interface);
}

