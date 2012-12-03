#include "../interfaces/temperature_sensor.h"

struct TemperatureInterface tempSensors[2];


uint8_t defaultGetTemperature(int32_t *val)
{
    val = 0;
    return 0;
}

void defaultTempInit()
{
}

void temperatureSensors_init()
{
    int i;
    for(i = 0; i < 2; i++)
    {
	tempSensors[i].getTemperature = defaultGetTemperature;
	tempSensors[i].sensorInit = defaultTempInit;
	tempSensors[i].sensorDeInit = defaultTempInit;
    }
}

void temperatureSensors_setImpl(enum TempSensorPositions pos, struct TemperatureInterface impl)
{
    tempSensors[pos] = impl;
}

struct TemperatureInterface *temperatureSensors_getSensorHandle(enum TempSensorPositions pos)
{
    return &(tempSensors[pos]);
}
