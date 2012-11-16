#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H

#include <stdint.h>

enum TempSensorPositions
{
    POSITION_MOTOR,
    POSITION_PCB,
};

struct TemperatureInterface
{
    /**
     * This function sets up the temperature sensor.
     * */
    void (*sensorInit) (void);
    /**
     * Polls for a new temperature value
     * */
    uint8_t (*getTemperature) (int32_t* val);
    /**
     * Deactivates the temperature sensor
     * */
    void (*sensorDeInit) (void);
};

/**
 * Set ups the internal data structures 
 * to return the correct handles.
 */
void temperatureSensorsInit();

void setTemperatureSensorImpl(enum TempSensorPositions pos, struct TemperatureInterface impl);

/**
 * Returns a handle to the sensor for the given
 * position.
 * 
 * Through this handle the sensor can be polled for readings
 *
 **/
struct TemperatureInterface *getSensorHandle(enum TempSensorPositions);

#endif
