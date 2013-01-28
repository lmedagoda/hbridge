#ifndef HBRIDGE_CMD2_H
#define HBRIDGE_CMD2_H

#include <stdint.h>
#include "../common/packets.h"

void hbridge_init(uint16_t numHbridges);

/**
 * This method assumes that is is called
 * roughly every millisecond. It processes
 * state changes and incomming packets.
 * */
void hbridge_process();

/**
 * Returns the sensor configuration for the given
 * hbridge
 * */
struct sensorConfig *getSensorConfig(uint16_t hbridgeNr);

void hbridge_requestStates();

void hbridge_triggerSensorConfiguration();
uint8_t hbridge_sensorsConfigured();

/**
 * Returns the actuator configuration for the given
 * hbridge
 * */
struct actuatorConfig *getActuatorConfig(uint16_t hbridgeNr);

void hbridge_triggerActuatorConfiguration();
uint8_t hbridge_actuatorsConfigured();

uint8_t hbridge_configureError();

void hbridge_resetDevices();




#endif