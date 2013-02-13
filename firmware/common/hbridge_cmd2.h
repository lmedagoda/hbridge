#ifndef HBRIDGE_CMD2_H
#define HBRIDGE_CMD2_H

#include <stdint.h>
#include "../common/packets.h"



void hbridge_init(uint16_t numHbridges);

uint8_t hbridge_getControlledHbridges();

/**
 * Sets the used controller and its configuration
 * data. 
 * 
 * In order to set no configuration data
 * pass a dataSize of Zero.
 * */
void hbridge_setControllerWithData(const uint16_t hbridgeId, enum controllerModes controller, const int packetId, const char *data, const uint8_t dataSize);

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
enum STATES hbridge_getState(uint16_t hbridgeNr);
enum STATES hbridge_getLowestHBState();

uint8_t hbridge_configureSensors();

uint8_t hbridge_sensorsConfigured();

/**
 * Returns the actuator configuration for the given
 * hbridge
 * */
struct actuatorConfig *getActuatorConfig(uint16_t hbridgeNr);

/**
 * Configures the actuators of all controlled 
 * motor drivers. 
 * Returns 1 if all motor drivers were configured successfully.
 * */
uint8_t hbridge_configureActuators();

uint8_t hbridge_actuatorsConfigured();
uint8_t hbridge_configureControllers();

/**
 * Returns true if one of the controlled 
 * hbridges is in an actuator error state.
 * */
uint8_t hbridge_hasActuatorError();

/**
 * Returns true if one of the controlled 
 * hbridges is in a sensor error state.
 * */
uint8_t hbridge_hasSensorError();

void hbridge_resetActuators();
void hbridge_resetSensors();




#endif