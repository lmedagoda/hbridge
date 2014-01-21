#ifndef HBRIDGE_CMD_H
#define HBRIDGE_CMD_H
#include "packets.h"
#include "protocol.h"

/**
 * Initiates a sensor-config struct with default-values
 * No Encoder, no temperature-sensor, status-messages every 100ms
 * @param sc empty sensorConfig-struct
 */
void hbridge_sensorStructInit(struct sensorConfig *sc);

/**
 * Sends a sensorConfig to a given HBridge.
 * @param hbridgeId use protocol.h enum hostIDs
 * @param sc a filled! sensorconfig
 */
void hbridge_sendSensorConfiguration(enum hostIDs hbridgeId, struct sensorConfig *sc);

/**
 * Initiates a actuator-config struct with default-values
 * maxCurrent 3000, timeout 2000ms and much more
 * @param ac empty actuatorConfig-struct
 */
void hbridge_actuatorStructInit(struct actuatorConfig *ac);

/**
 * Sends an actuatorConfig to a given HBridge.
 * @param hbridgeId use protocol.h enum hostIDs
 * @param sc a filled! actuatorconfig
 */
void hbridge_sendActuatorConfiguration(enum hostIDs hbridgeId, struct actuatorConfig *ac);

/**
 * Initiates a controller-config struct with default-values
 * ControllerMode = PWM
 * @param ac empty setActiveControllerData-struct
 */
void hbridge_controllerStructInit(struct setActiveControllerData *cd);

/**
 * Sends a controllerconfiguration to a given HBridge.
 * @param hbridgeId use protocol.h enum hostIDs
 * @param cd a filled! setActiveControllerData
 */
void hbridge_sendControllerConfiguration(enum hostIDs hbridgeId, struct setActiveControllerData *cd);

/**
 * Sends a value to the 4 HBridges used in setValueData (see HBridge documentation)
 * values are interpretated as documented in HBridge-documentation, but this method sends them as they are
 * TODO!!!! get this out of this!
 * value1/2 * 9
 * value3/4 * 4
 * @param value1 
 * @param value2
 * @param value3
 * @param value4
 */
void hbridge_setValue( int16_t value1, int16_t value2, int16_t value3, int16_t value4);
void hbridge_setValues( int16_t value1, int16_t value2, int16_t value3, int16_t value4, enum HIGH_PRIORITY_IDs hbSet);

/**
 * Clears a sensorerror of the given HBridge
 * @param hbridgeId
 */
void hbridge_sendClearSensorError(enum hostIDs hbridgeId);

/**
 * Clears an actuatorerror of the given HBridge
 * @param hbridgeId
 */
void hbridge_sendClearActuatorError(enum hostIDs hbridgeId);

/**
 * Asks the given HBridge to send the current state
 * @param hbridgeId
 */
void hbridge_requestState(enum hostIDs hbridgeId);

/**
 * Sets the given HBridge to state unconfigured
 * @param hbridgeId
 */
void hbridge_setUnconfigured(enum hostIDs hbridgeId);

/**
 * Sets the given HBridge to state unconfigured
 * @param hbridgeId
 */
void hbridge_setActuatorUnconfigured(enum hostIDs hbridgeId);

/**
 * Sets the sender restrictions of the hbridges
 * @param hostId - ID of the HBridge which should receive the message
 * @param allAlowed - if true everyone is allowed to send commands otherwise only commands from mainboard are allowed
 */
void hbridge_sendAllowedSenderConfiguration(enum hostIDs hbridgeId, int allAllowed);

#endif
