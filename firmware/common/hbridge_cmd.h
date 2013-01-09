#include "packets.h"

void hbridge_sensorStructInit(struct sensorConfig *sc);
void hbridge_sendSensorConfiguration(int hbridgeId, struct sensorConfig *sc);
void hbridge_actuatorStructInit(struct actuatorConfig *ac);
void hbridge_sendActuatorConfiguration(int hbridgeId, struct actuatorConfig *ac);
void hbridge_controllerStructInit(struct setActiveControllerData *cd);
void hbridge_sendControllerConfiguration(int hbridgeId, struct setActiveControllerData *cd);
void hbridge_setValue( int value1, int value2, int value3, int value4);
void hbridge_sendClearSensorError(int hbridgeId);
void hbridge_sendClearActuatorError(int hbridgeId);
void hbridge_requestState(int hbridgeId);
