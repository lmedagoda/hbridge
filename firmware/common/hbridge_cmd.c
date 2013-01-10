#include "hbridge_cmd.h"
#include "protocol.h"
#include "printf.h"
#include <stddef.h>

void hbridge_sendClearSensorError(int hbridgeId){
    protocol_sendData(hbridgeId, PACKET_ID_CLEAR_SENSOR_ERROR, NULL, 0);
}

void hbridge_sendClearActuatorError(int hbridgeId){
    protocol_sendData(hbridgeId, PACKET_ID_CLEAR_ACTUATOR_ERROR, NULL, 0);
}

void hbridge_sensorStructInit(struct sensorConfig *sc){
    struct encoderConfiguration e1c;
        e1c.encoderType = NO_ENCODER;
        e1c.tickDivider = 1;
        e1c.ticksPerTurn = 1;
    
        sc->encoder1Config = e1c;
        sc->encoder2Config = e1c;
        sc->externalTempSensor = 0;
        sc->statusEveryMs = 0;
}

void hbridge_sendSensorConfiguration(int hbridgeId, struct sensorConfig *sc){
  protocol_sendData(hbridgeId, PACKET_ID_SET_SENSOR_CONFIG, (unsigned char *) sc, sizeof(struct sensorConfig));
}

void hbridge_actuatorStructInit(struct actuatorConfig *ac){
    ac->pwmStepPerMs = 200;
    ac->openCircuit = 1;
    ac->maxCurrent = 2000;
    ac->maxCurrentCount = 100;
    ac->maxBoardTemp = 80;
    ac->maxBoardTempCount = 200;
    ac->maxMotorTemp = 60;
    ac->maxMotorTempCount = 200;
    ac->controllerInputEncoder = INTERNAL;
    ac->timeout = 200;
}

void hbridge_sendActuatorConfiguration(int hbridgeId, struct actuatorConfig *ac){    
  protocol_sendData(hbridgeId, PACKET_ID_SET_ACTUATOR_CONFIG, (unsigned char *) ac, sizeof(struct actuatorConfig));
}

void hbridge_controllerStructInit(struct setActiveControllerData *cd){
    cd->controllerId = CONTROLLER_MODE_PWM;
}

void hbridge_sendControllerConfiguration(int hbridgeId, struct setActiveControllerData *cd){
    protocol_sendData(hbridgeId, PACKET_ID_SET_ACTIVE_CONTROLLER, (unsigned char *) cd, sizeof(struct setActiveControllerData));
}

void hbridge_setValue(int value1, int value2, int value3, int value4){   
    struct setValueData values;
    values.board1Value = value1 * 18;
    values.board2Value = value2 * 18;
    values.board3Value = value3 * 18;
    values.board4Value = value4 * 18;
    protocol_sendData(RECEIVER_ID_ALL, PACKET_ID_SET_VALUE58, (unsigned char *) &values, sizeof(struct setValueData));
}

void hbridge_requestState(int hbridgeId){
    protocol_sendData(hbridgeId, PACKET_ID_REQUEST_STATE, NULL, 0);    
}

void hbridge_setUnconfigured(int hbridgeId){
    protocol_sendData(hbridgeId, PACKET_ID_SET_UNCONFIGURED, NULL, 0);    
}