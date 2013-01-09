#include "hbridge_cmd.h"
#include "protocol.h"
#include "printf.h"

void hbridge_sendClearError(int id){
    printf("clear\n");
//    protocol_sendData(id, PACKET_ID_CLEAR_ACTUATOR_ERROR, 1, sizeof(1));
    protocol_sendData(id, PACKET_ID_CLEAR_SENSOR_ERROR, 1, sizeof(1));
    protocol_sendData(id, PACKET_ID_REQUEST_STATE, 1, sizeof(1));
}

void hbridge_sendEncoderConfiguration(int id){
  struct encoderConfiguration e1c;
    e1c.encoderType = NO_ENCODER;
    e1c.tickDivider = 1;
    e1c.ticksPerTurn = 1;
    
  struct sensorConfig sc;
    sc.encoder1Config = e1c;
    sc.encoder2Config = e1c;
    sc.externalTempSensor = 0;
    sc.statusEveryMs = 5;
    
  protocol_sendData(id, PACKET_ID_SET_SENSOR_CONFIG, (unsigned char *) &sc, sizeof(struct sensorConfig));
}

void hbridge_sendActuatorConfiguration(int id){
  
  struct actuatorConfig ac;
    ac.pwmStepPerMs = 200;
    ac.openCircuit = 1;
    ac.maxCurrent = 2000;
    ac.maxCurrentCount = 100;
    ac.maxBoardTemp = 80;
    ac.maxBoardTempCount = 200;
    ac.maxMotorTemp = 60;
    ac.maxMotorTempCount = 200;
    ac.controllerInputEncoder = INTERNAL;
    ac.timeout = 200;
    
  protocol_sendData(id, PACKET_ID_SET_ACTUATOR_CONFIG, (unsigned char *) &ac, sizeof(struct actuatorConfig));
}

void hbridge_sendControllerConfiguration(int id){
    struct setActiveControllerData controller;
    controller.controllerId = CONTROLLER_MODE_PWM;
    
    protocol_sendData(id, PACKET_ID_SET_ACTIVE_CONTROLLER, (unsigned char *) &controller, sizeof(struct setActiveControllerData));
}

void hbridge_setValue(int value1, int value2, int value3, int value4){
    //printf("MOTORENWERTE %i, %i, %i, %i \n", value1, value2, value3, value4);   
    struct setValueData values;
    values.board1Value = value1 * 18;
    values.board2Value = value2 * 18;
    values.board3Value = value3 * 18;
    values.board4Value = value4 * 18;
    protocol_sendData(RECEIVER_ID_ALL, PACKET_ID_SET_VALUE58, (unsigned char *) &values, sizeof(struct setValueData));
}
