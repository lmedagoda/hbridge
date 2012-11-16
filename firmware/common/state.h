#ifndef __STATE_H
#define __STATE_H

#include <stdint.h>
#include "packets.h"
#include "../interfaces/controllers.h"

struct ErrorState {
    unsigned motorOverheated:1;
    unsigned boardOverheated:1;
    unsigned overCurrent:1;
    unsigned timeout:1;
    unsigned badConfig:1;
    unsigned encodersNotInitalized:1;
    unsigned controllersNotConfigured:1;
    unsigned hardwareShutdown:1;
};

struct SensorConfiguration
{
    enum encoderTypes internalEncoder;
    enum encoderTypes externalEncoder;
    uint16_t statusEveryMs;
    uint8_t useExternalTempSensor;
};

struct ActuatorConfiguration
{
    uint8_t useOpenLoop;
    uint16_t pwmStepPerMillisecond;
    uint16_t maxCurrent;
    uint8_t maxCurrentCount;
    uint8_t maxMotorTemp;
    uint8_t maxMotorTempCount;
    uint8_t maxBoardTemp;
    uint8_t maxBoardTempCount;
    uint16_t timeout;

};

struct GlobalState {
    enum STATES internalState;
    struct SensorConfiguration sensorConfig;
    struct ActuatorConfiguration actuatorConfig;
    struct ControllerTargetData targetData;
    enum controllerModes controllMode;
    enum controllerInputEncoder controllerInputEncoder;
    uint8_t resetTimeoutCounter;
};


extern volatile struct GlobalState *activeCState;
extern volatile struct GlobalState *lastActiveCState;

void state_initStruct(volatile struct GlobalState *cs);
void state_printDebug(volatile struct GlobalState *cs);

void state_switchToErrorState();

void state_printErrorState();
uint8_t state_inErrorState();
volatile struct ErrorState *state_getErrorState();
void state_clearErrors();

void state_init();

#endif
