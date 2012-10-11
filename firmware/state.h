#ifndef __STATE_H
#define __STATE_H

#include <stdint.h>
#include "protocol.h"

enum internalState {
  STATE_UNCONFIGURED,
  STATE_CONFIG1_RECEIVED,
  STATE_CONFIG2_RECEIVED,
  STATE_CONFIGURED,
  STATE_GOT_TARGET_VAL,
  STATE_ERROR,
};

struct ErrorState {
    uint8_t motorOverheated;
    uint8_t boardOverheated;
    uint8_t overCurrent;
    uint8_t timeout;
    uint8_t badConfig;
    uint8_t encodersNotInitalized;
    uint8_t hardwareShutdown;
};


struct ControllerState {
  enum controllerModes controllMode;
  enum internalState internalState;
  enum encoderTypes internalEncoder;
  enum encoderTypes externalEncoder;
  enum controllerInputEncoder controllerInputEncoder;
  uint8_t useExternalTempSensor;
  uint8_t useOpenLoop;
  uint16_t pwmStepPerMillisecond;
  uint16_t maxCurrent;
  uint8_t maxCurrentCount;
  uint8_t maxMotorTemp;
  uint8_t maxMotorTempCount;
  uint8_t maxBoardTemp;
  uint8_t maxBoardTempCount;
  uint16_t timeout;
  int32_t targetValue;
  uint8_t resetTimeoutCounter;
};


extern volatile struct ControllerState *activeCState;
extern volatile struct ControllerState *lastActiveCState;

void initStateStruct(volatile struct ControllerState *cs);
void printStateDebug(volatile struct ControllerState *cs);

void printErrorState();
uint8_t inErrorState();
volatile struct ErrorState *getErrorState();
void clearErrors();

#endif
