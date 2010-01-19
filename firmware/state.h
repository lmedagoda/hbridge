#ifndef __STATE_H
#define __STATE_H

#include "inc/stm32f10x_type.h"
#include "protocol.h"

enum internalState {
  STATE_UNCONFIGURED,
  STATE_CONFIG1_RECEIVED,
  STATE_CONFIG2_RECEIVED,
  STATE_CONFIGURED,
  STATE_ERROR,
};

struct ErrorState {
    unsigned motorOverheated:1;
    unsigned boardOverheated:1;
    unsigned overCurrent:1;
    unsigned timeout:1;
    unsigned badConfig:1;
    unsigned encodersNotInitalized:1;
    unsigned unused:3;
} __attribute__ ((packed));

struct PIDValues {
  s16 kp;
  s16 ki;
  s16 kd;
  u16 minMaxPidOutput;
};

struct ControllerState {
  enum controllerModes controllMode;
  enum internalState internalState;
  struct PIDValues speedPIDValues;
  struct PIDValues positionPIDValues;
  u8 useBackInduction;
  u8 useOpenLoop;
  u8 enablePIDDebug;
  u8 cascadedPositionController;
  u16 pwmStepPerMillisecond;
  u16 maxCurrent;
  u8 maxCurrentCount;
  u8 maxMotorTemp;
  u8 maxMotorTempCount;
  u8 maxBoardTemp;
  u8 maxBoardTempCount;
  u16 timeout;
  s32 targetValue;
  u8 resetTimeoutCounter;
  u32 ticksPerTurn;
};


extern volatile struct ControllerState *activeCState;
extern volatile struct ControllerState *lastActiveCState;

void initStateStruct(volatile struct ControllerState *cs);
void printStateDebug(volatile struct ControllerState *cs);

u8 inErrorState();
struct ErrorState *getErrorState();
void clearErrors();

#endif
