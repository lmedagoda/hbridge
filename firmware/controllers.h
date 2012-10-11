#ifndef __CONTROLLERS_H
#define __CONTROLLERS_H

#include <stdint.h>
#include "drivers/pid.h"

struct ControllerData {
    struct pid_data pidData;
    int32_t lastWheelPos;
    uint8_t debugActive;
};

struct ControllerInterface {
    void (*init) (void);
    void (*reset) (int32_t wheelPos);
    int32_t (*step) (int32_t targetPos, int32_t wheelPos, uint32_t ticksPerTurn);
    void (*deInit) (void);
};

#define NUM_CONTROLLERS 3

extern struct ControllerInterface controllers[NUM_CONTROLLERS];

/**
* This function initializes internal values of the controllers
**/
void initControllers();

/**
* This function sould be called, if the h-bridge goes into an
* error state. In this function the internal data of the 
* controllers should be reset, so that no quirks occure on
* reactivation, but configuration data should be kept.
**/
void resetControllers(int32_t wheelPos);


#endif
