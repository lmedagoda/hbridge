#ifndef __CONTROLLERS_H
#define __CONTROLLERS_H

#include <stdint.h>
#include <common/pid.h>

#define MAX_CONTROLLER_DATA_SIZE 8

struct ControllerTargetData
{
    uint8_t data[MAX_CONTROLLER_DATA_SIZE];
    uint8_t dataSize;
};

struct ControllerData {
    struct pid_data pidData;
    int32_t lastWheelPos;
    uint8_t debugActive;
    uint8_t isConfigured;
};

struct ControllerInterface {
    /**
     * Initializes the controller
     * */
    void (*init) (void);
    
    /**
     * Resets the controller. This only resets
     * state variables, but not the configuration
     * */
    void (*reset) (int32_t wheelPos);
    
    /**
     * Returns wether the controller is propperly configured
     * */
    uint8_t (*isConfigured) (void);
    
    /**
     * This function 'steps' the controller. Meaning it computes the 
     * next pwm value in respect to its configuration and the given parameters
     * 
     * @param targetData This structure contains the target value and needs
     * 			 be interpreted/casted by the controller.
     * 
     * @param actuatorPos current position of the actuator in ticks
     * 
     * @param ticksPerTurn Number of ticks per turn of the actuator
     * */
    int32_t (*step) (struct ControllerTargetData *targetData, int32_t actuatorPos, uint32_t ticksPerTurn);
    
    /**
     * Deinitializes the controller
     * This function call should also remove the configuration
     * */
    void (*deInit) (void);
};

/**
* This function initializes internal values of the controllers
**/
void controllers_init();

/**
* This function sould be called, if the h-bridge goes into an
* error state. In this function the internal data of the 
* controllers should be reset, so that no quirks occure on
* reactivation, but configuration data should be kept.
**/
void controllers_resetControllers(int32_t wheelPos);

/**
 * This function returns 1 if all controllers
 * have been sucessfully configured.
 * */
uint8_t controllers_areControllersConfigured();

/**
 * Returns a pointer to a registered controller, or Zero if no
 * controller with the given id was registered
 * */
const struct ControllerInterface *controllers_getController(uint8_t ctrlID);

#endif
