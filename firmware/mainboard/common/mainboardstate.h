#ifndef MAINBOARD_STATE_H
#define MAINBOARD_STATE_H
#include "stdint.h"
#include "mb_types.h"

enum COMSTATES {
    COM_RUNNING,
    COM_OFFLINE,
    COM_ZOMBIE
};



enum MAINBOARDERROR{
    MAINBOARD_NOERROR,
    MAINBOARD_HBRIDGE_ERROR,
    MAINBOARD_CAN_ERROR,
    MAINBOARD_WRONG_STATE_ERROR
};

struct MainboardState 
{
    ///Is called when switching to state
    uint8_t (*enterStateHandler)(void);
    
    ///is called while in state
    void (*stateHandler)(void);    
};


void mbstate_init();

/**
 * Switches to the given state
 * 
 * Returns 0 in success
 * */
uint8_t mbstate_changeState(enum MAINBOARDSTATE newState);

/**
 * Returns the current state
 * */
enum MAINBOARDSTATE mbstate_getCurrentState();

/**
 * Calls the handler for the current state
 * */
void mbstate_processState();

/**
 * Returns the state object for the given state.
 * This can be used to register own state and
 * stateSwitch functions.
 * */
struct MainboardState *mbstate_getState(enum MAINBOARDSTATE state);

#endif
