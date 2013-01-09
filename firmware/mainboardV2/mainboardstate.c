#include "mainboardstate.h"
#include "state.h"
#include "bool.h"
#include "../hbridgeCommon/drivers/printf.h"

volatile extern MainState currentState;
volatile extern MainState wantedState;
bool toOff();
bool toRunning();
bool toAutonomous(bool full_autonomous);
bool toSurface();
void processMainboardstate(){
    //printf("PROCESS MAINBOARD STATE\n");
    if (currentState.mainboardstate != wantedState.mainboardstate){
        bool allowed = FALSE;
        printf("CHANGE STATE\n");
        switch(wantedState.mainboardstate){
            case OFF:
                allowed = toOff();
                break;
            case RUNNING:
                allowed = toRunning();
                break;
            case HALT:
                printf("WANTED STATE HALT IS DEPRECATED\n");
                allowed = FALSE;
                break;
            case AUTONOMOUS:
                allowed = toAutonomous(FALSE);
                break;
            case FULL_AUTONOMOUS:
                allowed = toAutonomous(TRUE);
                break;
            case SURFACE:
                allowed = toSurface();
                break;
            default:
                printf("WARNING: Unknown state\n");
                break;
        }
        if (allowed == FALSE){
            //TODO error handling
            currentState.mainboardstate = ERROR_WRONG_STATE;
            wantedState.mainboardstate = ERROR_WRONG_STATE;
        } else {
            currentState.mainboardstate = wantedState.mainboardstate;
        }
    }
}
bool toOff(){
    //current state doesn't matter
    //toOff is allowed every time
    //TODO hbridges unconfigure
    return TRUE;
}

bool toRunning(){
    //current state doesn't matter
    //toRunning is allowed every time
    if (currentState.mainboardstate == UNDEFINED){
        return FALSE;
    }
    //TODO hbridges unconfigure
    //TODO hbridges configure
    return TRUE; 
}
bool toAutonomous(bool full_autonomous){
    if (currentState.mainboardstate == UNDEFINED){
        return FALSE;
    }
    //TODO everything for autonomous

    return TRUE;
}

bool toSurface(){
    if (currentState.mainboardstate == UNDEFINED){
        return FALSE;
    }
    //TODO get the control from the PC
    //unconfigure Hbridges
    //configure Hbrdiges
    //Thrusterup
    return TRUE;
}
