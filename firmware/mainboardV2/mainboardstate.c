#include "mainboardstate.h"
#include "state.h"
#include "bool.h"
#include "../hbridgeCommon/drivers/printf.h"

volatile extern MainState currentState;
volatile extern MainState wantedState;
volatile uint8_t pending=0;
bool toOff();
bool toRunning();
bool toAutonomous(bool full_autonomous);
bool toSurface();
void processMainboardstate(){
    //printf("PROCESS MAINBOARD STATE\n");
//    printf("wantedState.mainboardstate %i %i %i\n",wantedState.mainboardstate,currentState.mainboardstate,pending);
    if (currentState.mainboardstate != wantedState.mainboardstate){
        bool allowed = FALSE;
        //printf("CHANGE STATE\n");
        switch(wantedState.mainboardstate){
            case MAINBOARD_OFF:
                allowed = toOff();
                break;
            case MAINBOARD_RUNNING:
                allowed = toRunning();
                break;
            case MAINBOARD_HALT:
                printf("WANTED STATE HALT IS DEPRECATED\n");
                allowed = FALSE;
                break;
            case MAINBOARD_AUTONOMOUS:
                allowed = toAutonomous(FALSE);
                break;
            case MAINBOARD_FULL_AUTONOMOUS:
                allowed = toAutonomous(TRUE);
                break;
            case MAINBOARD_SURFACE:
                allowed = toSurface();
                break;
            default:
                printf("WARNING: Unknown state or senseless state\n");
                break;
        }
        if (!allowed){
            //wantedState.mainboardstate = 0;
            //TODO error handling
            currentState.mainboarderror = MAINBOARD_WRONG_STATE_ERROR;
            wantedState.mainboarderror = MAINBOARD_WRONG_STATE_ERROR;
        } else {
            //behandelt
        }
    }
}
bool toOff(){
    //current state doesn't matter
    //toOff is allowed every time
    //TODO hbridges unconfigure
    int i;
    if (currentState.mainboardstate != MAINBOARD_CONFIGURING_TO_OFF){
        for(i=0;i<4;i++){
            wantedState.hbridges[i].state = STATE_UNCONFIGURED;
        }
        printf("to OFF\n");
        currentState.mainboardstate = MAINBOARD_CONFIGURING_TO_OFF; 
    } else {
        bool off_all = TRUE;
        for (i=0; i<4; i++){
            if (currentState.hbridges[i].state != STATE_UNCONFIGURED){
                off_all = FALSE;
            }
        }
        if (off_all) {
            printf("CURRENT TO OFF\n");
            currentState.mainboardstate = MAINBOARD_OFF;
        }
        //else {printf("still not in OFF\n");}
    }
    return TRUE;
}

bool toRunning(){
    if (currentState.mainboardstate == MAINBOARD_UNDEFINED){
        printf("undefined\n");
        return FALSE;
    }
    
    int i;
    if (currentState.mainboardstate != MAINBOARD_CONFIGURING_TO_RUNNING){ 
            for(i=0;i<4;i++){
                wantedState.hbridges[i].state = STATE_RUNNING;
            }
            currentState.mainboardstate = MAINBOARD_CONFIGURING_TO_RUNNING;
            printf("to RUNNING\n");
    } else {
        bool run_all = TRUE;
        for (i=0; i<4; i++){
            if (currentState.hbridges[i].state != STATE_RUNNING){
                run_all = FALSE;
            }
        }
        if (run_all) {
            currentState.mainboardstate = MAINBOARD_RUNNING;
            printf("Now in Running\n");

        }
        //else {printf("Still not in Running\n");}
    }
    return TRUE; 
}
bool toAutonomous(bool full_autonomous){
    if (currentState.mainboardstate == MAINBOARD_UNDEFINED){
        return FALSE;
    }
    //TODO everything for autonomous

    return TRUE;
}

bool toSurface(){
    if (currentState.mainboardstate == MAINBOARD_UNDEFINED){
        return FALSE;
    }
    //TODO get the control from the PC
    //unconfigure Hbridges
    //configure Hbrdiges
    //Thrusterup
    return TRUE;
}
