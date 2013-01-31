#include "mainboardstate.h"
#include "state.h"
#include "bool.h"
#include "../hbridgeCommon/drivers/printf.h"

//TODO: This is a hack, and should be done in hbridgestate.c ...
#include "../common/hbridge_cmd.h"

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

void switchHBridgeState(enum MAINBOARDSTATE pendingState, enum MAINBOARDSTATE finishedState, int wantedHBridgeState) {
    int i;
    if (currentState.mainboardstate != pendingState){
        for(i=0;i<4;i++){
            wantedState.hbridges[i].state = wantedHBridgeState;
        }
        printf("to %d\n", wantedHBridgeState);
        currentState.mainboardstate = pendingState; 
    } else {
        bool still_pending = FALSE;
        for (i=0; i<4; i++){
            if (currentState.hbridges[i].state != wantedHBridgeState){
                still_pending = TRUE;
            }
        }
        if (!still_pending) {
            printf("finished HBridge transition to %d\n", wantedHBridgeState);
            currentState.mainboardstate = finishedState;
        }
    }

}

bool toOff(){
    //current state doesn't matter
    //toOff is allowed every time
    switchHBridgeState(MAINBOARD_CONFIGURING_TO_OFF, MAINBOARD_OFF, STATE_UNCONFIGURED);
    return TRUE;
}

bool toRunning(){
    if (currentState.mainboardstate == MAINBOARD_UNDEFINED){
        printf("undefined\n");
        return FALSE;
    }
    
    switchHBridgeState(MAINBOARD_CONFIGURING_TO_RUNNING, MAINBOARD_RUNNING, STATE_RUNNING);
    return TRUE; 
}
bool toAutonomous(bool full_autonomous){
    if (currentState.mainboardstate == MAINBOARD_UNDEFINED){
        return FALSE;
    }
    switchHBridgeState(MAINBOARD_CONFIGURING_TO_AUTONOMOUS, MAINBOARD_AUTONOMOUS, STATE_UNCONFIGURED);
    //Allow the hbridges to communicate with the pc
    //TODO: This is a hack, and should be done in hbridgestate.c ...
    hbridge_sendAllowedSenderConfiguration(RECEIVER_ID_ALL, TRUE);

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
