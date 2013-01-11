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
    if (currentState.mainboardstate != wantedState.mainboardstate || pending){
        bool allowed = FALSE;
        //printf("CHANGE STATE\n");
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
//        printf("Pending: %i, Allowed: %i\n",pending,allowed);
        if (!allowed){
            //wantedState.mainboardstate = 0;
            //TODO error handling
            //currentState.mainboardstate = ERROR_WRONG_STATE;
            //wantedState.mainboardstate = ERROR_WRONG_STATE;
        } else {
            if(!pending)
                    currentState.mainboardstate = wantedState.mainboardstate;
        }
    }
}
bool toOff(){
    //current state doesn't matter
    //toOff is allowed every time
    //TODO hbridges unconfigure
//    printf("TO_OFF\n");
            int i;
            pending=0;
            for(i=0;i<4;i++){
                wantedState.hbridges[i].state = STATE_UNCONFIGURED;
                if(currentState.hbridges[i].state != STATE_UNCONFIGURED){
//                        printf("Hbridghe %i is in %i\n",i,currentState.hbridges[i].state);
                        pending = 1;
                }
            }
    return TRUE;
}

bool toRunning(){
    
    //printf("MB_TO_RUNNING\n");
    //current state doesn't matter
    //toRunning is allowed every time
    if (currentState.mainboardstate == UNDEFINED){
        printf("undefined\n");
        return FALSE;
    }

   int i;
   char do_change = 1;

   for(i=0;i<4;i++)
    if(wantedState.hbridges[i].state == STATE_SENSOR_ERROR || wantedState.hbridges[i].state  == STATE_ACTUATOR_ERROR){
        do_change = 0;
        break;
    }
  
   if(do_change){
            int i;
            pending=0;
            for(i=0;i<4;i++){
                wantedState.hbridges[i].state = STATE_RUNNING;
                if(currentState.hbridges[i].state != STATE_RUNNING){
                        pending = 1;
                }
            }
    }else{
        printf("We stil lhave an error %i\n", currentState.mainboardstate);
        return FALSE;
    }

    if(currentState.hbridges[0].state != STATE_RUNNING && currentState.hbridges[1].state != STATE_RUNNING && currentState.hbridges[2].state != STATE_RUNNING && currentState.hbridges[3].state != STATE_RUNNING){
        printf("Not running yet %i \n",currentState.mainboardstate);
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
