#ifndef STATE_H
#define STATE_H

#include "packets.h"
#include "arc_packet.h"
#include "bool.h"


enum COMSTATES {
    COM_RUNNING,
    COM_OFFLINE,
    COM_ZOMBIE
};


struct HBRIDGESTATE {
    enum STATES state;
    bool pending;
};

typedef struct {
    ARC_SYSTEM_STATE mainboardstate;
    struct HBRIDGESTATE hbridges[4];
    enum COMSTATES computer1;
    enum COMSTATES computer2;
} MainState;
    
volatile extern MainState currentState;
volatile extern MainState wantedState;
#endif
