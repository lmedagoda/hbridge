/* 
    * File:   mainboard_state.h
    * Author: clausen
    *
    * Created on 23. Dezember 2012, 10:24
*/

#ifndef MAINBOARD_STATE_H
#define MAINBOARD_STATE_H

#include "packets.h"
#include "arc_packet.h"


enum COMSTATES {
    RUNNING,
    OFFLINE,
    ZOMBIE
};


struct HBRIDGESTATE {
    enum STATES state;
    enum COMSTATES connection;
};

typedef struct {
    ARC_SYSTEM_STATE state;
    struct HBRIDGESTATE hbridge1;
    struct HBRIDGESTATE hbridge2;
    struct HBRIDGESTATE hbridge3;
    struct HBRIDGESTATE hbridge4;
    enum COMSTATES computer1;
    enum COMSTATES computer2;
} MainState;
    
volatile extern struct MainboardState currentState;
