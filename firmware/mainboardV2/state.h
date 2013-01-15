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
enum MAINBOARDSTATE{
    MAINBOARD_OFF = 0x00,
    MAINBOARD_HALT = 0x01,
    MAINBOARD_RUNNING = 0x02,
    MAINBOARD_AUTONOMOUS = 0x03,
    MAINBOARD_EMERGENCY = 0x04,
    MAINBOARD_FULL_AUTONOMOUS = 0x05,
    MAINBOARD_SURFACE = 0x06,
    MAINBOARD_CONFIGURING_TO_RUNNING = 0x07,
    MAINBOARD_CONFIGURING_TO_OFF = 0x08,
    MAINBOARD_CONFIGURING_TO_AUTONOMOUS = 0x09,
    MAINBOARD_UNDEFINED = 0xff
};
enum MAINBOARDERROR{
    MAINBOARD_NOERROR,
    MAINBOARD_HBRIDGE_ERROR,
    MAINBOARD_CAN_ERROR,
    MAINBOARD_WRONG_STATE_ERROR
};

typedef struct {
    struct HBRIDGESTATE hbridges[4];
    enum COMSTATES computer1;
    enum COMSTATES computer2;
    enum MAINBOARDSTATE mainboardstate;
    enum MAINBOARDERROR mainboarderror;
} MainState;
    
volatile extern MainState currentState;
volatile extern MainState wantedState;
#endif
