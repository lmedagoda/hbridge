#ifndef MB_TYPES_H
#define MB_TYPES_H

enum MAINBOARDSTATE {
    MAINBOARD_OFF = 0,
    MAINBOARD_RUNNING = 2,
    MAINBOARD_AUTONOMOUS,
    MAINBOARD_EMERGENCY,
    MAINBOARD_FULL_AUTONOMOUS,
    MAINBOARD_SURFACE,
    MAINBOARD_UNDEFINED,
    MAINBOARD_NUM_STATES,
};

// typedef for the different command headers
// packet ids can only be 5 bits wide
typedef enum {
    MB_PING = 0x01,        // ping packet
    MB_SET_STATE = 0x02,   // set runtime state of the system
    MB_CONTROL = 0x04, 
    MB_STATUS = 0x08,      // status information from asguard
    MB_ID_CAN = 0x0b,        // CAN passthrough
    MB_MODE_CHANGE = 0x0c,      // mode changed on asguard
    MB_ID_CAN_ACKED = 0x0d,    // CAN passthrough, with ack response
    MB_ID_CAN_ACK = 0x0d,    // CAN passthrough ack
    MB_GIVE_TOKEN = 0x0e,    //Packet contains only an token to handle ansync communications, needed for ppp tunnel to make the conn more robus, this token should ONLY create internal of driver (arc_driver) 
    MB_GIVE_BACK = 0x0f,
    MB_REGISTER = 0x03
} MB_PACKET_ID;

struct canAckMsg
{
    unsigned canId:12;
    unsigned index:4;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct canMsg
{
    unsigned canId:12;
    unsigned index:4;
    //warning this may be invalid depending on the size...
    uint8_t data[8];
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct MotionCommand
{
    uint8_t traversalSpeed;
    uint8_t rotationalSpeed;
} __attribute__ ((packed)) __attribute__((__may_alias__));

#endif