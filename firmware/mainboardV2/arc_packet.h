#ifndef ARC_PACKET_H
#define ARC_PACKET_H

#include <inttypes.h>

// definitions for the packet protocol used in the cesar project
// the packet is defined as
//
// sync           (1 byte) - sync marker
// system_id      (3 bits) - id of the originator of the packet 
// packet_id      (5 bits) - command header as per the definitions below  
// length         (8 bits) - data size
// data           (0-MAX_DATA_LENGTH bytes) - data content
// crc            (1 byte) - crc value of the packet
//
// The index field can be used in transmissions with multiple packets for one payload
//

// polynomial to use for the crc
#define ARC_POLYVAL 0x8C

// sync marker to decrease the changes of wrong packet identification
#define SYNC_MARKER 0xaa

// the packet length
#define ARC_MAX_DATA_LENGTH 16 
#define ARC_MIN_FRAME_LENGTH 4 
#define ARC_MAX_FRAME_LENGTH (ARC_MAX_DATA_LENGTH + ARC_MIN_FRAME_LENGTH)

// typedef for the different command headers
// packet ids can only be 5 bits wide
typedef enum {
    PING = 0x01,        // ping packet
    SET_STATE = 0x02,   // set runtime state of the system
    CONTROL = 0x04, 
    STATUS = 0x08,      // status information from asguard
    DGPS = 0x0a,        // DGPS correction data
    ID_CAN = 0x0b,        // CAN passthrough
    MODE_CHANGE = 0x0c,      // mode changed on asguard
    ID_CAN_ACKED = 0x0d,    // CAN passthrough, with ack response
    ID_CAN_ACK = 0x0d,    // CAN passthrough ack
    GIVE_TOKEN = 0x0e,    //Packet contains only an token to handle ansync communications, needed for ppp tunnel to make the conn more robus, this token should ONLY create internal of driver (arc_driver) 
    GIVE_BACK = 0x0f,
    REGISTER = 0x03
} ARC_PACKET_ID;

typedef enum {
    REQUESTED = 0,
    TIMEOUT = 1,
    INITIAL = 2,
    ILLEGAL = 3,
} MODE_CHANGE_REASON;

// different runtime system states
typedef enum {
    OFF = 0x00,
    HALT = 0x01,
    RUNNING = 0x02,
    AUTONOMOUS = 0x03,
    EMERGENCY = 0x04,
    FULL_AUTONOMOUS = 0x05, //LONG timeout for real autonomous mission without any connection to the vehicle
    SURFACE = 0x06, //Surfacing in case of abort, activly diving thruster up
    UNDEFINED = 0xff
} ARC_SYSTEM_STATE;

// system ids of the involved hosts
// please note, system ids can only be 2 bits wide
typedef enum {
    REGISTER_CHANCE = 0x00,
    OCU = 0x01,
    ASV = 0x02,
    AVALON = 0x03
} ARC_SYSTEM_ID;

// this flag controls the interpretation of the system_id field.
// if the originator is MASTER, system_id is the device that is allowed
// to send next.
// in case its SLAVE, system_id is the actual originator of the package.
typedef enum {
    MASTER = 0,
    SLAVE = 1,
} ARC_ORIGINATOR_FLAG;

/**
 * Control Packet with PWM Values for all engines on the ASV
 */
typedef struct {
	uint8_t quer_vorne;
	uint8_t quer_hinten;
	uint8_t motor_rechts;
		uint8_t motor_links;	
} arc_asv_control_packet_t;

typedef struct {
    uint8_t sync;
    ARC_ORIGINATOR_FLAG originator;
    ARC_SYSTEM_ID system_id;
    ARC_PACKET_ID packet_id;
    uint8_t length;
    uint8_t data[ARC_MAX_DATA_LENGTH]; 
    uint8_t crc;
} arc_packet_t;

/** 
 * Parses an input buffer for a valid packet and returns it in a packet structure
 * returns
 * n - if a packet was found and the struct is valid, n is the length of the packet
 * 0 - buffer too small for packet or packet incomplete
 * -1 - invalid packet at start of buffer 
*/
int16_t parsePacket(const uint8_t *buffer, uint16_t buffer_length, arc_packet_t *packet);

/** 
 * uses a packet struct and adds the crc based on the other values. The provided buffer
 * needs to be at least MAX_FRAME_LENGTH in size. 
 * returns the length of the data frame
 *
 * all of the following methods will actually assume the OCU as the originator
 * of the packet.
 */
uint16_t createPacket(const arc_packet_t *packet, uint8_t *buffer);

arc_packet_t createTokenPacket();

#endif
