#include "arc_packet.h"

/**
 * generates and returns a checksum of the given Parameters
 *
 */
uint8_t createCRC(uint8_t start, const uint8_t *crc) { 
    /* calculates 8-bit CRC of given data */ 
    /* based on the polynomial POLYVAL */ 
    uint8_t c, i; 
    c = *crc; 
    for (i = 0; i < 8; i++) {
        if ((c ^ start) & 1) c = (c >> 1 ) ^ ARC_POLYVAL; 
        else c >>= 1; 
        start >>= 1; 
    } 
    return c; 
} 

int16_t parsePacket(const uint8_t *buffer, uint16_t buffer_length, arc_packet_t *packet) {
    uint16_t i = 0, length = 0, crc = 0;	

    // First remove everything until the first SYNC_MARKER
    for (i = 0; i < buffer_length && buffer[i] != SYNC_MARKER; ++i)
    {
	;
    }
    if (i > 0)
	return -i;
    
    // see if a packet could fit into the buffer
    if( buffer_length >= ARC_MIN_FRAME_LENGTH) {
        length = buffer[2];

        if( length > ARC_MAX_DATA_LENGTH ) {
            return -1; // definitely not a valid packet
        } else {
            packet->sync = SYNC_MARKER;
        }

        if( buffer_length >= (ARC_MIN_FRAME_LENGTH + length) ) {
            // copy data and check crc
            for( i=0; i<(length+ARC_MIN_FRAME_LENGTH); i++ ) {
                if( i == 1 ) {
                    packet->packet_id = (ARC_PACKET_ID)(buffer[i] & 0x1f);
                    packet->system_id = (ARC_SYSTEM_ID) ((buffer[i] >> 5) & 0x03);
                    packet->originator = (ARC_ORIGINATOR_FLAG) ((buffer[i] >> 7) & 0x01);
                } else if( i == 2 ) {
                    packet->length = buffer[i];
                } else if( i == length + 3) {
                    packet->crc = buffer[i] ^ SYNC_MARKER; // unscramble the CRC marker
                    
                    if(  packet->crc == crc ) {
                        return length + ARC_MIN_FRAME_LENGTH; // valid packet return frame length
                    } else {
                        return -1; // bad crc
                    }
                } else {
                    packet->data[i-3] = buffer[i];
                }

                crc = createCRC( crc, buffer+i );
            }
        }
    } 

    return 0; // buffer not long enough... could still be a valid packet
}

uint16_t createPacket(const arc_packet_t *packet, uint8_t *buffer) {
    uint16_t i, crc = 0, length;

    length = packet->length + ARC_MIN_FRAME_LENGTH;
    if( length > ARC_MAX_FRAME_LENGTH )
        return 0;

    // flatten packet content and create crc checksum
    for(i=0;i<length;i++) {
        if( i == 0 ) {
            buffer[i] = SYNC_MARKER;
        } else if( i == 1 ) {
            buffer[i] = (packet->packet_id & 0x1f) | (packet->system_id & 0x03) << 5 | (packet->originator & 0x01) << 7;
        } else if( i == 2 ) {
            buffer[i] = packet->length;
        } else if( i == length-1 ) {
            buffer[i] = crc ^ SYNC_MARKER;
        } else {
            buffer[i] = packet->data[i-3];
        }

        crc = createCRC( crc, buffer+i );
    }

    return length;
}

arc_packet_t createTokenPacket(){
  arc_packet_t packet;
  packet.originator = MASTER; //Dosnt matter, catched early
  packet.system_id = OCU; //dito, currently until we have multiple systems
  //packet.packet_id = TOKEN;
  packet.length = 0;
  return packet;
}

