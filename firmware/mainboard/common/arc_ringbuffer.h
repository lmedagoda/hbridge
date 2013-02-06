#ifndef ARC_RINGBUFFER_H
#define ARC_RINGBUFFER_H
#include "arc_packet.h"
#define BUFFERSIZE 200
typedef struct {
    uint8_t first;
    uint8_t  last;
    arc_packet_t elements[BUFFERSIZE];
} RING_BUFFER;


arc_packet_t* first(RING_BUFFER* buffer);
arc_packet_t* last(RING_BUFFER* buffer);
int pop_front(RING_BUFFER* buffer, arc_packet_t* packet);
arc_packet_t* pop_back(RING_BUFFER* buffer);
int push_back(arc_packet_t packet, RING_BUFFER* buffer);
int push_front(arc_packet_t packet, RING_BUFFER* buffer);
int size(RING_BUFFER* buffer);
#endif
