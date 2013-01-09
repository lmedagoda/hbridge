#include "arc_packet.h"
#include "arc_ringbuffer.h"
int inc(int a);
int dec(int a);

arc_packet_t* first(RING_BUFFER* buffer){
    if (buffer->first != buffer->last) {
        return &buffer->elements[buffer->first]; 
    } else {
        return 0;
    }
}
arc_packet_t* last(RING_BUFFER* buffer){
    if (buffer->first != buffer->last) {
        return &buffer->elements[buffer->last]; 
    } else {
        return 0;
    }
}
arc_packet_t* pop_front(RING_BUFFER* buffer){
    if (buffer->first != buffer->last){
        arc_packet_t* packet = &(buffer->elements[buffer->first]);
        buffer->first = inc(buffer->first);
        return packet;
    } else {
        printf("RETURN 0i\n");
        return 0;
    }
}

arc_packet_t* pop_back(RING_BUFFER* buffer){
    if (buffer->first != buffer->last){
        arc_packet_t* packet = &(buffer->elements[buffer->last]);
        buffer->last = dec(buffer->last); 
        return packet;
    } else {
       return 0;
    } 
}
int push_back(arc_packet_t packet, RING_BUFFER* buffer){
    if (buffer->first != inc(buffer->last)){
       buffer->last = inc(buffer->last);
       packet_copy(&packet, &(buffer->elements[buffer->last]));
       return 1;
    } else {
        return 0;
    }
}
int push_front(arc_packet_t packet, RING_BUFFER* buffer){
    if (dec(buffer->first) != buffer->last){
        buffer->first = dec(buffer->first);
        packet_copy(&packet, &(buffer->elements[buffer->first]));
        return 1;
    } else {
        return 0;
    }
}
int size(RING_BUFFER* buffer){
    //TODO to implement
    return 0;
}

int inc(int a){
    return ++a % BUFFERSIZE;
}
int dec(int a){
    if (a == 0){
        return (BUFFERSIZE-1);
    } else {
        return --a;
    }
}
int packet_copy(arc_packet_t* src, arc_packet_t* dest){
    int i = 0;
    for (i = 0; i < sizeof(arc_packet_t); i++){
        ((char*) dest)[i] =  ((char*) src)[i];
    }
}
