#ifndef TOKENHANDLER_H
#define TOKENHANDLER_H

#include "arc_packet.h"
#define MAX_HANDLERS 20
//How many Tokens to other Stations are allowed?
#define TIMEOUT_TOKEN 20

typedef enum {
    AMBER_UNREGISTERED,
    AMBER_REGISTERED} AMBER_STATE;
extern volatile AMBER_STATE amber_state;

typedef uint8_t (*token_handler_t)(arc_packet_t* packet);

uint8_t handleTokenPacket(arc_packet_t *packet);
void registerTokenHandler(token_handler_t handler);
void initTokenhandling();
extern volatile uint8_t has_token;

#endif
