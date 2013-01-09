#ifndef TOKENHANDLER_H
#define TOKENHANDLER_H

#include "bool.h"
#include "arc_packet.h"
#define MAX_HANDLERS 20
//How many Tokens to other Stations are allowed?
#define TIMEOUT_TOKEN 20
typedef enum {
    AMBER_UNREGISTERED,
    AMBER_REGISTERED} AMBER_STATE;
extern volatile AMBER_STATE amber_state;

bool handleTokenPacket(arc_packet_t *packet);
void registerTokenHandler(bool (*handler)(arc_packet_t* packet));
void initTokenhandling();
extern volatile bool has_token;

#endif
