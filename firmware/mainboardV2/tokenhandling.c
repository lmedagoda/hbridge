#include "arc_driver.h"
#include "bool.h"
#include "tokenhandling.h"
#include "printf.h"
//How many Tokens wasn't my Token since my last Token
int given_tokens;
int random_register = 0;
bool (*tokenHandlers[MAX_HANDLERS])(arc_packet_t* packet);
int token_handlers = 0;
bool giveTokenHandler(arc_packet_t* packet);
bool giveBackHandler(arc_packet_t* packet);
bool registerPacketHandler(arc_packet_t* packet);
initTokenhandling(){
    registerTokenHandler(giveTokenHandler);
    registerTokenHandler(registerPacketHandler);
    registerTokenHandler(giveBackHandler);
}

bool handleTokenPacket(arc_packet_t* packet){
    printf("handle Token Packet \n");
    int i = 0;
        printf("tokenhandlers: %i \n", token_handlers); 
    token_handlers = 3;
    for (i=0; i < token_handlers; i++){
        printf("i: %i \n", i); 
        if (tokenHandlers[i](packet) == TRUE){
            return TRUE;
        }
    }
    printf("handle Token Packet \n");
    return FALSE;
}

void registerTokenHandler(bool (*handler)(arc_packet_t* packet)){
    printf("register \n");
    if (token_handlers >= MAX_HANDLERS){
        //TODO assert
        return;
    }
    tokenHandlers[token_handlers++] = handler;
}

bool giveTokenHandler(arc_packet_t* packet){
    //printf("GIVE TOKEN HANDLER\n");
    if (packet->packet_id == GIVE_TOKEN){
        //printf("Das war ein Token Packet\n");
        if (given_tokens++ > TIMEOUT_TOKEN){
            amber_state = AMBER_UNREGISTERED;
        }
        //still unregistered
        if (amber_state == AMBER_UNREGISTERED){
            if (packet->system_id == REGISTER_CHANCE){
                printf("REGISTER CHANCE\n");
                if (random_register == 0){
                    sendProtocolPacket(REGISTER);
                    //TODO random_register zufaellig setzen
                } else {
                    random_register--;
                }
            }
        //token to me
        } 
        if (packet->system_id == (ARC_SYSTEM_ID) MY_SYSTEM_ID){
            if (amber_state == AMBER_UNREGISTERED){
                amber_state = AMBER_REGISTERED; 
                printf("GOT THE FIRST TOKEN; NOW REGISTERED\n");
            }
            given_tokens = 0;
            has_token = TRUE;
        }
        return TRUE;
    } else {
        return FALSE;
    }
}

bool giveBackHandler(arc_packet_t* packet){
    //printf("TokenBackHandler");
    if (packet->packet_id == GIVE_TOKEN){
        //Nothing to do while master is not implemented 
        return TRUE;
    } else {
        return FALSE;
    }
}

bool registerPacketHandler(arc_packet_t* packet){
    //printf("Register Packet handler");
    if (packet->packet_id == REGISTER){
        //Nothing to do while master is not implemented 
        return TRUE;
    } else {
        return FALSE;
    }
}
