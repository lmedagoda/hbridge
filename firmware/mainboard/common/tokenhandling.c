#include "arc_driver.h"
#include "tokenhandling.h"
#include "printf.h"
//How many Tokens wasn't my Token since my last Token
int given_tokens;
int random_register = 0;

token_handler_t tokenHandlers[MAX_HANDLERS];

#define TRUE 1
#define FALSE 0

int token_handlers = 0;
uint8_t giveTokenHandler(arc_packet_t* packet);
uint8_t giveBackHandler(arc_packet_t* packet);
uint8_t registerPacketHandler(arc_packet_t* packet);

void initTokenhandling(){
    registerTokenHandler(giveTokenHandler);
    registerTokenHandler(registerPacketHandler);
    registerTokenHandler(giveBackHandler);
}

uint8_t handleTokenPacket(arc_packet_t* packet){
    //printf("handle Token Packet \n");
    int i = 0;
        //printf("tokenhandlers: %i \n", token_handlers); 
    token_handlers = 3;
    for (i=0; i < token_handlers; i++){
        //printf("i: %i \n", i); 
        if (tokenHandlers[i](packet) == TRUE){
            return TRUE;
        }
    }
    //printf("handle Token Packet \n");
    return FALSE;
}

void registerTokenHandler(token_handler_t handler){
    if (token_handlers >= MAX_HANDLERS){
        //TODO assert
        return;
    }
    tokenHandlers[token_handlers++] = handler;
}

uint8_t giveTokenHandler(arc_packet_t* packet){
    //printf("GIVE TOKEN HANDLER\n");
    if (packet->packet_id == MB_GIVE_TOKEN){
        //printf("Das war ein Token Packet\n");
        if (given_tokens++ > TIMEOUT_TOKEN){
            amber_state = AMBER_UNREGISTERED;
        }
        //still unregistered
        if (amber_state == AMBER_UNREGISTERED){
            if (packet->system_id == REGISTER_CHANCE){
                if (random_register == 0){
                    arc_sendProtocolPacket(MB_REGISTER);
                    printf("Tried register with ID %i\n", MY_SYSTEM_ID); 
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
        if (amber_state == AMBER_UNREGISTERED){
           //while i'm unregistered. i hearing only to Registerchance packets
           //This Packet isn't a registerchance, so it's handled
            return TRUE;
        }
        return FALSE;
    }
}

uint8_t giveBackHandler(arc_packet_t* packet){
    //printf("TokenBackHandler");
    if (packet->packet_id == MB_GIVE_TOKEN){
        //Nothing to do while master is not implemented 
        return TRUE;
    } else {
        return FALSE;
    }
}

uint8_t registerPacketHandler(arc_packet_t* packet){
    //printf("Register Packet handler");
    if (packet->packet_id == MB_REGISTER){
        //Nothing to do while master is not implemented 
        return TRUE;
    } else {
        return FALSE;
    }
}
