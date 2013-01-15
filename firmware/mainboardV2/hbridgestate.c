#include "hbridgestate.h"
#include "state.h"
#include "bool.h"
#include "../common/hbridge_cmd.h"
#include "../common/protocol.h"
#include "../hbridgeCommon/drivers/printf.h"

int mapState(enum STATES);
void switchTo(int id, int state);

void ackHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size){
    /*struct ackData *ack = (struct ackData*) &data;
    if(ack->packetId == lastPacket)
    {
        lastPacket = 0;
    }*/
    //printf("ack\n");
}

void hbridgestateHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size){
    //printf("StateHandler\n");
    
    struct announceStateData *asd = (struct announceStateData*) data;
    currentState.hbridges[senderId - RECEIVER_ID_H_BRIDGE_1].state = asd->curState;
    currentState.hbridges[senderId - RECEIVER_ID_H_BRIDGE_1].pending = FALSE;
    
}

void initHbridgeState(){
    protocol_registerHandler(PACKET_ID_ANNOUNCE_STATE, &hbridgestateHandler);
    protocol_registerHandler(PACKET_ID_ACK, &ackHandler);
    /*protocol_registerHandler(PACKET_ID_STATUS, &statusHandler);
    protocol_registerHandler(PACKET_ID_EXTENDED_STATUS, &statusHandler);*/
}

void processHbridgestate(){
    int i,j;
    for(i = 0; i < 4; i++){

        //printf("wanted: %i current: %i Pending: %i\n", wantedState.hbridges[i].state, currentState.hbridges[i].state, currentState.hbridges[i].pending);
        
        if(wantedState.hbridges[i].state != currentState.hbridges[i].state && currentState.hbridges[i].pending == FALSE){
            currentState.hbridges[i].pending = TRUE;
                //printf("Brücke: %i\n", i);
            
            
            int wanted;
            if((wanted = mapState(wantedState.hbridges[i].state)) < 0){
                printf("Are you mad?\n");
            }
            int current;
            if((current = mapState(currentState.hbridges[i].state)) < 0){
                    //printf("current %i", current);
                if(wanted != 0){
                    //printf("test: %i\n", wantedState.hbridges[i].state);
                    currentState.hbridges[i].pending = FALSE;
                    //wantedState.hbridges[i].state = currentState.hbridges[i].state;
                    //printf("HBridgeError: %i %i\n", currentState.hbridges[i].state,i);
                //printf("Fehler erkannt\n");
                //if(wanted != 0)
printf("WANTED UNGLEICH 0\n");
                    continue;
                }
                if(currentState.hbridges[i].state == STATE_SENSOR_ERROR){
                    printf("Clean error sensor %i",i);
                    hbridge_sendClearSensorError(i+RECEIVER_ID_H_BRIDGE_1);
                    hbridge_requestState(i+RECEIVER_ID_H_BRIDGE_1);
                    continue;
                }
                if(currentState.hbridges[i].state == STATE_ACTUATOR_ERROR){
                    printf("Clean error actuator%i",i);
                    hbridge_sendClearActuatorError(i+RECEIVER_ID_H_BRIDGE_1);
                   
                    hbridge_requestState(i+RECEIVER_ID_H_BRIDGE_1);
                    //currentState.hbridges[i].state = STATE_SENSOR_ERROR;
                    continue;
                }
            }
            
            if(wanted < current){
                printf("Set %i to UNCONFIGURED\n", i);
                hbridge_setUnconfigured(i+RECEIVER_ID_H_BRIDGE_1);
            } else if(wanted == current){
                continue;
            } else {
                switchTo(i+RECEIVER_ID_H_BRIDGE_1, current+1);
            }
        }
       //printf("DAS I HIER: %i\n", i); 
    }
}
    
void switchTo(int id, int state){
    //printf("switchToState: %i\n", state);
    struct sensorConfig sc;
    struct actuatorConfig ac;
    struct setActiveControllerData cd;
    switch(state){
        case 0:
            //TODO
            break;
        case 1:
            printf("HBrücke %i auf State Sensors\n", id);
            hbridge_sensorStructInit(&sc);
            hbridge_sendSensorConfiguration(id, &sc);
            break;
        case 2:
            printf("HBrücke %i auf State Actuators\n", id);
            hbridge_actuatorStructInit(&ac);
            if(id == RECEIVER_ID_H_BRIDGE_1 || id == RECEIVER_ID_H_BRIDGE_2)
                ac.maxCurrent = 3300;
            hbridge_sendActuatorConfiguration(id, &ac);
            break;
        case 3:
            printf("HBrücke %i auf State Controller\n", id);
            hbridge_controllerStructInit(&cd);
            hbridge_sendControllerConfiguration(id, &cd);
            break;
        case 4:
            printf("HBrücke %i auf State Running\n", id);
            hbridge_setValue(0,0,0,0);
            break;
    }
}
    
    
int mapState(enum STATES state){
    switch(state){
            case STATE_SENSOR_ERROR:
            case STATE_ACTUATOR_ERROR:
                return -1;
            case STATE_UNCONFIGURED:
                return 0;
            case STATE_SENSORS_CONFIGURED:
                return 1;
            case STATE_ACTUATOR_CONFIGURED:
                return 2;
            case STATE_CONTROLLER_CONFIGURED:
                return 3;
            case STATE_RUNNING:
                return 4;
            default: return -2;
    }
}
