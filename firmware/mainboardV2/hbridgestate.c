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
    printf("ack");
}

void hbridgestateHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size){
    printf("senderId: %i\n", senderId);
    printf("receiverId: %i\n", receiverId);
    
    struct announceStateData *asd = (struct announceStateData*) data;
    
    currentState.hbridges[senderId - 1].state = asd->curState;
    currentState.hbridges[senderId - 1].pending = FALSE;
}

void initHbridgeState(){
    protocol_registerHandler(PACKET_ID_ANNOUNCE_STATE, &hbridgestateHandler);
    protocol_registerHandler(PACKET_ID_ACK, &ackHandler);
    /*protocol_registerHandler(PACKET_ID_STATUS, &statusHandler);
    protocol_registerHandler(PACKET_ID_EXTENDED_STATUS, &statusHandler);*/
}

void processHbridgestate(){
    int i;
    for(i = 0; i < 4; i++){
               
        if(wantedState.hbridges[i].state != currentState.hbridges[i].state && currentState.hbridges[i].pending == FALSE){
            currentState.hbridges[i].pending = TRUE;
            
            int wanted;
            if((wanted = mapState(wantedState.hbridges[i].state) < 0)){
                printf("Are you mad?\n");
            }
            int current;
            if((current = mapState(currentState.hbridges[i].state) < 0)){
                if(wanted != 0)
                    return;
            }
            
            
            if(wanted < current){
                //TODO Unconfigure
            } else {
                switchTo(i+1, wanted+1);
            }
        }
    }
}
    
void switchTo(int id, int state){
    printf("switchToState: %i\n", state);
    struct sensorConfig sc;
    struct actuatorConfig ac;
    struct setActiveControllerData cd;
    switch(state){
        case 0:
            //TODO
            break;
        case 1:
            
            hbridge_sensorStructInit(&sc);
            hbridge_sendSensorConfiguration(id, &sc);
            break;
        case 2:
            hbridge_actuatorStructInit(&ac);
            hbridge_sendActuatorConfiguration(id, &ac);
            break;
        case 3:
            hbridge_controllerStructInit(&cd);
            hbridge_sendControllerConfiguration(id, &cd);
            break;
        case 4:
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
            default: return -1;
    }
}
