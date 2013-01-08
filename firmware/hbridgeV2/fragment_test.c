#include "../protocol.h"
#include "../common/protocol_low_priority.h"
#include "../common/protocol.h"
#include "../common/packets.h"

signed int sendfunc(uint16_t senderId, uint16_t receiverId, uint16_t packetId, const unsigned char *data, const unsigned int size){
    protocol_processLowPrio(senderId, receiverId, packetId, data, size);
    return 0;
}



int main(int argc, char **argv){
    protocol_init();
    protocolLowPriority_init();
    struct encoderConfiguration e1c;
        e1c.encoderType = NO_ENCODER;
        e1c.tickDivider = 1;
        e1c.ticksPerTurn = 1;
    
    struct sensorConfig sc;
        sc.encoder1Config = e1c;
        sc.encoder2Config = e1c;
        sc.externalTempSensor = 0;
        sc.statusEveryMs = 5;
    
        struct actuatorConfig fakeConf;
        memset(&fakeConf, 0, sizeof(struct actuatorConfig));
        
        
    protocol_setSendFunc(sendfunc);
    protocol_setMaxPacketSize(8);
    protocol_sendData(RECEIVER_ID_ALL, PACKET_ID_SET_SENSOR_CONFIG, (unsigned char *) &sc, sizeof(struct sensorConfig));
    protocol_sendData(RECEIVER_ID_ALL, PACKET_ID_SET_ACTUATOR_CONFIG, (unsigned char *) &fakeConf, sizeof(struct actuatorConfig));
    protocol_sendData(RECEIVER_ID_ALL, PACKET_ID_REQUEST_STATE, (unsigned char *) 0, 0);

    return 0;
}
