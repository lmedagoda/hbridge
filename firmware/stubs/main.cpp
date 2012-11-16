
extern "C" {
#include "../common/systick.h"
#include "../interfaces/encoder.h"
}

signed int firmwareSendPacket(uint16_t senderId, uint16_t packetId, const unsigned char *data, const unsigned int size);
signed int firmwareReceivePacket(uint16_t *senderId, uint16_t *packetId, unsigned char *data, const unsigned int dataSize);

void encodersStubInit();

int main(int argc, char **argv)
{
    baseInit();
    
    //set implementation for available encoders
    encodersStubInit();
    
    //set implementation for sending of packets
    protocol_setSendFunc(firmwareSendPacket);
    protocol_setRecvFunc(firmwareReceivePacket);
    
        
    platformInit();
    
    run();
}