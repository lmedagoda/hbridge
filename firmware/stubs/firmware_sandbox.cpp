extern "C" {   
#include "common/systick.h"
#include "interfaces/current_measurement.h"
#include "interfaces/encoder.h"
#include <stdlib.h>
}

#include <iostream>

signed int firmwareSendPacket(uint16_t senderId, uint16_t receiverId, uint16_t packetId, const unsigned char *data, const unsigned int size);
signed int firmwareReceivePacket(uint16_t *senderId, uint16_t *receiverId, uint16_t *packetId, unsigned char *data, const unsigned int dataSize);

void encodersStubInit();

int fw_main(void)
{
    //note, this mesage can only be send AFTER usart configuration
    std::cout << "Entered main loop\n" << std::endl;

    //init basic functionality
    //read address, turn on peripherals etc.
    baseInit();
    
    //set implementation for available encoders
    encodersStubInit();
    
    //set implementation for sending of packets
    protocol_setSendFunc(firmwareSendPacket);
    protocol_setRecvFunc(firmwareReceivePacket);
    protocol_setMaxPacketSize(8);
        
    platformInit();

    std::cout << "Peripheral configuration finished\n" << std::endl;

    run();
    
    

 
 
    while(1) {

    }
    
    return 0;
}
