#include "uwmodem.h"

#include <hbridgeCommon/lib/STM32F10x_StdPeriph_Driver/inc/stm32f10x_can.h>
#include <hbridgeCommon/drivers/can.h>
#include <printf.h>
#define MAX_SURFACE_COUNT 1
uwmodem_send_func_t uwmodem_sendFunc;
uwmodem_recv_func_t uwmodem_recvFunc;
uwmodem_seek_func_t uwmodem_seekFunc;
uwmodem_surface_func_t uwmodem_surfaceFunc;

int uwmodem_sendCanData(unsigned char *buffer, const unsigned int size);
int uwmodem_surfaceCheck(unsigned char *buffer, const unsigned int size);
int surface_count;
void uwmodem_init(uwmodem_send_func_t sendFunc, uwmodem_recv_func_t recvFunc, uwmodem_seek_func_t seekFunc, uwmodem_surface_func_t surfaceFunc){

    uwmodem_sendFunc = sendFunc;
    uwmodem_recvFunc = recvFunc;
    uwmodem_seekFunc = seekFunc;
    uwmodem_surfaceFunc = surfaceFunc;
    surface_count = 0;

}
void uwmodem_handleIncommingData(){
    unsigned char buffer[50];
    int seek;
    while ((seek = uwmodem_seekFunc(buffer, 50)) >= 1){
        int send_bytes = 0;
        while (send_bytes < seek){
            printf(".");
            int fragmentSize = 8;
            if ((seek-send_bytes) < 8)
                fragmentSize = seek - send_bytes;
            int ret = uwmodem_sendCanData(buffer+send_bytes, fragmentSize);
            if (ret<0)
                return;
            else
                send_bytes += seek;
        }
        uwmodem_recvFunc(buffer, seek);
        if (uwmodem_surfaceCheck(buffer, seek)){
            uwmodem_surfaceFunc();
        }
    }
}

int uwmodem_surfaceCheck(unsigned char *buffer, const unsigned int size){
    int i;
    for (i=0; i<size; i++){
        if (buffer[i] == 0xAA || buffer[i] == 0x55){
            surface_count++; 
            if (surface_count >= MAX_SURFACE_COUNT){
                return 1;
            }
        } else {
            surface_count = 0;
        }
    }
    return 0;
}
int uwmodem_sendData(unsigned char *data, const unsigned int size){
    return uwmodem_sendFunc(data, size);
}

int uwmodem_sendCanData(unsigned char *buffer, const unsigned int size){
    CanTxMsg msg;
    msg.StdId = 0x504;
    msg.DLC = size;
    int i;
    for(i=0; i < size; i++){
        msg.Data[i] = buffer[i];
    }
    CAN_SendMessage(&msg);
    printf("send a can packet with data from the uwmoden (size: %i)\n", size);
    return size;
}
void uwmodem_process(){
    uwmodem_handleIncommingData();
}
