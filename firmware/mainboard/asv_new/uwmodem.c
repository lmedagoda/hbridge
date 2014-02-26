#include "uwmodem.h"

#include <hbridgeCommon/lib/STM32F10x_StdPeriph_Driver/inc/stm32f10x_can.h>
#include <hbridgeCommon/drivers/can.h>
#include <printf.h>
uwmodem_send_func_t uwmodem_sendFunc;
uwmodem_recv_func_t uwmodem_recvFunc;
uwmodem_seek_func_t uwmodem_seekFunc;

int uwmodem_sendCanData(unsigned char *buffer, const unsigned int size);
void uwmodem_init(uwmodem_send_func_t sendFunc, uwmodem_recv_func_t recvFunc, uwmodem_seek_func_t seekFunc){

    uwmodem_sendFunc = sendFunc;
    uwmodem_recvFunc = recvFunc;
    uwmodem_seekFunc = seekFunc;

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
    }
}
int uwmodem_sendData(unsigned char *data, const unsigned int size){
    return uwmodem_sendFunc(data, size);
}

int uwmodem_sendCanData(unsigned char *buffer, const unsigned int size){
    CanTxMsg msg;
    msg.StdId = 0x501;
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
