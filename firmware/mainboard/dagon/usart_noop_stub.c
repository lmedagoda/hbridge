#include "../../hbridgeCommon/drivers/usart.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/select.h>
#include <sys/ioctl.h>

/**
 * Deinitializes the USART1
 **/

int socked;
struct sockaddr_in si_me, si_other;


void printRawData(const char *source, const unsigned char *data, const unsigned int len){
    printf("%s :",source);
    for(unsigned int i = 0; i < len; i++){
        printf("%c",data[i]);
    }
    printf("\n");
}

void USART1_Init(enum USART_MODE mode)
{
}

signed int USART1_SendData(const unsigned char *data, const unsigned int size)
{
    printRawData("USART1",data,size);
    return size;
}

signed int USART1_SeekData (unsigned char *buffer, const unsigned int buffer_length)
{
    return 0;
}

signed int USART1_GetData (unsigned char *buffer, const unsigned int buffer_length)
{
    return 0;
}

void USART2_Init(enum USART_MODE mode)
{
}

signed int USART2_SendData(const unsigned char *data, const unsigned int size)
{
    
    printRawData("USART2",data,size);
    return size;
}

signed int USART2_SeekData (unsigned char *buffer, const unsigned int buffer_length)
{
    return 0;
}

signed int USART2_GetData (unsigned char *buffer, const unsigned int buffer_length)
{
    return 0;
}


void USART3_Init(enum USART_MODE mode, unsigned int speed)
{
}

signed int USART3_SendData(const unsigned char *data, const unsigned int size)
{
    printRawData("USART3",data,size);
    return size;
}
signed int USART3_SeekData (unsigned char *buffer, const unsigned int buffer_length)
{
    return 0;
}

signed int USART3_GetData (unsigned char *buffer, const unsigned int buffer_length)
{
    return 0;
}


void USART5_Init(enum USART_MODE mode)
{
    int i;
    int intslen=sizeof(si_other);
    char buf[1024];

    if ((socked=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1){
        fprintf(stderr,"Couldnot open socked\n");
        exit(-1);
    }
    memset((char *) &si_me, 0, sizeof(si_me));
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(12345);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(socked, &si_me, sizeof(si_me))==-1){
        fprintf(stderr,"Bind Failed\n");
        exit(-1);
    }
    printf("Bind Successfull\n");


}

signed int USART5_SendData(const unsigned char *data, const unsigned int size)
{
    sendto(socked, data, size, 0, (struct sockaddr *)&si_other, sizeof(si_other));
    printRawData("USART5",data,size);
    return size;
}
signed int USART5_SeekData (unsigned char *buffer, const unsigned int buffer_length)
{
    return USART5_GetData(buffer,buffer_length);
}

signed int USART5_GetData (unsigned char *buffer, const unsigned int buffer_length)
{
    unsigned int bytes_available;
    int slen=sizeof(si_other);
    ioctl(socked,FIONREAD,&bytes_available);
    if(bytes_available > buffer_length){
        bytes_available = buffer_length;
    }
    int read = 0;
    if(bytes_available > 0){
        read = recvfrom(socked, buffer, bytes_available, 0, &si_other, &slen);
        if(read==-1){
            fprintf(stderr,"Read Failed\n");
            exit(-1);
        }
    }
    return read;
}

