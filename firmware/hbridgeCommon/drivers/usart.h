#ifndef __USART_H
#define __USART_H


enum USART_MODE
{
    USART_POLL,
    USART_USE_INTERRUPTS,
};

/**
 * Initializes the USART 1 with 115200 Baud
 * 
 * The parameter mode defines wheather the driver will use 
 * an interrupt to send and receive the data.
 **/
void USART1_Init(enum USART_MODE mode);

/**
 * Deinitializes the USART1
 **/
void USART1_DeInit(void);

/**
 * Sends data out via USART1. 
 * 
 * Arguments are a pointer to the data and the size of the
 * data to be send.
 *
 * returns nr ob bytes send or -1 in case the data could not be sent.
 **/
signed int USART1_SendData(const unsigned char *data, const unsigned int size);

/**
 * Receives data via USART1
 *
 * Arguments are a pointer to an array were the received data
 * should be saved. Also the maximum number of bytes which may
 * be stored in the array, is given.
 *
 * Note this function will only work if the driver uses the 
 * interrupt mode.
 *
 * Returns the amount of received bytes. Or -1 in case that
 * the internal receive buffer overrun. 
 **/
signed int USART1_GetData (unsigned char *buffer, const unsigned int buffer_length);

/**
 * This function get a copy of the data that is currently available in 
 * the receive buffer. This data will not be removed from the receive
 * buffer.
 * 
 * Note this function will only work if the driver uses the 
 * interrupt mode.
 *
 * Returns the amount of received bytes. Or -1 in case that
 * the internal receive buffer overrun. 
 * */
signed int USART1_SeekData (unsigned char *buffer, const unsigned int buffer_length);



void USART3_Init(enum USART_MODE mode);
void USART3_DeInit(void);
signed int USART3_SendData(const unsigned char *data, const unsigned int size);
signed int USART3_GetData (unsigned char *buffer, const unsigned int buffer_length);
signed int USART3_SeekData (unsigned char *buffer, const unsigned int buffer_length);

void USART2_Init(enum USART_MODE mode);
// void USART2_DeInit(void); //Not implemented so far
signed int USART2_SendData(const unsigned char *data, const unsigned int size);
signed int USART2_GetData (unsigned char *buffer, const unsigned int buffer_length);
signed int USART2_SeekData (unsigned char *buffer, const unsigned int buffer_length);
#ifdef STM32F10X_HD
void USART5_Init(enum USART_MODE mode);
#endif 

//void UART5_DeInit(void); //Not implemented so far
signed int USART5_SendData(const unsigned char *data, const unsigned int size);
signed int USART5_GetData (unsigned char *buffer, const unsigned int buffer_length);
signed int USART5_SeekData (unsigned char *buffer, const unsigned int buffer_length);


#endif
