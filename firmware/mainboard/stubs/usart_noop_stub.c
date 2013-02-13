#include "../../hbridgeCommon/drivers/usart.h"

/**
 * Deinitializes the USART1
 **/
void USART1_Init(enum USART_MODE mode)
{
}

signed int USART1_SendData(const unsigned char *data, const unsigned int size)
{
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

void USART3_Init(enum USART_MODE mode)
{
}

signed int USART3_SendData(const unsigned char *data, const unsigned int size)
{
    return size;
}

signed int USART3_GetData (unsigned char *buffer, const unsigned int buffer_length)
{
    return 0;
}

