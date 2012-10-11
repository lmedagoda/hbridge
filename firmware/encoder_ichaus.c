#include "encoder_ichaus.h"
#include "encoder.h"
#include "printf.h"
#include "inc/stm32f10x_spi.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_gpio.h"
#include "state.h"

#define ERROR_REPETITIONS 3

extern unsigned int systemTick;

int icHausOffset;

void encoderInitIcHaus()
{
    print("encoderInitIcHaus\n");
    
    icHausOffset = 0;
    if(icHausOffset > 4095)
    {
        print("icHausOffset > 4095 - ignoring!\n");
        icHausOffset = 0;
    }
    
    /* SPI2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
	
    //Configure SPI2 pins: SCK, MISO and MOSI
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    SPI_InitTypeDef SPI_InitStructure;

    /* Disable SPI2 for configuration */
    SPI_Cmd(SPI2, DISABLE);

    /* SPI2 Master */
    /* SPI2 Config -----------------------------------------------------------*/
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; // maybe change to 64 for stability
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI2, &SPI_InitStructure);


    /* Enable SPI2 */
    SPI_Cmd(SPI2, ENABLE);
    printf("encoderInitICHaus finished \n ");
}

void setTicksPerTurnIcHaus(uint32_t ticks, uint8_t tickDivider)
{
}

/**
 * Calculate the CRC-Checksum according to protocol specification.
 * @param data The data bits (sensor value + nE + nW)
 * @return The crc6 result
 */

unsigned int bissCRC6(unsigned int data)
{
    unsigned int const poly = 0b1000011;
    uint32_t dividend = data << 6;
    int currPos = 31;
    
    while (currPos >= 6)
    {
        // skip leading zeros
        while ((currPos >= 6) && ((dividend & (1 << currPos)) == 0))
        {
            --currPos;
        }
        if (currPos >= 6)
        {
            dividend = dividend ^ (poly << (currPos - 6));
            --currPos;
        }
    }
    // invert and cut to 6 bits
    return (~dividend) & ((1 << 6) - 1);
}

int isInvalidData_IcHaus(uint32_t data)
{
    unsigned int result = (data >> 13) & 16383;
    int ret = 0;
    static int error_counter = 0;
    if ((data >> 27) != 26) // start sequence 11010
    {
        if (error_counter < ERROR_REPETITIONS)
        {
            print("IcHaus: Data does not start with 11010\n");
        }
        ++error_counter;
        ret = 1;
    }
    else
    {
        if (((data >> 13) & 1) != 1) // nW-bit must be 1
        {
            if (error_counter < ERROR_REPETITIONS)
            {
                print("IcHaus: nW-bit not found\n");
            }
            ++error_counter;
            ret = 2;
        }
        else
        {
            if ((data & 63) != 0) // last 6 bits must be 0
            {
                if (error_counter < ERROR_REPETITIONS)
                {
                    print("IcHaus: Data too long");
                }
                ++error_counter;
                ret = 3;
            }
            else
            {
                unsigned int crc6sensor = (data >> 7) & ((1 << 6) - 1);
                unsigned int crc6 = bissCRC6(result);
                if (crc6 != crc6sensor)
                {
                    if (error_counter < ERROR_REPETITIONS)
                    {
                        print("IcHaus: CRC failed ");
                        printf("was: %du expected: %du  result: %du\n", crc6, crc6sensor, result);
                    }
                    ++error_counter;
                    ret = 4;
                }
            }
        }
    }

    if(error_counter > ERROR_REPETITIONS + 1000)
    {
        error_counter = ERROR_REPETITIONS - 1; // allow message in next step
    }
    else
    {
        if (error_counter > 0)
        {
            --error_counter;
        }
    }
    return ret;
}

uint32_t getTicksIcHaus(void)
{
       print("#");
    static unsigned int lastTick = 0;
    static uint32_t lastValue = 0;

//     if (lastTick == systemTick) // if sensor was already polled in this system tick
//     {
//         return lastValue;
//     }
    /* Wait for SPI1 Tx buffer empty */
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
    {
//        print("*");
    }
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);

    uint16_t dataArray[2] = {0, 0};
    SPI_I2S_SendData(SPI2, 0);

    /* Wait for SPI2 data reception */
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
    {
//              print(".");
    }

    //read current data from data register
    dataArray[1] = SPI_I2S_ReceiveData(SPI2);
    SPI_I2S_SendData(SPI2, 0);

    /* Wait for SPI2 data reception */
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
    {
//               print("_");
    }

    //read current data from data register
    dataArray[0] = SPI_I2S_ReceiveData(SPI2);
    uint32_t value = (dataArray[1] << 16) | dataArray[0];
    unsigned int result = (value >> 15) & 4095;

    lastTick = systemTick;

    static int errorCounter = 0;
    if (isInvalidData_IcHaus(value) != 0)
    {
        print(":-( ");
        if (errorCounter > 50)
        {
	    print("ICHouse is dead\n");
            // some error handling
//             getErrorState()->sensorFailure = 1;
        }
        else
        {
            errorCounter += 2;
        }
        return lastValue;
    }
    else
    {
        if(errorCounter != 0)
        {
            --errorCounter;
        }
    }
    value = (result + icHausOffset);
    if(value > 4095)
    {
        value -= 4096;
    }

    return lastValue = value;
}

uint16_t getTickint16_tIcHaus(void)
{
  return getTicksIcHaus() & 0xffff;
}

uint32_t from16BitIcHaus(uint16_t const ticks)
{
  return ticks;
}
