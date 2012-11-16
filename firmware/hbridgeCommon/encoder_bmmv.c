#include "drivers/printf.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_spi.h"
#include "inc/stm32f10x_gpio.h"
#include "encoder.h"

extern unsigned int systemTick;
struct encoderData externalEncoderConfig;
static volatile uint8_t configured = 0;

// Baumer multiturn absolute encoder
void encoderInitBMMV()
{
    print("encoderInitBMMV\n");
    /* SPI2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

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
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; // maybe change to 64 for stability
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI2, &SPI_InitStructure);


    /* Enable SPI2 */
    SPI_Cmd(SPI2, ENABLE);

     // TODO: Does something need be changed here?
    externalEncoderConfig.tickDivider = 1;
    externalEncoderConfig.ticksPerTurn = 0;
  
}

void setTicksPerTurnBMMV(uint32_t ticks, uint8_t tickDivider)
{
    if(configured && externalEncoderConfig.ticksPerTurn == ticks && externalEncoderConfig.tickDivider == tickDivider)
	return;

    externalEncoderConfig.tickDivider = tickDivider;
    externalEncoderConfig.ticksPerTurn = ticks;
    configured = 1;
}

uint32_t getTicksBMMV()
{
    static unsigned int lastTick = 0;
    static uint32_t lastValue = 0;
    
    if(lastTick == systemTick) // if sensor was already polled in this system tick
    {
        return lastValue;
    }
    
    /* Wait for SPI1 Tx buffer empty */
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
    {
  //      print("*");
    }

    GPIO_ResetBits(GPIOB, GPIO_Pin_12);

    uint16_t dataArray[2] = {0,0};
    SPI_I2S_SendData(SPI2, 0);

    /* Wait for SPI2 data reception */
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
    {
  //      print(".");
    }

    //read current data from data register
    dataArray[1] = SPI_I2S_ReceiveData(SPI2) & ~0x8000; // first bit is not from encoder, discard it

    SPI_I2S_SendData(SPI2, 0);

    /* Wait for SPI2 data reception */
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
    {
 //       print("_");
    }

    //read current data from data register
    dataArray[0] = SPI_I2S_ReceiveData(SPI2);

    uint32_t value = (dataArray[1] << 16) | dataArray[0];
    value = value >> 6; // align the 25 data bits

    lastTick = systemTick;
    return lastValue = value;
}

uint16_t getDividedTicksBMMV()
{
     // TODO: Implement more useful algorithm?
    uint32_t ticks = getTicksBMMV() / externalEncoderConfig.tickDivider;
    return ticks & 4095; // limit to 12 bit
}

