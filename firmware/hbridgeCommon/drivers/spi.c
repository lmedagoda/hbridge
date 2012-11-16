
#include "stm32f10x_it.h"
#include "inc/stm32f10x_gpio.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_spi.h"
#include "spi.h"

volatile uint8_t SPI1_Buffer_Tx[8];
volatile uint8_t SPI1_Buffer_Rx[8];

volatile uint8_t SPI1_Tx_Size = 0;
volatile uint8_t SPI1_Rx_Size = 0;
volatile uint8_t SPI1_Tx_Idx = 0;
volatile uint8_t SPI1_Rx_Idx = 0;

volatile uint8_t SPI1Mode;

void (* volatile SPI1TransactionComplete) (void) = 0;


void dataCallback();
void initCallback();

void SPISendBytes(uint8_t *data, uint8_t size, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin );
void SPISendReadBytes(uint8_t *txdata, uint8_t txsize, uint8_t rxsize, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

/*******************************************************************************
* Function Name  : SPI_Configuration
* Description    : Configures the SPI interface.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_Configuration(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);
    
  //Configure SPI2 pins: SCK, MISO and MOSI
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

    
  //TODO is SOFT_NSS suficient

  /* SPI1 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  SPI_InitTypeDef SPI_InitStructure;

  /* Disable SPI1 for configuration */
  SPI_Cmd(SPI2, DISABLE);

  /* SPI1 Master */
  /* SPI2 Config -----------------------------------------------------------*/
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);


  /* Enable SPI1 */
  SPI_Cmd(SPI2, ENABLE);
}

uint32_t MSBtoLSB(uint32_t x) {
  uint8_t *buf = (uint8_t *) &x;
  uint32_t ret = (*buf << 24) + (*(buf +1) << 16) + (*(buf+2) << 8) + *(buf+3);

  return ret;
}

void selectSlave(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    GPIO_ResetBits(GPIOx, GPIO_Pin);
//  activeSPIDevice = device;
  //SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);

}

void deselectSlave(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    GPIO_SetBits(GPIOx, GPIO_Pin);
  //SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Reset);

}

void SPISendBytes(uint8_t *data, uint8_t size, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
  SPI1_Tx_Idx = 0;
  SPI1_Tx_Size = size;

  SPI1Mode = SPI_WRITE;

  int i;
  
  for(i = 0; i < size; i++) {
    SPI1_Buffer_Tx[i] = data[i];
  }

  selectSlave(GPIOx, GPIO_Pin);

  SPI_I2S_SendData(SPI1, SPI1_Buffer_Tx[SPI1_Tx_Idx]);
  SPI1_Tx_Idx++;

  /* Enable SPI1 TXE interrupt */
  SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
}

void SPISendReadBytes(uint8_t *txdata, uint8_t txsize, uint8_t rxsize, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
  SPI1_Tx_Idx = 0;
  SPI1_Tx_Size = txsize;

  SPI1_Rx_Idx = 0;
  SPI1_Rx_Size = rxsize;

  SPI1Mode = SPI_WRITE_READ;

  int i;
  
  for(i = 0; i < txsize; i++) {
    SPI1_Buffer_Tx[i] = txdata[i];
  }

  selectSlave(GPIOx, GPIO_Pin);

  SPI_I2S_SendData(SPI1, SPI1_Buffer_Tx[SPI1_Tx_Idx]);
  SPI1_Tx_Idx++;

  /* Enable SPI1 TXE interrupt */
  SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
}

/*******************************************************************************
* Function Name  : SPI1_IRQHandler
* Description    : This function handles SPI1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI1_IRQHandler(void)
{ 
  if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE)) {
 
    if(SPI1_Tx_Idx < SPI1_Tx_Size) {
      /* Send SPI1 data */
      SPI_I2S_SendData(SPI1, SPI1_Buffer_Tx[SPI1_Tx_Idx]);
      SPI1_Tx_Idx++; 
    } else {
      if(SPI1Mode == SPI_WRITE) {
	SPI1Mode = SPI_FINISHED;
	
	/* Disable SPI1 TXE interrupt */
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);
      
	if(SPI1TransactionComplete)
	  SPI1TransactionComplete();
      }
      //start "reading" instantly after last byte written
      if(SPI1Mode == SPI_WRITE_READ) {
	/* Disable SPI1 TXE interrupt */
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);
	SPI1Mode = SPI_READ;

	//read current data from data register
	SPI_I2S_ReceiveData(SPI1);

	//Enable SPI1 RXNE interrupt 
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);

	//send garbage, so that the clock pulses and we can receive data
	SPI_I2S_SendData(SPI1, 0);
      }

    }
  }
  
  if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE)) {

    SPI1_Buffer_Rx[SPI1_Rx_Idx] = SPI_I2S_ReceiveData(SPI1);
    SPI1_Rx_Idx++;
    if(SPI1_Rx_Idx >= SPI1_Rx_Size) {
      /* Disable SPI1 RXNE interrupt */
      SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, DISABLE);
      
      if(SPI1Mode == SPI_READ) {
	SPI1Mode = SPI_FINISHED;
	
	if(SPI1TransactionComplete)
	  SPI1TransactionComplete();
      }
    } else {
      //send garbage, so that the clock pulses and we can receive data
      SPI_I2S_SendData(SPI1, 0);
    }
  }
}
