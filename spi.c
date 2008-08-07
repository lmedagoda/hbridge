
#include "stm32f10x_type.h"
#include "stm32f10x_it.h"
#include "stm32f10x_gpio.h"
#include "spi.h"

enum LS7366Commands {
  LS7366_CLR = 0,
  LS7366_RD = 64,
  LS7366_WR = 128,
  LS7366_LOAD = 192,
};

enum LS7366Registers {
  LS7366_MDR0 = 8,
  LS7366_MDR1 = 16,
  LS7366_DTR = 24,
  LS7366_CNTR = 32,
  LS7366_OTR = 40,
  LS7366_STR = 48,
  LS7366_NONE = 56,
};

enum LS7366CountingModes {
  LS7366_NON_QUADRATURE_MODE = 0,
  LS7366_X1_QUADRATURE_MODE = 1,
  LS7366_X2_QUADRATURE_MODE = 2,
  LS7366_X4_QUADRATURE_MODE = 3,
};

enum LS7366FilterClockDivisors {
  LS7366_FILTER_DIVISOR_1 = 0,
  LS7366_FILTER_DIVISOR_2 = 128,
};

struct LS7366Data LS7366Data[4];

volatile enum LS7366Devices activeSPIDevice;

volatile u8 LS7366InitDone = 0;
volatile u8 LS7366DataAcquired = 0;

void dataCallback();
void initCallback();

void selectSlave(enum LS7366Devices device);
void deselectSlave(enum LS7366Devices device);
void SPISendBytes(u8 *data, u8 size, enum LS7366Devices device);
void SPISendReadBytes(u8 *txdata, u8 txsize, u8 rxsize, enum LS7366Devices device);


void initLS7366() {
  u8 buffer[2];
  
  //Write to MDR0
  buffer[0] = LS7366_WR | LS7366_MDR0;
  //value
  buffer[1] = LS7366_X2_QUADRATURE_MODE;

  SPI1TransactionComplete = initCallback;

  SPISendBytes(buffer, 2, LS7366_FRONT_LEFT);
}

u8 initLS7366Finished() {
  return LS7366InitDone;
}

u8 LS7366DataReady() {
  return LS7366DataAcquired;
}

void initCallback(void) {
  //deactivate current device
  deselectSlave(activeSPIDevice);

  u8 buffer[2];
  
  //Write to MDR0
  buffer[0] = LS7366_WR | LS7366_MDR0;
  //value
  buffer[1] = LS7366_X2_QUADRATURE_MODE;

  //MSB/LSB test
  //buffer[0] = 0x00010001;
  //buffer[1] = 0x01000000;

  switch(activeSPIDevice) {
  case LS7366_FRONT_LEFT:
    SPISendBytes(buffer, 2, LS7366_FRONT_RIGHT);
    break;
  case LS7366_FRONT_RIGHT:
    SPISendBytes(buffer, 2, LS7366_REAR_LEFT);
    break;
  case LS7366_REAR_LEFT:
    SPISendBytes(buffer, 2, LS7366_REAR_RIGHT);
    break;
  case LS7366_REAR_RIGHT:
    LS7366InitDone = 1;
    
    SPI1TransactionComplete = dataCallback;

    break;
  }
}

u32 MSBtoLSB(u32 x) {
  u8 *buf = (u8 *) &x;
  u32 ret = (*buf << 24) + (*(buf +1) << 16) + (*(buf+2) << 8) + *(buf+3);

  return ret;
}

void requestLS7366Data() {
  u8 buffer[2];
  int i;
  
  LS7366DataAcquired = 0;
  
  for(i = 0; i < 4;i++) {
    LS7366Data[i].lastTicks = LS7366Data[i].curTicks;
  }
  
  //Read CNTR
  //this is equal to Transfer CNTR to OTR and
  //send OTR out
  buffer[0] = LS7366_RD | LS7366_CNTR;
 
 //MSB/LSB test
  //buffer[0] = 0x00000110;
  
  SPISendReadBytes(buffer, 1, 4, LS7366_FRONT_LEFT);
}

void dataCallback() {
  //deactivate current device
  deselectSlave(activeSPIDevice);

  u8 buffer[2];
  
  //Read CNTR
  //this is equal to Transfer CNTR to OTR and
  //send OTR out
  buffer[0] = LS7366_RD | LS7366_CNTR;
 //MSB/LSB test
  //buffer[0] = 0x00000110;

  switch(activeSPIDevice) {
  case LS7366_FRONT_LEFT:
    LS7366Data[activeSPIDevice].curTicks = MSBtoLSB(*((u32 *) SPI1_Buffer_Rx));
    SPISendReadBytes(buffer, 1, 4, LS7366_FRONT_RIGHT);
    break;
  case LS7366_FRONT_RIGHT:
    LS7366Data[activeSPIDevice].curTicks = MSBtoLSB(*((u32 *) SPI1_Buffer_Rx));
    SPISendReadBytes(buffer, 1, 4, LS7366_REAR_LEFT);
    break;
  case LS7366_REAR_LEFT:
    LS7366Data[activeSPIDevice].curTicks = MSBtoLSB(*((u32 *) SPI1_Buffer_Rx));
    SPISendReadBytes(buffer, 1, 4, LS7366_REAR_RIGHT);
    break;
  case LS7366_REAR_RIGHT:
    LS7366Data[activeSPIDevice].curTicks = MSBtoLSB(*((u32 *) SPI1_Buffer_Rx));
    LS7366DataAcquired = 1;
    break;
  }
}

void selectSlave(enum LS7366Devices device) {
  switch(device) {
  case LS7366_FRONT_LEFT:
    GPIO_ResetBits(GPIOC, GPIO_Pin_6);    
    break;    
  case LS7366_FRONT_RIGHT:
    GPIO_ResetBits(GPIOC, GPIO_Pin_7);
    break;    
  case LS7366_REAR_LEFT:
    GPIO_ResetBits(GPIOC, GPIO_Pin_8);
    break;    
  case LS7366_REAR_RIGHT:
    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
    break;
  }
  activeSPIDevice = device;
  //SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);

}

void deselectSlave(enum LS7366Devices device) {
  switch(device) {
  case LS7366_FRONT_LEFT:
    GPIO_SetBits(GPIOC, GPIO_Pin_6);
    break;    
  case LS7366_FRONT_RIGHT:
    GPIO_SetBits(GPIOC, GPIO_Pin_7);
    break;    
  case LS7366_REAR_LEFT:
    GPIO_SetBits(GPIOC, GPIO_Pin_8);
    break;    
  case LS7366_REAR_RIGHT:
    GPIO_SetBits(GPIOC, GPIO_Pin_9);
    break;
  }
  //SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Reset);

}

void SPISendBytes(u8 *data, u8 size, enum LS7366Devices device) {
  SPI1_Tx_Idx = 0;
  SPI1_Tx_Size = size;

  SPI1Mode = SPI_WRITE;

  int i;
  
  for(i = 0; i < size; i++) {
    SPI1_Buffer_Tx[i] = data[i];
  }

  selectSlave(device);

  SPI_I2S_SendData(SPI1, SPI1_Buffer_Tx[SPI1_Tx_Idx]);
  SPI1_Tx_Idx++;

  /* Enable SPI1 TXE interrupt */
  SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
}

void SPISendReadBytes(u8 *txdata, u8 txsize, u8 rxsize, enum LS7366Devices device) {
  SPI1_Tx_Idx = 0;
  SPI1_Tx_Size = txsize;

  SPI1_Rx_Idx = 0;
  SPI1_Rx_Size = rxsize;

  SPI1Mode = SPI_WRITE_READ;

  int i;
  
  for(i = 0; i < txsize; i++) {
    SPI1_Buffer_Tx[i] = txdata[i];
  }

  selectSlave(device);

  SPI_I2S_SendData(SPI1, SPI1_Buffer_Tx[SPI1_Tx_Idx]);
  SPI1_Tx_Idx++;

  /* Enable SPI1 TXE interrupt */
  SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
}
