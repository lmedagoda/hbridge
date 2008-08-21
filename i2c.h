#ifndef __I2C_H
#define __I2C_H

#include "stm32f10x_type.h"
#include "stm32f10x_i2c.h"

enum I2CModes {
  I2C_READ,
  I2C_WRITE,
  I2C_WRITE_READ,
  I2C_FINISHED,
};


struct I2C_Data {
  u8 I2C_Buffer_Tx[4];
  u8 I2C_Buffer_Rx[4];
  u8 I2C_Tx_Idx;
  u8 I2C_Rx_Idx;
  u8 I2C_Tx_Size;
  u8 I2C_Rx_Size;
  u8 curI2CAddr;
  u8 I2CMode;
  u8 I2CError;
  u32 I2CErrorReason;
};

extern volatile struct I2C_Data I2C1_Data;
extern volatile struct I2C_Data I2C2_Data;

void I2C_EV_IRQHandler(I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data);
void I2C_ER_IRQHandler(I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data);

u8 I2C1OperationFinished();

void I2CSendBytes(u8 *data, u8 size, u8 addr, I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data);
void I2CReadBytes(u8 size, u8 addr,I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data);
void I2CWriteReadBytes(u8 *txdata, u8 txsize, u8 rxsize, u8 addr, I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data);

void I2C1SendBytes(u8 *data, u8 size, u8 addr);
void I2C1ReadBytes(u8 size, u8 addr);
void I2C1WriteReadBytes(u8 *txdata, u8 txsize, u8 rxsize, u8 addr);

void I2C2SendBytes(u8 *data, u8 size, u8 addr);
void I2C2ReadBytes(u8 size, u8 addr);
void I2C2WriteReadBytes(u8 *txdata, u8 txsize, u8 rxsize, u8 addr);


#endif
