#ifndef __I2C_H
#define __I2C_H

#include "inc/stm32f10x_type.h"
#include "inc/stm32f10x_i2c.h"

enum I2CModes {
    I2C_READ,
    I2C_WRITE,
    I2C_WRITE_READ,
    I2C_FINISHED,
};

enum I2CState {
    START_WRITTEN,
    ADDRESS_WRITTEN,
    HANDLING_DATA,
    STOP_WRITTEN,
};


struct I2C_Data {
    enum I2CState state;
    u8 I2C_Buffer_Tx[4];
    u8 I2C_Buffer_Rx[4];
    u8 I2C_Tx_Idx;
    u8 I2C_Rx_Idx;
    u8 I2C_Tx_Size;
    u8 I2C_Rx_Size;
    u8 I2CMode;
    u8 I2CError;
    u32 I2CErrorReason;
    // counter to detect i2c interrupt storm (flooding)
    u32 i2cSafetyCounter;
    u8 curI2CAddr;
    int curI2CSpeed;
    FunctionalState curI2CIsRemapped;
};

/*extern volatile u16 dbgWrite;
extern volatile u16 dbgRead;
extern volatile u32 dbgBuffer[dbgBufferSize];
*/
extern volatile struct I2C_Data I2C1_Data;
extern volatile struct I2C_Data I2C2_Data;

void printfI2CDbg();
void I2C_EV_IRQHandler(I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data);
void I2C_ER_IRQHandler(I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data);

u8 I2C1OperationFinished();

/**
 * This function checks weather the current i2c operation 
 * e.g. I2C2SendBytes, I2C2ReadBytes or I2C2WriteReadBytes
 * was finished.
 *
 * returns 0 if unfinished and !0 if finished
 */
u8 I2C2OperationFinished();

u8 handleI2CxErrors(I2C_TypeDef* I2Cx, volatile struct I2C_Data* I2Cx_Data);
u8 I2CSendBytes(u8 *data, u8 size, u8 addr, I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data);
u8 I2CReadBytes(u8 size, u8 addr,I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data);
u8 I2CWriteReadBytes(u8 *txdata, u8 txsize, u8 rxsize, u8 addr, I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data);
u8 I2CxOperationFinished(I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data);
void setupI2Cx(u16 address, int speed, I2C_TypeDef* I2Cx, FunctionalState remapped);

u8 I2C1SendBytes(u8 *data, u8 size, u8 addr);
u8 I2C1ReadBytes(u8 size, u8 addr);
u8 I2C1WriteReadBytes(u8 *txdata, u8 txsize, u8 rxsize, u8 addr);

/**
 * This function will try to start an asynchronious I2C
 * data transfer on the I2C2 interface. It will write
 * the given data on the bus, addressed to the given
 * address.
 *
 * Note this function may fail.
 *
 * Returns 0 on success and 1 on failure
 */
u8 I2C2SendBytes(u8 *data, u8 size, u8 addr);
u8 I2C2ReadBytes(u8 size, u8 addr);
u8 I2C2WriteReadBytes(u8 *txdata, u8 txsize, u8 rxsize, u8 addr);

/**
 * This function tries to write the given data on 
 * the I2C2 bus. This function will only return
 * if the write was sucessfull and completed. 
 *
 * Note if this function fails it will block the
 * programm infinite.
 */
void I2C2SendBytesBlocking(u8 *data, u8 size, u8 addr);

/**
 * This function sets up the I2C2 bus with the given
 * Adress as its own address (for receving data) and
 * the given speed. It will also configure the IO Pins,
 * activate the interrupts and the RCC.
 */
void setupI2C2(u16 address, int speed);

/**
 * This function sets up the I2C1 bus with the given
 * Adress as its own address (for receving data) and
 * the given speed. It will also configure the IO Pins,
 * activate the interrupts and the RCC.
 */
void setupI2C1(u16 address, int speed, FunctionalState remapped);

/**
 * This function checks weather an error occured during the
 * current I2C operation and tries to handle it. If an error
 * occured the the current operation is canceled and needs
 * to be restarted.
 *
 * Known Bugs: In case of Bus error the recovery does not allways work
 *
 * Return 1 if an error occured 0 otherwise
 */
u8 handleI2C2Errors();

/**
 * This function checks weather an error occured during the
 * current I2C operation and tries to handle it. If an error
 * occured the the current operation is canceled and needs
 * to be restarted.
 *
 * Known Bugs: In case of Bus error the recovery does not allways work
 *
 * Return 1 if an error occured 0 otherwise
 */
u8 handleI2C1Errors();


#endif
