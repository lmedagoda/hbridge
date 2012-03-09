#ifndef __I2C_H
#define __I2C_H

#include "inc/stm32f10x_type.h"
#include "inc/stm32f10x_i2c.h"

struct I2C_Handle;

struct I2C_CommandResult {
    u8 rxData[4];
    u8 rxSize;
    u8 I2CError;
    u32 I2CErrorReason;
};

void setupI2Cx(u16 address, int speed, I2C_TypeDef* I2Cx, FunctionalState remapped);

/**
 * This function resets the I2C bus, this may be called from
 * drivers if they think the bus is faulty and needs a complete
 * reset.
 **/
void resetI2C(struct I2C_Handle *handle);


/**
 * This function returns a handle to the given I2C bus which
 * is needed to perform write or read operations on the bus.
 * */
struct I2C_Handle *I2C_getHandle(I2C_TypeDef* I2Cx);

/**
 * This function will try to start an asynchronious I2C
 * data transfer on the I2C interface. It will write
 * the given data on the bus, addressed to the given
 * address.
 *
 * Note this function may fail.
 *
 * Returns 0 on success and 1 on failure
 */
u8 I2C_writeBytes(struct I2C_Handle *handle, u8 *data, u8 size, u8 addr);
u8 I2C_writeReadBytes(struct I2C_Handle *handle, u8 *txdata, u8 txsize, u8 rxsize, u8 addr);
u8 I2C_readBytes(struct I2C_Handle *handle, u8 size, u8 addr);

/**
 * This function tries to write the given data on 
 * the I2C bus. This function will only return
 * if the write was sucessfull and completed. 
 *
 * Note if this function fails it will block the
 * programm infinite.
 */
void I2C_writeBytesBlocking(struct I2C_Handle *handle, u8 *data, u8 size, u8 addr);

/**
 * Checks if the last issued I2C operation was finished.
 * If this is the case a pointer to an I2C_CommandResult 
 * is returned.
 * 
 * If the operation is still pending 0 is returned
 * */
struct I2C_CommandResult *I2C_getResult(struct I2C_Handle *handle);

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


#endif
