
#include "i2c.h"
#include "inc/stm32f10x_i2c.h"
#include "stm32f10x_it.h"
#include "printf.h"

volatile struct I2C_Data I2C1_Data;
volatile struct I2C_Data I2C2_Data;

volatile int ledcounter = 0;
vu32 i2cSafetyCounter = 0;


#define I2C_SMBALERT (1<<15)
#define I2C_TIMEOUT (1<<14)
#define I2C_PECERR (1<<12)
#define I2C_OVR (1<<11)
#define I2C_AF (1<<10)
#define I2C_ARLO (1<<9)
#define I2C_BERR (1<<8)
#define I2C_TxE (1<<7)
#define I2C_RxNE (1<<6)
#define I2C_STOPF (1<<4)
#define I2C_ADD10 (1<<3)
#define I2C_BTF (1<<2)
#define I2C_ADDR (1<<1)
#define I2C_SB (1<<0)
#define I2C_DUALF (1<<7)
#define I2C_SMBHOST (1<<6)
#define I2C_SMBDEFAULT (1<<5)
#define I2C_GENCALL (1<<4)
#define I2C_TRA (1<<2)
#define I2C_BUSY (1<<1)
#define I2C_MSL (1<<0)

extern vu32 wasini2cit;

/*******************************************************************************
* Function Name  : I2C2_ER_IRQHandler
* Description    : This function handles I2C2 Error interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C2_ER_IRQHandler(void)
{
  I2C_ER_IRQHandler(I2C2, &I2C2_Data);
}

struct i2cDebug {
    u16 sr1;
    u16 sr2;
    u8 state;
    u8 mode;
    u8 idx;
    u8 size;
    u16 cnt;
    u8 error;
};

#define dbgBufferSize 200

volatile u16 dbgWrite = 0;
volatile u16 dbgRead = 0;
volatile struct i2cDebug dbgBuffer[dbgBufferSize];

void printfI2CDbg() {
    while(dbgWrite != dbgRead) {
	volatile struct i2cDebug *dbg = &(dbgBuffer[dbgRead]);
	dbgRead = (dbgRead +1 ) % dbgBufferSize;
	printf("cnt %hu, SR1 %hu, SR2 %hu, S %hu, M %hu, idx %hu, size %hu, error %hu\n", dbg->cnt, dbg->sr1, dbg->sr2, dbg->state, dbg->mode, dbg->idx, dbg->size, dbg->error);
    }
}

void I2C_EV_IRQHandler(I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data)
{
    wasini2cit = 1;
    u16 sr1 = I2Cx->SR1;
    u16 sr2 = I2Cx->SR2;
    static u16 cnt = 0;

    //flush debug buffer
    if(i2cSafetyCounter == 0)
	dbgWrite = dbgRead;

    i2cSafetyCounter++;

    
    if(i2cSafetyCounter > 120) {
	I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
    }
    
    vu8 nextRxWritePointer = (dbgWrite + 1) % dbgBufferSize;

    if(nextRxWritePointer != dbgRead) {
	volatile struct i2cDebug *dbg = &(dbgBuffer[dbgWrite]);
	dbg->sr1 = sr1;
	dbg->sr2 = sr2;
	dbg->state = I2Cx_Data->state;
	dbg->mode = I2Cx_Data->I2CMode;
	dbg->idx = I2Cx_Data->I2C_Rx_Idx;
	dbg->size = I2Cx_Data->I2C_Rx_Size;
	dbg->cnt = cnt;
	dbg->error = I2Cx_Data->I2CError;
	dbgWrite = nextRxWritePointer;
    }
    
    cnt++;
    
//    if(I2Cx_Data->I2CError)
//	I2Cx_Data->I2CMode = I2C_FINISHED;
    //printf("SR1 %hu, SR2 %hu S %hu, M %hu, idx %hu, size %hu\n", sr1, sr2, I2Cx_Data->state, mode, idx, size);
    
    /*if(cnt > 100) {
	I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
    }
    cnt++;
    */
    switch(I2Cx_Data->state) {
	case START_WRITTEN:
	    if(sr1& I2C_SB) {
		//we are master
		//write adress of slave
		if(I2Cx_Data->I2CMode == I2C_READ) {
		    // Send I2C2 slave Address for write
		    I2C_Send7bitAddress(I2Cx, I2Cx_Data->curI2CAddr, I2C_Direction_Receiver);
		} else {
		    if(I2Cx_Data->I2CMode != I2C_FINISHED) {//I2Cx_Data->I2CMode == I2C_WRITE || I2Cx_Data->I2CMode == I2C_WRITE_READ 
			// Send I2C2 slave Address for write
			I2C_Send7bitAddress(I2Cx, I2Cx_Data->curI2CAddr, I2C_Direction_Transmitter);
		    }
		}
		//enable buf it, so we get an interrupt if TxE or RxNE
		I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);

		I2Cx_Data->state = ADDRESS_WRITTEN;
	    }
	    break;
	case ADDRESS_WRITTEN:
	    //advance state for debug purposes
	    if(sr1 & I2C_ADDR) {
		I2Cx_Data->state = HANDLING_DATA;
		
	    }
	    //no break here by purpose
	case HANDLING_DATA:
	    //this basicly means, we are not in transmission
	    //mode, but the chip is waiting for data to transmit
	    //as we had this error, we are goint to handle this case
	    if(sr1 & I2C_TxE && !(sr2 & I2C_TRA)) {
		I2C_SendData(I2Cx, 0);		
	    }
	    
	    if(sr2 & I2C_TRA) {
		//Transmitter
		if(sr1 & I2C_TxE || sr1 & I2C_BTF) {
		    if(I2Cx_Data->I2C_Tx_Idx < I2Cx_Data->I2C_Tx_Size) {
			/* Transmit buffer data */
			I2C_SendData(I2Cx, I2Cx_Data->I2C_Buffer_Tx[I2Cx_Data->I2C_Tx_Idx]);
			I2Cx_Data->I2C_Tx_Idx++;
		    } else {
			if(I2Cx_Data->I2CMode == I2C_WRITE) {
			/* Disable I2C1 BUF interrupts */
			I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);
			// Send STOP Condition
			I2C_GenerateSTOP(I2Cx, ENABLE);

			//write some data to DR, so that BTF is never set
			//note this data is NOT sended
			I2C_SendData(I2Cx, 0);

			I2Cx_Data->I2CMode = I2C_FINISHED;
			} else {
			    if(I2Cx_Data->I2CMode == I2C_WRITE_READ) {
				//Write phase finished, switch to read
				I2Cx_Data->I2CMode = I2C_READ;

				/* Disable I2C1 BUF interrupts */
				I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);
				
				/* Send I2C1 START condition */
				I2C_GenerateSTART(I2Cx, ENABLE);	  

				//set state back to start written
				I2Cx_Data->state = START_WRITTEN;
				
				//write some data to DR, so that BTF is never set
				//note this data is NOT sended
				I2C_SendData(I2Cx, 0);
			    } else {
				//handle TxE or BTF in weired state
				I2C_SendData(I2Cx, 0);
			    }
			}
		    }
		}
	    } else {
		//receiver
		//for one byte receiving ack needs to be disabled just after writing the address
		if(I2Cx_Data->I2C_Rx_Size == 1 && sr1 & I2C_ADDR) {
		    // Disable Acknowledgement
		    I2C_AcknowledgeConfig(I2Cx, DISABLE);

		    // Send STOP Condition
		    I2C_GenerateSTOP(I2Cx, ENABLE);
		}

		//read data if rx is not empty
		if(sr1 & I2C_RxNE || sr1 & I2C_BTF) {
		    if(I2Cx_Data->I2C_Rx_Idx < I2Cx_Data->I2C_Rx_Size) {
			I2Cx_Data->I2C_Buffer_Rx[I2Cx_Data->I2C_Rx_Idx] = I2C_ReceiveData(I2Cx);
			I2Cx_Data->I2C_Rx_Idx++;
			//set mode to finish if we got all bytes
			if(I2Cx_Data->I2C_Rx_Idx >= I2Cx_Data->I2C_Rx_Size)
			    I2Cx_Data->I2CMode = I2C_FINISHED;	
		    } else {
			//clear RxNE
			I2C_ReceiveData(I2Cx);
		    }
		    
		    //test if next byte is last one and disable ack
		    if(I2Cx_Data->I2C_Rx_Idx + 1 == I2Cx_Data->I2C_Rx_Size) {
			// Disable Acknowledgement
			I2C_AcknowledgeConfig(I2Cx, DISABLE);
			// Send STOP Condition
			I2C_GenerateSTOP(I2Cx, ENABLE);
		    }
		}
	    }
	    break;
	case STOP_WRITTEN:
	    break;
    }    
}


void I2C_ER_IRQHandler(I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data) {
  I2Cx_Data->I2CError = 1;
  u16 sr2 = I2Cx->SR2;
  u16 cr1 = I2Cx->CR1;
  
  //we got an error, be shur we don't stop 
  //later by an orphan start condition
  I2C_GenerateSTART(I2Cx, DISABLE);
  
  vu8 nextRxWritePointer = (dbgWrite + 1) % dbgBufferSize;
  i2cSafetyCounter++;

  if(i2cSafetyCounter > 120) {
      I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
  }
  
  //if(nextRxWritePointer != dbgRead) {
	volatile struct i2cDebug *dbg = &(dbgBuffer[dbgWrite]);
	dbg->sr1 = cr1;
	dbg->sr2 = sr2;
	dbg->state = I2Cx_Data->state;
	dbg->mode = I2Cx_Data->I2CMode;
	dbg->idx = I2Cx_Data->I2C_Rx_Idx;
	dbg->size = I2Cx_Data->I2C_Rx_Size;
	dbg->cnt = 666;
	dbg->error = I2Cx_Data->I2CError;
	dbgWrite = nextRxWritePointer;
    //}
  //Acknowledge failure
  if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_AF)) {
    I2Cx_Data->I2CErrorReason = I2C_FLAG_AF;
  
    //Note, master only error handling
    if(!(cr1 & (1<<9) || cr1 & (1<<8)) && sr2 & I2C_BUSY)
	I2C_GenerateSTOP(I2Cx, ENABLE);

    /* Clear AF flag */
    I2C_ClearFlag(I2Cx, I2C_FLAG_AF);
  }

  if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_ARLO)) {
    I2C_ClearFlag(I2Cx, I2C_FLAG_ARLO);
    I2Cx_Data->I2CErrorReason = I2C_FLAG_ARLO;
    // Send STOP Condition
    if(!(cr1 & (1<<9) || cr1 & (1<<8)) && sr2 & I2C_BUSY)
	I2C_GenerateSTOP(I2Cx, ENABLE);
  }
  
  if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BERR)) {
    I2C_ClearFlag(I2Cx, I2C_FLAG_BERR);
    I2Cx_Data->I2CErrorReason = I2C_FLAG_BERR;
    I2C_SoftwareResetCmd(I2Cx, ENABLE);
  }
  
  if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_OVR)) {
    I2C_ClearFlag(I2Cx, I2C_FLAG_OVR);
    I2Cx_Data->I2CErrorReason = I2C_FLAG_OVR;
    // Send STOP Condition
    I2C_GenerateSTOP(I2Cx, ENABLE);
  }
}

/*******************************************************************************
* Function Name  : I2C1_EV_IRQHandler
* Description    : This function handles I2C1 Event interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void I2C1_EV_IRQHandler(void)
{
  I2C_EV_IRQHandler(I2C1, &I2C1_Data);
}

/*******************************************************************************
* Function Name  : I2C1_ER_IRQHandler
* Description    : This function handles I2C1 Error interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C1_ER_IRQHandler(void)
{
  I2C_ER_IRQHandler(I2C1, &I2C1_Data);
}

/*******************************************************************************
* Function Name  : I2C2_EV_IRQHandler
* Description    : This function handles I2C2 Event interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C2_EV_IRQHandler(void)
{ 
  I2C_EV_IRQHandler(I2C2, &I2C2_Data);
}

u8 I2C1OperationFinished() {
    if(i2cSafetyCounter >= 120) {
	printfI2CDbg();
    }
  return I2C1_Data.I2CMode == I2C_FINISHED;
};

u8 I2CSendBytes(u8 *data, u8 size, u8 addr, I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data) {
    
  if(I2Cx_Data->I2CError) {
    //print("I2C1 in error status \n");
    return 1;
  }
  
  if(I2Cx_Data->I2CMode != I2C_FINISHED) {
    print("I2C1 still active \n");
    return 1;
  }
  
  if(I2C_GetFlagStatus(I2C1, I2C_FLAG_MSL)) {
    //print("BAD, I am still a master \n");
    return 1;
  }

    if(i2cSafetyCounter >= 120) {
	printfI2CDbg();
    }
  i2cSafetyCounter = 0;

  //setup interrupt handler state machine
  I2Cx_Data->I2C_Tx_Idx = 0;
  I2Cx_Data->I2C_Tx_Size = size;
  I2Cx_Data->curI2CAddr = addr;
  I2Cx_Data->I2CMode = I2C_WRITE;
  int i;
  I2Cx_Data->state = START_WRITTEN;
  for(i = 0; i < size; i++) {
    I2Cx_Data->I2C_Buffer_Tx[i] = data[i];
  }

  //disable orphant stop conditions
  I2C_GenerateSTOP(I2Cx, DISABLE);

  // Enable Acknowledgement
  I2C_AcknowledgeConfig(I2Cx, ENABLE);
	
  /* Send I2C1 START condition */
  I2C_GenerateSTART(I2Cx, ENABLE);
  
  return 0;
}

u8 I2CReadBytes(u8 size, u8 addr,I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data) {
  if(I2Cx_Data->I2CError) {
    //print("I2C1 in error status \n");
    return 1;
  }
  
  if(I2Cx_Data->I2CMode != I2C_FINISHED) {
    print("I2C1 still active \n");
    return 1;
  }
  
  if(I2C_GetFlagStatus(I2C1, I2C_FLAG_MSL)) {
    //print("BAD, I am still a master \n");
    return 1;
  }

    if(i2cSafetyCounter >= 120) {
	printfI2CDbg();
    }
  i2cSafetyCounter = 0;

  //setup interrupt handler state machine
  I2Cx_Data->I2C_Rx_Idx = 0;
  I2Cx_Data->I2C_Rx_Size = size;
  I2Cx_Data->curI2CAddr = addr;
  I2Cx_Data->I2CMode = I2C_READ;
  I2Cx_Data->state = START_WRITTEN;

   //disable orphant stop conditions
  I2C_GenerateSTOP(I2Cx, DISABLE);

  // Enable Acknowledgement
  I2C_AcknowledgeConfig(I2Cx, ENABLE);
  
  /* Send I2C1 START condition */
  I2C_GenerateSTART(I2Cx, ENABLE);

  return 0;
}

u8 I2CWriteReadBytes(u8 *txdata, u8 txsize, u8 rxsize, u8 addr, I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data) {
  if(I2Cx_Data->I2CError) {
    //print("I2C1 in error status \n");
    return 1;
  }
  
  if(I2Cx_Data->I2CMode != I2C_FINISHED) {
    print("I2C1 still active \n");
    return 1;
  }
  
  if(I2C_GetFlagStatus(I2C1, I2C_FLAG_MSL)) {
    //print("BAD, I am still a master \n");
    return 1;
  }

    if(i2cSafetyCounter >= 120) {
	printfI2CDbg();
    }
  i2cSafetyCounter = 0;

  //setup interrupt handler state machine
  I2Cx_Data->I2C_Tx_Idx = 0;
  I2Cx_Data->I2C_Tx_Size = txsize;
  I2Cx_Data->I2C_Rx_Idx = 0;
  I2Cx_Data->I2C_Rx_Size = rxsize;
  I2Cx_Data->curI2CAddr = addr;
  I2Cx_Data->I2CMode = I2C_WRITE_READ;
  I2Cx_Data->state = START_WRITTEN;
  
  int i;
  
  for(i = 0; i < txsize; i++) {
    I2Cx_Data->I2C_Buffer_Tx[i] = txdata[i];
  }

  //disable orphant stop conditions
  I2C_GenerateSTOP(I2Cx, DISABLE);

  // Enable Acknowledgement
  I2C_AcknowledgeConfig(I2Cx, ENABLE);

  /* Send I2C1 START condition */
  I2C_GenerateSTART(I2Cx, ENABLE);

  return 0;
}


u8 I2C1SendBytes(u8 *data, u8 size, u8 addr) {
  return I2CSendBytes(data, size, addr, I2C1, &I2C1_Data);
}

u8 I2C1ReadBytes(u8 size, u8 addr) {
  return I2CReadBytes(size, addr, I2C1, &I2C1_Data);
}

u8 I2C1WriteReadBytes(u8 *txdata, u8 txsize, u8 rxsize, u8 addr) {
  return I2CWriteReadBytes(txdata, txsize, rxsize, addr, I2C1, &I2C1_Data);
}

u8 I2C2SendBytes(u8 *data, u8 size, u8 addr) {
  return I2CSendBytes(data, size, addr, I2C2, &I2C2_Data);
}

u8 I2C2ReadBytes(u8 size, u8 addr) {
  return I2CReadBytes(size, addr, I2C2, &I2C2_Data);
}

u8 I2C2WriteReadBytes(u8 *txdata, u8 txsize, u8 rxsize, u8 addr) {
  return I2CWriteReadBytes(txdata, txsize, rxsize, addr, I2C2, &I2C2_Data);
}
