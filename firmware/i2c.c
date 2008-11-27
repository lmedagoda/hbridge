
#include "i2c.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_it.h"

volatile u16 dbgWrite = 0;
volatile u16 dbgRead = 0;
volatile u32 dbgBuffer[dbgBufferSize];
volatile u32 number=0;

volatile struct I2C_Data I2C1_Data;
volatile struct I2C_Data I2C2_Data;


/*******************************************************************************
* Function Name  : I2C1_EV_IRQHandler
* Description    : This function handles I2C1 Event interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

extern volatile enum md03_i2c_state md03_I2C1_state;

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

void I2C_EV_IRQHandler(I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data)
{
  //assert_param(0);
  vu32 state = I2C_GetLastEvent(I2Cx);
  
  u8 test;

  switch (state)
  {
    case I2C_FLAG_SB:

    /* Test on I2C1 EV5 and clear it */
    case I2C_EVENT_MASTER_MODE_SELECT:
      test = 1;
      if(I2Cx_Data->I2CMode == I2C_READ) {
	// Send I2C2 slave Address for write
	I2C_Send7bitAddress(I2Cx, I2Cx_Data->curI2CAddr, I2C_Direction_Receiver);
      } else {
	if(I2Cx_Data->I2CMode != I2C_FINISHED) {//I2Cx_Data->I2CMode == I2C_WRITE || I2Cx_Data->I2CMode == I2C_WRITE_READ 
	  // Send I2C2 slave Address for write
	  I2C_Send7bitAddress(I2Cx, I2Cx_Data->curI2CAddr, I2C_Direction_Transmitter);
	}
	
      }
      break;

    case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
      test = 2;

      /* Enable I2C1 BUF interrupts */
      I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);

      if(I2Cx_Data->I2C_Rx_Size == 1) {
	// Disable Acknowledgement
	I2C_AcknowledgeConfig(I2Cx, DISABLE);

	// Send STOP Condition
	I2C_GenerateSTOP(I2Cx, ENABLE);
      }
      break;

    /* Test on I2C1 EV6 and first EV8 and clear them */
    case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
      test = 3;

      /* Enable I2C1 BUF interrupts */
      I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);

      /* Reset Idx */
      //I2C1_Tx_Idx = 0;
      /* Send the first data */
      /* EV8 just after EV6 */
      I2C_SendData(I2Cx, I2Cx_Data->I2C_Buffer_Tx[I2Cx_Data->I2C_Tx_Idx]);
      I2Cx_Data->I2C_Tx_Idx++;
      break;

    /* Test on I2C1 EV7 and clear it */
    case I2C_EVENT_MASTER_BYTE_RECEIVED:
      test = 4;

      I2Cx_Data->I2C_Buffer_Rx[I2Cx_Data->I2C_Rx_Idx] = I2C_ReceiveData(I2Cx);
      I2Cx_Data->I2C_Rx_Idx++;
      
      //test if next byte is last one and disable ack
      if(I2Cx_Data->I2C_Rx_Idx + 1 == I2Cx_Data->I2C_Rx_Size) {
	// Disable Acknowledgement
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	// Send STOP Condition
	I2C_GenerateSTOP(I2Cx, ENABLE);
      }

      if(I2Cx_Data->I2C_Rx_Idx >= I2Cx_Data->I2C_Rx_Size) {
	// Disable Acknowledgement
	I2C_AcknowledgeConfig(I2Cx, DISABLE);	

	/* Disable I2C1 interrupts */
        //I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF, DISABLE);

	I2Cx_Data->I2CMode = I2C_FINISHED;	
      }
      break;

    case I2C_EVENT_MASTER_BYTE_IN_SHIFT_REGISTER:
      test = 7;
      if(I2Cx_Data->I2C_Tx_Idx < I2Cx_Data->I2C_Tx_Size) {

	/*delay = 1000;
	while(delay)
	;*/
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

	    //write some data to DR, so that BTF is never set
	    //note this data is NOT sended
	    I2C_SendData(I2Cx, 0);
	  }
	}
      }      
      break;
    /* Test on I2C1 EV8 and clear it */
    case I2C_EVENT_MASTER_BYTE_TRANSMITTED:         
      test = 5;
      break;

    default:

      test = 6;
      
      break;
  }
  
  /*
  static int number = 0;

  number++;

  if(((dbgWrite +1) % dbgBufferSize) != dbgRead &&
     ((dbgWrite +2) % dbgBufferSize) != dbgRead &&
     ((dbgWrite +3) % dbgBufferSize) != dbgRead &&
     ((dbgWrite +4) % dbgBufferSize) != dbgRead &&
     ((dbgWrite +5) % dbgBufferSize) != dbgRead &&
     ((dbgWrite +6) % dbgBufferSize) != dbgRead) {
    
    dbgBuffer[dbgWrite] = number;
    
    dbgWrite = (dbgWrite +1) % dbgBufferSize;

    dbgBuffer[dbgWrite] = I2Cx_Data->I2CMode;
    
    dbgWrite = (dbgWrite +1) % dbgBufferSize;

    dbgBuffer[dbgWrite] = I2Cx;
    
    dbgWrite = (dbgWrite +1) % dbgBufferSize;    

    dbgBuffer[dbgWrite] = test;
    
    dbgWrite = (dbgWrite +1) % dbgBufferSize;    


    if(test == 6) {
      dbgBuffer[dbgWrite] = state;
    
      dbgWrite = (dbgWrite +1) % dbgBufferSize;
    }
    
    if(test == 7 || test == 5 || test == 3) {
      dbgBuffer[dbgWrite] = I2Cx_Data->I2C_Tx_Idx;
      dbgWrite = (dbgWrite +1) % dbgBufferSize;
      dbgBuffer[dbgWrite] = I2Cx_Data->I2C_Tx_Size;
      dbgWrite = (dbgWrite +1) % dbgBufferSize;
    }
    if(test == 4 || test == 2) {
      dbgBuffer[dbgWrite] = I2Cx_Data->I2C_Rx_Idx;
      dbgWrite = (dbgWrite +1) % dbgBufferSize;
      dbgBuffer[dbgWrite] = I2Cx_Data->I2C_Rx_Size;
      dbgWrite = (dbgWrite +1) % dbgBufferSize;
    }

  }
  
  if(number > 680) {
    // Disable I2Cx BUF interrupts
    //I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
    //I2C_ITConfig(I2C2, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
    }*/
  /*
  static int ledcounter = 0;

  ledcounter++;
  
  if(I2Cx == I2C2) {
    if(ledcounter < 2) {    
      GPIOC->BRR |= 0x00001000;
    } else {
      GPIOC->BSRR |= 0x00001000;
      //if(ledcounter > 3)
      //ledcounter = 0;
    }
    }*/
  
}

void sendDebugBuffer() {
  while(dbgWrite != dbgRead) {
    printf("Number is %lu ", dbgBuffer[dbgRead]);
    dbgRead = (dbgRead +1) % dbgBufferSize;

    if(dbgBuffer[dbgRead] == I2C_WRITE) { 
      print("Mode is I2C_WRITE ");
    } else {
      if(dbgBuffer[dbgRead] == I2C_READ) {
	print("Mode is I2C_READ "); 
      } else {
	print("Mode is I2C_WRITE_READ ");
      }
    }
    dbgRead = (dbgRead +1) % dbgBufferSize;

    if(((void *) dbgBuffer[dbgRead]) == I2C1) {  
      print("Channel is I2C1 ");
    } else {
      print("Channel is I2C2 ");      
    }
    
    dbgRead = (dbgRead +1) % dbgBufferSize;
    
    switch(dbgBuffer[dbgRead]) {
    case 2:
      dbgRead = (dbgRead +1) % dbgBufferSize;  
      printf("Got event 2 Rx Idx is %lu ", dbgBuffer[dbgRead]);
      dbgRead = (dbgRead +1) % dbgBufferSize;
      printf("Rx Size is %lu \n", dbgBuffer[dbgRead]);
      break;
    case 3:
      dbgRead = (dbgRead +1) % dbgBufferSize;  
      printf("Got event 3 Tx Idx is %lu ", dbgBuffer[dbgRead]);
      dbgRead = (dbgRead +1) % dbgBufferSize;
      printf("Tx Size is %lu \n", dbgBuffer[dbgRead]);
      break;
    case 4:
      dbgRead = (dbgRead +1) % dbgBufferSize;  
      printf("Got event 4 Rx Idx is %lu ", dbgBuffer[dbgRead]);
      dbgRead = (dbgRead +1) % dbgBufferSize;
      printf("Rx Size is %lu \n", dbgBuffer[dbgRead]);
      break;
    case 5:
      dbgRead = (dbgRead +1) % dbgBufferSize;  
      printf("Got event 5 Tx Idx is %lu ", dbgBuffer[dbgRead]);
      dbgRead = (dbgRead +1) % dbgBufferSize;
      printf("Tx Size is %lu \n", dbgBuffer[dbgRead]);
      break;
    case 6:
      dbgRead = (dbgRead +1) % dbgBufferSize;
      printf("Got unknown event %lu \n", dbgBuffer[dbgRead]);
      break;
    case 7:
      dbgRead = (dbgRead +1) % dbgBufferSize;  
      printf("Got event 7 Tx Idx is %lu ", dbgBuffer[dbgRead]);
      dbgRead = (dbgRead +1) % dbgBufferSize;
      printf("Tx Size is %lu \n", dbgBuffer[dbgRead]);
      break;
    case 200:
      dbgRead = (dbgRead +1) % dbgBufferSize;  
      printf("Got Fail Idx is %lu ", dbgBuffer[dbgRead]);
      dbgRead = (dbgRead +1) % dbgBufferSize;
      printf("Size is %lu \n", dbgBuffer[dbgRead]);
      break;
    case 300:
      dbgRead = (dbgRead +1) % dbgBufferSize;  
      printf("\nGlobal Md03 mode is is %lu ", dbgBuffer[dbgRead]);
      dbgRead = (dbgRead +1) % dbgBufferSize;  
      printf("single Md03 mode is is %lu ", dbgBuffer[dbgRead]);
      dbgRead = (dbgRead +1) % dbgBufferSize;  
      printf("device is is %lu \n", dbgBuffer[dbgRead]);
      break;
    default:
      printf("Got event %lu \n", dbgBuffer[dbgRead]);
      break;
    }
    
    dbgRead = (dbgRead +1) % dbgBufferSize;  
  }
}


u8 I2C1OperationFinished() {
  return I2C1_Data.I2CMode == I2C_FINISHED;
};



void I2C_ER_IRQHandler(I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data) {
  I2Cx_Data->I2CError = 1;
  
  //Acknowledge failure
  if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_AF)) {
    I2Cx_Data->I2CErrorReason = I2C_FLAG_AF;
  
    //Node, master only error handling
    I2C_GenerateSTOP(I2Cx, ENABLE);

    /* Clear AF flag */
    I2C_ClearFlag(I2Cx, I2C_FLAG_AF);
  } 

  if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_ARLO)) {
    I2Cx_Data->I2CErrorReason = I2C_FLAG_ARLO;
  }
  
  if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BERR)) {
    I2Cx_Data->I2CErrorReason = I2C_FLAG_BERR;
  }
  
  if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_OVR)) {
    I2Cx_Data->I2CErrorReason = I2C_FLAG_OVR;
  }

  
  static int ledcounter = 0;

  ledcounter++;
  
  if(ledcounter < 20000) {    
    GPIOC->BRR |= 0x00001000;
  } else {
    GPIOC->BSRR |= 0x00001000;
    if(ledcounter > 40000)
      ledcounter = 0;
  }

}

void I2CSendBytes(u8 *data, u8 size, u8 addr, I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data) {
  //setup interrupt handler state machine
  I2Cx_Data->I2C_Tx_Idx = 0;
  I2Cx_Data->I2C_Tx_Size = size;
  I2Cx_Data->curI2CAddr = addr;
  I2Cx_Data->I2CMode = I2C_WRITE;
  int i;
  
  for(i = 0; i < size; i++) {
    I2Cx_Data->I2C_Buffer_Tx[i] = data[i];
  }

  // Enable Acknowledgement
  I2C_AcknowledgeConfig(I2Cx, ENABLE);
	
  /* Send I2C1 START condition */
  I2C_GenerateSTART(I2Cx, ENABLE);
}

void I2CReadBytes(u8 size, u8 addr,I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data) {
  //setup interrupt handler state machine
  I2Cx_Data->I2C_Rx_Idx = 0;
  I2Cx_Data->I2C_Rx_Size = size;
  I2Cx_Data->curI2CAddr = addr;
  I2Cx_Data->I2CMode = I2C_READ;

  // Enable Acknowledgement
  I2C_AcknowledgeConfig(I2Cx, ENABLE);
  
  /* Send I2C1 START condition */
  I2C_GenerateSTART(I2Cx, ENABLE);
}

void I2CWriteReadBytes(u8 *txdata, u8 txsize, u8 rxsize, u8 addr, I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data) {
  //setup interrupt handler state machine
  I2Cx_Data->I2C_Tx_Idx = 0;
  I2Cx_Data->I2C_Tx_Size = txsize;
  I2Cx_Data->I2C_Rx_Idx = 0;
  I2Cx_Data->I2C_Rx_Size = rxsize;
  I2Cx_Data->curI2CAddr = addr;
  I2Cx_Data->I2CMode = I2C_WRITE_READ;
  
  int i;
  
  for(i = 0; i < txsize; i++) {
    I2Cx_Data->I2C_Buffer_Tx[i] = txdata[i];
  }

  // Enable Acknowledgement
  I2C_AcknowledgeConfig(I2Cx, ENABLE);

  /* Send I2C1 START condition */
  I2C_GenerateSTART(I2Cx, ENABLE);
}


void I2C1SendBytes(u8 *data, u8 size, u8 addr) {
  I2CSendBytes(data, size, addr, I2C1, &I2C1_Data);
}

void I2C1ReadBytes(u8 size, u8 addr) {
  I2CReadBytes(size, addr, I2C1, &I2C1_Data);
}

void I2C1WriteReadBytes(u8 *txdata, u8 txsize, u8 rxsize, u8 addr) {
  I2CWriteReadBytes(txdata, txsize, rxsize, addr, I2C1, &I2C1_Data);
}

void I2C2SendBytes(u8 *data, u8 size, u8 addr) {
  I2CSendBytes(data, size, addr, I2C2, &I2C2_Data);
}

void I2C2ReadBytes(u8 size, u8 addr) {
  I2CReadBytes(size, addr, I2C2, &I2C2_Data);
}

void I2C2WriteReadBytes(u8 *txdata, u8 txsize, u8 rxsize, u8 addr) {
  I2CWriteReadBytes(txdata, txsize, rxsize, addr, I2C2, &I2C2_Data);
}
