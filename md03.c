
#include "stm32f10x_lib.h"
#include "md03.h"
#include "i2c.h"
#include "stm32f10x_it.h"

//set to 24 000 instructions this should be equivalent to 1 ms
#define MD03_I2C_TIMEOUT 24000
#define MD03_01_ADDR 0xB0
#define MD03_02_ADDR 0xB2
#define MD03_03_ADDR 0xB4
#define MD03_04_ADDR 0xB6

volatile enum md03_global_state md03_global_state;
volatile enum md03_state single_md03_state;
volatile struct md03_data md03_data[4];

volatile enum md03_names cur_md03_device;

void I2C1CompleteCallback();


void setupI2C() {
  I2C_InitTypeDef  I2C_InitStructure;

  md03_data[MD03_01].address = MD03_01_ADDR;
  md03_data[MD03_02].address = MD03_02_ADDR;
  md03_data[MD03_03].address = MD03_03_ADDR;
  md03_data[MD03_04].address = MD03_04_ADDR;
  
  single_md03_state = MD03_IDLE;
  md03_global_state = MD03_WROTE_ALL_DATA;
  //md03_I2C2_state = MD03_IDLE;


  /* I2C1 configuration ----------------------------------------------------*/
  I2C_StructInit(&I2C_InitStructure);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  //I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_16_9;
  //we are master, no address
  I2C_InitStructure.I2C_OwnAddress1 = 0xA0;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  //I2C_InitStructure.I2C_ClockSpeed = 100000;
  I2C_InitStructure.I2C_ClockSpeed = 250000;
  //I2C_InitStructure.I2C_ClockSpeed = 400000;

  /* Enable I2C1 and I2C2 --------------------------------------------------*/
  I2C_Cmd(I2C1, ENABLE);

  I2C_Init(I2C1, &I2C_InitStructure);
  /* I2C2 configuration ----------------------------------------------------*/
  //I2C_InitStructure.I2C_OwnAddress1 = 0xA2;

  //I2C_Cmd(I2C2, ENABLE);
  //I2C_Init(I2C2, &I2C_InitStructure);


  /* Disable PEC for I2C1 and I2C2 -----------------------------------------*/
  I2C_CalculatePEC(I2C1, DISABLE);
  //I2C_CalculatePEC(I2C2, DISABLE);

  /* Enable I2C1 and I2C2 event and buffer interrupt */
  I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);
  //I2C_ITConfig(I2C2, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);
}

void MD03StateMachine(volatile enum md03_state *state, enum md03_names device, I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data) {
  u8 txdata[4];

  switch (*state) {
  case MD03_IDLE:
    //set correct state
    *state = MD03_READ_TEMP;
  
    //set i2c request
    txdata[0] = MD03_TEMPERATURE;
    I2CWriteReadBytes(txdata, 1, 1, md03_data[device].address, I2Cx, I2Cx_Data);
    break;
  case MD03_READ_TEMP:
    //save received data to data structure
    md03_data[device].temperature = I2Cx_Data->I2C_Buffer_Rx[0];

    *state = MD03_READ_CURRENT;
    
    //set next i2c request
    txdata[0] = MD03_MOTOR_CURRENT;
    I2CWriteReadBytes(txdata, 1, 1, md03_data[device].address, I2Cx, I2Cx_Data);
    break;
  case MD03_READ_CURRENT:
    //save received data to data structure
    md03_data[device].motor_current = I2Cx_Data->I2C_Buffer_Rx[0];
    
    *state = MD03_DATA_RECEIVED;
    break;
  case MD03_DATA_RECEIVED:
    assert_param(0);
    //this should never happen
    break;
  case MD03_WRITE_SPEED:
    //set next state
    *state = MD03_WRITE_ACC;
    //do what needs to be done in current state
    txdata[0] = MD03_SPEED;
    txdata[1] = md03_data[device].speed;
    I2CSendBytes(txdata, 2, md03_data[device].address, I2Cx, I2Cx_Data);
    break;
  case MD03_WRITE_ACC:
    //set next state
    *state = MD03_WRITE_CMD;
    //do what needs to be done in current state
    txdata[0] = MD03_ACCELERATION;
    txdata[1] = md03_data[device].acceleration;
    I2CSendBytes(txdata, 2, md03_data[device].address, I2Cx, I2Cx_Data);  
    break;
  case MD03_WRITE_CMD:
    //set next state
    *state = MD03_WAIT_FOR_CMD_WRITE;
    //do what needs to be done in current state
    txdata[0] = MD03_COMMAND;
    txdata[1] = md03_data[device].direction;
    I2CSendBytes(txdata, 2, md03_data[device].address, I2Cx, I2Cx_Data);
    break;
  case MD03_WAIT_FOR_CMD_WRITE:
    //the last state issues a write, we need to wait until
    //the i2c hardware tells us, that the write was finished
    *state = MD03_WRITE_FINISHED;
    break;
  case MD03_WRITE_FINISHED:
    assert_param(0);
    //should never happen
    break;
  }
}


void requestMD03Data() {
  assert_param(md03_global_state == MD03_WROTE_ALL_DATA);
  requestMD03DataI2C1();
}

void requestMD03DataI2C1() {
  md03_global_state = MD03_READING_ALL_DATA;
  cur_md03_device = MD03_01;
  single_md03_state = MD03_IDLE;
  MD03StateMachine(&single_md03_state, cur_md03_device, I2C1, &I2C1_Data);
}


int MD03DataReady() {
  if(I2C1OperationFinished()) {
    I2C1CompleteCallback();
  }
  return (md03_global_state == MD03_ALL_DATA_RECEIVED);
}

void requestMD03DataWrite() {
  assert_param(md03_global_state == MD03_ALL_DATA_RECEIVED);
  requestMD03DataWriteI2C1();
}

void requestMD03DataWriteI2C1() {
  md03_global_state = MD03_WRITING_ALL_DATA;
  cur_md03_device = MD03_01;
  single_md03_state = MD03_WRITE_SPEED;
  MD03StateMachine(&single_md03_state, cur_md03_device, I2C1, &I2C1_Data);
}

int MD03DataWritten() {
  if(I2C1OperationFinished()) {
    I2C1CompleteCallback();
  }
  
  return (md03_global_state == MD03_WROTE_ALL_DATA);
}


void I2C1CompleteCallback() {
  /*static int number = 0;
  if(((dbgWrite +1) % dbgBufferSize) != dbgRead &&
     ((dbgWrite +2) % dbgBufferSize) != dbgRead &&
     ((dbgWrite +3) % dbgBufferSize) != dbgRead &&
     ((dbgWrite +4) % dbgBufferSize) != dbgRead &&
     ((dbgWrite +5) % dbgBufferSize) != dbgRead &&
     ((dbgWrite +6) % dbgBufferSize) != dbgRead) {
    
    dbgBuffer[dbgWrite] = number;

    number++;

    dbgWrite = (dbgWrite +1) % dbgBufferSize;

    dbgBuffer[dbgWrite] = I2C1_Data.I2CMode;
    
    dbgWrite = (dbgWrite +1) % dbgBufferSize;

    dbgBuffer[dbgWrite] = I2C1;
    
    dbgWrite = (dbgWrite +1) % dbgBufferSize;    

    dbgBuffer[dbgWrite] = 300;
    
    dbgWrite = (dbgWrite +1) % dbgBufferSize;    

    dbgBuffer[dbgWrite] = md03_global_state;
    
    dbgWrite = (dbgWrite +1) % dbgBufferSize;    

    dbgBuffer[dbgWrite] = single_md03_state;
    
    dbgWrite = (dbgWrite +1) % dbgBufferSize;    

    dbgBuffer[dbgWrite] = cur_md03_device;
    
    dbgWrite = (dbgWrite +1) % dbgBufferSize;
    }*/


  if(md03_global_state == MD03_READING_ALL_DATA) {
    //advance the single md03 state
    MD03StateMachine(&single_md03_state, cur_md03_device, I2C1, &I2C1_Data);
    if(single_md03_state == MD03_DATA_RECEIVED) {
      if(cur_md03_device != MD03_02) {
	single_md03_state = MD03_IDLE;
	cur_md03_device++;
	MD03StateMachine(&single_md03_state, cur_md03_device, I2C1, &I2C1_Data);
      } else {
	md03_global_state = MD03_ALL_DATA_RECEIVED;
	return;
      }
    }
  }
  if(md03_global_state == MD03_WRITING_ALL_DATA) {
    //advance the single md03 state
    MD03StateMachine(&single_md03_state, cur_md03_device, I2C1, &I2C1_Data);
    if(single_md03_state == MD03_WRITE_FINISHED) {
      if(cur_md03_device != MD03_02) {
	single_md03_state = MD03_WRITE_SPEED;
	cur_md03_device++;
	MD03StateMachine(&single_md03_state, cur_md03_device, I2C1, &I2C1_Data);
      } else {
	md03_global_state = MD03_WROTE_ALL_DATA;
	return;
      }
    } 
  }

  /*
  if(((dbgWrite +1) % dbgBufferSize) != dbgRead &&
     ((dbgWrite +2) % dbgBufferSize) != dbgRead &&
     ((dbgWrite +3) % dbgBufferSize) != dbgRead &&
     ((dbgWrite +4) % dbgBufferSize) != dbgRead &&
     ((dbgWrite +5) % dbgBufferSize) != dbgRead &&
     ((dbgWrite +6) % dbgBufferSize) != dbgRead) {
    
    dbgBuffer[dbgWrite] = number;

    number++;

    dbgWrite = (dbgWrite +1) % dbgBufferSize;

    dbgBuffer[dbgWrite] = I2C1_Data.I2CMode;
    
    dbgWrite = (dbgWrite +1) % dbgBufferSize;

    dbgBuffer[dbgWrite] = I2C1;
    
    dbgWrite = (dbgWrite +1) % dbgBufferSize;    

    dbgBuffer[dbgWrite] = 300;
    
    dbgWrite = (dbgWrite +1) % dbgBufferSize;    

    dbgBuffer[dbgWrite] = md03_global_state;
    
    dbgWrite = (dbgWrite +1) % dbgBufferSize;    

    dbgBuffer[dbgWrite] = single_md03_state;
    
    dbgWrite = (dbgWrite +1) % dbgBufferSize;    

    dbgBuffer[dbgWrite] = cur_md03_device;
    
    dbgWrite = (dbgWrite +1) % dbgBufferSize;
    }*/

}



