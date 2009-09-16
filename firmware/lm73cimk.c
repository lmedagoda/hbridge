#include "lm73cimk.h"
#include "i2c.h"
#include "printf.h"

enum lm73cimkStates lm73cimkState = LM73_IDLE;

u32 currentTemperature;

u32 statistic1 = 0;
u32 statistic2 = 0;
u32 statistic_tr = 0;
u32 statistic_nr = 0;
u32 statistic_gt = 0;
u32 statistic_r = 0;


void moveLM73CIMKStateMachine(u8 addr) {
  if(statistic1 > 10000) {
    print("Did 10000 i2c operations \n");
    printf("Errors: %lu \n", statistic2);
    printf("Triggered: %lu \n", statistic_tr);
    printf("Not Ready: %lu \n", statistic_nr);
    printf("Requests: %lu \n", statistic_r);
    printf("Got Temp: %lu \n", statistic_gt);
    
    statistic1 = 0;
    statistic2 = 0;
    statistic_tr = 0;
    statistic_nr = 0;
    statistic_gt = 0;
    statistic_r = 0;
  }
  
  //handle I2C errors
  if(I2C1_Data.I2CError) {
    statistic2++;
    //print("Error on I2C1 \n"); 
    switch(I2C1_Data.I2CErrorReason) {
    case I2C_FLAG_ARLO:
      //print("I2C_FLAG_ARLO \n");
      break;
    case I2C_FLAG_BERR:
      //print("I2C_FLAG_BERR \n");
      I2C_SoftwareResetCmd(I2C1, DISABLE);
      setupI2CForLM73CIMK();
      break;
    case I2C_FLAG_OVR:
      //print("I2C_FLAG_OVR \n");
      break;
    case I2C_FLAG_AF:
      //print("I2C_FLAG_AF \n");
      break;	
    default:
      printf("Unknown %lu", I2C1_Data.I2CErrorReason);
    }
    
    I2C1_Data.I2CMode = I2C_FINISHED;
    
    I2C1_Data.I2CError = 0;
    //we were not acked, restart
    //request next temperature
    
    if(lm73cimkState == LM73_TRIGGERED)
      //retrigger
      lm73cimkState = LM73_IDLE;
    else
      //restart polling
      lm73cimkState = LM73_TRIGGERED;
  }


  switch(lm73cimkState) {
  case LM73_IDLE:
    //trigger new conversation
    if(!lm73cimk_triggerTemeratureConversation(addr)) {
      statistic1++;
      statistic_tr++;
      lm73cimkState = LM73_TRIGGERED;
      //print("Triggered\n");
    }
    
    break;
    
  case LM73_TRIGGERED:
    //test if last operation was finished
    if(I2C1OperationFinished()) {
      //request next temperature
      if(!requestTemperature(addr)) {
	statistic1++;
	statistic_r++;
	//print("Requested\n");
	lm73cimkState = LM73_TEMP_REQUESTED;
      }
    }
    break;
    
  case LM73_TEMP_REQUESTED:
    if(I2C1OperationFinished()) {
      //print("Got Temp\n");
      u16 value = ((I2C1_Data.I2C_Buffer_Rx[0]) << 8) + I2C1_Data.I2C_Buffer_Rx[1];
      I2C1_Data.I2C_Buffer_Rx[0] = 0;
      I2C1_Data.I2C_Buffer_Rx[1] = 1;

      if(value >= 0x8000) {
	//print("Temp Conversation not finished\n");
	statistic_nr++;
	lm73cimkState = LM73_TRIGGERED;
      } else {
	currentTemperature = value;
	currentTemperature *= 100;
	currentTemperature = currentTemperature >> 7;
	statistic_gt++;
	lm73cimkState = LM73_ACQUIRED_TEMP;
      }
    }
    break;
  default:
    break;
  }
}

/**
 * Note, this function also configures the 
 * LM73CIMK.
 * The LM73CIMK needs 14 ms for a conversation
 **/
u8 lm73cimk_triggerTemeratureConversation(u8 addr) {
  u8 config = 0;
  
  //set in shutdown mode, ie singe shot mode
  config |= (1<<7);
  
  //disable alert
  config |= (1<<5);

  //reset alert
  config |= (1<<3);
  
  //request new conversation
  config |= (1<<2);

  u8 data[3];
  
  //adress of configuration register
  data[0] = 1;
  
  data[1] = config;
  

  //send data to device
  return I2C1SendBytes(data, 2, addr);
}

u8 requestTemperature(u8 addr) {
  u8 tempRequest = 0;
  return I2C1WriteReadBytes(&tempRequest, 1, 2, addr);
}

u8 getTemperature(u8 addr, u32 *val) {
  if(lm73cimkState != LM73_ACQUIRED_TEMP) {
    moveLM73CIMKStateMachine(addr);
  } else {
    *val = currentTemperature;
    lm73cimkState = LM73_IDLE;
    return 0;
  }
  return 1;  
}


void setupI2CForLM73CIMK() {
  I2C_InitTypeDef  I2C_InitStructure;

  /* I2C1 configuration ----------------------------------------------------*/
  I2C_StructInit(&I2C_InitStructure);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  //I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_16_9;
  //we are master, no address
  I2C_InitStructure.I2C_OwnAddress1 = 0xA0;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  //I2C_InitStructure.I2C_ClockSpeed = 50000;
  I2C_InitStructure.I2C_ClockSpeed = 350000;
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

  I2C1_Data.I2CMode = I2C_FINISHED;
  lm73cimkState = LM73_IDLE;
}






