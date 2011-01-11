#include "lm73cimk.h"
#include "i2c.h"
#include "printf.h"
#include "inc/stm32f10x_i2c.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_nvic.h"
#include "inc/stm32f10x_gpio.h"

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
    print("move state machine\n");
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
    //trigger new conversion
    if(!lm73cimk_triggerTemeratureConversion(addr)) {
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

      // value is bigger than 0x8000 when sensor is still working on temperature conversion
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
      print("Error, lm73 polling statemachine in unknown state\n");
    break;
  }
}

/**
 * Note, this function also configures the 
 * LM73CIMK.
 * The LM73CIMK needs 14 ms for a conversion
 **/
u8 lm73cimk_triggerTemeratureConversion(u8 addr) {
  u8 config = 0;
  
  //set in shutdown mode, ie single shot mode
  config |= (1<<7);
  
  //disable alert
  config |= (1<<5);

  //reset alert
  config |= (1<<3);
  
  //request new conversion
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
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_StructInit(&NVIC_InitStructure);
  
  /* Configure and enable I2C1 interrupt ------------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQChannel;
  NVIC_Init(&NVIC_InitStructure);
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 | RCC_APB1Periph_I2C2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
  
  GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
    
  I2C_InitTypeDef  I2C_InitStructure;

  /* I2C1 configuration ----------------------------------------------------*/
  I2C_StructInit(&I2C_InitStructure);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2; // for < 100 kHz
  //I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_16_9; // for > 100 kHz
  //we are master, no address
  I2C_InitStructure.I2C_OwnAddress1 = 0xA0;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = 100000; // tested with no errors
  //I2C_InitStructure.I2C_ClockSpeed = 400000; // tested with some errors

  /* Enable I2C1 and I2C2 --------------------------------------------------*/
  I2C_Cmd(I2C1, ENABLE);

  I2C_Init(I2C1, &I2C_InitStructure);

  /* configure I2C pins ----------------------------------------------------*/
  //Configure I2C1 Pins, SDA and SCL
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  

  /* Disable PEC for I2C1 -----------------------------------------*/
  I2C_CalculatePEC(I2C1, DISABLE);

  /* Enable I2C1 and I2C2 event and buffer interrupt */
  I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);
 
  I2C1_Data.I2CMode = I2C_FINISHED;
  lm73cimkState = LM73_IDLE;
}






