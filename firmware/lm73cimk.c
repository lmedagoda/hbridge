#include "lm73cimk.h"
#include "i2c.h"
#include "printf.h"
#include "inc/stm32f10x_i2c.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_nvic.h"
#include "inc/stm32f10x_gpio.h"

enum lm73cimkStates lm73cimkState = LM73_IDLE;

volatile struct I2C_Data *I2C_Data;
I2C_TypeDef* I2C_Bus;

u32 currentTemperature;

u32 statistic1 = 0;
u32 statistic2 = 0;
u32 statistic_tr = 0;
u32 statistic_nr = 0;
u32 statistic_gt = 0;
u32 statistic_r = 0;

void lm73cimk_init(I2C_TypeDef* I2C_Bus_l, FunctionalState remapped)
{
    I2C_Bus = I2C_Bus_l;
    if(I2C_Bus == I2C1)
    {
	I2C_Data = &I2C1_Data;
    }
    else
    {
	I2C_Data = &I2C2_Data;
	//I2C2 can not be remapped
	assert_param(!remapped);
    }

    setupI2Cx(0xA0, 100000, I2C_Bus, remapped);
    
    lm73cimkState = LM73_IDLE;
}


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
    if(handleI2CxErrors(I2C_Bus, I2C_Data))
    {
	statistic2++;
	
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
    {
	static int count = 0;
	
	//test if last operation was finished
	if(I2CxOperationFinished(I2C_Bus, I2C_Data)) {
	    //request next temperature
	    if(!lm73cimk_requestTemperature(addr)) {
		statistic1++;
		statistic_r++;
		count = 0;
		//print("Requested\n");
		lm73cimkState = LM73_TEMP_REQUESTED;
	    }
	}
	else
	{
	    if(++count > 2000)
	    {
		I2C_SoftwareResetCmd(I2C_Bus, DISABLE);
		lm73cimk_init(I2C_Bus, I2C_Data->curI2CIsRemapped);
		count = 0;
		lm73cimkState = LM73_IDLE;
	    }
	}
	break;
    }
    case LM73_TEMP_REQUESTED:
	if(I2CxOperationFinished(I2C_Bus, I2C_Data)) {
	    //print("Got Temp\n");
	    s16 value = ((I2C_Data->I2C_Buffer_Rx[0]) << 8) + I2C_Data->I2C_Buffer_Rx[1];
	    I2C_Data->I2C_Buffer_Rx[0] = 0;
	    I2C_Data->I2C_Buffer_Rx[1] = 1;

	    // value is -32768 when sensor is still working on temperature conversion
	    if (value == -32768)
	    {
		//print("Temp Conversation not finished\n");
		++statistic_nr;
		lm73cimkState = LM73_TRIGGERED;
	    }
	    else
	    {
		value = value >> 7; // remove tailing zeros and decimal places
		if ((value & (1 << 8)) != 0) // if negative fill with 1s (right shift is not defined with negative numbers)
		{
		    value |= 0xFE00; 
		}
		currentTemperature = (value > 127) ? 127 : ((value < -128) ? -128 : value); // cut to 8 bits
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
    return I2CSendBytes(data, 2, addr, I2C_Bus, I2C_Data);
}

u8 lm73cimk_requestTemperature(u8 addr)
{
    u8 tempRequest = 0;
    return I2CWriteReadBytes(&tempRequest, 1, 2, addr, I2C_Bus, I2C_Data);
}

u8 lm73cimk_getTemperature(u8 addr, u32* val)
{
    if(lm73cimkState != LM73_ACQUIRED_TEMP) {
	moveLM73CIMKStateMachine(addr);
    } else {
	*val = currentTemperature;
	lm73cimkState = LM73_IDLE;
	return 0;
    }
  return 1;  
}






