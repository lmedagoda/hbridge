#include "lm73cimk.h"
#include "i2c.h"
#include "printf.h"
#include "inc/stm32f10x_i2c.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_nvic.h"
#include "inc/stm32f10x_gpio.h"

enum lm73cimkState {
  LM73_IDLE,
  LM73_TRIGGERED,
  LM73_TRIGGER_FINISHED,
  LM73_TEMP_REQUESTED,
  LM73_ERROR,
  LM73_ACQUIRED_TEMP,
};


struct LM73Data {
    u8 enabled;
    enum lm73cimkState state;
    s32 temperature;
    u8 address;
    struct I2C_Handle *i2cHandle;
    u32 errorCounter;
};

#define NUM_LM73 2

struct LM73Data lm73Data[NUM_LM73];


u32 statistic1 = 0;
u32 statistic2 = 0;
u32 statistic_tr = 0;
u32 statistic_nr = 0;
u32 statistic_gt = 0;
u32 statistic_r = 0;

u8 lm73cimk_triggerTemeratureConversion(enum LM73_SENSORS sensor);
void moveLM73CIMKStateMachine(enum LM73_SENSORS sensor);
u8 lm73cimk_requestTemperature(enum LM73_SENSORS sensor);

void lm73cimk_init(I2C_TypeDef* I2C_Bus_l)
{
    int i;
    for(i = 0; i < NUM_LM73; i++)
    {
	lm73Data[i].enabled = 0;
	lm73Data[i].state = LM73_IDLE;
	lm73Data[i].temperature = 0;
	lm73Data[i].address = 0;	
	lm73Data[i].i2cHandle = I2C_getHandle(I2C_Bus_l);
	lm73Data[i].errorCounter = 0;
    }
}

void lm73cimk_setup_sensor(enum LM73_SENSORS sensor, u8 i2c_addr)
{
    lm73Data[sensor].state = LM73_IDLE;    
    lm73Data[sensor].enabled = 1;
    lm73Data[sensor].address = i2c_addr;
}


void moveLM73CIMKStateMachine(enum LM73_SENSORS sensor) 
{
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

    switch(lm73Data[sensor].state) {
    case LM73_IDLE:
	//trigger new conversion
	if(!lm73cimk_triggerTemeratureConversion(sensor)) {
	    statistic1++;
	    statistic_tr++;
	    lm73Data[sensor].state = LM73_TRIGGERED;
	    //print("Triggered\n");
	}    
	else
	{
	    statistic2++;
	}
	break;
	
    case LM73_TRIGGERED:
    {
	struct I2C_CommandResult *i2cResult = I2C_getResult(lm73Data[sensor].i2cHandle);
	if(i2cResult)
	{
	    if(i2cResult->I2CError)
	    {
		lm73Data[sensor].state = LM73_IDLE;
		if(++(lm73Data[sensor].errorCounter) > 2000)
		{
		    print("bus reset\n");
		    //reset bus and start over
		    resetI2C(lm73Data[sensor].i2cHandle);
		    lm73Data[sensor].errorCounter = 0;
		}
		statistic2++;
	    }
	    else
	    {
		lm73Data[sensor].state = LM73_TRIGGER_FINISHED;
		lm73Data[sensor].errorCounter = 0;
	    }
	}
	break;
    }
    case LM73_TRIGGER_FINISHED:
	//request next temperature
	if(!lm73cimk_requestTemperature(sensor)) {
	    statistic1++;
	    statistic_r++;
	    //print("Requested\n");
	    lm73Data[sensor].state = LM73_TEMP_REQUESTED;
	} 
	else
	{
	    statistic2++;
	}
	break;

    case LM73_TEMP_REQUESTED:
    {
	struct I2C_CommandResult *i2cResult = I2C_getResult(lm73Data[sensor].i2cHandle);
	if(i2cResult)
	{
	    if(i2cResult->I2CError)
	    {
		statistic2++;
		lm73Data[sensor].state = LM73_TRIGGER_FINISHED;
	    } else {
		s16 value = ((i2cResult->rxData[0]) << 8) + i2cResult->rxData[1];
		
		// value is -32768 when sensor is still working on temperature conversion
		if (value == -32768)
		{
		    //print("Temp Conversation not finished\n");
		    ++statistic_nr;
		    lm73Data[sensor].state = LM73_TRIGGER_FINISHED;
		}
		else
		{
		    value = value >> 7; // remove tailing zeros and decimal places
		    if ((value & (1 << 8)) != 0) // if negative fill with 1s (right shift is not defined with negative numbers)
		    {
			value |= 0xFE00; 
		    }
		    lm73Data[sensor].temperature = (value > 127) ? 127 : ((value < -128) ? -128 : value); // cut to 8 bits
		    statistic_gt++;
		    lm73Data[sensor].state = LM73_ACQUIRED_TEMP;
		}
	    }
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
u8 lm73cimk_triggerTemeratureConversion(enum LM73_SENSORS sensor) 
{
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
    return I2C_writeBytes(lm73Data[sensor].i2cHandle, data, 2, lm73Data[sensor].address);
}

u8 lm73cimk_requestTemperature(enum LM73_SENSORS sensor)
{
    u8 tempRequest = 0;    
    return I2C_writeReadBytes(lm73Data[sensor].i2cHandle, &tempRequest, 1, 2, lm73Data[sensor].address);
}

u8 lm73cimk_getTemperature(enum LM73_SENSORS sensor, s32* val)
{
    moveLM73CIMKStateMachine(sensor);
    if(lm73Data[sensor].state == LM73_ACQUIRED_TEMP) 
    {
	*val = lm73Data[sensor].temperature;
	lm73Data[sensor].state = LM73_IDLE;
	return 0;
    }
    return 1;  
}






