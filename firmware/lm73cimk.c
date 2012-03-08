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
    u32 temperature;
    u8 address;
    volatile struct I2C_Data *I2C_Data;
    I2C_TypeDef* I2C_Bus;
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

enum LM73_SENSORS curI2COwner;
enum LM73_SENSORS lastI2COwner;

u8 lm73cimk_triggerTemeratureConversion(enum LM73_SENSORS sensor);
void moveLM73CIMKStateMachine();
u8 lm73cimk_requestTemperature(enum LM73_SENSORS sensor);

void lm73cimk_init(I2C_TypeDef* I2C_Bus_l)
{
    lastI2COwner = LM73_NONE;
    curI2COwner = LM73_NONE;

    int i;
    for(i = 0; i < NUM_LM73; i++)
    {
	lm73Data[i].enabled = 0;
	lm73Data[i].state = LM73_IDLE;
	lm73Data[i].temperature = 0;
	lm73Data[i].address = 0;
	lm73Data[i].I2C_Bus = I2C_Bus_l;
	if(lm73Data[i].I2C_Bus == I2C1)
	{
	    lm73Data[i].I2C_Data = &I2C1_Data;
	}
	else
	{
	    lm73Data[i].I2C_Data = &I2C2_Data;
	}
	lm73Data[i].errorCounter = 0;
    }
}

void lm73cimk_setup_sensor(enum LM73_SENSORS sensor, u8 i2c_addr)
{
    lm73Data[sensor].state = LM73_IDLE;    
    lm73Data[sensor].enabled = 1;
    lm73Data[sensor].address = i2c_addr;
}

void scheduleLM73()
{
    if(curI2COwner == LM73_NONE)
    {
	//round robing sceduling
	int i = lastI2COwner;
	i++;
	
	if(i >= LM73_NONE)
	    i = 0;
	
	//look for next enabled sensor
	for(; i < LM73_NONE; i++)
	{
	    if(lm73Data[i].enabled)
	    {
		break;
	    }
	}
	
	curI2COwner = i;
	lastI2COwner = curI2COwner;
    }
}

void moveLM73CIMKStateMachine() 
{
    scheduleLM73();
    
    if(curI2COwner == LM73_NONE)
	return;
    
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
    
    enum LM73_SENSORS sensor = curI2COwner;
    
    //handle I2C errors
    if(handleI2CxErrors(lm73Data[sensor].I2C_Bus, lm73Data[sensor].I2C_Data))
    {
	statistic2++;
	
	//we were not acked, restart
	//request next temperature
    
	//retrigger
	lm73Data[sensor].state = LM73_IDLE;
	//give away bus for other sensor
	curI2COwner = LM73_NONE;
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
	break;
	
    case LM73_TRIGGERED:
    {
	//test if last operation was finished
	if(I2CxOperationFinished(lm73Data[sensor].I2C_Bus, lm73Data[sensor].I2C_Data)) {
	    lm73Data[sensor].state = LM73_TRIGGER_FINISHED;
	    //give away bus for other sensor
	    curI2COwner = LM73_NONE;
	}
	else
	{
	    if(++(lm73Data[sensor].errorCounter) > 2000)
	    {
	        print("bus reset\n");
		//reset bus and start over
		resetI2C(lm73Data[sensor].I2C_Bus, lm73Data[sensor].I2C_Data);
		lm73Data[sensor].errorCounter = 0;
		lm73Data[sensor].state = LM73_IDLE;
		//give away bus for other sensor
		curI2COwner = LM73_NONE;
	    }
	}
	break;
    }
    case LM73_TRIGGER_FINISHED:
	//request next temperature
	if(!lm73cimk_requestTemperature(sensor)) {
	    statistic1++;
	    statistic_r++;
	    lm73Data[sensor].errorCounter = 0;
	    //print("Requested\n");
	    lm73Data[sensor].state = LM73_TEMP_REQUESTED;
	}
	break;

    case LM73_TEMP_REQUESTED:
	if(I2CxOperationFinished(lm73Data[sensor].I2C_Bus, lm73Data[sensor].I2C_Data)) {
	    //print("Got Temp\n");
	    s16 value = ((lm73Data[sensor].I2C_Data->I2C_Buffer_Rx[0]) << 8) + lm73Data[sensor].I2C_Data->I2C_Buffer_Rx[1];
	    lm73Data[sensor].I2C_Data->I2C_Buffer_Rx[0] = 0;
	    lm73Data[sensor].I2C_Data->I2C_Buffer_Rx[1] = 1;

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
	    //give away bus for other sensor
	    curI2COwner = LM73_NONE;
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
    return I2CSendBytes(data, 2, lm73Data[sensor].address, lm73Data[sensor].I2C_Bus, lm73Data[sensor].I2C_Data);
}

u8 lm73cimk_requestTemperature(enum LM73_SENSORS sensor)
{
    u8 tempRequest = 0;
    return I2CWriteReadBytes(&tempRequest, 1, 2, lm73Data[sensor].address, lm73Data[sensor].I2C_Bus, lm73Data[sensor].I2C_Data);
}

u8 lm73cimk_getTemperature(enum LM73_SENSORS sensor, u32* val)
{
    if(lm73Data[sensor].state != LM73_ACQUIRED_TEMP) {
	moveLM73CIMKStateMachine();
    } else {
	*val = lm73Data[sensor].temperature;
	lm73Data[sensor].state = LM73_IDLE;
	return 0;
    }
    return 1;  
}






