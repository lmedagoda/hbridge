#include "tmp100.h"
#include "i2c.h"
#include "printf.h"

enum tmp100State {
  TMP100_IDLE,
  TMP100_TRIGGERED,
  TMP100_TRIGGER_FINISHED,
  TMP100_TEMP_REQUESTED,
  TMP100_ERROR,
  TMP100_ACQUIRED_TEMP,
};


struct TMP100Data {
    uint8_t enabled;
    enum tmp100State state;
    int32_t temperature;
    uint8_t address;
    struct I2C_Handle *i2cHandle;
    uint32_t errorCounter;
};

struct TMP100Data tmp100Data[1];

void tmp100_init(I2C_TypeDef* I2C_Bus)
{
    tmp100Data[0].enabled = 0;
    tmp100Data[0].state = TMP100_IDLE;
    tmp100Data[0].temperature = 0;
    tmp100Data[0].address = 0;
    tmp100Data[0].i2cHandle = I2C_getHandle(I2C_Bus);
    tmp100Data[0].errorCounter = 0;
}

void tmp100_setup_sensor(uint8_t i2c_addr)
{
    tmp100Data[0].enabled = 1;
    tmp100Data[0].address = i2c_addr;
}

uint8_t tmp100_triggerTemeratureConversion(enum TMP100_SENSORS sensor);
uint8_t tmp100_requestTemperature(enum TMP100_SENSORS sensor);

void moveTMP100CIMKStateMachine(uint8_t sensor) 
{
    switch(tmp100Data[sensor].state) {
    case TMP100_IDLE:
	//trigger new conversion
	if(!tmp100_triggerTemeratureConversion(sensor)) {
	    tmp100Data[sensor].state = TMP100_TRIGGERED;
	    //printf("Triggered\n");
	}    
	break;
	
    case TMP100_TRIGGERED:
    {
	struct I2C_CommandResult *i2cResult = I2C_getResult(tmp100Data[sensor].i2cHandle);
	if(i2cResult)
	{
	    if(i2cResult->I2CError)
	    {
		tmp100Data[sensor].state = TMP100_IDLE;
		if(++(tmp100Data[sensor].errorCounter) > 2000)
		{
		    printf("bus reset\n");
		    //reset bus and start over
		    resetI2C(tmp100Data[sensor].i2cHandle);
		    tmp100Data[sensor].errorCounter = 0;
		}
	    }
	    else
	    {
		tmp100Data[sensor].state = TMP100_TRIGGER_FINISHED;
		tmp100Data[sensor].errorCounter = 0;
	    }
	}
	break;
    }
    case TMP100_TRIGGER_FINISHED:
	//request next temperature
	if(!tmp100_requestTemperature(sensor)) {
	    tmp100Data[sensor].state = TMP100_TEMP_REQUESTED;
	}
	break;

    case TMP100_TEMP_REQUESTED:
    {
	struct I2C_CommandResult *i2cResult = I2C_getResult(tmp100Data[sensor].i2cHandle);
	if(i2cResult)
	{
	    if(i2cResult->I2CError)
	    {
		tmp100Data[sensor].state = TMP100_TRIGGER_FINISHED;
	    } else {
		int16_t value = (i2cResult->rxData[0] << 8) + (i2cResult->rxData[1]);

		value = value / 256;
				
		tmp100Data[sensor].temperature = value;
		tmp100Data[sensor].state = TMP100_ACQUIRED_TEMP;
	    }
	}
    }
	break;
    default:
	printf("Error, tmp100 polling statemachine in unknown state\n");
	break;
    }
}


uint8_t tmp100_triggerTemeratureConversion(enum TMP100_SENSORS sensor) 
{
    uint8_t config = 0;
    
    //set in shutdown mode, ie single shot mode
    config |= (1);
    
    //configure as interrupt mode
    config |= (1<<1);
    
    //ignore polarity, it is only good for alert mode
    
    //ignore faulty setting

    //bit 5 and 6 are set to zero means half degree resolution

    //set bit 7 (OS/Alert) to trigger temperature conversion
    config |= (1<<7);

    uint8_t data[3];
    
    //adress of configuration register
    data[0] = 1;
    
    data[1] = config;
    

    //send data to device
    return I2C_writeBytes(tmp100Data[sensor].i2cHandle, data, 2, tmp100Data[sensor].address);
}

uint8_t tmp100_requestTemperature(enum TMP100_SENSORS sensor)
{
    uint8_t tempRequest = 0;    
    return I2C_writeReadBytes(tmp100Data[sensor].i2cHandle, &tempRequest, 1, 2, tmp100Data[sensor].address);
}

uint8_t tmp100_getTemperature(enum TMP100_SENSORS sensor, int32_t* val)
{
    moveTMP100CIMKStateMachine(sensor);
    if(tmp100Data[sensor].state == TMP100_ACQUIRED_TEMP) 
    {
	*val = tmp100Data[sensor].temperature;
	tmp100Data[sensor].state = TMP100_IDLE;
	return 0;
    }
    return 1;  
}

