#ifndef HBRIDGE_TYPES_HPP
#define HBRIDGE_TYPES_HPP

#include <stdint.h>
#include <stdexcept>
#include <iostream>

#include <base/time.h>
#include <base/actuators/commands.h>

namespace hbridge
{
    const int BOARD_COUNT = 8;
    
    typedef int64_t Ticks;

    enum CONTROLLER_INPUT_ENCODER {
        INTERNAL = 1,
        EXTERNAL = 2
    };
    
    struct ActuatorConfiguration
    {
        std::string name;
	unsigned short maxPWM;
        unsigned char openCircuit;
        unsigned char maxMotorTemp;
        unsigned char maxBoardTemp;
        unsigned short timeout;
        unsigned short maxCurrent;
        unsigned char maxCurrentCount;
        unsigned short pwmStepPerMs;
	CONTROLLER_INPUT_ENCODER controllerInputEncoder;
        /**
         * Initialise all fields of the configuration structure with 0
         */
        ActuatorConfiguration() :
        {}
    };
    
    enum ENCODER_TYPE {
        ENCODER_NONE = 0,
        ENCODER_QUADRATURE = 1,
        ENCODER_QUADRATURE_WITH_ZERO = 2,
        ENCODER_IC_HOUSE_MH_Y = 3,
        ENCODER_BMMV30_SSI = 4,
        ENCODER_ANALOG_VOLTAGE = 5,
        ENCODER_END_SWITCH = 6,
    };

    struct EncoderConfiguration
    {
	///real amount of ticks per turn
	uint32_t ticksPerTurn;
	
	/**
	 * If this value is != 0 every
	 * leapTickValue encoder ticks,
	 * the encoder value will be increased/decreased
	 * by one. This is usefull if your gear/encoder ratio
	 * is not even.
	 * */
	uint32_t leapTickValue;
	unsigned char tickDivider;
	unsigned int ticksPerTurnDivided;
	double zeroPosition;
	enum ENCODER_TYPE type;

	/**
         * Initialise with sane values
         */
        EncoderConfiguration() :
            ticksPerTurn(0), leapTickValue(0), tickDivider(1), ticksPerTurnDivided(0), zeroPosition(0), type(ENCODER_NONE)
        {}
        
        EncoderConfiguration(uint32_t ticksPerTurn, uint32_t leapTickValue, ENCODER_TYPE type) :
	    ticksPerTurn(ticksPerTurn), leapTickValue(leapTickValue), tickDivider(1), zeroPosition(0), type(type)
	{
	    validate();
	}
	
	void setZeroPosition(double zeroPos) 
	{
	    zeroPosition = zeroPos;
	}
	
	void validate() 
	{
	    if(tickDivider == 0)
		throw std::out_of_range("Invalid tick divider given");
	    
	    this->ticksPerTurnDivided = ticksPerTurn / tickDivider;
	}
    };    
    
    struct SensorConfiguration
    {
	SensorConfiguration() : externalTempSensor(0), statusFrequency(10)
	{
	}
	
        std::string name;
	
        unsigned char externalTempSensor;
	/**
	 * The frequency in hz in which the firmware should
	 * report the sensor status. Not, only 'round
	 * /integer' frequencies are allowed.
	 * */
	int statusFrequency;
	
	EncoderConfiguration encoder_config_intern;
	EncoderConfiguration encoder_config_extern;
    };
    
    struct ErrorState {
	bool motorOverheated;
	bool boardOverheated;
	bool overCurrent;
	bool timeout;
	bool badConfig;
	bool encodersNotInitialized;
        bool hardwareShutdown;
        bool emergencyOff;

	ErrorState() :
	  motorOverheated(false), boardOverheated(false), overCurrent(false), timeout(false), badConfig(false), encodersNotInitialized(false), hardwareShutdown(false), emergencyOff(false)
        {}
        
	/** Returns true if any of the error flags is set */
        bool hasError() const
        {
	    return hasConfigurationError() || timeout;
	}

	/** Returns true if an error that forbids driving the motors is set. In
	 * practice, it is any error but timeout */
        bool hasConfigurationError() const
        {
	    return (motorOverheated || boardOverheated || overCurrent || badConfig || encodersNotInitialized || hardwareShutdown || emergencyOff);
	}

        bool operator != (ErrorState const& other) const
        {
            return
                motorOverheated        != other.motorOverheated ||
                boardOverheated        != other.boardOverheated ||
                overCurrent            != other.overCurrent ||
                timeout                != other.timeout ||
                badConfig              != other.badConfig ||
                encodersNotInitialized != other.encodersNotInitialized ||
                hardwareShutdown       != other.hardwareShutdown ||
	        emergencyOff           != other.emergencyOff;
        }

        bool operator == (ErrorState const& other) const
        { return !(other != *this); }
		
	void printError()
	{
	    std::cout << "MotorOverheated      :" << motorOverheated << std::endl;
	    std::cout << "boardOverheated      :" << boardOverheated << std::endl;
	    std::cout << "overCurrent          :" << overCurrent << std::endl;
	    std::cout << "timeout              :" << timeout << std::endl;
	    std::cout << "badConfig            :" << badConfig << std::endl;
	    std::cout << "encodersNotInitialized:" << encodersNotInitialized << std::endl;
	    std::cout << "hardwareShutdown     :" << hardwareShutdown << std::endl;
	    std::cout << "emergencyOff         :" << emergencyOff<< std::endl;
	}
	
	ErrorState operator | (ErrorState const& es) const
	{
	    ErrorState ret;
	    ret.motorOverheated = this->motorOverheated || es.motorOverheated;
	    ret.boardOverheated = this->boardOverheated || es.boardOverheated;
	    ret.overCurrent     = this->overCurrent || es.overCurrent;
	    ret.timeout         = this->timeout || es.timeout;
	    ret.badConfig       = this->badConfig || es.badConfig;
	    ret.encodersNotInitialized = this->encodersNotInitialized || es.encodersNotInitialized;
	    ret.hardwareShutdown = this->hardwareShutdown || es.hardwareShutdown;
	    ret.emergencyOff = this->emergencyOff || es.emergencyOff;
	    return ret;
	}
    };
    
    /**
     * Drive mode constants for the hbridges
     */
    typedef base::actuators::DRIVE_MODE DRIVE_MODE;

    struct BoardState
    {
        int index;
        int current;
	//position of first encoder in turns
        double position;
	//position of second encoder in turns
        double positionExtern;
	float temperature;
	float motorTemperature;
        float pwm;
	struct ErrorState error;
        base::Time can_time;
    };
    
    struct MotorConfiguration {
	SensorConfiguration sensorConfig;
	ActuatorConfiguration actuatorConfig;
    };
}

#endif /* HBRIDGE_DRIVER_HPP */

