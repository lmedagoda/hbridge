#ifndef HBRIDGE_HPP
#define HBRIDGE_HPP

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
    
    struct Configuration
    {
	unsigned short maxPWM;
	unsigned short maxSpeed;
        unsigned char openCircuit;
        unsigned char activeFieldCollapse;
        unsigned char externalTempSensor;
        unsigned char cascadedPositionController;
        unsigned char pidDebugActive;
        unsigned char maxMotorTemp;
        unsigned char maxMotorTempCount;
        unsigned char maxBoardTemp;
        unsigned char maxBoardTempCount;
        unsigned short timeout;
        unsigned short maxCurrent;
        unsigned char maxCurrentCount;
        unsigned short pwmStepPerMs;
	CONTROLLER_INPUT_ENCODER controllerInputEncoder;
        /**
         * Initialise all fields of the configuration structure with 0
         */
        Configuration() :
            openCircuit(0), activeFieldCollapse(0), externalTempSensor(0),
            cascadedPositionController(0), pidDebugActive(0), maxMotorTemp(0),
            maxMotorTempCount(0), maxBoardTemp(0), maxBoardTempCount(0),
            timeout(0), maxCurrent(0), maxCurrentCount(0), pwmStepPerMs(0), controllerInputEncoder(INTERNAL)
        {}
    };
    
    enum ENCODER_TYPE {
        ENCODER_NONE = 0,
        ENCODER_QUADRATURE = 1,
        ENCODER_QUADRATURE_WITH_ZERO = 2,
        ENCODER_IC_HOUSE_MH_Y = 3,
        ENCODER_BMMV30_SSI = 4,
        ENCODER_ANALOG_VOLTAGE = 5
    };

    struct EncoderConfiguration
    {
	//real amount of ticks per turn
	double ticksPerTurn;
	/**
	* the value that is passed to the motor driver 
	* as ticks for one full evolution, as the motor 
	* driver can only process integers. 
	* Note that this values needs to be >= ticksPerTurn
	* and % tickDivider == 0
	* If these constrains are not met, the encoder will 
	* loose ticks and drift.
	**/ 
	unsigned int ticksPerTurnMotorDriver;
	unsigned char tickDivider;
	unsigned int ticksPerTurnDivided;
	double zeroPosition;
	enum ENCODER_TYPE type;

	/**
         * Initialise with sane values
         */
        EncoderConfiguration() :
            ticksPerTurn(0), tickDivider(1), ticksPerTurnDivided(0), zeroPosition(0), type(ENCODER_NONE)
        {}
        
        EncoderConfiguration(double ticksPerTurn, ENCODER_TYPE type) :
	    ticksPerTurn(ticksPerTurn), tickDivider(1), zeroPosition(0), type(type)
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
	    
	    //we calculate a value >= cfg.ticksPerTurn were
	    //value % cfg.ticksPerTurnDivided = 0;    
	    int value = ((int)(ticksPerTurn / tickDivider)) * tickDivider;
	    if(value < ticksPerTurn)
	    {
		value = (((int)(ticksPerTurn / tickDivider)) + 1) * tickDivider;
	    }
	    
	    ticksPerTurnMotorDriver = value;
	    
	    this->ticksPerTurnDivided = ticksPerTurnMotorDriver / tickDivider;
	}
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
    
    struct PositionControllerConfiguration
    {
	PositionControllerConfiguration () :minHystDist(0), maxHystDist(0), 
	hysteresisActive(false), allowWrapAround(false), overDistCount(0) {};
	
	double minHystDist;
	double maxHystDist;
	bool hysteresisActive;
	bool allowWrapAround;
	short overDistCount;
    };
    
    struct PID_Debug
    {
	PID_Debug() : pPart(0), iPart(0), dPart(0), minMaxPidOutput(0) {};
	signed short pPart;
	signed short iPart;
	signed short dPart;
	unsigned short minMaxPidOutput;
    };
    
    struct PositionControllerDebug
    {
	PositionControllerDebug() : targetValue(0), pwmValue(0), encoderValue(0), positionValue(0) {};
	unsigned short targetValue;
	signed short pwmValue;
	unsigned int encoderValue;
	unsigned int positionValue;
	PID_Debug pidDebug;
    };
    
    struct SpeedControllerDebug
    {
	SpeedControllerDebug() : targetValue(0), pwmValue(0), encoderValue(0), speedValue(0) {};
	unsigned short targetValue;
	signed short pwmValue;
	unsigned int encoderValue;
	unsigned int speedValue;
	PID_Debug pidDebug;
    };

    struct MotorConfiguration {
        Configuration base_config;
	EncoderConfiguration encoder_config_intern;
	EncoderConfiguration encoder_config_extern;
        base::actuators::DRIVE_MODE    mode;
        base::actuators::PIDValues     pid_position;
        base::actuators::PIDValues     pid_speed;
    };
}

#endif /* HBRIDGE_DRIVER_HPP */

