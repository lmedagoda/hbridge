#ifndef HBRIDGE_HPP
#define HBRIDGE_HPP

#ifndef __orogen
#include <stdint.h>
#include <stdexcept>
#include <iostream>
#endif

#include <base/time.h>

namespace hbridge
{
#define HBRIGE_MAXIMUM_BOARDS 8
#ifndef __orogen
    const int TICKS_PER_TURN = 512 * 729 / 16;
    const int BOARD_COUNT = HBRIGE_MAXIMUM_BOARDS;
    
#endif /* __orogen */

    typedef int64_t Ticks;

    struct Configuration
    {
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

#ifndef __orogen
        /**
         * Initialise all fields of the configuration structure with 0
         */
        Configuration() :
            openCircuit(0), activeFieldCollapse(0), externalTempSensor(0),
            cascadedPositionController(0), pidDebugActive(0), maxMotorTemp(0),
            maxMotorTempCount(0), maxBoardTemp(0), maxBoardTempCount(0),
            timeout(0), maxCurrent(0), maxCurrentCount(0), pwmStepPerMs(0)
        {}
#endif
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
	unsigned int ticksPerTurn;
	unsigned char tickDivider;
	unsigned int ticksPerTurnDivided;
	Ticks zeroPosition;
	enum ENCODER_TYPE type;
#ifndef __orogen
        /**
         * Initialise with sane values
         */
        EncoderConfiguration() :
            ticksPerTurn(0), tickDivider(1), ticksPerTurnDivided(0), zeroPosition(0), type(ENCODER_NONE)
        {}
        
        EncoderConfiguration(uint32_t ticksPerTurn, uint8_t tickDivider, ENCODER_TYPE type) :
	    ticksPerTurn(ticksPerTurn), tickDivider(tickDivider), zeroPosition(0), type(type)
	{
	    validate();
	}
	
	void setZeroPosition(Ticks zeroPos) 
	{
	    zeroPosition = zeroPos;
	}
	
	void validate() 
	{
	    if(tickDivider == 0)
		throw std::out_of_range("Invalid tick divider given");
	    
	    this->ticksPerTurnDivided = ticksPerTurn / tickDivider;
	}
#endif
    };    
    enum MOTOR_ID
    {
        MOTOR_REAR_LEFT  = 0,
        MOTOR_REAR_RIGHT = 1,
        MOTOR_FRONT_RIGHT = 2,
        MOTOR_FRONT_LEFT = 3
    };
    
    struct ErrorState {
	bool motorOverheated;
	bool boardOverheated;
	bool overCurrent;
	bool timeout;
	bool badConfig;
	bool encodersNotInitialized;
        bool hardwareShutdown;
#ifndef __orogen

	ErrorState() :
            motorOverheated(false), boardOverheated(false), overCurrent(false), timeout(false), badConfig(false), encodersNotInitialized(false), hardwareShutdown(false)
        {}
        
        bool hasError() 
        {
	    return (motorOverheated || boardOverheated || overCurrent || timeout || badConfig || encodersNotInitialized || hardwareShutdown);
	}
		
	void printError()
	{
	    std::cout << "MotorOverheated      :" << motorOverheated << std::endl;
	    std::cout << "boardOverheated      :" << boardOverheated << std::endl;
	    std::cout << "overCurrent          :" << overCurrent << std::endl;
	    std::cout << "timeout              :" << timeout << std::endl;
	    std::cout << "badConfig            :" << badConfig << std::endl;
	    std::cout << "encodersNotInitialized:" << encodersNotInitialized << std::endl;
	    std::cout << "hardwareShutdown     :" << hardwareShutdown << std::endl;
	}
	
	ErrorState operator + (ErrorState const& es) const
	{
	    ErrorState ret;
	    ret.motorOverheated = this->motorOverheated || es.motorOverheated;
	    ret.boardOverheated = this->boardOverheated || es.boardOverheated;
	    ret.overCurrent     = this->overCurrent || es.overCurrent;
	    ret.timeout         = this->timeout || es.timeout;
	    ret.badConfig       = this->badConfig || es.badConfig;
	    ret.encodersNotInitialized = this->encodersNotInitialized || es.encodersNotInitialized;
	    ret.hardwareShutdown = this->hardwareShutdown || es.hardwareShutdown;
	    return ret;
	}
#endif
    };
    
    /**
     * Drive mode constants for the hbridges
     */
    enum DRIVE_MODE
    {
        DM_PWM = 0,
        DM_SPEED = 1,
        DM_POSITION = 2,
        DM_UNINITIALIZED = 3
    };

    struct BoardState
    {
        int index;
        int current;
        Ticks position;
        Ticks positionExtern;
	int temperature;
        int delta;
        float pwm;
	struct ErrorState error;
        base::Time can_time;
    };
}

#endif /* HBRIDGE_DRIVER_HPP */

