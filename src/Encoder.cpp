#include "Encoder.hpp"
#include <base/float.h>

namespace hbridge {
    
    
Encoder::Encoder()
{
    lastPositionInTurn = 0;
    turns = 0;
    zeroPosition = 0;
    gotValidReading = false;    

}

void Encoder::setConfiguration(EncoderConfiguration& cfg)
{
    if(encoderConfig.ticksPerTurn == cfg.ticksPerTurn)
        return;
    
    encoderConfig = cfg;
    
    //we go sane and reset the encoder if wrap value changes
    lastPositionInTurn = 0;
    turns = 0;
    gotValidReading = false;    
}

void Encoder::setZeroPosition(Ticks zeroPos) 
{
    zeroPosition = zeroPos;
};

void Encoder::setRawEncoderValue(uint value)
{
    //reverse scale down for transmission
    value *= encoderConfig.tickDivider;
    
    if(!gotValidReading)
    {
	if(value > 0)
	{
	    gotValidReading = true;
	    lastPositionInTurn = value;
	}
    }
    else 
    {
	int diff = value - lastPositionInTurn;
	lastPositionInTurn = value;
	
	// We assume that a motor rotates less than half a turn per [ms]
	// (a status packet is sent every [ms])
	if ((uint) abs(diff) > encoderConfig.ticksPerTurnMotorDriver / 2)
	{
	    turns += (diff < 0 ? 1 : -1);
	}
    }
}

double Encoder::getAbsoluteTurns() const
{
    if(!gotValidReading)
	return base::unknown<double>();
    
    Ticks allMotorTicks = turns* ((int64_t)encoderConfig.ticksPerTurnMotorDriver) + ((int)lastPositionInTurn) - zeroPosition;
    double turns = allMotorTicks / encoderConfig.ticksPerTurn;
    return turns;
}


Ticks Encoder::getMotorTicksFromAbsoluteTurn(double targetValue) const
{
    double curPos = getAbsoluteTurns();
    if(fabs(curPos - targetValue) > 1)
	throw std::runtime_error("Target value is more than one turn apart");
    
    if(base::isUnknown<double>(curPos))
	throw std::runtime_error("Tried to access encoder while it is unknown (did not pass zero mark yet)");
    
    int64_t target_ticks = targetValue * encoderConfig.ticksPerTurn + zeroPosition;
    int64_t targetInTurn_ticks = target_ticks % encoderConfig.ticksPerTurnMotorDriver;
    
    return targetInTurn_ticks;
}


const EncoderConfiguration& Encoder::getEncoderConfig() const
{
    return encoderConfig;
}

}
