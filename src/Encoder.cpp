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
    gotValidReading = true;
    
    //reverse scale down for transmission
    value *= encoderConfig.tickDivider;
    
    int diff = value - lastPositionInTurn;
    lastPositionInTurn = value;
    
    // We assume that a motor rotates less than half a turn per [ms]
    // (a status packet is sent every [ms])
    if ((uint) abs(diff) > encoderConfig.ticksPerTurn / 2)
    {
	turns += (diff < 0 ? 1 : -1);
    }
}

double Encoder::getAbsoluteTurns() const
{
    if(!gotValidReading)
	return base::unknown<double>();
    
    double ret = turns + ((double)lastPositionInTurn) / encoderConfig.ticksPerTurn;
    return ret - zeroPosition;
}

const EncoderConfiguration& Encoder::getEncoderConfig() const
{
    return encoderConfig;
}

}
