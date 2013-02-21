#ifndef ENCODER_HPP
#define ENCODER_HPP

#include "MotorDriverTypes.hpp"

namespace hbridge {

class Encoder
{
private:
    EncoderConfiguration encoderConfig;
    Ticks zeroPosition;
    unsigned int lastPositionInTurn;
    int turns;
    bool gotValidReading;

public:
    Encoder();
    void setConfiguration(EncoderConfiguration &cfg);
    void setZeroPosition(Ticks zeroPos);
    void setRawEncoderValue(unsigned int value);
    //returns all motor turns acumulated since configuration
    double getAbsoluteTurns() const;
    
    //computes the motor tick value for a given absolute turn value
    Ticks getMotorTicksFromAbsoluteTurn(double targetValue) const;
    
    const EncoderConfiguration &getEncoderConfig() const;
};

}

#endif // ENCODER_HPP
