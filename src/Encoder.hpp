#ifndef ENCODER_HPP
#define ENCODER_HPP

#include "MotorDriverTypes.hpp"

namespace hbridge {

class Encoder
{
private:
    EncoderConfiguration encoderConfig;
    double zeroPosition;
    unsigned int lastPositionInTurn;
    int turns;
    bool gotValidReading;

public:
    Encoder();
    void setConfiguration(EncoderConfiguration &cfg);
    void setZeroPosition(double zeroPos);
    void setRawEncoderValue(unsigned int value);
    //returns all motor turns acumulated since configuration
    double getAbsoluteTurns() const;
    
    const EncoderConfiguration &getEncoderConfig() const;
};

}

#endif // ENCODER_HPP
