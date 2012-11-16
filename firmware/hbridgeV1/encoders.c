#include "../interfaces/encoder.h"
#include "../hbridgeCommon/encoder_adc.h"
#include "../hbridgeCommon/encoder_quadrature.h"
#include "../hbridgeCommon/encoder_ichaus.h"
#include "../hbridgeCommon/encoder_quadrature_exti.h"

void encodersInit()
{
    struct EncoderInterface encoder;
    defaultInitEncoder(&encoder);

    encoder.encoderInit = encoderInitQuadrature;
    encoder.getTicks = getTicksQuadrature;
    encoder.setTicksPerTurn = setTicksPerTurnQuadrature;
    setEncoderImplementation(QUADRATURE, encoder);

    encoder.encoderInit = encoderInitQuadratureWithZero;
    encoder.getTicks = getTicksQuadratureWithZero;
    encoder.setTicksPerTurn = setTicksPerTurnQuadratureWithZero;
    setEncoderImplementation(QUADRATURE_WITH_ZERO, encoder);

    encoder.encoderInit = encoderInitIcHaus;
    encoder.getTicks = getTicksIcHaus;
    encoder.setTicksPerTurn = setTicksPerTurnIcHaus;
    setEncoderImplementation(IC_HOUSE_MH_Y, encoder);
    
    encoder.encoderInit = encoderInitADC;
    encoder.encoderDeInit = encoderDeInitADC;
    encoder.getTicks = getTicksADC;
    encoder.setTicksPerTurn = setTicksPerTurnADC;
    setEncoderImplementation(ANALOG_VOLTAGE, encoder);
}