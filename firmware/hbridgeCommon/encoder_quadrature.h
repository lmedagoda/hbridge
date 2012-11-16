#ifndef ENCODER_QUADRATURE_H
#define ENCODER_QUADRATURE_H

void encoderInitQuadrature();
uint32_t getTicksQuadrature();
void setTicksPerTurnQuadrature(uint32_t ticks, uint8_t tickDivider);

void encoderInitQuadratureV2();
uint32_t getTicksQuadratureV2();
void setTicksPerTurnQuadratureV2(uint32_t ticks, uint8_t tickDivider);

void encoderInitQuadratureWithZeroV2();
uint32_t getTicksQuadratureWithZeroV2();
void setTicksPerTurnQuadratureWithZeroV2(uint32_t ticks, uint8_t tickDivider);

#endif
