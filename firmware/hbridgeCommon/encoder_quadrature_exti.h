#ifndef ENCODER_QUADRATURE_EXTI_H
#define ENCODER_QUADRATURE_EXTI_H

void encoderInitQuadratureWithZero();
void setTicksPerTurnQuadratureWithZero(uint32_t ticks, uint8_t tickDivider);
uint32_t getTicksQuadratureWithZero();

#endif