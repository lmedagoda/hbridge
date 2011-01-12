#ifndef ENCODER_QUADRATURE_H
#define ENCODER_QUADRATURE_H

void encoderInitQuadrature();
u32 getTicksPerTurnQuadrature(void);
u32 getTicksPerTurnQuadrature(void);
u32 getTicksQuadrature();
u16 getDividedTicksQuadrature();
void setTicksPerTurnQuadrature(u32 ticks, u8 tickDivider);
#endif