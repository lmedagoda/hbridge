#ifndef ENCODER_QUADRATURE_H
#define ENCODER_QUADRATURE_H

void encoderInitQuadrature();
u32 getTicksQuadrature();
void setTicksPerTurnQuadrature(u32 ticks, u8 tickDivider);

void encoderInitQuadratureV2();
u32 getTicksQuadratureV2();
void setTicksPerTurnQuadratureV2(u32 ticks, u8 tickDivider);

void encoderInitQuadratureWithZeroV2();
u32 getTicksQuadratureWithZeroV2();
void setTicksPerTurnQuadratureWithZeroV2(u32 ticks, u8 tickDivider);

#endif