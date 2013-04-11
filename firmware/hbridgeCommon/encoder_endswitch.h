#ifndef ENCODER_END_SWITCH_H
#define ENCODER_END_SWITCH_H
#include <stdint.h>

void endSwitchEncoder_Init();
void endSwitchEncoder_setTicksPerTurn(uint32_t ticks, uint8_t tickDivider);
uint32_t endSwitchEncoder_getTicks(void);
void endSwitchEncoder_encoderDeInit(void);

#endif
