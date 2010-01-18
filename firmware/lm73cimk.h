#ifndef __LM73CIMK_H
#define __LM73CIMK_H

#include "inc/stm32f10x_type.h"

enum lm73cimkStates {
  LM73_IDLE,
  LM73_TRIGGERED,
  LM73_TEMP_REQUESTED,
  LM73_ERROR,
  LM73_ACQUIRED_TEMP,
};

extern enum lm73cimkStates lm73cimkState;

u8 getTemperature(u8 addr, u32 *val);

void setupI2CForLM73CIMK();

//for internal use only
u8 lm73cimk_triggerTemeratureConversation(u8 addr);

u8 requestTemperature(u8 addr);


#endif
