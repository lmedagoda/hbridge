#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "protocol.h"

extern volatile enum hostIDs ownHostId;

void baseNvicInit();
void baseInit();
void pollCanMessages(); 
void SysTick_Configuration(void);

#endif
