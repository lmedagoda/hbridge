#ifndef __PRINTF_H
#define __PRINTF_H

#include "inc/stm32f10x_type.h"
#undef printf

int print(const char *format);
int printf(const char *format, ...);

void testprintf();

#endif
