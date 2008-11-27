#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f10x_type.h"
#include "controller_interface.h"

#undef printf

extern int print(const char *format);
extern int printf(const char *format, ...);

extern volatile controller_func controller;

#endif
