
#ifndef __SPI_H
#define __SPI_H

#include "stm32f10x_type.h"

enum LS7366Devices {
  LS7366_FRONT_LEFT,
  LS7366_FRONT_RIGHT,
  LS7366_REAR_LEFT,
  LS7366_REAR_RIGHT,
};

struct LS7366Data {
  u32 curTicks;
  u32 lastTicks;
};

enum SPIModes {
  SPI_READ,
  SPI_WRITE,
  SPI_WRITE_READ,
  SPI_FINISHED,
};


extern struct LS7366Data LS7366Data[4];

void initLS7366();
void requestLS7366Data();
u8 initLS7366Finished();
u8 LS7366DataReady();


#endif
