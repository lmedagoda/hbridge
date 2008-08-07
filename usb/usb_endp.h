
#ifndef __USB_ENDP_H
#define __USB_ENDP_H

#include "usb_type.h"

#define USBBUFFER_SIZE 1024

extern volatile u8 USBBufferIN[USBBUFFER_SIZE];
extern volatile u8 USBBufferOUT[USBBUFFER_SIZE];
extern volatile u16 USBInRead;
extern volatile u16 USBInWrite;
extern volatile u16 USBOutRead;
extern volatile u16 USBOutWrite;

#endif
