
#ifndef __USB_H
#define __USB_H

void Set_USBClock(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Cable_Config (FunctionalState NewState);
void USB_To_USART_Send_Data(u8* data_buffer, u8 Nb_bytes);
void USART_To_USB_Send_Data(void);
void USB_Send_Data(const u8 *buffer, u32 count);
void Get_SerialNum(void);

#endif
