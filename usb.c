
#include "stm32f10x_it.h"
#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "platform_config.h"
#include "usb_pwr.h"
#include "usb_endp.h"


u8 buffer_in[VIRTUAL_COM_PORT_DATA_SIZE];
extern u32 count_in;


/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz)
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
  /* USBCLK = PLLCLK / 1.5 */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  /* Enable USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

/*******************************************************************************
* Function Name  : Enter_LowPowerMode
* Description    : Power-off system clocks and power while entering suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void)
{
  /* Set the device state to suspend */
  bDeviceState = SUSPENDED;
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode
* Description    : Restores system clocks and power while exiting suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }
}

/*******************************************************************************
* Function Name  : USB_Cable_Config
* Description    : Software Connection/Disconnection of USB Cable
* Input          : None.
* Return         : Status
*******************************************************************************/
void USB_Cable_Config (FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
  else
  {
    GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
}

/*******************************************************************************
* Function Name  : USB_To_UART_Send_Data.
* Description    : send the received data from USB to the UART 0.
* Input          : data_buffer: data address.
                   Nb_bytes: number of bytes to send.
* Return         : none.
*******************************************************************************/
void USB_To_USART_Send_Data(u8* data_buffer, u8 Nb_bytes)
{
  /*
  u32 i;

  for (i = 0; i < Nb_bytes; i++)
  {
    USART_SendData(USART1, *(data_buffer + i));
  }
  */
}

/*******************************************************************************
* Function Name  : UART_To_USB_Send_Data.
* Description    : send the received data from UART 0 to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
void USART_To_USB_Send_Data(void)
{
  /*
  if (USART_InitStructure.USART_WordLength == USART_WordLength_8b)
  {
    buffer_in[count_in] = USART_ReceiveData(USART1) & 0x7F;
  }
  else if (USART_InitStructure.USART_WordLength == USART_WordLength_9b)
  {
    buffer_in[count_in] = USART_ReceiveData(USART1);
  }
  count_in++;
  UserToPMABufferCopy(buffer_in, ENDP1_TXADDR, count_in);
  SetEPTxCount(ENDP1, count_in);
  SetEPTxValid(ENDP1);
  */
}

void USB_Send_Data(const u8 *buffer, u32 count) {
  if(bDeviceState == CONFIGURED) {
    int size = 0;
    int timeout = 0;
    volatile int free_space = 0;
    
    free_space = (USBBUFFER_SIZE - (USBOutWrite - USBOutRead) - 1) % USBBUFFER_SIZE;
    if(free_space < count) {
      return;
    }
    
    /*
    do {
      free_space = (USBBUFFER_SIZE - (USBOutWrite - USBOutRead) - 1) % USBBUFFER_SIZE;
      timeout++;
      if(timeout > 10000)
	return;
    } while (free_space < count) ;
    */
    volatile int noTransmissionBeforeCopy = _GetEPTxStatus(ENDP1) == EP_TX_NAK;
    
    //write to ringbuffer
    while((((USBOutWrite +1) % USBBUFFER_SIZE) != USBOutRead) 
	  && size < count) {
      USBBufferOUT[USBOutWrite] = buffer[size];
      USBOutWrite = (USBOutWrite + 1) % USBBUFFER_SIZE;
      size++;
    }

    if(_GetEPTxStatus(ENDP1) == EP_TX_NAK && noTransmissionBeforeCopy) {
      size = 0;
      u8 localbuffer[BULK_MAX_PACKET_SIZE];
      while((USBOutRead != USBOutWrite) && size < BULK_MAX_PACKET_SIZE) {
	localbuffer[size] =  USBBufferOUT[USBOutRead];
	size++;
	USBOutRead = (USBOutRead + 1) % USBBUFFER_SIZE;
      }
      UserToPMABufferCopy(localbuffer, ENDP1_TXADDR, size);
      SetEPTxCount(ENDP1, size);
      SetEPTxValid(ENDP1);
    }
    /*
    //transfer in progress
    if(USBOutWrite != USBOutRead || _GetEPTxStatus(ENDP1) != EP_TX_NAK) {
      //write to ringbuffer
      while((((USBOutWrite +1) % USBBUFFER_SIZE) != USBOutRead) 
	    && size < count) {
	USBBufferOUT[USBOutWrite] = buffer[size];
	USBOutWrite = (USBOutWrite + 1) % USBBUFFER_SIZE;
	size++;
      }

      //there could be the extreme rare case, that transmission
      //finished in the moment we wrote data to the ringbuffer
      if(_GetEPTxStatus(ENDP1) == EP_TX_NAK && USBOutWrite != USBOutRead) {
	size = 0;
	u8 localbuffer[BULK_MAX_PACKET_SIZE];
	while((USBOutRead != USBOutWrite) && size < BULK_MAX_PACKET_SIZE) {
	  localbuffer[size] =  USBBufferOUT[USBOutRead];
	  size++;
	  USBOutRead = (USBOutRead + 1) % USBBUFFER_SIZE;
	}
	UserToPMABufferCopy(localbuffer, ENDP1_TXADDR, size);
	SetEPTxCount(ENDP1, size);
	SetEPTxValid(ENDP1);
      }
    } else {
      //start new transfer
      if(count > BULK_MAX_PACKET_SIZE) {
	//transmission to big for one packet
	//copy the rest into the ringbuffer and start transfer
	size = BULK_MAX_PACKET_SIZE;
	while((((USBOutWrite +1) % USBBUFFER_SIZE) != USBOutRead) 
	      && size < count) {
	  USBBufferOUT[USBOutWrite] = buffer[size];
	  size++;
	  USBOutWrite = (USBOutWrite + 1) % USBBUFFER_SIZE;
	}
	UserToPMABufferCopy(buffer, ENDP1_TXADDR, BULK_MAX_PACKET_SIZE);
	SetEPTxCount(ENDP1, BULK_MAX_PACKET_SIZE);
      } else {
	//transmission smaller than packet
	//just transfer the buffer
	UserToPMABufferCopy(buffer, ENDP1_TXADDR, count);
	SetEPTxCount(ENDP1, count);
      }
      SetEPTxValid(ENDP1);
      }*/
  }
}


/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
  u32 Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(vu32*)(0x1FFFF7E8);
  Device_Serial1 = *(vu32*)(0x1FFFF7EC);
  Device_Serial2 = *(vu32*)(0x1FFFF7F0);

  if (Device_Serial0 != 0)
  {
    Virtual_Com_Port_StringSerial[2] = (u8)(Device_Serial0 & 0x000000FF);
    Virtual_Com_Port_StringSerial[4] = (u8)((Device_Serial0 & 0x0000FF00) >> 8);
    Virtual_Com_Port_StringSerial[6] = (u8)((Device_Serial0 & 0x00FF0000) >> 16);
    Virtual_Com_Port_StringSerial[8] = (u8)((Device_Serial0 & 0xFF000000) >> 24);

    Virtual_Com_Port_StringSerial[10] = (u8)(Device_Serial1 & 0x000000FF);
    Virtual_Com_Port_StringSerial[12] = (u8)((Device_Serial1 & 0x0000FF00) >> 8);
    Virtual_Com_Port_StringSerial[14] = (u8)((Device_Serial1 & 0x00FF0000) >> 16);
    Virtual_Com_Port_StringSerial[16] = (u8)((Device_Serial1 & 0xFF000000) >> 24);

    Virtual_Com_Port_StringSerial[18] = (u8)(Device_Serial2 & 0x000000FF);
    Virtual_Com_Port_StringSerial[20] = (u8)((Device_Serial2 & 0x0000FF00) >> 8);
    Virtual_Com_Port_StringSerial[22] = (u8)((Device_Serial2 & 0x00FF0000) >> 16);
    Virtual_Com_Port_StringSerial[24] = (u8)((Device_Serial2 & 0xFF000000) >> 24);
  }
}
