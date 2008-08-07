/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : usb_endp.c
* Author             : MCD Application Team
* Version            : V2.2.0
* Date               : 06/13/2008
* Description        : Endpoint routines
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "hw_config.h"
#include "usb_istr.h"
#include "usb_endp.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile u8 USBBufferIN[USBBUFFER_SIZE];
volatile u8 USBBufferOUT[USBBUFFER_SIZE];
volatile u16 USBInRead = 0;
volatile u16 USBInWrite = 0;
volatile u16 USBOutRead = 0;
volatile u16 USBOutWrite = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : EP3_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP3_OUT_Callback(void)
{
  u8 localbuffer[BULK_MAX_PACKET_SIZE];
  
  u32 count_out = 0;
  count_out = GetEPRxCount(ENDP3);
  //assert_param(count_out <= 32);
  PMAToUserBufferCopy(localbuffer, ENDP3_RXADDR, count_out);

  int i;

  //Note, there is no check for buffer wrap arounds here
  for(i = 0; i < count_out; i++) {
    USBBufferIN[USBInWrite] = localbuffer[i];
    USBInWrite = (USBInWrite + 1) % USBBUFFER_SIZE;
  }
  SetEPRxValid(ENDP3);
}

/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback(void)
{
  u8 localbuffer[BULK_MAX_PACKET_SIZE];
  int size = 0;

  while((USBOutRead != USBOutWrite) && size < BULK_MAX_PACKET_SIZE) {
    localbuffer[size] =  USBBufferOUT[USBOutRead];
    size++;
    USBOutRead = (USBOutRead + 1) % USBBUFFER_SIZE;
  }

  if(size) {
    UserToPMABufferCopy(localbuffer, ENDP1_TXADDR, size);
    SetEPTxCount(ENDP1, size);
    SetEPTxValid(ENDP1);
  }
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/

