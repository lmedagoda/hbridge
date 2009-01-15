
#include "stm32f10x_can.h"
#include "stm32f10x_rcc.h"
#include "can.h"

#define CAN_BUFFER_SIZE 36

CanRxMsg canRXBuffer[CAN_BUFFER_SIZE];
u8 canRxWritePointer;
u8 canRxReadPointer;
u8 canRxError;

u16 wasincanit = 0;

void CAN_Configuration(void)
{

  /* CAN Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN, ENABLE);

  CAN_InitTypeDef        CAN_InitStructure;

  /* CAN register init */
  CAN_DeInit();
  CAN_StructInit(&CAN_InitStructure);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM=DISABLE;
  CAN_InitStructure.CAN_ABOM=ENABLE;
  CAN_InitStructure.CAN_AWUM=DISABLE;
  CAN_InitStructure.CAN_NART=DISABLE;
  CAN_InitStructure.CAN_RFLM=DISABLE;
  CAN_InitStructure.CAN_TXFP=DISABLE;
  CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
  CAN_InitStructure.CAN_BS1=CAN_BS1_5tq;
  CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;
  CAN_InitStructure.CAN_Prescaler=4;
  if(CAN_Init(&CAN_InitStructure) == CANINITFAILED) {
    print("Can init failed \n");
  }
  



  /* CAN FIFO0 message pending interrupt enable */ 
  CAN_ITConfig(CAN_IT_FMP0, ENABLE);

  /* CAN FIFO0 message pending interrupt enable */ 
  CAN_ITConfig(CAN_IT_FMP1, ENABLE);
}

void CAN_ConfigureFilters(enum hostIDs boardNr) {
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;

  //TODO perhaps filter ide seem to be a bit different(stdId + RTR + IDE + EXId)
  //Emergency stop, setMode and setValue are matched to FIFO0
  CAN_FilterInitStructure.CAN_FilterNumber=0;
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList;
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh=PACKET_ID_EMERGENCY_STOP<<5;
  CAN_FilterInitStructure.CAN_FilterIdLow=PACKET_ID_SET_VALUE<<5;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=PACKET_ID_SET_MODE<<5;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xFFFF;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);

  //configure, configure 2 and set pis values matched to FIFO1
  CAN_FilterInitStructure.CAN_FilterNumber=1;
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList;
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh=(PACKET_ID_SET_CONFIGURE + boardNr)<<5;
  CAN_FilterInitStructure.CAN_FilterIdLow=(PACKET_ID_SET_CONFIGURE2 + boardNr)<<5;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=(PACKET_ID_SET_PID_POS + boardNr)<<5;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=(PACKET_ID_SET_PID_SPEED + boardNr)<<5;;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO1;
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
  
  CAN_EnterNormalMode();
}


void USB_LP_CAN_RX0_IRQHandler(void)
{
  wasincanit++;
  
  vu8 nextRxWritePointer = (canRxWritePointer + 1) % CAN_BUFFER_SIZE;
  while((u8)(CAN->RF0R&(u32)0x03) && nextRxWritePointer != canRxReadPointer) {
    CAN_Receive(CAN_FIFO0, canRXBuffer + canRxWritePointer);
    canRxWritePointer = nextRxWritePointer;

    nextRxWritePointer = (canRxWritePointer + 1) % CAN_BUFFER_SIZE;    
  }  
}

void CAN_RX1_IRQHandler(void)
{
  wasincanit++;
  
  vu8 nextRxWritePointer = (canRxWritePointer + 1) % CAN_BUFFER_SIZE;
  while((u8)(CAN->RF1R&(u32)0x03) && nextRxWritePointer != canRxReadPointer) {
    CAN_Receive(CAN_FIFO1, canRXBuffer + canRxWritePointer);
    canRxWritePointer = nextRxWritePointer;

    nextRxWritePointer = (canRxWritePointer + 1) % CAN_BUFFER_SIZE;    
  }
}

CanRxMsg *CAN_GetNextData() {
  if(canRxWritePointer != canRxReadPointer) {
    return canRXBuffer + canRxReadPointer;
  }
  return 0;
}

void CAN_MarkNextDataAsRead() {
  if(canRxWritePointer != canRxReadPointer) {
    canRxReadPointer = (canRxReadPointer + 1) % CAN_BUFFER_SIZE;
  }  
}
