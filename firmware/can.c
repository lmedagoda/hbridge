
#include "inc/stm32f10x_can.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_gpio.h"
#include "can.h"
#include "printf.h"

#define CAN_BUFFER_SIZE 36

CanRxMsg canRXBuffer[CAN_BUFFER_SIZE];
u8 canRxWritePointer;
u8 canRxReadPointer;
u8 canRxError;

u16 wasincanit = 0;

void CAN_Configuration(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);

	//Configure CAN pin: RX 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //Configure CAN pin: TX 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	
    /* CAN Periph clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN, ENABLE);

    CAN_InitTypeDef CAN_InitStructure;

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

void CAN_CancelAllTransmits() {
    int i;
    for(i = 0; i < 3; i++) {
    if(CAN_TransmitStatus(i) == CANTXPENDING)
	CAN_CancelTransmit(i);
    }    
}

void CAN_ConfigureFilters(enum hostIDs boardNr) {
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;

  //TODO perhaps filter ide seem to be a bit different(stdId + RTR + IDE + EXId)
  //Emergency stop, setMode and setValue are matched to FIFO0
  CAN_FilterInitStructure.CAN_FilterNumber=0;
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList;
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh=PACKET_ID_EMERGENCY_STOP<<5;
  if(boardNr == RECEIVER_ID_H_BRIDGE_1 || boardNr == RECEIVER_ID_H_BRIDGE_2 || boardNr == RECEIVER_ID_H_BRIDGE_3 || boardNr == RECEIVER_ID_H_BRIDGE_4) {
    CAN_FilterInitStructure.CAN_FilterIdLow=PACKET_ID_SET_VALUE14<<5;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=PACKET_ID_SET_MODE14<<5;
  } else {
    CAN_FilterInitStructure.CAN_FilterIdLow=PACKET_ID_SET_VALUE58<<5;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=PACKET_ID_SET_MODE58<<5;      
  }
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=(PACKET_ID_ENCODER_CONFIG + boardNr)<<5;
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
  int received = 0;
  wasincanit++;
  
  vu8 nextRxWritePointer = (canRxWritePointer + 1) % CAN_BUFFER_SIZE;
  while((u8)(CAN->RF0R&(u32)0x03) && nextRxWritePointer != canRxReadPointer) {
    CAN_Receive(CAN_FIFO0, canRXBuffer + canRxWritePointer);
    canRxWritePointer = nextRxWritePointer;

    nextRxWritePointer = (canRxWritePointer + 1) % CAN_BUFFER_SIZE;
    received = 1;
  }

  //discard message to avoid interrupt strom
  if(!received) {
    CanRxMsg nirvana;
    CAN_Receive(CAN_FIFO0, &nirvana);
  }
}

void CAN_RX1_IRQHandler(void)
{
  int received = 0;
  wasincanit++;
  
  vu8 nextRxWritePointer = (canRxWritePointer + 1) % CAN_BUFFER_SIZE;
  while((u8)(CAN->RF1R&(u32)0x03) && nextRxWritePointer != canRxReadPointer) {
    CAN_Receive(CAN_FIFO1, canRXBuffer + canRxWritePointer);
    canRxWritePointer = nextRxWritePointer;

    nextRxWritePointer = (canRxWritePointer + 1) % CAN_BUFFER_SIZE;    
    received = 1;
  }
  //discard message to avoid interrupt strom
  if(!received) {
    CanRxMsg nirvana;
    CAN_Receive(CAN_FIFO1, &nirvana);
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
