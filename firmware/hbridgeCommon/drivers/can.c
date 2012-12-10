
#include "inc/stm32f10x_can.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_gpio.h"
#include "core_cm3.h"
#include "can.h"
#include "printf.h"

#define CAN_BUFFER_SIZE 36

CanRxMsg canRXBuffer[CAN_BUFFER_SIZE];
uint8_t canRxWritePointer;
uint8_t canRxReadPointer;
uint8_t canRxError;

void CAN_Configuration(enum CAN_REMAP remap)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);

    switch(remap)
    {
	case CAN_NO_REMAP:
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	    
	    //Configure CAN pin: RX 
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_Init(GPIOA, &GPIO_InitStructure);

	    //Configure CAN pin: TX 
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	    GPIO_Init(GPIOA, &GPIO_InitStructure);
	    break;
	case CAN_REMAP1:
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	    //do the remap
	    GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);
	    
	    //Configure CAN pin: RX 
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_Init(GPIOB, &GPIO_InitStructure);

	    //Configure CAN pin: TX 
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	    GPIO_Init(GPIOB, &GPIO_InitStructure);
	    break;
	case CAN_REMAP2:
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

	    //do the remap
	    GPIO_PinRemapConfig(GPIO_Remap2_CAN1, ENABLE);	    

	    //Configure CAN pin: RX 
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_Init(GPIOD, &GPIO_InitStructure);

	    //Configure CAN pin: TX 
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	    GPIO_Init(GPIOD, &GPIO_InitStructure);
	    break;
    }	    

	
    /* CAN Periph clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    CAN_InitTypeDef CAN_InitStructure;

    /* CAN register init */
    CAN_DeInit(CAN1);
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
    if(CAN_Init(CAN1, &CAN_InitStructure) == CANINITFAILED) {
	printf("Can init failed \n");
        assert_param(0);
    }

    NVIC_InitTypeDef NVIC_InitStructure;
    
    /* Enable CAN RX0 interrupt IRQ channel */
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable CAN RX0 interrupt IRQ channel */
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* CAN FIFO0 message pending interrupt enable */ 
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    /* CAN FIFO0 message pending interrupt enable */ 
    CAN_ITConfig(CAN1, CAN_IT_FMP1, ENABLE);
}

void CAN_CancelAllTransmits() {
    int i;
    for(i = 0; i < 3; ++i) {
    if(CAN_TransmitStatus(CAN1, i) == CANTXPENDING)
	CAN_CancelTransmit(CAN1, i);
    }    
}

unsigned char CAN_SendMessage(CanTxMsg* TxMessage)
{
    unsigned char ret = 0;
    
    //make this thread non interruptable
    __disable_irq();
    
    if(CAN_Transmit(CAN1, TxMessage) == CAN_NO_MB)
	ret = 1;
    //finished, let's allow interrupts again
    __enable_irq();
    
    return ret;
}

void CAN_ConfigureFilters(enum hostIDs boardNr) {
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;

  //Emergency stop, setMode and setValue are matched to FIFO0
  CAN_FilterInitStructure.CAN_FilterNumber=0;
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList;
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh=PACKET_ID_EMERGENCY_STOP<<5;
  if(boardNr == RECEIVER_ID_H_BRIDGE_1 || boardNr == RECEIVER_ID_H_BRIDGE_2 || boardNr == RECEIVER_ID_H_BRIDGE_3 || boardNr == RECEIVER_ID_H_BRIDGE_4) {
    CAN_FilterInitStructure.CAN_FilterIdLow=PACKET_ID_SET_VALUE14<<5;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=PACKET_LOW_PRIORITY_DATA<<5;
  } else {
    CAN_FilterInitStructure.CAN_FilterIdLow=PACKET_ID_SET_VALUE58<<5;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0;      
  }
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);

  //configure, configure 2 and set pis values matched to FIFO1
  CAN_FilterInitStructure.CAN_FilterNumber=1;
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit;
  //let pass every message, that matches the boardNr
  CAN_FilterInitStructure.CAN_FilterIdHigh=(boardNr)<<5;
  CAN_FilterInitStructure.CAN_FilterIdLow=0;
  //0x1E0 is the mask that matches all RECEIVER_IDs
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh= (0x1E0) << 5;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xFF;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO1;
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
  int received = 0;
 
  volatile uint8_t nextRxWritePointer = (canRxWritePointer + 1) % CAN_BUFFER_SIZE;
  while((uint8_t)(CAN1->RF0R&(uint32_t)0x03) && nextRxWritePointer != canRxReadPointer) {
    CAN_Receive(CAN1, CAN_FIFO0, canRXBuffer + canRxWritePointer);
    canRxWritePointer = nextRxWritePointer;

    nextRxWritePointer = (canRxWritePointer + 1) % CAN_BUFFER_SIZE;
    received = 1;
  }

  //discard message to avoid interrupt strom
  if(!received) {
    CanRxMsg nirvana;
    CAN_Receive(CAN1, CAN_FIFO0, &nirvana);
  }
}

void CAN1_RX1_IRQHandler(void)
{
  int received = 0;
  
  volatile uint8_t nextRxWritePointer = (canRxWritePointer + 1) % CAN_BUFFER_SIZE;
  while((uint8_t)(CAN1->RF1R&(uint32_t)0x03) && nextRxWritePointer != canRxReadPointer) {
    CAN_Receive(CAN1, CAN_FIFO1, canRXBuffer + canRxWritePointer);
    canRxWritePointer = nextRxWritePointer;

    nextRxWritePointer = (canRxWritePointer + 1) % CAN_BUFFER_SIZE;    
    received = 1;
  }
  //discard message to avoid interrupt strom
  if(!received) {
    CanRxMsg nirvana;
    CAN_Receive(CAN1, CAN_FIFO1, &nirvana);
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

int can_send_message_hard(CanTxMsg * message) {
    int mb;
    int resend_counter = 0;
    volatile int counter = 0;

    // need to keep the id of the message since the CAN_Transmit can 
    // actually change that field!
    int id = message->StdId;

    while(resend_counter < CAN_SEND_ARBITRATION_RETRIES) {
      resend_counter++;
      counter = 0;

      message->StdId = id;

      while((mb = CAN_Transmit(CAN1, message)) == CAN_NO_MB) {
	counter++;
	if (counter > CAN_SEND_RETRIES)
	  return CANTXFAILE;
      }
      
      counter = 0;
#define CAN_TSR_RQCP    ((uint32_t)0x00000001)    /* Request completed mailbox0 */
#define CAN_TSR_TXOK    ((uint32_t)0x00000002)    /* Transmission OK of mailbox0 */
#define CAN_TSR_ALST    ((uint32_t)0x00000004)    /* Arbitation Lost */
#define CAN_TSR_TERR    ((uint32_t)0x00000008)    /* Transmission Error */

      //wait till message is written on the bus
      //check if errors occured
      while(!(CAN1->TSR & (CAN_TSR_ALST << (8*mb))) && !(CAN1->TSR & (CAN_TSR_TERR << (8*mb)))) {
	counter++;
	if (counter > CAN_SEND_RETRIES) {
	  printf("Timeout sending can message");
	  break;
	}
	
	//jump out if message was send sucessfull
	if((CAN1->TSR & (CAN_TSR_RQCP << (8*mb))) && (CAN1->TSR & (CAN_TSR_TXOK << (8*mb)))) {
	  return CANTXOK;
	}
      }
      
      if(CAN1->TSR & (CAN_TSR_ALST << (8*mb))) {
	printf("Error ARLO mb is \n");
      }
      
      if(CAN1->TSR & (CAN_TSR_TERR << (8*mb))) {
	printf("Error TERR mb is \n");
      }
    }

    printf("Error sending can message\n");
    return CANTXFAILE;
}


