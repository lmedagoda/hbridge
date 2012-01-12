#include "inc/stm32f10x_lib.h"
#include "inc/stm32f10x_gpio.h"
#include "inc/stm32f10x_tim.h"
#include "inc/stm32f10x_rcc.h"
#include "stm32f10x_it.h"
#include "inc/stm32f10x_can.h"
#include "spi.h"
#include "i2c.h"
#include "pid.h"
#include "usart.h"
#include "protocol.h"
#include "can.h"
#include "state.h"
#include "controllers.h"
#include "hbridge.h"
#include "encoder.h"
#include "current_measurement.h"
#include "lm73cimk.h"
#include "printf.h"
#include <stdlib.h>

void NVIC_Configuration(void);

void GPIO_Configuration(void);

void SysTick_Configuration(void);

#define MIN_S16 (-(1<<15))
#define MAX_S16 ((1<<15) -1)

volatile enum hostIDs ownHostId;

unsigned int systemTick;

/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{
  vu32 delay;
  systemTick = 0;
  //Enable peripheral clock for GPIO
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

  NVIC_Configuration();

  GPIO_Configuration();

  // re-route timer1 output to pins not included in chip package (non-existing pins)
  // needed for timer chaining
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);

  USART_Configuration();

//note, this mesage can only be send AFTER usart configuration
  print("Entered main loop\n");

  //turn of red led
  GPIO_SetBits(GPIOA, GPIO_Pin_8);

  //read out dip switches
  ownHostId = getOwnHostId();

  CAN_Configuration(CAN_NO_REMAP);
  CAN_ConfigureFilters(ownHostId);

  currentMeasurementInit();

  encodersInit();
  
  //init i2c to read out temperature sensor
  setupI2CForLM73CIMK();  

  //wait until 5V rail get's stable
  delay = 20000000;
  while(delay)
    delay--;

  volatile struct ControllerState cs1;
  volatile struct ControllerState cs2;
  
  activeCState = &(cs1);
  lastActiveCState = &(cs2);

  initControllers();

  //init cotroller state with sane values
  initStateStruct(activeCState);
  initStateStruct(lastActiveCState);

  //clear all errors, btw initialize error state struct
  clearErrors();

  hbridgeInit();


  measureACS712BaseVoltage(); 

  //activate systick interrupt, at this point
  //activeCState1 hast to be initalized completely sane !
  SysTick_Configuration();


  
  print("Peripheral configuration finished\n");
 
  /** DEBUG**/
/*
  u16 lastTime = 0;
  u16 time;
  u16 counter = 0;  
*/

  u16 cnt = 0;
    
//   u32 temp = 0;
//   u32 gotTmpCnt = 0;
//   u8 lmk72addr = (0x4E<<1);

  /** END DEBUG **/
 
  while(1) {

    /** START DEBUG **/
    /*if(!getTemperature(lmk72addr, &temp)) {
	gotTmpCnt++;
	    //printf("got temp %lu\n", temp);
    }

    printfI2CDbg();
    if(counter > 10000) {
      printf("cur temp is %lu got tmp %lu times\n", temp, gotTmpCnt);
      gotTmpCnt = 0;
      counter = 0;
      print(".");
      u32 eet = getTicksExtern();
      u32 iet = getTicks();
      printf("externalEncoderTicks are %lu internalTicks %lu \n", eet, iet);
      //printf("Error is %h \n", error);
      //print("ActiveCstate: ");
      //printStateDebug(activeCState);
      //print("LastActiveCstate: ");
      //printStateDebug(lastActiveCState);
    }

    time = TIM_GetCounter(TIM1);
    if(lastTime > time) {
      counter++;
    }
    lastTime = time;
*/

    /* END DEBUG */

    CanRxMsg *curMsg;
    
    //check if we got a new message
    if((curMsg = CAN_GetNextData()) != 0) {
      //print("Got a Msg \n");
      print("P");

      u8 forceSynchronisation = updateStateFromMsg(curMsg, lastActiveCState, ownHostId);
    
      //mark current message als processed
      CAN_MarkNextDataAsRead();

      //this is concurrency proof, as this code can not run, while
      //systick Handler is active !
      volatile struct ControllerState *tempstate = activeCState;

      //we go an new packet, so reset the timeout
      lastActiveCState->resetTimeoutCounter = 1;
      
      //this is atomar as only the write is relevant!
      activeCState = lastActiveCState;

      lastActiveCState = tempstate;
      
      *lastActiveCState = *activeCState;      
      
      if(forceSynchronisation) {
          while(activeCState->resetTimeoutCounter)
              ;
      }

      u16 errorDbg = inErrorState();
      if((curMsg->StdId != PACKET_ID_SET_VALUE14 && curMsg->StdId != PACKET_ID_SET_VALUE58)  || cnt == 50) {
	cnt = 0;  
	printf("Error is %hu \n", errorDbg);
	print("ActiveCstate: ");
	printStateDebug(activeCState);
	printf("P:%hi I:%hi D:%hi \n", activeCState->speedPIDValues.kp,  activeCState->speedPIDValues.ki,  activeCState->speedPIDValues.kd );
	print(" LastActiveCstate: ");
	printStateDebug(lastActiveCState);
	printf("P:%hi I:%hi D:%hi \n", lastActiveCState->speedPIDValues.kp,  lastActiveCState->speedPIDValues.ki, lastActiveCState->speedPIDValues.kd );
      } else {
	  cnt++;
      }
    } 
  }
}

void SysTickHandler(void) {  
    ++systemTick;
    //request switch of adc value struct
    requestNewADCValues();

    static s32 currentPwmValue = 0;
    static u16 index = 0;
    static u16 overCurrentCounter = 0;
    static u16 timeoutCounter = 0;
    static u32 temperature = 0;

    s32 pwmValue = 0;
    s32 wheelPos = 0;
    u32 ticksPerTurn = 0;
    u8 tickDivider = 0;
    
    switch(activeCState->controllerInputEncoder) {
        case INTERNAL:
            wheelPos = getTicks(activeCState->internalEncoder);
            ticksPerTurn = getTicksPerTurn(activeCState->internalEncoder);
            tickDivider = getTickDivider(activeCState->internalEncoder);
            break;
        case EXTERNAL:
            wheelPos = getTicks(activeCState->externalEncoder);
            ticksPerTurn = getTicksPerTurn(activeCState->externalEncoder);
            tickDivider = getTickDivider(activeCState->externalEncoder);
            break;
    }


    index++;
    if(index >= (1<<10))
	index = 0;

    setNewSpeedPIDValues(activeCState->speedPIDValues.kp, activeCState->speedPIDValues.ki, activeCState->speedPIDValues.kd, activeCState->speedPIDValues.minMaxPidOutput);
    setNewPosPIDValues(activeCState->positionPIDValues.kp, activeCState->positionPIDValues.ki, activeCState->positionPIDValues.kd, activeCState->positionPIDValues.minMaxPidOutput);
  
    //wait for adc struct to be switched
    waitForNewADCValues();

    u32 currentValue = calculateCurrent(currentPwmValue);

    // get temperature
    getTemperature(LM73_SENSOR_1,&temperature);    

    //reset timeout, if "userspace" requested it
    if(activeCState->resetTimeoutCounter) {
	activeCState->resetTimeoutCounter = 0;
	timeoutCounter = 0;
    }

    //change state to error if error is set
    if(inErrorState() && (activeCState->internalState == STATE_CONFIGURED || activeCState->internalState == STATE_GOT_TARGET_VAL)) {
	activeCState->internalState = STATE_ERROR;
    }

    //switch into error stat if GPIO is pulled low
    //this should securly switch off the Hbridge
    if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)) {
	activeCState->internalState = STATE_ERROR;
	getErrorState()->hardwareShutdown = 1;
    }

    //only check for overcurrent if configured
    if(activeCState->internalState == STATE_CONFIGURED || activeCState->internalState == STATE_GOT_TARGET_VAL) {
	//check for overcurrent
	if(currentValue > activeCState->maxCurrent) {
	    overCurrentCounter++;

	    if(overCurrentCounter > activeCState->maxCurrentCount) {
		activeCState->internalState = STATE_ERROR;
		getErrorState()->overCurrent = 1;
	    }
	} else {
	    overCurrentCounter = 0;
	}
	
	//check for timeout and go into error state
	if(activeCState->timeout && (timeoutCounter > activeCState->timeout)) {
	    activeCState->internalState = STATE_ERROR;
	    getErrorState()->timeout = 1;
	}
    
	//only run controllers if we got an target value
	if(activeCState->internalState == STATE_GOT_TARGET_VAL) {
	    //calculate pwm value
	    switch(activeCState->controllMode) {
		case CONTROLLER_MODE_PWM:
		    pwmValue = activeCState->targetValue;
		    break;
		
		case CONTROLLER_MODE_POSITION: {
                    //the target value needs to be multiplied by the tickDivider
                    //in case of position mode, as the "external" and "internal" ticks 
                    //differ by the factor of tickDivider
                    //TargetValue is divided by TickDivider
                    //wheelPos is not divided
                    u32 targetPositionValue = activeCState->targetValue * tickDivider;
                    if(activeCState->cascadedPositionController) {
			pwmValue = cascadedPositionController(targetPositionValue, wheelPos, ticksPerTurn, activeCState->enablePIDDebug);
		    } else {
			pwmValue = positionController(targetPositionValue, wheelPos, ticksPerTurn, activeCState->enablePIDDebug);
		    }
		    break;
		}   
		case CONTROLLER_MODE_SPEED:
		    pwmValue = speedController(activeCState->targetValue, wheelPos, ticksPerTurn, activeCState->enablePIDDebug);
		    break; 
		default:
		    pwmValue = 0;
		    break;
	    }

	    //trunkcate to s16
	    if(pwmValue > MAX_S16) 
		pwmValue = MAX_S16;
	    if(pwmValue < MIN_S16)
		pwmValue = MIN_S16;

	    if(abs(currentPwmValue - pwmValue) < activeCState->pwmStepPerMillisecond) {
		currentPwmValue = pwmValue;
	    } else {
		if(currentPwmValue - pwmValue < 0) {
		    currentPwmValue += activeCState->pwmStepPerMillisecond;
		} else {
		    currentPwmValue -= activeCState->pwmStepPerMillisecond;      
		}
	    }
	} else {
	    currentPwmValue = 0;
	}
  
	//send status message over CAN
	CanTxMsg statusMessage;
	statusMessage.StdId= PACKET_ID_STATUS + ownHostId;
	statusMessage.RTR=CAN_RTR_DATA;
	statusMessage.IDE=CAN_ID_STD;
	statusMessage.DLC= sizeof(struct statusData);
	
	struct statusData *sdata = (struct statusData *) statusMessage.Data;
	
	sdata->pwm = currentPwmValue;
	sdata->externalPosition = getDividedTicks(activeCState->externalEncoder);
	sdata->position = getDividedTicks(activeCState->internalEncoder);
	sdata->currentValue = currentValue;
	sdata->index = index;
	
	//cancel out old messages
	CAN_CancelAllTransmits();
	
	if(CAN_Transmit(&statusMessage) == CAN_NO_MB) {
	    print("Error Tranmitting status Message : No free TxMailbox \n");
	} else {
	    //print("Tranmitting status Message : OK \n");  
	}
	
	//increase timeout
	timeoutCounter++;

	//set pwm
	setNewPWM(currentPwmValue, activeCState->useOpenLoop);
    } else {
	//send error message
	if(activeCState->internalState == STATE_ERROR) {  
	    //send status message over CAN
	    CanTxMsg errorMessage;
	    errorMessage.StdId= PACKET_ID_ERROR + ownHostId;
	    errorMessage.RTR=CAN_RTR_DATA;
	    errorMessage.IDE=CAN_ID_STD;
	    errorMessage.DLC= sizeof(struct errorData);
	    
	    struct errorData *edata = (struct errorData *) errorMessage.Data;

	    volatile struct ErrorState *es = getErrorState();

	    edata->temperature = temperature;
	    edata->position = getDividedTicks(activeCState->internalEncoder);
	    edata->index = index;
	    edata->externalPosition = getDividedTicks(activeCState->externalEncoder);
	    edata->motorOverheated = es->motorOverheated;
	    edata->boardOverheated = es->boardOverheated;
	    edata->overCurrent = es->overCurrent;
	    edata->timeout = es->timeout;
	    edata->badConfig = es->badConfig;
	    edata->encodersNotInitalized = es->encodersNotInitalized;

	    //cancel out old messages
	    CAN_CancelAllTransmits();

	    if(CAN_Transmit(&errorMessage) == CAN_NO_MB) {
		print("Error Tranmitting status Message : No free TxMailbox \n");
	    } else {
	    //print("Tranmitting status Message : OK \n");  
	    }
	}
    
	//reset to zero values
	overCurrentCounter = 0;
	timeoutCounter = 0;
	setNewPWM(0, activeCState->useOpenLoop);

	//reset PID struct, to avoid bad controller 
	//behavior an reactivation due to big I part
	resetControllers(wheelPos);
    }
}

void SysTick_Configuration(void) {
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
  
  // SysTick end of count event each 1ms with input clock equal to 9MHz (HCLK/8)
  SysTick_SetReload(9000);

  // Enable SysTick interrupt
  SysTick_ITConfig(ENABLE);

  // Enable the SysTick Counter
  SysTick_CounterCmd(SysTick_Counter_Enable);
}



/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
******************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  //get default GPIO config
  GPIO_StructInit(&GPIO_InitStructure);

  /* Enable GPIOA, GPIOD, USB_DISCONNECT(GPIOC) and USART1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB 
			 | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD
                         | RCC_APB2Periph_USART1, ENABLE);

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;


  //configure PA2 (ADC Channel2) as analog input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //configure PA3 (ADC Channel3) as analog input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //configure PA4 (ADC Channel4) as analog input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //configure PA5 (ADC Channel5) as analog input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //LED (PA8)
  //configure as Push Pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //configure PB0 (ADC Channel8) as analog input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //configure pb 1 input pull up (maeuseklavier)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Configure SMBA
  /*
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  */
  //Configure GPIOB Pin 11 as input pull up for emergency switch off
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  //Configure SPI2 pins: SCK, MISO and MOSI
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Configure PC.12 as output push-pull (LED)
  GPIO_WriteBit(GPIOC,GPIO_Pin_12,Bit_SET);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //configure pc 13/14/15 as input pull up (maeuseklavier)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

}


/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures NVIC and Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_StructInit(&NVIC_InitStructure);

  /* Set the Vector Table base location at 0x08000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   

  /* 2 bit for pre-emption priority, 2 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  //Configure and enable TIM1 Update interrupt
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  //Configure and enable TIM1 Update interrupt
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable ADC interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable USB interrupt -------------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable CAN RX0 interrupt IRQ channel */
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN_RX0_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable CAN RX0 interrupt IRQ channel */
  NVIC_InitStructure.NVIC_IRQChannel = CAN_RX1_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Systick with Preemption Priority 2 and Sub Priority as 0 */ 
  NVIC_SystemHandlerPriorityConfig(SystemHandler_SysTick, 3, 0);
}

/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  printf("Wrong parameters value: file %s on line %d\n", file, (int) line);

  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  // Configure PA.8 as output push-pull (LED)
  GPIO_WriteBit(GPIOA,GPIO_Pin_8,Bit_SET);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  volatile int delay;
  int waittime = 500000;
  

 // blink red LED
  while(1)
    {    
      GPIO_SetBits(GPIOA, GPIO_Pin_8);
      delay = waittime;
      while(delay) {
	delay--;
      }
      
      GPIO_ResetBits(GPIOA, GPIO_Pin_8);
      delay = waittime;
      while(delay) {
	delay--;
      }
    }
}
