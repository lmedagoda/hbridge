#include "drivers/printf.h"
#include "drivers/usart.h"
#include "systick.h"
#include "current_measurement.h"
#include "drivers/can.h"
#include "drivers//i2c.h"
#include "encoder.h"
#include "encoder_quadrature.h"
#include <stdlib.h>

enum hostIDs getOwnHostId() {
    
    enum hostIDs id = RECEIVER_ID_H_BRIDGE_1; 
    return id;
}

extern struct EncoderInterface encoders[NUM_ENCODERS];

/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int fw_main(void)
{

    vu32 delay;

    baseNvicInit();

    USART3_Init(DISABLE);

    //note, this mesage can only be send AFTER usart configuration
    print("Entered main loop\n");

    //init basic functionality
    //read address, turn on peripherals etc.
    baseInit();

    print("Peripheral configuration finished\n");

    CAN_Configuration(CAN_REMAP1);
    CAN_ConfigureFilters(ownHostId);

    //this is hacky, but the best way at,
    encoders[QUADRATURE].encoderInit = encoderInitQuadratureV2;
    encoders[QUADRATURE].getTicks = getTicksQuadratureV2;
    encoders[QUADRATURE].setTicksPerTurn = setTicksPerTurnQuadratureV2;

    encoders[QUADRATURE_WITH_ZERO].encoderInit = encoderInitQuadratureWithZeroV2;
    encoders[QUADRATURE_WITH_ZERO].getTicks = getTicksQuadratureWithZeroV2;
    encoders[QUADRATURE_WITH_ZERO].setTicksPerTurn = setTicksPerTurnQuadratureWithZeroV2;
    //activate systick interrupt, at this point
    //activeCState1 hast to be initalized completely sane !
    SysTick_Init();

 
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

    pollCanMessages();
  }
}
