
#include "rc.h"
#include "stdlib.h"

extern int printf(const char *format, ...);

vu16 edgeTimeBuffer[EDGETIMEBUFFER_SIZE];
vu16 etbRead = 0;
vu16 etbWrite = 0;

volatile s32 rc_values[MAX_RC_CHANNELS];




void updateRCValues() {
  static u8 index = 0;
  static u32 lastFallingEdgeTime = 0;
  static u32 lastRaisingEdgeTime = 0;
  static u8 contact = 0;

  while(etbRead != etbWrite) {
    /* Get the Input Capture value */
    u32 raisingEdgeTime = edgeTimeBuffer[etbRead];
    etbRead = (etbRead + 1) % EDGETIMEBUFFER_SIZE;
  
    u32 fallingEdgeTime = edgeTimeBuffer[etbRead];
    etbRead = (etbRead + 1) % EDGETIMEBUFFER_SIZE;
    
    s32 dutyCycle = fallingEdgeTime - lastRaisingEdgeTime;
    s32 downtime =  raisingEdgeTime - fallingEdgeTime;

    //wrap around checks
    if(dutyCycle < 0) {
      dutyCycle += 65000;
    }

    if(downtime < 0) {
      downtime += 65000;
    }

    lastFallingEdgeTime = fallingEdgeTime;
    lastRaisingEdgeTime = raisingEdgeTime;

    //for some reason we get twice as much counter ticks as expected
    //as a quickfix we devide dutyCycle and downtime by two
    dutyCycle /= 2;
    downtime /=2;

    /* DEBUG CODE */
    /*printf("raisingEdgeTime is %lu ", raisingEdgeTime);
    printf("fallingEdgeTime is %lu ", fallingEdgeTime);
    printf("dutyCycle is %lu ", dutyCycle);
    printf("downtime is %lu \n", downtime);
    number++;*/
    /* END DEBUG CODE */
    
    
    
    //the duty cycle is allways 0.4 ms
    //if it is smaller or bigger this means,
    //we have no radio contact
    if(abs(dutyCycle - 400) > 100) {
      if(contact) {
	for(index = 0; index < MAX_RC_CHANNELS; index++) {
	  rc_values[index] = 0;
	}
	contact = 0;
      }
      return;
    }
    
    
    //wait for start, to get correct channels
    if(!contact && downtime < 4000) {
      return;
    }
    
    
    //if down period is bigger than 4 ms 
    //this is a start of a new signal 
    //1000 is 1 ms
    if(downtime > 4000) {
      contact = 1;
      index = 0;
    } else {
      //Values from Osciloscope
      //minimum is 0.75 ms
      //mean is 1.15 ms
      //maximum is 1.55 ms
      
      //looking at the measurment values this seems to be correct :
      //min 700
      //mean 1075
      //max 1450

      //mapped to -100 +100
      rc_values[index] = ((downtime -1075) * 100) / 375;
      
      if(index < MAX_RC_CHANNELS)
	index++;
    }
  }

  /* DEBUG */
  /*printf("rc[0] is %d \n", rc_values[0]);    
  printf("rc[1] is %d \n", rc_values[1]);    
  printf("rc[2] is %d \n", rc_values[2]);    
  printf("rc[3] is %d \n", rc_values[3]);    
  printf("rc[4] is %d \n", rc_values[4]);    
  printf("rc[5] is %d \n", rc_values[5]);*/
  /*END DEBUG*/

}


