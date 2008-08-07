
#include "controller_interface.h"
#include <stdlib.h>
#include "pid.h"

void dummycontroller(void) {
  int i;
  /*
  printf("Got Data from MD03 \n", 0);
  printf("FL Temp is %d, Cur is %d \n",getTemperature(FRONT_LEFT), getCurrent(FRONT_LEFT));
  printf("FR Temp is %d, Cur is %d \n",getTemperature(FRONT_RIGHT), getCurrent(FRONT_RIGHT));
  printf("RL Temp is %d, Cur is %d \n",getTemperature(REAR_LEFT), getCurrent(REAR_LEFT));
  printf("RR Temp is %d, Cur is %d \n",getTemperature(REAR_RIGHT), getCurrent(REAR_RIGHT));
  

  printf("Dev %d, CurTicks %d, LastTicks %d \n",FRONT_LEFT, getCurTicks(FRONT_LEFT), getLastTicks(FRONT_LEFT));
  printf("Dev %d, CurTicks %d, LastTicks %d \n",FRONT_RIGHT, getCurTicks(FRONT_RIGHT), getLastTicks(FRONT_RIGHT));
  printf("Dev %d, CurTicks %d, LastTicks %d \n",REAR_LEFT, getCurTicks(REAR_LEFT), getLastTicks(REAR_LEFT));
  printf("Dev %d, CurTicks %d, LastTicks %d \n",REAR_RIGHT, getCurTicks(REAR_RIGHT), getLastTicks(REAR_RIGHT));
  */
  //md03_I2C1_state = MD03_DATA_RECEIVED;
  
  setNewAcceleration(40, FRONT_LEFT);
  setNewAcceleration(40, FRONT_RIGHT);
  setNewAcceleration(40, REAR_LEFT);
  setNewAcceleration(40, REAR_RIGHT);

  int tv = getValueForChannel(CHANNEL_1);  

  if( abs(tv) > 10 ) {
    if( tv > 0 ) {
      tv -= 10;
    } else {
      tv += 10;
    }
  } else {
    tv = 0;
  }

  tv *=2;

  printf("TV is %d, rc[0] is %d \n",tv, getValueForChannel(CHANNEL_1));

  /*static int test = 0;  

  if(test < 100) {
    tv = 50;
  } else {
    tv = 0;
    if(test > 200)
      test = 0;
      }

  //tv = 100;
  //setNewSpeedPID(tv, FRONT_LEFT);


  test++;
  
  printf("TV is %d , test is %d \n",tv, test);
  */
  
  setNewSpeed(tv, FRONT_LEFT);  
  setNewSpeed(tv, FRONT_RIGHT);  
  setNewSpeed(tv, REAR_LEFT);
  setNewSpeed(tv, REAR_RIGHT);
  
  /*
  setNewSpeed(0, FRONT_LEFT);  
  setNewSpeed(50, FRONT_RIGHT);  
  setNewSpeed(0, REAR_LEFT);
  setNewSpeed(0, REAR_RIGHT);
  */
}
