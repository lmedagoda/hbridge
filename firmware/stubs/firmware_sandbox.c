#include "common/systick.h"
#include "interfaces/current_measurement.h"
#include "interfaces/encoder.h"
#include <stdlib.h>


int fw_main(void)
{
    //note, this mesage can only be send AFTER usart configuration
    print("Entered main loop\n");

    //init basic functionality
    //read address, turn on peripherals etc.
    baseInit();

    
    
    print("Peripheral configuration finished\n");

 
 
    while(1) {

    }
}
