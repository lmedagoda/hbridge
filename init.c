
#include "controller_interface.h"
#include "dummycontroller.h"


/**
 * This function is called from the main loop after initalization.
 * You should initalize internal data structures of your controller
 * here and install it using installController().
 **/
void init(void) {
  installController(dummycontroller);
}
