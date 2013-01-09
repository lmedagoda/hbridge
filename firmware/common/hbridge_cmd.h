#include "packets.h"

void hbridge_sendEncoderConfiguration(int id);
void hbridge_sendActuatorConfiguration(int id);
void hbridge_setValue( int value1, int value2, int value3, int value4);
void hbridge_sendControllerConfiguration(int id);
void hbridge_sendClearError(int id);
