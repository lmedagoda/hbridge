#include "timeout.h"
#include "time.h"

int timeout_timeout;
unsigned int timeout_lastReset;

void timeout_init(int timeoutInMs)
{
    timeout_timeout = timeoutInMs;
}

uint8_t timeout_hasTimeout()
{
    unsigned int curTime = time_getTimeInMs();
    unsigned int diff = curTime - timeout_lastReset;
    if(diff < 0)
	//TODO TEST THIS !!!
	diff += (unsigned int) (-1);

    if(diff > timeout_timeout)
	return 1;
    
    return 0;
}

void timeout_reset()
{
    timeout_lastReset = time_getTimeInMs();
}


