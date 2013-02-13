#include "time.h"

unsigned int time_curTime = 0;

unsigned int time_getTimeInMs()
{
    return time_curTime;
}

void time_msPassed()
{
    time_curTime++;
}