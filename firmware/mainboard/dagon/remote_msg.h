#ifndef _REMOTE_MSG_
#define _REMOTE_MSG_

#include <inttypes.h>

struct remote_msg{
    uint8_t state;
    double target;
} __attribute__ ((packed)) __attribute__((__may_alias__));

#endif //_REMOTE_MSG_
