#ifndef _REMOTE_MSG_
#define _REMOTE_MSG_

#include <inttypes.h>

struct remote_msg{
    uint8_t state;
    double target;
    uint8_t modemdata_size;
    uint8_t modemdata[10];
} __attribute__ ((packed)) __attribute__((__may_alias__));

#endif //_REMOTE_MSG_
