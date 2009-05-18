#ifndef HBRIDGE_PROTOCOL_HPP
#define HBRIDGE_PROTOCOL_HPP

namespace firmware
{
#define __NO_STM32

    //define types similar to STM32 firmeware lib,
    //so we can use same protocoll.h
    typedef uint8_t u8;
    typedef uint16_t u16;
    typedef int16_t s16;

#include "firmware/protocol.h"
}

#endif

