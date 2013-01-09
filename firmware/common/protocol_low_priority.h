#ifndef __PROTOCOL_LOW_PRIO_H
#define __PROTOCOL_LOW_PRIO_H

#include <stdint.h>

void protocolLowPriority_init();
uint8_t protocol_sendLowPrio(uint16_t senderId, uint16_t receiverId, uint16_t id, const uint8_t *data, uint8_t size);
#endif