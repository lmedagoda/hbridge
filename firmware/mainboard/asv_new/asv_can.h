#ifndef ASVCAN_H
#define ASVCAN_H
#include "../common/mainboardstate.h"
#include "../common/packethandling.h"
#include "../common/arc_packet.h"
#include "../common/time.h"
#include "../common/timeout.h"
#include "../common/arc_tokendriver.h"
#include "../../common/hbridge_cmd2.h"
#include "../../common/hbridge_cmd.h"
#include "../../common/protocol.h"
#include "../../interfaces/thread.h"
#include "../../hbridgeCommon/protocol_can.h"
#include "../../hbridgeCommon/drivers/can.h"
#include "../../hbridgeCommon/drivers/usart.h"
#include "../../hbridgeCommon/drivers/assert.h"
#include "../../hbridgeCommon/drivers/printf.h"
#include <stddef.h>
#include "uwmodem.h"
int asvcan_recvPacket(uint16_t *senderId, uint16_t *receiverId, uint16_t *packetId, unsigned char *data, const unsigned int dataSize);
void asvcan_handlePacket(CanRxMsg *msg);
#endif
