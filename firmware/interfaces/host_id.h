#ifndef HOST_ID_H
#define HOST_ID_H

#include "../common/protocol.h"

/**
 * This function must be implemented by the platform.
 * It returns the ID that will be used as source 
 * address for all protocol messages.
 * */
enum hostIDs getOwnHostId();

#endif
