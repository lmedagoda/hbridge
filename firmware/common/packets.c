#include "packets.h"

const char *getPacketName(uint16_t packetId)
{
    switch(packetId)
    {
	case PACKET_ID_SET_OVERWRITE: 
	    return "PACKET_ID_SET_OVERWRITE";
	    break;
	
	case PACKET_ID_ERROR: 
	    return "PACKET_ID_ERROR";
	    break;
	case PACKET_ID_STATUS: 
	    return "PACKET_ID_STATUS";
	    break;
	case PACKET_ID_EXTENDED_STATUS: 
	    return "PACKET_ID_EXTENDED_STATUS";
	    break;
	case PACKET_ID_ACK: 
	    return "PACKET_ID_ACK";
	    break;
	
	case PACKET_ID_SET_VALUE: 
	    return "PACKET_ID_SET_VALUE";
	    break;
	case PACKET_ID_SET_VALUE14: 
	    return "PACKET_ID_SET_VALUE14";
	    break;
	case PACKET_ID_SET_VALUE58: 
	    return "PACKET_ID_SET_VALUE58";
	    break;
	
	//case PACKET_LOW_PRIORITY_DATA: return "";break;
	case PACKET_ID_LOWIDS_START: 
	    return "PACKET_LOW_PRIORITY_DATA";
	    break;
    
	case PACKET_ID_SET_BASE_CONFIG: 
	    return "PACKET_ID_SET_BASE_CONFIG";
	    break;
	case PACKET_ID_SET_ACTIVE_CONTROLLER: 
	    return "PACKET_ID_SET_ACTIVE_CONTROLLER";
	    break;
	
	case PACKED_ID_REQUEST_VERSION: 
	    return "PACKED_ID_REQUEST_VERSION";
	    break;
	case PACKED_ID_VERSION: 
	    return "PACKED_ID_VERSION";
	    break;

	case PACKET_ID_SET_SPEED_CONTROLLER_DATA: 
	    return "PACKET_ID_SET_SPEED_CONTROLLER_DATA";
	    break;
	case PACKET_ID_SPEED_CONTROLLER_DEBUG: 
	    return "PACKET_ID_SPEED_CONTROLLER_DEBUG";
	    break;

	case PACKET_ID_POS_CONTROLLER_DEBUG: 
	    return "PACKET_ID_POS_CONTROLLER_DEBUG";
	    break;
	case PACKET_ID_SET_POS_CONTROLLER_DATA: 
	    return "PACKET_ID_SET_POS_CONTROLLER_DATA";
	    break;
	
	case PACKET_ID_TOTAL_COUNT: 
	    return "PACKET_ID_TOTAL_COUNT";
	    break;
    }
    
    return "";
}
