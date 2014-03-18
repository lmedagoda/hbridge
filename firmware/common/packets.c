#include "packets.h"

const char *getStateName(enum STATES state)
{
    switch(state)
    {
        case STATE_UNCONFIGURED:
	    return "STATE_UNCONFIGURED";
	    break;
        case STATE_SENSOR_ERROR:
	    return "STATE_SENSOR_ERROR";
	    break;
        case STATE_SENSORS_CONFIGURED:
	    return "STATE_SENSORS_CONFIGURED";
	    break;
        case STATE_ACTUATOR_ERROR:
	    return "STATE_ACTUATOR_ERROR";
	    break;
        case STATE_ACTUATOR_CONFIGURED:
	    return "STATE_ACTUATOR_CONFIGURED";
	    break;
        case STATE_CONTROLLER_CONFIGURED:
	    return "STATE_CONTROLLER_CONFIGURED";
	    break;
        case STATE_RUNNING:
	    return "STATE_RUNNING";
	    break;
    }
    return "";
}

const char *getPacketName(uint16_t packetId)
{
    switch(packetId)
    {
	case PACKET_ID_EMERGENCY_STOP:
	    return "PACKET_ID_EMERGENCY_STOP";
	    break;
	case PACKET_ID_SET_ALLOWED_SENDER: 
	    return "PACKET_ID_SET_ALLOWED_SENDER";
	    break;
	case PACKET_ID_ERROR: 
	    return "PACKET_ID_ERROR";
	    break;
	case PACKET_ID_ANNOUNCE_STATE:
	    return "PACKET_ID_ANNOUNCE_STATE";
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
    
	case PACKET_ID_SET_SENSOR_CONFIG: 
	    return "PACKET_ID_SET_SENSOR_CONFIG";
	    break;
	
	case PACKET_ID_CLEAR_SENSOR_ERROR:
	    return "PACKET_ID_CLEAR_SENSOR_ERROR";
	    break;
	
	case PACKET_ID_SET_ACTUATOR_CONFIG: 
	    return "PACKET_ID_SET_ACTUATOR_CONFIG";
	    break;
	
	case PACKET_ID_CLEAR_ACTUATOR_ERROR:
	    return "PACKET_ID_CLEAR_ACTUATOR_ERROR";
	    break;

	case PACKET_ID_SET_ACTIVE_CONTROLLER: 
	    return "PACKET_ID_SET_ACTIVE_CONTROLLER";
	    break;
	
	case PACKET_ID_REQUEST_STATE:
	    return "PACKET_ID_REQUEST_STATE";	    
	    break;
	    
	case PACKET_ID_REQUEST_SENSOR_CONFIG:
	    return "PACKET_ID_REQUEST_SENSOR_CONFIG";
	    break;
	    
	case PACKED_ID_REQUEST_VERSION: 
	    return "PACKED_ID_REQUEST_VERSION";
	    break;
	    
	case PACKED_ID_ANNOUNCE_VERSION: 
	    return "PACKED_ID_ANNOUNCE_VERSION";
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
	
	case PACKET_ID_SET_UNCONFIGURED:
	    return "PACKET_ID_SET_UNCONFIGURED";
	    break;
	    
	case PACKET_ID_SET_ACTUATOR_UNCONFIGURED:
	    return "PACKET_ID_SET_ACTUATOR_UNCONFIGURED";
	    break;
	    
	case PACKET_ID_TOTAL_COUNT: 
	    return "PACKET_ID_TOTAL_COUNT";
	    break;
    }
    
    return "UNKNOWN ID";
}
