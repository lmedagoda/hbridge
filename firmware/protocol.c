#include "protocol.h"
#include "state.h"
#include "encoder.h"
#include "inc/stm32f10x_can.h"
#include "drivers/printf.h"
#include "drivers/can.h"

void (*protocolHandlers[NUM_PACKET_IDS - END_BASE_PACKETS])(int id, unsigned char *data, unsigned short size);

void protocol_defaultHandler(int id, unsigned char *data, unsigned short size)
{
    unsigned short idl = id;
    printf("Warning, packet with id %hu was not handled \n", idl);
}

volatile enum hostIDs ownHostId;

void protocol_init(enum hostIDs id)
{
    ownHostId = id;
    
    int i = 0;
    for(i = 0; i < NUM_PACKET_IDS - END_BASE_PACKETS; i++)
    {
	protocolHandlers[i] = protocol_defaultHandler;
    }
}

void protocol_registerHandler(int id, void (*handler)(int id, unsigned char *data, unsigned short size))
{
    protocolHandlers[id - END_BASE_PACKETS] = handler;
}

void protocol_ackPacket(int id)
{
    CanTxMsg ackMessage;

    //send status message over CAN
    ackMessage.StdId= PACKET_ID_ACK + ownHostId;
    ackMessage.RTR=CAN_RTR_DATA;
    ackMessage.IDE=CAN_ID_STD;
    ackMessage.DLC= sizeof(struct ackData);

    struct ackData *data = (struct ackData *) ackMessage.Data;
    data->packetId = id + ownHostId;

    printf("acking %i id %i\n", id, id + ownHostId);

    while(CAN_SendMessage(&ackMessage))
	;

}


uint8_t updateStateFromMsg(CanRxMsg *curMsg, volatile struct ControllerState *state, enum hostIDs ownHostId) {
    //clear device specific adress bits
    curMsg->StdId &= ~ownHostId;

    uint8_t forceSynchronisation = 0;

    switch(curMsg->StdId) {
	case PACKET_ID_EMERGENCY_STOP:
	    //print("Got PACKET_ID_EMERGENCY_STOP Msg \n");
	    state->internalState = STATE_UNCONFIGURED;
	    state->targetValue = 0;
	    break;

	case PACKET_ID_SET_VALUE58: 
	case PACKET_ID_SET_VALUE14: {
	    //print("Got PACKET_ID_SET_VALUE Msg \n");
	    if(state->internalState == STATE_CONFIGURED || state->internalState == STATE_GOT_TARGET_VAL) {
		struct setValueData *data = (struct setValueData *) curMsg->Data;
		int16_t targetVal = 0;
		switch(ownHostId) {
		    case RECEIVER_ID_H_BRIDGE_1:
		    case RECEIVER_ID_H_BRIDGE_5:
			targetVal = data->board1Value;
			state->internalState = STATE_GOT_TARGET_VAL;
			break;
		    case RECEIVER_ID_H_BRIDGE_2:
		    case RECEIVER_ID_H_BRIDGE_6:
			targetVal = data->board2Value;
			state->internalState = STATE_GOT_TARGET_VAL;
			break;
		    case RECEIVER_ID_H_BRIDGE_3:
		    case RECEIVER_ID_H_BRIDGE_7:
			targetVal = data->board3Value;
			state->internalState = STATE_GOT_TARGET_VAL;
			break;
		    case RECEIVER_ID_H_BRIDGE_4:
		    case RECEIVER_ID_H_BRIDGE_8:
			targetVal = data->board4Value;
			state->internalState = STATE_GOT_TARGET_VAL;
			break;
		}
		uint16_t posVal = targetVal;
		if(state->controllMode == CONTROLLER_MODE_POSITION)
		    state->targetValue = posVal;
		else
		    state->targetValue = targetVal;

	    } else {
		print("Error, not configured \n");
		state->internalState = STATE_ERROR;
		getErrorState()->badConfig = 1;
	    }
	    break;
	}

	case PACKET_ID_SET_MODE14:
	case PACKET_ID_SET_MODE58: {
	    print("set mode \n");
	    if(state->internalState == STATE_CONFIGURED || state->internalState == STATE_GOT_TARGET_VAL) {
		struct setModeData *data = (struct setModeData *) curMsg->Data;
                enum controllerModes lastMode = state->controllMode;
		switch(ownHostId) {
		    case RECEIVER_ID_H_BRIDGE_1:
		    case RECEIVER_ID_H_BRIDGE_5:
			state->controllMode = data->board1Mode;
			break;
		    case RECEIVER_ID_H_BRIDGE_2:
		    case RECEIVER_ID_H_BRIDGE_6:
			state->controllMode = data->board2Mode;
			break;
		    case RECEIVER_ID_H_BRIDGE_3:
		    case RECEIVER_ID_H_BRIDGE_7:
			state->controllMode = data->board3Mode;
			break;
		    case RECEIVER_ID_H_BRIDGE_4:
		    case RECEIVER_ID_H_BRIDGE_8:
			state->controllMode = data->board4Mode;
			break;
		}
                if(lastMode != state->controllMode)
                    state->internalState = STATE_CONFIGURED;
	    } else {
		print("Error, not configured \n");
		state->internalState = STATE_ERROR;
		getErrorState()->badConfig = 1;
	    }
	    break;
	}
        case PACKET_ID_SET_CONFIGURE: {
            if(!encoderConfigured(state->internalEncoder) || !encoderConfigured(state->externalEncoder)) {
                state->internalState = STATE_ERROR;
                getErrorState()->encodersNotInitalized = 1;
                break;
            }
	    print("Got PACKET_ID_SET_CONFIGURE Msg \n");
	    struct configure1Data *data = (struct configure1Data *) curMsg->Data;
	    state->useExternalTempSensor = data->externalTempSensor;
	    state->useOpenLoop = data->openCircuit;
            state->controllerInputEncoder = data->controllerInputEncoder;

	    state->maxMotorTemp = data->maxMotorTemp;
	    state->maxMotorTempCount = data->maxMotorTempCount;
	    state->maxBoardTemp = data->maxBoardTemp;
	    state->maxBoardTempCount = data->maxBoardTempCount;
	    state->timeout = data->timeout;

	    if(state->internalState == STATE_CONFIG2_RECEIVED) {
                state->internalState = STATE_CONFIGURED;
                clearErrors();
	    } else {
		state->internalState = STATE_CONFIG1_RECEIVED;
	    }
	    protocol_ackPacket(curMsg->StdId);

	    break;
	}
	  
        case PACKET_ID_SET_CONFIGURE2: {
            if(!encoderConfigured(state->internalEncoder) || !encoderConfigured(state->externalEncoder)) {
                state->internalState = STATE_ERROR;
                getErrorState()->encodersNotInitalized = 1;
                break;
            }
	    print("Got PACKET_ID_SET_CONFIGURE2 Msg \n");
	    struct configure2Data *data = (struct configure2Data *) curMsg->Data;
	    state->maxCurrent = data->maxCurrent;
	    state->maxCurrentCount = data->maxCurrentCount;
	    state->pwmStepPerMillisecond = data->pwmStepPerMs;

	    if(state->internalState == STATE_CONFIG1_RECEIVED) {
                state->internalState = STATE_CONFIGURED;
                clearErrors();
	    } else {
		state->internalState = STATE_CONFIG2_RECEIVED;
	    }
	    protocol_ackPacket(curMsg->StdId);

	    break;
	}
	case PACKET_ID_ENCODER_CONFIG_EXTERN:
	case PACKET_ID_ENCODER_CONFIG_INTERN: {
            uint8_t error = 0;
	    if(state->internalState == STATE_CONFIGURED || state->internalState == STATE_GOT_TARGET_VAL) {
		print("Bad, state is configured \n");
		//do not allow to configure encoders while PID might be active
		getErrorState()->badConfig = 1;
                error = 1;
	    } 
	    
            struct encoderConfiguration *encData = (struct encoderConfiguration *) curMsg->Data;
            printf("configuring encoders %lu \n",encData->ticksPerTurn);		
            if(curMsg->StdId == PACKET_ID_ENCODER_CONFIG_INTERN) {
		print("Got PACKET_ID_ENCODER_CONFIG intern Msg \n");
                if (state->internalEncoder != encData->encoderType)
                {
                    deinitEncoder(state->internalEncoder);
                    initEncoder(encData->encoderType);
                    state->internalEncoder = encData->encoderType;
                }
            } else {
		print("Got PACKET_ID_ENCODER_CONFIG extern Msg \n");
                if (state->externalEncoder != encData->encoderType)
                {
                    deinitEncoder(state->externalEncoder);
                    initEncoder(encData->encoderType);
                    state->externalEncoder = encData->encoderType;
                }
            }
            setTicksPerTurn(encData->encoderType, encData->ticksPerTurn, encData->tickDivider);
            if(!error)
            {
                clearErrors();
                state->internalState = STATE_UNCONFIGURED;
            }
            //systick needs to run one time after new encoder values are set
            forceSynchronisation = 1;
	    protocol_ackPacket(curMsg->StdId);
	}
	break;
     default: 
	{
	    if(curMsg->StdId > END_BASE_PACKETS)
	    {
		if(state->internalState == STATE_GOT_TARGET_VAL) {
		    print("Error, configuring controllers is not allowed while running \n");
		    state->internalState = STATE_ERROR;
		    getErrorState()->badConfig = 1;
		}
		else
		{
		    protocolHandlers[curMsg->StdId - END_BASE_PACKETS](curMsg->StdId, curMsg->Data, curMsg->DLC);
		}
	    }
	    else
	    {
		uint16_t id = curMsg->StdId;
		printf("Got unknown packet id : %hu ! \n", id);
	    }
	    break;
	}
    }

    return forceSynchronisation;
}
