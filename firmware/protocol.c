#include "protocol.h"
#include "state.h"
#include "encoder.h"
#include "inc/stm32f10x_can.h"
#include "inc/stm32f10x_gpio.h"
#include "printf.h"

//HACK this should be moved to a callback interface for message ids...
void positionControllerSetControllerConfiguration(volatile struct posControllerData* data);

u8 updateStateFromMsg(CanRxMsg *curMsg, volatile struct ControllerState *state, enum hostIDs ownHostId) {
    //clear device specific adress bits
    curMsg->StdId &= ~ownHostId;

    u8 forceSynchronisation = 0;

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
		s16 targetVal = 0;
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
		u16 posVal = targetVal;
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
	  
        case PACKET_ID_SET_PID_POS: {
	  if(state->internalState == STATE_CONFIGURED || state->internalState == STATE_GOT_TARGET_VAL) {
	    struct setPidData *data = (struct setPidData *) curMsg->Data;
	    state->positionPIDValues.kp = data->kp;
	    state->positionPIDValues.ki = data->ki;
	    state->positionPIDValues.kd = data->kd;
	    state->positionPIDValues.minMaxPidOutput = data->minMaxPidOutput;
	    printf("Got PACKET_ID_SET_PI_POS Msg %hi %hi %hi %hu\n", data->kp, data->ki, data->kd, data->minMaxPidOutput);
	  
	  } else {
	    print("Error, not configured \n");
	    state->internalState = STATE_ERROR;
	    getErrorState()->badConfig = 1;
	  }
	  break;
	}
        case PACKET_ID_SET_PID_SPEED: {
	  if(state->internalState == STATE_CONFIGURED || state->internalState == STATE_GOT_TARGET_VAL) {
	    struct setPidData *data = (struct setPidData *) curMsg->Data;
	    state->speedPIDValues.kp = data->kp;
	    state->speedPIDValues.ki = data->ki;
	    state->speedPIDValues.kd = data->kd;
	    state->speedPIDValues.minMaxPidOutput = data->minMaxPidOutput;
	    printf("Got PACKET_ID_SET_PID_SET Msg %hi %hi %hi %hu\n", data->kp, data->ki, data->kd, data->minMaxPidOutput);
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
	    state->cascadedPositionController = data->cascadedPositionController;
            state->controllerInputEncoder = data->controllerInputEncoder;
	    state->enablePIDDebug = data->enablePIDDebug;

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
	    break;
	}
	case PACKET_ID_ENCODER_CONFIG_EXTERN:
	case PACKET_ID_ENCODER_CONFIG_INTERN: {
            u8 error = 0;
	    print("Got PACKET_ID_ENCODER_CONFIG Msg \n");
	    if(state->internalState == STATE_CONFIGURED || state->internalState == STATE_GOT_TARGET_VAL) {
		print("Bad, state is configured \n");
		//do not allow to configure encoders while PID might be active
		getErrorState()->badConfig = 1;
                error = 1;
	    } 
	    
            struct encoderConfiguration *encData = (struct encoderConfiguration *) curMsg->Data;
            printf("configuring encoders %lu \n",encData->ticksPerTurn);		
            if(curMsg->StdId == PACKET_ID_ENCODER_CONFIG_INTERN) {
                if (state->internalEncoder != encData->encoderType)
                {
                    deinitEncoder(state->internalEncoder);
                    initEncoder(encData->encoderType);
                    state->internalEncoder = encData->encoderType;
                }
            } else {
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
	}
	break;
	case PACKET_ID_POS_CONTROLLER_DATA:
	{
	    print("Got Pos Ctrl data");
	    if(curMsg->DLC != sizeof(struct posControllerData))
	    {
		print("Error PosCtrlData has wrong size");
		break;
	    }
	    struct posControllerData *data = (struct posControllerData *) curMsg->Data;
	    positionControllerSetControllerConfiguration(data);
	}
	break;
     default: 
	{
	  u16 id = curMsg->StdId;
	  
	  printf("Got unknown packet id : %hu ! \n", id);
	  break;
	}
    }

    return forceSynchronisation;
}
