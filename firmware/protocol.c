#include "protocol.h"
#include "state.h"
#include "encoder.h"
#include "inc/stm32f10x_can.h"
#include "inc/stm32f10x_gpio.h"
#include "printf.h"

enum hostIDs getOwnHostId() {
    enum hostIDs id = RECEIVER_ID_H_BRIDGE_1; 
    u16 gpioData = 0;
    gpioData |= GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);
    gpioData |= (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14) << 1);
    gpioData |= (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15) << 2);
    gpioData |= (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) << 3);

    //get correct host id from gpio pins
    switch(gpioData) {
    case 1 :
	print("Configured as H_BRIDGE_1\n");
	id = RECEIVER_ID_H_BRIDGE_1;
	break;
    case 2 :
	print("Configured as H_BRIDGE_2\n");
	id = RECEIVER_ID_H_BRIDGE_2;
	break;
    case 4 :
	print("Configured as H_BRIDGE_3\n");
	id = RECEIVER_ID_H_BRIDGE_3;
	break;
    case 8 :
	print("Configured as H_BRIDGE_4\n");
	id = RECEIVER_ID_H_BRIDGE_4;
	break;
    default:
	printf("Wrong host ide configured %hu\n", gpioData);
	//blink and do nothing
	assert_failed((u8 *)__FILE__, __LINE__);
	break;
    }    
    return id;
}


void updateStateFromMsg(CanRxMsg *curMsg, volatile struct ControllerState *state, enum hostIDs ownHostId) {
      //clear device specific adress bits
      curMsg->StdId &= ~ownHostId;
    
      switch(curMsg->StdId) {
        case PACKET_ID_EMERGENCY_STOP:
	  //print("Got PACKET_ID_EMERGENCY_STOP Msg \n");
	  state->internalState = STATE_UNCONFIGURED;
	  state->targetValue = 0;
	  break;
        case PACKET_ID_SET_VALUE: {
	  //print("Got PACKET_ID_SET_VALUE Msg \n");
	  if(state->internalState == STATE_CONFIGURED) {
	    struct setValueData *data = (struct setValueData *) curMsg->Data;
	    switch(ownHostId) {
	    case RECEIVER_ID_H_BRIDGE_1:
	      state->targetValue = data->board1Value;
	      break;
	    case RECEIVER_ID_H_BRIDGE_2:
	      state->targetValue = data->board2Value;
	      break;
	    case RECEIVER_ID_H_BRIDGE_3:
	      state->targetValue = data->board3Value;
	      break;
	    case RECEIVER_ID_H_BRIDGE_4:
	      state->targetValue = data->board4Value;
	      break;
	    }
	  } else {
	    print("Error, not configured \n");
	    state->internalState = STATE_ERROR;
	    getErrorState()->badConfig = 1;
	  }
	  break;
	}

        case PACKET_ID_SET_MODE: {
	  if(state->internalState == STATE_CONFIGURED) {
	    struct setModeData *data = (struct setModeData *) curMsg->Data;
	    switch(ownHostId) {
	    case RECEIVER_ID_H_BRIDGE_1:
	      
	      state->controllMode = data->board1Mode;
	      break;
	    case RECEIVER_ID_H_BRIDGE_2:
	      state->controllMode = data->board2Mode;
	      break;
	    case RECEIVER_ID_H_BRIDGE_3:
	      state->controllMode = data->board3Mode;
	      break;
	    case RECEIVER_ID_H_BRIDGE_4:
	      state->controllMode = data->board4Mode;
	      break;
	    }
	  } else {
	    print("Error, not configured \n");
	    state->internalState = STATE_ERROR;
	    getErrorState()->badConfig = 1;
	  }
	  break;
	}
	  
        case PACKET_ID_SET_PID_POS: {
	  if(state->internalState == STATE_CONFIGURED) {
	    struct setPidData *data = (struct setPidData *) curMsg->Data;
	    state->positionPIDValues.kp = data->kp;
	    state->positionPIDValues.ki = data->ki;
	    state->positionPIDValues.kd = data->kd;
	    state->positionPIDValues.minMaxPidOutput = data->minMaxPidOutput;
	    printf("Got PACKET_ID_SET_PI_POS Msg %h %h %h %hu\n", data->kp, data->ki, data->kd, data->minMaxPidOutput);
	  
	  } else {
	    print("Error, not configured \n");
	    state->internalState = STATE_ERROR;
	    getErrorState()->badConfig = 1;
	  }
	  break;
	}
        case PACKET_ID_SET_PID_SPEED: {
	  if(state->internalState == STATE_CONFIGURED) {
	    struct setPidData *data = (struct setPidData *) curMsg->Data;
	    state->speedPIDValues.kp = data->kp;
	    state->speedPIDValues.ki = data->ki;
	    state->speedPIDValues.kd = data->kd;
	    state->speedPIDValues.minMaxPidOutput = data->minMaxPidOutput;
	    printf("Got PACKET_ID_SET_PID_SET Msg %h %h %h %hu\n", data->kp, data->ki, data->kd, data->minMaxPidOutput);
	  } else {
	    print("Error, not configured \n");
	    state->internalState = STATE_ERROR;
	    getErrorState()->badConfig = 1;
	  }
	  break;
	}
        case PACKET_ID_SET_CONFIGURE: {
	  print("Got PACKET_ID_SET_CONFIGURE Msg \n");
	  struct configure1Data *data = (struct configure1Data *) curMsg->Data;
	  state->useBackInduction = data->activeFieldCollapse;
	  state->useOpenLoop = data->openCircuit;
	  state->cascadedPositionController = data->cascadedPositionController;
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
	case PACKET_ID_ENCODER_CONFIG: {
	    if(state->internalState == STATE_CONFIGURED) {
		//do not allow to configure encoders while PID might be active
		getErrorState()->badConfig = 1;
		getErrorState()->encodersNotInitalized = 1;
	    } else {
		struct encoderConfiguration *encData = (struct encoderConfiguration *) curMsg->Data;
		encoderInit(encData->ticksPerTurnIntern, encData->tickDividerIntern);
		encoderInitExtern(encData->ticksPerTurnExtern, encData->tickDividerExtern);
	    }
	}
	break;
      default: 
	{
	  u16 id = curMsg->StdId;
	  
	  printf("Got unknown packet id : %hu ! \n", id);
	  break;
	}
      }
    
}
