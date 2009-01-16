//
// C++ Implementation: interface
//
// Description: 
//
//
// Author:  <>, (C) 2008
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "hbridge.h"
#include <stdint.h>

//define types similar to STM32 firmeware lib,
//so we can use same protocoll.h
typedef uint8_t u8;
typedef uint16_t u16;
typedef int16_t s16;
#define __NO_STM32

#include "firmware/protocol.h"
#include "hico_api.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/select.h>
#include <iostream>
#include <errno.h>	
	 
namespace hbridge {

Interface *Interface::instance;

  Interface::Interface() : initalized(false)
{
}

Interface &Interface::getInstance() {
  if(!instance)
    instance = new Interface();

  return *instance;
};
  

Interface::~Interface()
{
}

/*!
    \fn Hbridge::Interface::setNewTargetValues(const short int board1, const short int board2, const short int board3, const short int board4)
 */
int Interface::setNewTargetValues(const short int board1, const short int board2, const short int board3, const short int board4)
{
    if(!initalized)
	return -1;

    struct can_msg msg;
    struct setValueData *data = (struct setValueData *) msg.data;

    data->board1Value = board1;
    data->board2Value = board2;
    data->board3Value = board3;
    data->board4Value = board4;

    return sendCanMessage(&msg, sizeof(struct setValueData), PACKET_ID_SET_VALUE);
}


int Interface::setDriveMode(enum DRIVE_MODES board1, enum DRIVE_MODES board2, enum DRIVE_MODES board3, enum DRIVE_MODES board4) {
    if(!initalized)
	return -1;

    struct can_msg msg;
    struct setModeData *data = (struct setModeData *) msg.data;

    data->board1Mode = board1;
    data->board2Mode = board2;
    data->board3Mode = board3;
    data->board4Mode = board4;

    return sendCanMessage(&msg, sizeof(struct setValueData), PACKET_ID_SET_MODE);
}


int Interface::setSpeedPIDValues(const enum HOST_IDS host, const double kp,const double ki, const double kd, unsigned int minMaxValue) {
    if(!initalized)
	return -1;

    struct can_msg msg;
    struct setPidData *data = (struct setPidData *) msg.data;

    data->kp = kp * 100;
    data->ki = ki * 100;
    data->kd = kd * 100;
    data->minMaxPidOutput = minMaxValue;
    return sendCanMessage(&msg, sizeof(struct setModeData), host | PACKET_ID_SET_PID_SPEED);
}

int Interface::setPositionPIDValues(const enum HOST_IDS host, const double kp,const double ki, const double kd, unsigned int minMaxValue) {
    if(!initalized)
	return -1;
    
    struct can_msg msg;
    struct setPidData *data = (struct setPidData *) msg.data;

    data->kp = kp * 100;
    data->ki = ki * 100;
    data->kd = kd * 100;
    data->minMaxPidOutput = minMaxValue;
    return sendCanMessage(&msg, sizeof(struct setModeData), host | PACKET_ID_SET_PID_POS);
}

int Interface::emergencyShutdown() {
    struct can_msg msg;
    if(sendCanMessage(&msg, 0, PACKET_ID_EMERGENCY_STOP) < 0 ) {
	return -1;
    }
    return 0;
}

int Interface::getFileDescriptor() const {
  if(!initalized)
    return -1;
  return canFd;
}

bool Interface::canInitalized() {
  return initalized;
}


int Interface::setConfiguration(const enum HOST_IDS host, const Configuration newConfig) {
    if(!initalized)
	return -1;

    struct can_msg msg1;
    struct can_msg msg2;

    struct configure1Data *conf1 = (struct configure1Data *) msg1.data;
    struct configure2Data *conf2 = (struct configure2Data *) msg2.data;

    conf1->openCircuit = newConfig.openCircuit;
    conf1->activeFieldCollapse = newConfig.activeFieldCollapse;
    conf1->externalTempSensor = newConfig.externalTempSensor;
    conf1->cascadedPositionController = newConfig.cascadedPositionController;
    conf1->enablePIDDebug = 0;
    conf1->unused = 0;
    conf1->maxMotorTemp = newConfig.maxMotorTemp;
    conf1->maxMotorTempCount = newConfig.maxMotorTempCount;
    conf1->maxBoardTemp = newConfig.maxBoardTemp;
    conf1->maxBoardTempCount = newConfig.maxBoardTempCount;
    conf1->timeout = newConfig.timeout;

    conf2->maxCurrent = newConfig.maxCurrent;
    conf2->maxCurrentCount = newConfig.maxCurrentCount;
    conf2->pwmStepPerMs = newConfig.pwmStepPerMs;

    if(sendCanMessage(&msg1, sizeof(struct configure1Data), PACKET_ID_SET_CONFIGURE | host) < 0 ) {
	return -1;
    }
    
    if(sendCanMessage(&msg2, sizeof(struct configure2Data), PACKET_ID_SET_CONFIGURE2 | host) < 0 ) {
	return -1;
    }

    return 0;
}

bool Interface::getNextCanMessage(can_msg &msg) {
  if(!initalized)
    return false;

  int ret = receiveCanMessage(&msg, 100);
  if(ret <= 0) {
    return false;
  }
  return true;
}


bool Interface::isStatusPacket(can_msg &msg) {
  return (msg.id & 0x1F) == PACKET_ID_STATUS;
}

bool Interface::isSpeedDebugPacket(can_msg &msg) {
  return (msg.id & 0x1F) == PACKET_ID_SPEED_DEBUG;
}

bool Interface::isPosDebugPacket(can_msg &msg) {
  return (msg.id & 0x1F) == PACKET_ID_POS_DEBUG;
}

bool Interface::isPIDSpeedDebugPacket(can_msg &msg) {
  return (msg.id & 0x1F) == PACKET_ID_PID_DEBUG_SPEED;
}

bool Interface::isPIDPositionDebugPacket(can_msg &msg) {
  return (msg.id & 0x1F) == PACKET_ID_PID_DEBUG_POS;
}

void Interface::getStatusFromCanMessage(can_msg &msg, Status &status) {
  struct statusData *data = (struct statusData *) msg.data; 
  
  status.current = data->currentValue;
  status.position = data->position;
  status.motorTemp = data->tempMotor;
  status.hbridgeTemp = data->tempHBrigde;
  status.index = data->index;
  status.errors = data->error;
  status.host = (enum HOST_IDS) (msg.id & ~0x1F);  
}

void Interface::getSpeedDebugFromCanMessage(can_msg &msg, SpeedDebug &sdbg) {
  struct speedDebugData *data = (struct speedDebugData *) msg.data;

  sdbg.targetVal = data->targetVal;
  sdbg.pwmVal = data->pwmVal;
  sdbg.encoderVal = data->encoderVal;
  sdbg.speedVal = data->speedVal;
  sdbg.host = (enum HOST_IDS) (msg.id & ~0x1F);  
}

void Interface::getPosDebugFromCanMessage(can_msg &msg, PosDebug &sdbg) {
  struct posDebugData *data = (struct posDebugData *) msg.data;

  sdbg.targetVal = data->targetVal;
  sdbg.pwmVal = data->pwmVal;
  sdbg.encoderVal = data->encoderVal;
  sdbg.posVal = data->posVal;
  sdbg.host = (enum HOST_IDS) (msg.id & ~0x1F);  
}


void Interface::getPIDDebugFromCanMessage(can_msg &msg, PIDDebug &dbg) {
  struct pidDebugData *data = (struct pidDebugData *) msg.data;
  dbg.pPart = data->pPart;
  dbg.iPart = data->iPart;
  dbg.dPart = data->dPart;
  dbg.host = (enum HOST_IDS) (msg.id & ~0x1F);
}


bool Interface::getNextStatus(Status &status) {
    if(!initalized)
	return false;

    can_msg msg;

    while(1) {
	int ret = receiveCanMessage(&msg, 100);
	if(ret <= 0) {
	    return false;
	}

	//check if we got a status packet
	if((msg.id & 0x1F) != PACKET_ID_STATUS) {
	    std::cout << "Got a packet, that did not have status ID" << std::endl;
	    continue;
	}
	
	struct statusData *data = (struct statusData *) msg.data; 
	
	status.current = data->currentValue;
	status.position = data->position;
	status.motorTemp = data->tempMotor;
	status.hbridgeTemp = data->tempHBrigde;
	status.index = data->index;
	status.errors = data->error;
	status.host = (enum HOST_IDS) (msg.id & ~0x1F);

	return true;
    }
    return false;
}

int Interface::receiveCanMessage(struct can_msg *msg, unsigned int timeout) {
  int ret;
  unsigned int readCount = 0;
  while(readCount < sizeof(struct can_msg)) { 
    ret=read(canFd, msg+ readCount,sizeof(struct can_msg) - readCount);
    if(ret < 0) {
      if(errno != EAGAIN)
	return -1;

      if(errno == EAGAIN && readCount == 0)
	return 0;
    } else {  
      readCount += ret;
    }
  }
  return 1;
}

int Interface::sendCanMessage(struct can_msg *msg, const unsigned char dlc, const unsigned short id) {
  msg->ff = 0;
  msg->rtr = 0;
  msg->id = id;
  msg->dlc = dlc;
  
  unsigned int send = 0;
  int ret;
  while(send < sizeof(struct can_msg)) {
    ret = write(canFd, msg + send, sizeof(struct can_msg) - send);
    if(ret < 0) {
      if(errno != EAGAIN)
	return -1;
    } else {
      send +=ret;
    }
  }
  return 0;
}

/*!
    \fn Hbridge::Interface::openCanDevice(char *path)
 */
int Interface::openCanDevice(std::string &path)
{
    if(initalized)
      return true;
  
    int ret;

    canFd = open(path.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
    if (canFd == -1) {
	perror(path.c_str());
        return -1;
    }

    //reset board, to have it in defined state
    ret = ioctl(canFd, IOC_RESET_BOARD);
    if (ret == -1) {
	perror("IOC_RESET_BOARD");
	return -1;
    }

    //set bitrate to 1 mbit
    int bitrate = BITRATE_1000k;
    ret = ioctl(canFd, IOC_SET_BITRATE, &bitrate);
    if (ret == -1) {
	perror("IOC_SET_BITRATE");
        return -1;
    }

    //set to active mode
    ret=ioctl(canFd,IOC_STOP);
    if (ret == -1) {
	perror("IOC_STOP");
        return -1;
    }
    ret=ioctl(canFd,IOC_START);
    if (ret == -1) {
	perror("IOC_START");
        return -1;
    }

    initalized = true;

    return true;
}

}




