//
// C++ Interface: interface
//
// Description: 
//
//
// Author:  <>, (C) 2008
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef HBRIDGEINTERFACE_H
#define HBRIDGEINTERFACE_H

#ifndef __orogen
#include "hico_api.h"
#include <stdint.h>
#include <string>
#endif

namespace hbridge {

enum HOST_IDS {
  H_BRIDGE_1 = (1<<5),
  H_BRIDGE_2 = (2<<5),
  H_BRIDGE_3 = (3<<5),
  H_BRIDGE_4 = (4<<5)
};

struct Status {
  unsigned short current;
  unsigned short index;
  unsigned short position;
  unsigned char hbridgeTemp;
  unsigned char motorTemp;
  unsigned char errors;
  enum HOST_IDS host;
};

struct Configuration {
  uint8_t openCircuit;
  uint8_t activeFieldCollapse;
  uint8_t externalTempSensor;
  uint8_t cascadedPositionController;
  unsigned char maxMotorTemp;
  unsigned char maxMotorTempCount;
  unsigned char maxBoardTemp;
  unsigned char maxBoardTempCount;
  unsigned short timeout;
  unsigned short maxCurrent;
  unsigned char maxCurrentCount;
  unsigned short pwmStepPerMs;
};

struct SpeedDebug {
  int16_t targetVal;
  int16_t pwmVal;
  uint16_t encoderVal;
  int16_t speedVal;
  enum HOST_IDS host;  
};

struct PosDebug {
  uint16_t targetVal;
  int16_t pwmVal;
  uint16_t encoderVal;
  uint16_t posVal;
  enum HOST_IDS host;  
};

struct PIDDebug {
  int16_t pPart;
  int16_t iPart;
  int16_t dPart;
  int16_t errorSum;
  enum HOST_IDS host;  
};


struct SpeedAndPIDDebug {
  struct PIDDebug pidDebugSpeed;
  struct SpeedDebug speedDebug;
  struct PIDDebug pidDebugPos;
  struct PosDebug posDebug;
};
 
 

struct AllStatus {
  struct Status status[4];
};

struct AllConfiguration {
  struct Configuration configs[4];
};
 
 

#ifndef __orogen
/**
	@author 
*/
class Interface{
public:

    Interface();

    ~Interface();

    int emergencyShutdown();

    int setNewTargetValues(const short int board1, const short int board2, const short int board3, const short int board4);

    int setPWMMode(const enum HOST_IDS host);
    int setSpeedMode(const enum HOST_IDS host, const double kp,const double ki, const double kd);
    int setPositionMode(const enum HOST_IDS host, const double kp,const double ki, const double kd);

    int setConfiguration(const enum HOST_IDS host, const Configuration);

    bool getNextStatus(Status &status);
    int openCanDevice(std::string &path);
    void getStatusFromCanMessage(can_msg &msg, Status &status);
    void getSpeedDebugFromCanMessage(can_msg &msg, SpeedDebug &sdbg);
    void getPIDDebugFromCanMessage(can_msg &msg, PIDDebug &dbg);
    void getPosDebugFromCanMessage(can_msg &msg, PosDebug &sdbg);
    
    bool getNextCanMessage(can_msg &msg);
    bool isStatusPacket(can_msg &msg);
    bool isSpeedDebugPacket(can_msg &msg);
    bool isPIDSpeedDebugPacket(can_msg &msg);
    bool isPIDPositionDebugPacket(can_msg &msg);
    bool isPosDebugPacket(can_msg &msg);
    
    int getFileDescriptor() const;
    
private:
    int canFd;
    bool initalized;

    int sendCanMessage(struct can_msg *msg, const unsigned char dlc, const unsigned short id);
    int receiveCanMessage(struct can_msg *msg, unsigned int timeout);

};
#endif

}

#endif
