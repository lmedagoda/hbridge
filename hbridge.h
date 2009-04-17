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

enum DRIVE_MODES {
  PWM = 0,
  SPEED = 1,
  POSITION = 2
};

struct Status {
  unsigned short current;
  unsigned short index;
  unsigned short position;
  signed short pwm;
  unsigned char errors;
  enum HOST_IDS host;
};

struct Configuration {
  uint8_t openCircuit;
  uint8_t activeFieldCollapse;
  uint8_t externalTempSensor;
  uint8_t cascadedPositionController;
  uint8_t pidDebugActive;
  unsigned char maxMotorTemp;
  unsigned char maxMotorTempCount;
  unsigned char maxBoardTemp;
  unsigned char maxBoardTempCount;
  unsigned short timeout;
  unsigned short maxCurrent;
  unsigned char maxCurrentCount;
  unsigned short pwmStepPerMs;

#ifndef __orogen
  Configuration() : openCircuit(0), activeFieldCollapse(0), externalTempSensor(0), cascadedPositionController(0), maxMotorTemp(0), maxMotorTempCount(0), maxBoardTemp(0), maxBoardTempCount(0), timeout(0), maxCurrent(0), maxCurrentCount(0), pwmStepPerMs(0) {};
  
  bool operator == (const Configuration &val) const {
    return openCircuit == val.openCircuit &&
    activeFieldCollapse  == val.activeFieldCollapse &&
    externalTempSensor == val.externalTempSensor &&
    cascadedPositionController == val.cascadedPositionController &&
    maxMotorTemp == val.maxMotorTemp &&
    maxMotorTempCount == val.maxMotorTempCount &&
    maxBoardTemp == val.maxBoardTemp &&
    maxBoardTempCount == val.maxBoardTempCount &&
    timeout == val.timeout &&
    maxCurrent == val.maxCurrent &&
    maxCurrentCount == val.maxCurrentCount &&
      pwmStepPerMs == val.pwmStepPerMs;

  };

  bool operator != (const Configuration &val) const {
    return openCircuit != val.openCircuit ||
    activeFieldCollapse  != val.activeFieldCollapse ||
    externalTempSensor != val.externalTempSensor ||
    cascadedPositionController != val.cascadedPositionController ||
    maxMotorTemp != val.maxMotorTemp ||
    maxMotorTempCount != val.maxMotorTempCount ||
    maxBoardTemp != val.maxBoardTemp ||
    maxBoardTempCount != val.maxBoardTempCount ||
    timeout != val.timeout ||
    maxCurrent != val.maxCurrent ||
    maxCurrentCount != val.maxCurrentCount ||
    pwmStepPerMs != val.pwmStepPerMs;
  };
#endif  


};

struct PiezoStatus {
  int piezo[4];
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
 

#ifndef __orogen
/**
	@author 
*/
class Driver {
public:

  static Driver &getInstance();
  

    ~Driver();

    int emergencyShutdown();

    int setNewTargetValues(const short int board1, const short int board2, const short int board3, const short int board4);
    int setDriveMode(enum DRIVE_MODES board1, enum DRIVE_MODES board2, enum DRIVE_MODES board3, enum DRIVE_MODES board4);

    int setSpeedPIDValues(const enum HOST_IDS host, const double kp,const double ki, const double kd, unsigned int minMaxValue);
    int setPositionPIDValues(const enum HOST_IDS host, const double kp,const double ki, const double kd, unsigned int minMaxValue);

    int setConfiguration(const enum HOST_IDS host, const Configuration);

    bool getNextStatus(Status &status);
    int openCanDevice(std::string &path);
    bool canInitalized();
    void getStatusFromCanMessage(can_msg &msg, Status &status);
    void getSpeedDebugFromCanMessage(can_msg &msg, SpeedDebug &sdbg);
    void getPIDDebugFromCanMessage(can_msg &msg, PIDDebug &dbg);
    void getPosDebugFromCanMessage(can_msg &msg, PosDebug &sdbg);
    void getPizeoFromCanMessage(can_msg &msg, PiezoStatus &status);
    
    bool getNextCanMessage(can_msg &msg);
    bool isStatusPacket(can_msg &msg);
    bool isSpeedDebugPacket(can_msg &msg);
    bool isPIDSpeedDebugPacket(can_msg &msg);
    bool isPIDPositionDebugPacket(can_msg &msg);
    bool isPosDebugPacket(can_msg &msg);
    bool isPiezoPacket(can_msg &msg);
    
    int getFileDescriptor() const;
    
private:
    Driver();
    int canFd;
    bool initalized;
    static Driver *instance;


    int sendCanMessage(struct can_msg *msg, const unsigned char dlc, const unsigned short id);
    int receiveCanMessage(struct can_msg *msg, unsigned int timeout);

};
#endif

}

#endif
