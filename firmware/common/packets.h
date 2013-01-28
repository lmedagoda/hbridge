#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>

enum STATES
{
    STATE_UNCONFIGURED,
    STATE_SENSOR_ERROR,
    STATE_SENSORS_CONFIGURED,
    STATE_ACTUATOR_ERROR,
    STATE_ACTUATOR_CONFIGURED,
    STATE_CONTROLLER_CONFIGURED,
    STATE_RUNNING,
};

enum HIGH_PRIORITY_IDs
{
    PACKET_ID_EMERGENCY_STOP,
    PACKET_ID_SET_ALLOWED_SENDER,
    PACKET_ID_ERROR,
    PACKET_ID_ANNOUNCE_STATE,
    PACKET_ID_STATUS,
    PACKET_ID_EXTENDED_STATUS,
    PACKET_ID_ACK,
    
    PACKET_ID_SET_VALUE,
    PACKET_ID_SET_VALUE14,
    PACKET_ID_SET_VALUE58,
    
    PACKET_LOW_PRIORITY_DATA,
};

enum LOW_PRIORITY_IDs
{
    /**
     * DO NOT USE THIS ENTRY
     * This is a 'trick' to have coninous 
     * ids over both enums
     */
    PACKET_ID_LOWIDS_START = PACKET_LOW_PRIORITY_DATA,
    
    PACKET_ID_SET_SENSOR_CONFIG,
    PACKET_ID_CLEAR_SENSOR_ERROR,
    PACKET_ID_SET_ACTUATOR_CONFIG,
    PACKET_ID_CLEAR_ACTUATOR_ERROR,
    PACKET_ID_SET_ACTIVE_CONTROLLER,

    PACKET_ID_REQUEST_STATE,
    
    PACKED_ID_REQUEST_VERSION,
    PACKED_ID_VERSION,

    PACKET_ID_SET_SPEED_CONTROLLER_DATA,
    PACKET_ID_SPEED_CONTROLLER_DEBUG,

    PACKET_ID_POS_CONTROLLER_DEBUG,
    PACKET_ID_SET_POS_CONTROLLER_DATA,
    
    PACKET_ID_SET_UNCONFIGURED,
    PACKET_ID_SET_ACTUATOR_UNCONFIGURED,
    
    /**
     * DO NOT USE THIS ENTRY
     * This is a trick to get the total number of
     * of IDs in use. This entry must allway be 
     * the last one in the enum.
     * */
    PACKET_ID_TOTAL_COUNT,
};

enum encoderTypes {
    NO_ENCODER = 0,
    QUADRATURE,
    QUADRATURE_WITH_ZERO,
    IC_HOUSE_MH_Y,
    BMMV30_SSI,
    ANALOG_VOLTAGE,
    NUM_ENCODERS,
};

enum controllerModes {
  CONTROLLER_MODE_NONE = 0,
  CONTROLLER_MODE_PWM = 1,
  CONTROLLER_MODE_SPEED = 2,
  CONTROLLER_MODE_POSITION = 3,
  NUM_CONTROLLERS
};

enum controllerInputEncoder {
    INTERNAL = 0,
    EXTERNAL = 1,
};

struct ackData {
    unsigned short packetId;
    unsigned short crc;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct announceStateData {
    enum STATES curState:8;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct setAllowedSenderData {
    /**
     * If this value is != 0 this
     * means that only the mainboard 
     * is allowed to send. The effect 
     * of this is that any packet originating
     * from an different sender is discarded.
     * 
     * To allow the PC to communicate with
     * the motor driver set this value to
     * Zero.
     * */
    uint8_t onlyMainboard;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct encoderConfiguration {
    enum encoderTypes encoderType:8;
    uint32_t ticksPerTurn;
    uint8_t tickDivider;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct sensorConfig 
{
    unsigned externalTempSensor :1;
    uint16_t statusEveryMs;
    struct encoderConfiguration encoder1Config;
    struct encoderConfiguration encoder2Config; 
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct actuatorConfig
{
  unsigned openCircuit :1;
  enum controllerInputEncoder controllerInputEncoder :1;
  unsigned unused :6;
  uint8_t maxMotorTemp;
  uint8_t maxMotorTempCount;
  uint8_t maxBoardTemp;
  uint8_t maxBoardTempCount;
  uint16_t timeout;
  uint16_t maxCurrent;
  uint8_t maxCurrentCount;
  uint16_t pwmStepPerMs;
} __attribute__ ((packed)) __attribute__((__may_alias__));


struct pidDebugData {
  int16_t pPart;
  int16_t iPart;
  int16_t dPart;
  uint16_t minMaxPidOutput;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct speedDebugData {
  int16_t targetVal;
  int16_t pwmVal;
  unsigned short encoderVal;
  int16_t speedVal;
  struct pidDebugData pidData;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct posDebugData {
  uint16_t targetVal;
  int16_t pwmVal;
  uint16_t encoderVal;
  uint16_t posVal;
  struct pidDebugData pidData;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct setPidData {
  int16_t kp;
  int16_t ki;
  int16_t kd;
  uint16_t minMaxPidOutput;
} __attribute__ ((packed)) __attribute__((__may_alias__)) ;

struct posControllerData {
    uint16_t minHystDist;
    uint16_t maxHystDist;
    unsigned hysteresisActive:1;
    unsigned allowWrapAround:1;
    unsigned debugActive:1;
    unsigned unused:5;
    uint8_t overDistCount;
    struct setPidData pidData;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct errorData {
    uint8_t temperature;
    uint16_t position;
    unsigned index :10;
    unsigned externalPosition:12;
    unsigned motorOverheated:1;
    unsigned boardOverheated:1;
    unsigned overCurrent:1;
    unsigned timeout:1;
    unsigned badConfig:1;
    unsigned encodersNotInitalized:1;
    unsigned hardwareShutdown:1;
    unsigned unused:3;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct statusData {
  signed pwm :12;
  unsigned externalPosition:12;
  uint16_t position;
  unsigned currentValue :14;
  unsigned index :10;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct extendedStatusData {
    uint8_t temperature;
    uint8_t motorTemperature;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct setValueData {
  int16_t board1Value;
  int16_t board2Value;
  int16_t board3Value;
  int16_t board4Value;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct setActiveControllerData {
  enum controllerModes controllerId:8;
} __attribute__ ((packed)) __attribute__((__may_alias__));

enum LOW_PRIO_TYPE
{
    ///Header, containing description of the upcomming data packets
    TYPE_HEADER = 0,
//     ///Everything is fine, message received
//     TYPE_ACK,
//     ///Message was broken, resend    
//     TYPE_REQUEST_RESEND,
//     ///Firmware does not know this message
//     TYPE_NACK,
    ///Data packet(s) followed by header
    TYPE_DATA,
};

struct LowPrioHeader
{
    enum LOW_PRIORITY_IDs id:8;
    ///Size in bytes of the data
    ///Note the data may be distributed over multiple packets
    uint8_t size;
    ///checksum of the data
    uint16_t crc;
    ///Sender ID ?
    ///DO ack ?
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct LowPrioPacket
{
    enum LOW_PRIO_TYPE type:2;
    unsigned sequenceNumber:6;
} __attribute__ ((packed)) __attribute__((__may_alias__));

const char *getPacketName(uint16_t packetId);

#endif