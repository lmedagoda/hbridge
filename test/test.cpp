#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE "test"
#define BOOST_AUTO_TEST_MAIN

#include <iostream>
#include <strings.h>

#include <boost/test/unit_test.hpp>
#include <boost/test/execution_monitor.hpp>  
#include <arpa/inet.h>

#include <canbus.hh>
#include "../HBridgeDriver.hpp"
#include "../protocol.hpp"

can::Driver driver;
hbridge::Driver hbd;

// Checks for kp=400, ki=5, kd=0, minmax=1800
unsigned char pidData[] = { 144, 1, 5, 0, 0, 0, 8, 7 }; // 8 byte
size_t pidDataSize = 8;
// Configuration according to arc-ocu
unsigned char config1Data[] = { 1, 0, 60, 200, 60, 200, 50, 0 }; // 8 byte
size_t config1DataSize = 8;
unsigned char config2Data[] = { 136, 19, 250, 200, 0, 0, 0, 0 }; // 5 byte
size_t config2DataSize = 5;
// Drive mode config data
unsigned char dmData[] = { 1, 1, 1, 1, 0, 0, 0, 0}; // 4 byte
size_t dmDataSize = 4;
// Set value data
size_t valueDataSize = 8;
unsigned char value1Data[] = { 0xec, 0xff, 0, 0, 0, 0, 0, 0 }; // 8 byte
unsigned char value2Data[] = { 0, 0, 20, 0, 0, 0, 0, 0 }; // 8 byte
unsigned char value3Data[] = { 0, 0, 0, 0, 20, 0, 0, 0 }; // 8 byte
unsigned char value4Data[] = { 0, 0, 0, 0, 0, 0, 20, 0 }; // 8 byte

bool checkMessage(
        int     board_index,
        firmware::packetIDs  pid,
        const unsigned char *testData,
        size_t               testDataSize,
        can::Message        &msg)
{
    unsigned int reqID = (board_index << 5) | pid;

     if (msg.can_id != reqID)
     {
         std::cerr << "Error: wrong CAN id (" << msg.can_id << "!=" << reqID << ")" << std::endl;
         return false;
     }

     if (msg.size != testDataSize)
     {
         std::cerr << "Error: wrong size (" << msg.size << "!=" << testDataSize << ")" << std::endl;
         return false;
     }

     for (int i = 0; i < msg.size; i++)
     {
         if (msg.data[i] != testData[i])
         {
             std::cerr << "Error: data different at position " << i << std::endl;
             std::cerr << "Expected: " << std::hex;
             for (unsigned int i = 0; i < testDataSize; i++)
             {
                 std::cerr << (int)(testData[i] & 0xff) << " ";
             }
             std::cerr << std::dec << std::endl;
             std::cerr << "Got: " << std::hex;
             for (unsigned int i = 0; i < msg.size; i++)
             {
                 std::cerr << (int)(msg.data[i] & 0xff) << " ";
             }
             std::cerr << std::dec << std::endl;

             return false;
         }
     }

     return true;
}

BOOST_AUTO_TEST_CASE(test_case)
{
    ::boost::execution_monitor ex_mon;

    std::cout << "Trying to open CAN device /dev/can0" << std::endl;
    BOOST_CHECK(driver.open("/dev/can0"));

    // Initialise hbridge hbridge instance


    // Still needs configuration (where are the config values stored?)
    hbridge::Configuration config;

    bzero(&config, sizeof(hbridge::Configuration));

    config.openCircuit = 1;
    config.maxMotorTemp = 60;
    config.maxMotorTempCount = 200;
    config.maxBoardTemp = 60;
    config.maxBoardTempCount = 200;
    config.timeout = 50;
    config.maxCurrent = 5000;
    config.maxCurrentCount = 250;
    config.pwmStepPerMs = 200;

    std::cout << "Configuring hbridges" << std::endl;
    for (int i = 0; i < 4; ++i)
    {
        hbridge::MessagePair config_msgs = hbd.setConfiguration(i, config);
        BOOST_CHECK(checkMessage(i + 1, firmware::PACKET_ID_SET_CONFIGURE, config1Data, config1DataSize, config_msgs.first));
        BOOST_CHECK(checkMessage(i + 1, firmware::PACKET_ID_SET_CONFIGURE2, config2Data, config2DataSize, config_msgs.second));
        driver.write(config_msgs.first);
        driver.write(config_msgs.second);
    }

    std::cout << "Set PID configuration" << std::endl;
    for (int i = 0; i < 4; ++i)
    {
        can::Message pidmsg = hbd.setSpeedPID(i, 400.0, 5.0, 0.0, 1800.0);
        BOOST_CHECK(checkMessage(i + 1, firmware::PACKET_ID_SET_PID_SPEED, pidData, pidDataSize, pidmsg));
        driver.write(pidmsg);
    }

    std::cout << "Set drive modes" << std::endl;
    can::Message dmmsg = hbd.setDriveMode(hbridge::DM_SPEED);
    BOOST_CHECK(checkMessage(0, firmware::PACKET_ID_SET_MODE, dmData, dmDataSize, dmmsg));
    driver.write(dmmsg);

    std::cout << "Rotate wheel 1 for one second" << std::endl;
    for (int i = 0; i < 100; i++)
    {
    	can::Message msg = hbd.setTargetValues(20, 0, 0, 0);
        BOOST_CHECK(checkMessage(0, firmware::PACKET_ID_SET_VALUE, value1Data, valueDataSize, msg));
        driver.write(msg);
        usleep(10000);
    }
/*
    std::cout << "Rotate wheel 2 for one second" << std::endl;
    for (int i = 0; i < 100; i++)
    {
        driver.write(hbd.setTargetValues(0, hbridge::TICKS_PER_TURN / 1000.0, 0, 0));
        usleep(10000);
    }

    std::cout << "Rotate wheel 3 for one second" << std::endl;
    for (int i = 0; i < 100; i++)
    {
        driver.write(hbd.setTargetValues(0, 0, hbridge::TICKS_PER_TURN / 1000.0, 0));
        usleep(10000);
    }

    std::cout << "Rotate wheel 4 for one second" << std::endl;
    for (int i = 0; i < 100; i++)
    {
        driver.write(hbd.setTargetValues(0, 0, 0, hbridge::TICKS_PER_TURN / 1000.0));
        usleep(10000);
    }*/

    can::Message reset = hbd.emergencyShutdown();
    driver.write(reset);
};

// EOF
