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
#include <HBridgeDriver.hpp>
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
unsigned char value1Data[] = { 20, 0, 0, 0, 0, 0, 0, 0 }; // 8 byte
unsigned char value2Data[] = { 0, 0, 20, 0, 0, 0, 0, 0 }; // 8 byte
unsigned char value3Data[] = { 0, 0, 0, 0, 20, 0, 0, 0 }; // 8 byte
unsigned char value4Data[] = { 0, 0, 0, 0, 0, 0, 20, 0 }; // 8 byte

bool checkMessage(
        hbridge::BOARDID     board,
        firmware::packetIDs  pid,
        const unsigned char *testData,
        size_t               testDataSize,
        can::Message        &msg)
{
    unsigned int reqID = board | pid;

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
    hbridge::MessagePair config1 = hbd.setConfiguration(hbridge::H_BRIDGE1, config);
    BOOST_CHECK(checkMessage(hbridge::H_BRIDGE1, firmware::PACKET_ID_SET_CONFIGURE, config1Data, config1DataSize, config1.first));
    BOOST_CHECK(checkMessage(hbridge::H_BRIDGE1, firmware::PACKET_ID_SET_CONFIGURE2, config2Data, config2DataSize, config1.second));
    driver.write(config1.first);
    driver.write(config1.second);
    hbridge::MessagePair config2 = hbd.setConfiguration(hbridge::H_BRIDGE2, config);
    BOOST_CHECK(checkMessage(hbridge::H_BRIDGE2, firmware::PACKET_ID_SET_CONFIGURE, config1Data, config1DataSize, config2.first));
    BOOST_CHECK(checkMessage(hbridge::H_BRIDGE2, firmware::PACKET_ID_SET_CONFIGURE2, config2Data, config2DataSize, config2.second));
    driver.write(config2.first);
    driver.write(config2.second);
    hbridge::MessagePair config3 = hbd.setConfiguration(hbridge::H_BRIDGE3, config);
    BOOST_CHECK(checkMessage(hbridge::H_BRIDGE3, firmware::PACKET_ID_SET_CONFIGURE, config1Data, config1DataSize, config3.first));
    BOOST_CHECK(checkMessage(hbridge::H_BRIDGE3, firmware::PACKET_ID_SET_CONFIGURE2, config2Data, config2DataSize, config3.second));
    driver.write(config3.first);
    driver.write(config3.second);
    hbridge::MessagePair config4 = hbd.setConfiguration(hbridge::H_BRIDGE4, config);
    BOOST_CHECK(checkMessage(hbridge::H_BRIDGE4, firmware::PACKET_ID_SET_CONFIGURE, config1Data, config1DataSize, config4.first));
    BOOST_CHECK(checkMessage(hbridge::H_BRIDGE4, firmware::PACKET_ID_SET_CONFIGURE2, config2Data, config2DataSize, config4.second));
    driver.write(config4.first);
    driver.write(config4.second);

    std::cout << "Set PID configuration" << std::endl;
    can::Message pidmsg1 = hbd.setSpeedPID(hbridge::H_BRIDGE1, 400.0, 5.0, 0.0, 1800.0);
    BOOST_CHECK(checkMessage(hbridge::H_BRIDGE1, firmware::PACKET_ID_SET_PID_SPEED, pidData, pidDataSize, pidmsg1));
    driver.write(pidmsg1);
    can::Message pidmsg2 = hbd.setSpeedPID(hbridge::H_BRIDGE2, 400.0, 5.0, 0.0, 1800.0);
    BOOST_CHECK(checkMessage(hbridge::H_BRIDGE2, firmware::PACKET_ID_SET_PID_SPEED, pidData, pidDataSize, pidmsg2));
    driver.write(pidmsg2);
    can::Message pidmsg3 = hbd.setSpeedPID(hbridge::H_BRIDGE3, 400.0, 5.0, 0.0, 1800.0);
    BOOST_CHECK(checkMessage(hbridge::H_BRIDGE3, firmware::PACKET_ID_SET_PID_SPEED, pidData, pidDataSize, pidmsg3));
    driver.write(pidmsg3);
    can::Message pidmsg4 = hbd.setSpeedPID(hbridge::H_BRIDGE4, 400.0, 5.0, 0.0, 1800.0);
    BOOST_CHECK(checkMessage(hbridge::H_BRIDGE4, firmware::PACKET_ID_SET_PID_SPEED, pidData, pidDataSize, pidmsg4));
    driver.write(pidmsg4);

    std::cout << "Set drive modes" << std::endl;
    can::Message dmmsg = hbd.setDriveMode(hbridge::DM_SPEED);
    BOOST_CHECK(checkMessage(hbridge::H_BROADCAST, firmware::PACKET_ID_SET_MODE, dmData, dmDataSize, dmmsg));
    driver.write(dmmsg);

    std::cout << "Rotate wheel 1 for one second" << std::endl;
    for (int i = 0; i < 100; i++)
    {
    	can::Message msg = hbd.setTargetValues(20, 0, 0, 0);
        BOOST_CHECK(checkMessage(hbridge::H_BROADCAST, firmware::PACKET_ID_SET_VALUE, value1Data, valueDataSize, msg));
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
