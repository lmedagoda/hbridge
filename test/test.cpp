#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE "test"
#define BOOST_AUTO_TEST_MAIN

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/test/execution_monitor.hpp>  
#include <canbus.hh>
#include <Protocol.hpp>

can::Driver driver;
hbridge::Protocol protocol;

bool driveMotor(int value1, int value2, int value3, int value4)
{
/*
    // Start motor
    BOOST_CHECK(driver->setNewTargetValues(value1, value2, value3, value4) >= 0);
    // Wait some ms
    usleep(500000);
   
    bool started = false;

    // Old status message
    hbridge::Status statusOld;
    bzero(&statusOld, sizeof(hbridge::Status));

    // Current status message
    hbridge::Status status;
    bzero(&status, sizeof(hbridge::Status));

    unsigned long ticks = 0;

    // Start timestamp
    timeval tvStart;
    bzero(&tvStart, sizeof(timeval));

    // Intermediate timestamp
    timeval tv;
    bzero(&tv, sizeof(timeval));

    time_t timeElapsed = 0;

    // CAN message
    can_msg msg;
    bzero(&msg, sizeof(can_msg));

    gettimeofday(&tvStart, NULL);

    while (timeElapsed < 1000000)
    {
        int ret = driver->receiveCanMessage(&msg, 1000);

        if (driver->isStatusPacket(msg))
        {
            driver->getStatusFromCanMessage(msg, status);
            
            if (started)
            {
                statusOld = status;
                started = false;
            }

            int tickDiff = status.position - statusOld.position;
            tickDiff += (tickDiff < 0 ? ticksPerTurn : 0);

            ticks += tickDiff;

            gettimeofday(&tv, NULL);
            timeElapsed =
                (tv.tv_sec - tvStart.tv_sec) * 1000000 +
                (tv.tv_usec - tvStart.tv_usec);
        }
        else
        {
            // Better run a bit longer than quit to early
        }
    }

    BOOST_CHECK(driver->setNewTargetValues(0, 0, 0, 0) >= 0);

    usleep(500000);

    // Get rps
    double ticksPerSecond = (double)ticks * (1000000.0 / (double)timeElapsed);
    double rps = ticksPerSecond / (double)ticksPerTurn;
*/
    return false;
}

BOOST_AUTO_TEST_CASE(test_case)
{
    ::boost::execution_monitor ex_mon;

    std::cout << "Trying to open CAN device /dev/can0" << std::endl;
    BOOST_CHECK(driver.open("/dev/can0"));

    // Initialise hbridge protocol instance

    // Still needs configuration (where are the config values stored?)
    hbridge::Configuration config;

    // TODO: load configuration

    std::cout << "Configuring hbridges" << std::endl;
    hbridge::MessagePair config1 = protocol.setConfiguration(hbridge::H_BRIDGE1, config);
    driver.write(config1.first);
    driver.write(config1.second);
    hbridge::MessagePair config2 = protocol.setConfiguration(hbridge::H_BRIDGE2, config);
    driver.write(config2.first);
    driver.write(config2.second);
    hbridge::MessagePair config3 = protocol.setConfiguration(hbridge::H_BRIDGE3, config);
    driver.write(config3.first);
    driver.write(config3.second);
    hbridge::MessagePair config4 = protocol.setConfiguration(hbridge::H_BRIDGE4, config);
    driver.write(config4.first);
    driver.write(config4.second);

    std::cout << "Set drive modes" << std::endl;
    driver.write(protocol.setDriveMode(hbridge::DM_SPEED));

    std::cout << "Rotate wheel 1 for one second" << std::endl;
    for (int i = 0; i < 100; i++)
    {
        driver.write(protocol.setTargetValues(hbridge::TICKS_PER_TURN, 0, 0, 0));
        usleep(10000);
    }

    std::cout << "Rotate wheel 2 for one second" << std::endl;
    for (int i = 0; i < 100; i++)
    {
        driver.write(protocol.setTargetValues(0, hbridge::TICKS_PER_TURN, 0, 0));
        usleep(10000);
    }

    std::cout << "Rotate wheel 3 for one second" << std::endl;
    for (int i = 0; i < 100; i++)
    {
        driver.write(protocol.setTargetValues(0, 0, hbridge::TICKS_PER_TURN, 0));
        usleep(10000);
    }

    std::cout << "Rotate wheel 4 for one second" << std::endl;
    for (int i = 0; i < 100; i++)
    {
        driver.write(protocol.setTargetValues(0, 0, 0, hbridge::TICKS_PER_TURN));
        usleep(10000);
    }
};

// EOF
