#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE "test"
#define BOOST_AUTO_TEST_MAIN

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/test/execution_monitor.hpp>  

#include <canbus.hh>
#include <HBridgeDriver.hpp>

can::Driver driver;
hbridge::Driver hbd;

BOOST_AUTO_TEST_CASE(test_case)
{
    ::boost::execution_monitor ex_mon;

    std::cout << "Trying to open CAN device /dev/can0" << std::endl;
    BOOST_CHECK(driver.open("/dev/can0"));

    // Initialise hbridge hbridge instance

    // Still needs configuration (where are the config values stored?)
    hbridge::Configuration config;

    // TODO: load configuration

    std::cout << "Configuring hbridges" << std::endl;
    hbridge::MessagePair config1 = hbd.setConfiguration(hbridge::H_BRIDGE1, config);
    driver.write(config1.first);
    driver.write(config1.second);
    hbridge::MessagePair config2 = hbd.setConfiguration(hbridge::H_BRIDGE2, config);
    driver.write(config2.first);
    driver.write(config2.second);
    hbridge::MessagePair config3 = hbd.setConfiguration(hbridge::H_BRIDGE3, config);
    driver.write(config3.first);
    driver.write(config3.second);
    hbridge::MessagePair config4 = hbd.setConfiguration(hbridge::H_BRIDGE4, config);
    driver.write(config4.first);
    driver.write(config4.second);

    std::cout << "Set drive modes" << std::endl;
    driver.write(hbd.setDriveMode(hbridge::DM_SPEED));

    std::cout << "Rotate wheel 1 for one second" << std::endl;
    for (int i = 0; i < 100; i++)
    {
        driver.write(hbd.setTargetValues(hbridge::TICKS_PER_TURN, 0, 0, 0));
        usleep(10000);
    }

    std::cout << "Rotate wheel 2 for one second" << std::endl;
    for (int i = 0; i < 100; i++)
    {
        driver.write(hbd.setTargetValues(0, hbridge::TICKS_PER_TURN, 0, 0));
        usleep(10000);
    }

    std::cout << "Rotate wheel 3 for one second" << std::endl;
    for (int i = 0; i < 100; i++)
    {
        driver.write(hbd.setTargetValues(0, 0, hbridge::TICKS_PER_TURN, 0));
        usleep(10000);
    }

    std::cout << "Rotate wheel 4 for one second" << std::endl;
    for (int i = 0; i < 100; i++)
    {
        driver.write(hbd.setTargetValues(0, 0, 0, hbridge::TICKS_PER_TURN));
        usleep(10000);
    }
};

// EOF
