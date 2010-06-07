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
#include "../HBridge.hpp"

can::Driver *driver = 0;
hbridge::Driver hbd;
int hbridge_id = 0;

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

bool checkPrintError(struct hbridge::ErrorState &error) {
    bool ret = false;
    if(error.badConfig) {
	std::cout << "HBridge reported error:badConfig" << std::endl;
	ret = true;
    }
    if(error.boardOverheated) {
	std::cout << "HBridge reported error:boardOverheated" << std::endl;
	ret = true;
    }
    if(error.encodersNotInitalized) {
	std::cout << "HBridge reported error:encodersNotInitalized" << std::endl;
	ret = true;
    }
    if(error.hardwareShutdown) {
	std::cout << "HBridge reported error:hardwareShutdown" << std::endl;
	ret = true;
    }
    if(error.motorOverheated) {
	std::cout << "HBridge reported error:motorOverheated" << std::endl;
	ret = true;
    }
    if(error.overCurrent) {
	std::cout << "HBridge reported error:overCurrent" << std::endl;
	ret = true;
    }
    if(error.timeout) {
	std::cout << "HBridge reported error:timeout" << std::endl;
	ret = true;
    }
    return ret;
}

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

void initDriver() {
    if(driver)
      return;

    const char *can_device = "can0";

    std::cout << "Trying to open CAN device " << can_device << std::endl;
#if CANBUS_VERSION >= 101
    BOOST_CHECK(driver = can::openCanDevice(can_device));
#else
    BOOST_CHECK(driver = new can::Driver());
    BOOST_CHECK(driver->open(can_device));
#endif

}

hbridge::Configuration getDefaultConfig() {
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

    return config;
}

BOOST_AUTO_TEST_CASE(static_tests) {
    hbridge::Configuration config = getDefaultConfig();
    hbridge::MessagePair config_msgs;
    std::cout << "Testing packet building" << std::endl;

    for (int i = 0; i < 4; ++i)
    {
        hbridge::MessagePair config_msgs = hbd.setConfiguration(i, config);
        BOOST_CHECK(checkMessage(i + 1, firmware::PACKET_ID_SET_CONFIGURE, config1Data, config1DataSize, config_msgs.first));
        BOOST_CHECK(checkMessage(i + 1, firmware::PACKET_ID_SET_CONFIGURE2, config2Data, config2DataSize, config_msgs.second));

    }

    for (int i = 0; i < 4; ++i)
    {
        can::Message pidmsg = hbd.setSpeedPID(i, 400.0, 5.0, 0.0, 1800.0);
        BOOST_CHECK(checkMessage(i + 1, firmware::PACKET_ID_SET_PID_SPEED, pidData, pidDataSize, pidmsg));
    }

    can::Message dmmsg = hbd.setDriveMode(hbridge::DM_SPEED);
    BOOST_CHECK(checkMessage(0, firmware::PACKET_ID_SET_MODE, dmData, dmDataSize, dmmsg));

    can::Message msg = hbd.setTargetValues(20, 0, 0, 0);
    BOOST_CHECK(checkMessage(0, firmware::PACKET_ID_SET_VALUE, value1Data, valueDataSize, msg));



}

BOOST_AUTO_TEST_CASE(encoder_not_initalized) {
    initDriver();
    hbridge::Configuration config = getDefaultConfig();
    config.timeout = 0;
    hbridge::MessagePair config_msgs;
    can::Message msg;

    config_msgs = hbd.setConfiguration(hbridge_id, config);

    int i;
    std::cout << "Testing if HBridge goes into error state if encoders not initalized " << (hbridge_id+1) << std::endl;
    driver->write(config_msgs.first);
    driver->write(config_msgs.second);
    for (i = 0; i < 50; i++)
    {
        usleep(10000);
	while(driver->getPendingMessagesCount() > 0) {
	    msg = driver->read() ;
	    
	    hbd.updateFromCAN(msg);
	    }
	    hbridge::BoardState state = hbd.getState(hbridge_id);
	    checkPrintError(state.error);
	    
	    if(state.error.encodersNotInitalized) {
		std::cout << "Correct: HBridge reportet encoderNotInitalized Error " << std::endl;
		break;
	    }
    }    
    BOOST_CHECK(i < 50);

}


BOOST_AUTO_TEST_CASE(configure_encoder_clear_Error) {
    initDriver();
    can::Message msg;

    std::cout << "Configuring Encoders " << (hbridge_id +1) << std::endl;
    hbridge::EncoderConfiguration encConf;
    encConf.tickDivider = 4;
    encConf.ticksPerTurn = hbridge::TICKS_PER_TURN * 4;

    encConf.tickDividerExtern = 1;
    encConf.ticksPerTurnExtern = 4096;
    
    can::Message encConfMsg = hbd.setEncoderConfiguration(hbridge_id, encConf);
    driver->write(encConfMsg);

    //now reconfigure
    hbridge::Configuration config = getDefaultConfig();
    hbridge::MessagePair config_msgs;

    std::cout << "Configuring hbridge " << (hbridge_id+1) << std::endl;
    config_msgs = hbd.setConfiguration(hbridge_id, config);
    driver->write(config_msgs.first);
    driver->write(config_msgs.second);

    //wait two ms, so that HB can process config
    usleep(2000);
    std::cout << "Checking if hbridge cleared errors" << std::endl;
    int i;
    for(i = 0; i < 50; i++) {
        usleep(10000);

    	while(driver->getPendingMessagesCount() > 0) {
	    msg = driver->read() ;
	    hbd.updateFromCAN(msg);
	}
	
	hbridge::BoardState state = hbd.getState(hbridge_id);
	if(!checkPrintError(state.error))
	    break;
    }
    BOOST_CHECK(i < 50);

}


BOOST_AUTO_TEST_CASE(internal_encoder_test) {
    std::cout << "Internal Encoder Test" << std::endl;
    initDriver();
    hbridge::Configuration config = getDefaultConfig();
    //set infinite timeout
    config.timeout = 0;
    hbridge::MessagePair config_msgs;
    config_msgs = hbd.setConfiguration(hbridge_id, config);
    driver->write(config_msgs.first);
    driver->write(config_msgs.second);
    can::Message msg;

    
    
    while(true) {	
	usleep(10000);
	bool gotmsg = false;
    	while(driver->getPendingMessagesCount() > 0) {
	    msg = driver->read() ;
	    hbd.updateFromCAN(msg);
	    gotmsg = true;
	}
	if(gotmsg) {
	    hbridge::BoardState state = hbd.getState(hbridge_id);
	    checkPrintError(state.error);
	    std::cout << "\r Encoder Pos is " << state.position << "    Externen Enc is " << state.positionExtern << "          ";
	}
    }

}

BOOST_AUTO_TEST_CASE(timeout_test) {
    std::cout << "Testing timeout" << std::endl;
    initDriver();
    hbridge::Configuration config = getDefaultConfig();
    //set 50ms timeout
    config.timeout = 50;
    hbridge::MessagePair config_msgs;
    config_msgs = hbd.setConfiguration(hbridge_id, config);
    driver->write(config_msgs.first);
    driver->write(config_msgs.second);
    
    base::Time startTime = base::Time::now();
    
    can::Message msg;    
    
    int msgcnt = 0;
    
    while(true) {	
	usleep(1000);
    	while(driver->getPendingMessagesCount() > 0) {
	    msg = driver->read() ;
	    hbd.updateFromCAN(msg);
	    msgcnt++;
	    
	    if(hbd.getState(hbridge_id).error.timeout) {
		break;
	    }
	}
	
	if(msgcnt > 80)
	    break;
    }
    
    BOOST_CHECK(abs((startTime - base::Time::now()).toMicroseconds()) < 10);
    BOOST_CHECK(abs(msgcnt - 50) < 5);
    

}



BOOST_AUTO_TEST_CASE(test_case)
{
    ::boost::execution_monitor ex_mon;

    initDriver();

    // Still needs configuration (where are the config values stored?)
    hbridge::Configuration config = getDefaultConfig();
    hbridge::MessagePair config_msgs;
    can::Message msg;
    
    std::cout << "Configuring hbridge " << (hbridge_id+1) << std::endl;
    config_msgs = hbd.setConfiguration(hbridge_id, config);
    driver->write(config_msgs.first);
    driver->write(config_msgs.second);

    //wait two ms, so that HB can process config
    usleep(2000);
    std::cout << "Checking if hbridge cleared errors" << std::endl;
    int i;
    for(i = 0; i < 500; i++) {
        usleep(10000);

    	while(driver->getPendingMessagesCount() > 0) {
	    msg = driver->read() ;
	    hbd.updateFromCAN(msg);
	}
	
	hbridge::BoardState state = hbd.getState(hbridge_id);
	if(!checkPrintError(state.error))
	    break;
    }
    BOOST_CHECK(i < 500);
    
    std::cout << "Set drive modes" << std::endl;
    msg = hbd.setDriveMode(hbridge::DM_PWM);
    driver->write(msg);

    while(driver->getPendingMessagesCount() > 0) {
	msg = driver->read() ;	
	hbd.updateFromCAN(msg);
    }
    
    std::cout << "Testing encoders and current carrying electronics"
	      << std::endl;
    hbridge::Ticks initial_position = hbd.getState(hbridge_id).position;
    std::cout << "Rotate wheel "<<(hbridge_id+1)<<" for 1/2 turn(forwards)"
	      << std::endl;
    for (i = 0; i < 500; i++)
    {
    	can::Message msg = hbd.setTargetValues(200, 0, 0, 0);
        driver->write(msg);
        usleep(10000);

	while(driver->getPendingMessagesCount() > 0) {
	  msg = driver->read();
	  hbd.updateFromCAN(msg);
	}

	
	hbridge::BoardState state = hbd.getState(hbridge_id);
	checkPrintError(state.error);

	if(state.position - initial_position > hbridge::TICKS_PER_TURN/2 )
	    break;
    }
    BOOST_CHECK(i < 500);

    initial_position = hbd.getState(hbridge_id).position;
    std::cout << "Rotate wheel "<<(hbridge_id+1)<<" for 1/2 turn(backwards)" << std::endl;
    for (i = 0; i < 500; i++)
    {
    	can::Message msg = hbd.setTargetValues(-200, 0, 0, 0);
        driver->write(msg);
        usleep(10000);

	while(driver->getPendingMessagesCount() > 0) {
	  msg = driver->read() ;
	
	  hbd.updateFromCAN(msg);
	}

	hbridge::BoardState state = hbd.getState(hbridge_id);
	checkPrintError(state.error);
	if(state.position - initial_position < -hbridge::TICKS_PER_TURN/2 )
	    break;
    }
    BOOST_CHECK(i < 500);

    std::cout << "Testing overcurrent protection" << std::endl;
    std::cout << "Please block wheel and press return" << std::endl;
    std::cout << "(The wheel will go forwards then backwards)" << std::endl;
    std::string dummy;
    std::getline(std::cin,dummy);

    while(driver->getPendingMessagesCount() > 0) {//flush messages
      msg = driver->read() ;
      
      hbd.updateFromCAN(msg);
    }

    config.maxCurrent = 500;
    config.maxCurrentCount = 250;
    config_msgs = hbd.setConfiguration(hbridge_id, config);
    driver->write(config_msgs.first);
    driver->write(config_msgs.second);

    std::cout << "Set PID configuration" << std::endl;
    msg = hbd.setSpeedPID(hbridge_id, 400.0, 5.0, 0.0, 1800.0);
    driver->write(msg);


    
    msg = hbd.setDriveMode(hbridge::DM_SPEED);
    driver->write(msg);

    for (i = 0; i < 500; i++)
    {
    	can::Message msg = hbd.setTargetValues(20, 0, 0, 0);
        driver->write(msg);
        usleep(10000);

	while(driver->getPendingMessagesCount() > 0) {
	  msg = driver->read() ;
	
	  hbd.updateFromCAN(msg);
	}

	hbridge::BoardState state = hbd.getState(hbridge_id);
	checkPrintError(state.error);
	if (state.error.overCurrent )
	    break;
    }
    BOOST_CHECK(i < 500);

    config_msgs = hbd.setConfiguration(hbridge_id, config);
    driver->write(config_msgs.first);
    driver->write(config_msgs.second);

    for (i = 0; i < 500; i++)
    {
    	can::Message msg = hbd.setTargetValues(-20, 0, 0, 0);
        driver->write(msg);
        usleep(10000);

	while(driver->getPendingMessagesCount() > 0) {
	  msg = driver->read() ;
	
	  hbd.updateFromCAN(msg);
	}

	hbridge::BoardState state = hbd.getState(hbridge_id);
	checkPrintError(state.error);
	if (state.error.overCurrent )
	    break;
    }
    BOOST_CHECK(i < 500);

    can::Message reset = hbd.emergencyShutdown();
    driver->write(reset);
    delete driver;
};

// EOF
