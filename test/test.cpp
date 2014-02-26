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

canbus::Driver *driver = 0;
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

const unsigned int ticksPerTurnIntern = 512 * 729 / 16;
const unsigned int ticksPerTurnExtern = 4096;

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
    if(error.encodersNotInitialized) {
	std::cout << "HBridge reported error:encodersNotInitialized" << std::endl;
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
        canbus::Message        &msg)
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
    BOOST_CHECK(driver = canbus::openCanDevice(can_device));
#else
    BOOST_CHECK(driver = new canbus::Driver());
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

    canbus::Message dmmsg = hbd.setDriveMode(hbridge::BOARDS_14, base::actuators::DM_SPEED);
//     BOOST_CHECK(checkMessage(0, firmware::PACKET_ID_SET_MODE14, dmData, dmDataSize, dmmsg));

    for (int i = 0; i < 4; ++i)
    {
        hbridge::MessagePair config_msgs = hbd.setConfiguration(i, config);
        BOOST_CHECK(checkMessage(i + 1, firmware::PACKET_ID_SET_CONFIGURE, config1Data, config1DataSize, config_msgs.first));
        BOOST_CHECK(checkMessage(i + 1, firmware::PACKET_ID_SET_CONFIGURE2, config2Data, config2DataSize, config_msgs.second));
    }

    for (int i = 0; i < 4; ++i)
    {
        canbus::Message pidmsg = hbd.setSpeedPID(i, 400.0, 5.0, 0.0, 1800.0);
        BOOST_CHECK(checkMessage(i + 1, firmware::PACKET_ID_SET_PID_SPEED, pidData, pidDataSize, pidmsg));
    }

//     canbus::Message msg = hbd.setTargetValues(hbridge::BOARDS_14, 20, 0, 0, 0);
//     BOOST_CHECK(checkMessage(0, firmware::PACKET_ID_SET_VALUE14, value1Data, valueDataSize, msg));



}

BOOST_AUTO_TEST_CASE(encoder_not_initalized) {
    initDriver();
    hbridge::Configuration config = getDefaultConfig();
    config.timeout = 0;
    hbridge::MessagePair config_msgs;
    canbus::Message msg;

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
	    
	    if(state.error.encodersNotInitialized) {
		std::cout << "Correct: HBridge reportet encoderNotInitalized Error " << std::endl;
		break;
	    }
    }    
    BOOST_CHECK(i < 50);

    std::cout << "Waiting a second" << std::endl;
    usleep(1000000);
}


BOOST_AUTO_TEST_CASE(configure_encoder_clear_Error) {
    initDriver();
    canbus::Message msg;

    std::cout << "Configuring Encoders " << (hbridge_id +1) << std::endl;
    hbridge::EncoderConfiguration encConfInt(ticksPerTurnIntern * 4, hbridge::ENCODER_QUADRATURE);
    hbridge::EncoderConfiguration encConfExt(ticksPerTurnExtern, hbridge::ENCODER_QUADRATURE_WITH_ZERO);
    
    canbus::Message encConfMsg = hbd.setInternalEncoderConfiguration(hbridge_id, encConfInt);
    driver->write(encConfMsg);

    encConfMsg = hbd.setExternalEncoderConfiguration(hbridge_id, encConfExt);
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

    std::cout << "Waiting a second" << std::endl;
    usleep(1000000);
}


BOOST_AUTO_TEST_CASE(internal_encoder_test) {
    std::cout << "Internal Encoder Test" << std::endl;
    initDriver();
    hbridge::Configuration config = getDefaultConfig();
    //set infinite timeout
    config.timeout = 50;
    hbridge::MessagePair config_msgs;
    config_msgs = hbd.setConfiguration(hbridge_id, config);
    driver->write(config_msgs.first);
    driver->write(config_msgs.second);
    canbus::Message msg;

    msg = hbd.setDriveMode(hbridge::BOARDS_14, base::actuators::DM_PWM);
    driver->write(msg);
    
    hbridge::Ticks posInternStart; 
    hbridge::Ticks posExternStart; 

    hbridge::BoardState state;
    
    //get inital position
    while(true) {	
    	canbus::Message msg = hbd.setTargetValues(hbridge::BOARDS_14, 0, 0, 0, 0);
        driver->write(msg);
	usleep(10000);
	bool gotmsg = false;
    	while(driver->getPendingMessagesCount() > 0) {
	    msg = driver->read() ;
	    hbd.updateFromCAN(msg);
	    gotmsg = true;
	}
	if(gotmsg) {
	    state = hbd.getState(hbridge_id);
	    checkPrintError(state.error);
	    posInternStart = state.position;
	    posExternStart = state.positionExtern;
	    break;
	}
    }

//     while(!checkPrintError(state.error)) {	
//     	usleep(10000);
// 	canbus::Message msg = hbd.setTargetValues(1800, 0, 0, 0);
//         driver->write(msg);
// 	bool gotmsg = false;
//     	while(driver->getPendingMessagesCount() > 0) {
// 	    msg = driver->read() ;
// 	    hbd.updateFromCAN(msg);
// 	    gotmsg = true;
// 	}
//     }


    int speed = 200;
    std::cout << "Rotate wheel "<<(hbridge_id+1)<<" for 1/2 turn(forward)" << std::endl;
    while(!checkPrintError(state.error)) {	
    	usleep(10000);
	canbus::Message msg = hbd.setTargetValues(hbridge::BOARDS_14, speed, 0, 0, 0);
        driver->write(msg);
	bool gotmsg = false;
    	while(driver->getPendingMessagesCount() > 0) {
	    msg = driver->read() ;
	    hbd.updateFromCAN(msg);
	    gotmsg = true;
	}
	
	if(gotmsg) {
	    hbridge::BoardState state = hbd.getState(hbridge_id);
	    std::cout << "\r Encoder Pos is " << state.position << "    Externen Enc is " << state.positionExtern << "          ";
	    if((uint) abs(state.position - posInternStart) > ticksPerTurnIntern / 2) {
		if(speed < 0)
		    break;
		posInternStart = state.position;
		speed = -200;
		std::cout << "Rotate wheel "<<(hbridge_id+1)<<" for 1/2 turn(backwards)" << std::endl;
	    }
	}
    }
    
    while(driver->getPendingMessagesCount() > 0) {
	msg = driver->read() ;
	hbd.updateFromCAN(msg);
    }
    posExternStart = state.positionExtern;
    
    std::cout << "Testing external encoder" << std::endl;
    speed = 200;
    std::cout << "Rotate wheel "<<(hbridge_id+1)<<" for 1/2 turn(forward)" << std::endl;
    while(!checkPrintError(state.error)) {	
    	usleep(10000);
	canbus::Message msg = hbd.setTargetValues(hbridge::BOARDS_14, speed, 0, 0, 0);
        driver->write(msg);
	bool gotmsg = false;
    	while(driver->getPendingMessagesCount() > 0) {
	    msg = driver->read() ;
	    hbd.updateFromCAN(msg);
	    gotmsg = true;
	}
	
	if(gotmsg) {
	    hbridge::BoardState state = hbd.getState(hbridge_id);
	    std::cout << "\r Encoder Pos is " << state.position << "    Externen Enc is " << state.positionExtern << "          ";
	    if((uint) abs(state.positionExtern - posExternStart) > ticksPerTurnExtern / 2) {
		if(speed < 0)
		    break;
		posExternStart = state.positionExtern;
		speed = -200;
		std::cout << "Rotate wheel "<<(hbridge_id+1)<<" for 1/2 turn(backwards)" << std::endl;
	    }
	}
    }
    
}


BOOST_AUTO_TEST_CASE(timeout_test) {
    std::cout << "Testing timeout of 50 ms" << std::endl;
    initDriver();
    hbridge::Configuration config = getDefaultConfig();
    //set 50ms timeout
    config.timeout = 50;
    
    hbridge::MessagePair config_msgs;
    config_msgs = hbd.setConfiguration(hbridge_id, config);
    driver->write(config_msgs.first);
    driver->write(config_msgs.second);
    canbus::Message msg;    
    usleep(1000);
    
    //flush messages
    while(driver->getPendingMessagesCount() > 0) {
	msg = driver->read() ;
    }
  
    base::Time startTime = base::Time::now();

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

	if(hbd.getState(hbridge_id).error.timeout) {
	    break;
	}
    }
    
    base::Time endTime = base::Time::now();
    
    std::cout << "Time until timeout was reported in ms " << (endTime - startTime).toMilliseconds() << std::endl;
    std::cout << "Received " << msgcnt << " messages until timeout was reported" << std::endl;
    BOOST_CHECK(abs((endTime - startTime).toMilliseconds() - 50) < 5);
    BOOST_CHECK(abs(msgcnt - 50) < 5);
}



BOOST_AUTO_TEST_CASE(test_case)
{
    initDriver();

    // Still needs configuration (where are the config values stored?)
    hbridge::Configuration config = getDefaultConfig();
    hbridge::MessagePair config_msgs;
    canbus::Message msg;
    
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
    

    std::cout << "Testing overcurrent protection" << std::endl;
    std::cout << "Please block wheel and press return" << std::endl;
    std::cout << "(The wheel will go forwards then backwards)" << std::endl;
    std::string dummy;
    std::getline(std::cin,dummy);

    config.maxCurrent = 500;
    config.maxCurrentCount = 250;
    config_msgs = hbd.setConfiguration(hbridge_id, config);
    driver->write(config_msgs.first);
    driver->write(config_msgs.second);

    std::cout << "Set PID configuration" << std::endl;
    msg = hbd.setSpeedPID(hbridge_id, 400.0, 5.0, 0.0, 500.0);
    driver->write(msg);
    
    msg = hbd.setDriveMode(hbridge::BOARDS_14, base::actuators::DM_SPEED);
    driver->write(msg);

    //wait for hbridge to process messages
    usleep(5000);

    //flush messages
    while(driver->getPendingMessagesCount() > 0) {
      msg = driver->read() ; 
      hbd.updateFromCAN(msg);
    }
    
    hbridge::BoardState state = hbd.getState(hbridge_id);
    while(!checkPrintError(state.error))
    {
    	canbus::Message msg = hbd.setTargetValues(hbridge::BOARDS_14, 20, 0, 0, 0);
        driver->write(msg);
        usleep(5000);

	while(driver->getPendingMessagesCount() > 0) {
	  msg = driver->read() ;
	
	  hbd.updateFromCAN(msg);
	}

	state = hbd.getState(hbridge_id);
    }
    BOOST_CHECK(state.error.overCurrent);

    config_msgs = hbd.setConfiguration(hbridge_id, config);
    driver->write(config_msgs.first);
    driver->write(config_msgs.second);

    //wait for hbridge to process messages
    usleep(5000);

    //flush messages
    while(driver->getPendingMessagesCount() > 0) {
      msg = driver->read() ; 
      hbd.updateFromCAN(msg);
    }

    for (i = 0; i < 500; i++)
    {
    	canbus::Message msg = hbd.setTargetValues(hbridge::BOARDS_14, -20, 0, 0, 0);
        driver->write(msg);
        usleep(10000);

	while(driver->getPendingMessagesCount() > 0) {
	  msg = driver->read() ;
	
	  hbd.updateFromCAN(msg);
	}

	state = hbd.getState(hbridge_id);
	checkPrintError(state.error);
	if (state.error.overCurrent )
	    break;
    }
    BOOST_CHECK(i < 500);

    canbus::Message reset = hbd.emergencyShutdown();
    driver->write(reset);
    delete driver;
};

// EOF
