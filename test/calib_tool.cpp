#include <iostream>
#include <strings.h>
#include <stdlib.h>
#include <time.h>

#include <arpa/inet.h>

#include <canbus.hh>
#include "../HBridgeDriver.hpp"
#include "../protocol.hpp"
#include "../HBridge.hpp"

canbus::Driver *driver = 0;
hbridge::Driver hbd;
int hbridge_id;

const unsigned int ticksPerTurnIntern = 512 * 729 / 16;
const unsigned int ticksPerTurnExtern = 4096;

hbridge::Configuration getDefaultConfig() {
    hbridge::Configuration config;
    bzero(&config, sizeof(hbridge::Configuration));

    config.openCircuit = 1;
    config.maxMotorTemp = 60;
    config.maxMotorTempCount = 200;
    config.maxBoardTemp = 60;
    config.maxBoardTempCount = 200;
    config.timeout = 50;
    config.maxCurrent = 10000;
    config.maxCurrentCount = 250;
    config.pwmStepPerMs = 200;

    return config;
}


int main(int argc, char *argv[]) {


    if (argc != 3) 
    {  
        std::cerr << "usage: ./pwm_test can_device can_device_type" << std::endl;
	exit(1);
    }

    driver = canbus::openCanDevice(argv[1], argv[2]);
    if(!driver)
    {
	std::cout << "failed to open canbus" << std::endl;
	exit(1);
    }
    canbus::Message msg;

    for(int i = 0; i < 4; i++)
    {
	hbridge_id = i;
	std::cerr << "Configuring Encoders " << (hbridge_id +1) << std::endl;
	hbridge::EncoderConfiguration encConfInt;
	hbridge::EncoderConfiguration encConfExt;
	encConfInt.tickDivider = 4;
	encConfInt.ticksPerTurn = ticksPerTurnIntern * 4;
	
	encConfExt.tickDivider = 1;
	encConfExt.ticksPerTurn = ticksPerTurnExtern;
	
	canbus::Message encConfMsg = hbd.setEncoderConfiguration(hbridge_id, encConfInt, encConfExt);
	driver->write(encConfMsg);
	
	//now reconfigure
	hbridge::Configuration config = getDefaultConfig();
	hbridge::MessagePair config_msgs;
	
	std::cerr << "Configuring hbridge " << (hbridge_id+1) << std::endl;
	config_msgs = hbd.setConfiguration(hbridge_id, config);
	driver->write(config_msgs.first);
	driver->write(config_msgs.second);
	
	std::cerr << "Set drive mode " << std::endl;
	msg = hbd.setDriveMode(hbridge::BOARDS_14, base::actuators::DM_PWM);
	driver->write(msg);
	
    }
    int msg_cnt = 0;
    int no_msg_cnt = 0;

    std::cerr << "Start hbridge... " << std::endl;

    while(1) 
      {
	canbus::Message msg = hbd.setTargetValues(hbridge::BOARDS_14, 0, 0, 0, 0);
	
	driver->write(msg);
	//usleep(10000); // 10ms
	usleep(1000); // 1ms
	
	bool gotmsg = false;
	while(driver->getPendingMessagesCount() > 0) 
	{
	    msg = driver->read() ;
	    hbd.updateFromCAN(msg);
	    gotmsg = true;
	    no_msg_cnt = 0;
	} 
	
	if(gotmsg) 
	{
	    msg_cnt++;
	    if (msg_cnt > 20) 
	    {
		for(int i = 0; i < 4; i++)
		{
		    hbridge::BoardState state = hbd.getState(i);
		    std::cout << "Board " << i << " Ext enc pos " << state.positionExtern << std::endl; 
		}
		exit(0);
	    }  
	} else {
	  no_msg_cnt++;
	}
	
	if(no_msg_cnt > 100)
	{
	    std::cout << "Error Hbridges did not respond " << std::endl;
	    exit(1);
	}
    }
}

