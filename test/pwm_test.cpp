#include <iostream>
#include <strings.h>
#include <stdlib.h>
#include <time.h>

#include <arpa/inet.h>

#include <canbus.hh>
#include "../HBridgeDriver.hpp"
#include "../protocol.hpp"
#include "../HBridge.hpp"

can::Driver *driver = 0;
hbridge::Driver hbd;
int hbridge_id;

const unsigned int ticksPerTurnIntern = 512 * 729 / 16;
const unsigned int ticksPerTurnExtern = 4096;

void initDriver() {
    if(driver)
      return;

    const char *can_device = "can0";

    std::cerr << "Trying to open CAN device " << can_device << std::endl;
#if CANBUS_VERSION >= 101
    driver = can::openCanDevice(can_device);
#else
    driver = new can::Driver();
    driver->open(can_device);
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
    config.maxCurrent = 10000;
    config.maxCurrentCount = 250;
    config.pwmStepPerMs = 200;

    return config;
}


int main(int argc, char *argv[]) {


    timeval start, tick;
    gettimeofday(&start, 0);

    int pwm_in = 0;
    int pwm = 0;

    if (argc == 3) {
	pwm_in = strtol(argv[2],NULL,10);
	hbridge_id = strtol(argv[1],NULL,10);

	if (pwm_in > -1800 && pwm_in < 1800) {    
	    pwm = pwm_in;
	    std::cerr << "Set static PWM value to " << pwm << std::endl;
	}
	else {
	std::cerr << "Value must be between -1800 and 1800" << pwm << std::endl;
	exit(0);
	}
    } else {
      std::cerr << "usage: ./pwm_test <id> <pwm>" << std::endl;
	exit(0);
    }

    initDriver();
    can::Message msg;

    std::cerr << "Configuring Encoders " << (hbridge_id +1) << std::endl;
    hbridge::EncoderConfiguration encConfInt(ticksPerTurnIntern * 4, 4, hbridge::ENCODER_QUADRATURE);
    hbridge::EncoderConfiguration encConfExt(ticksPerTurnExtern, 1, hbridge::ENCODER_QUADRATURE_WITH_ZERO);
    
    can::Message encConfMsg = hbd.setInternalEncoderConfiguration(hbridge_id, encConfInt);
    driver->write(encConfMsg);

    encConfMsg = hbd.setExternalEncoderConfiguration(hbridge_id, encConfExt);
    driver->write(encConfMsg);    

    //now reconfigure
    hbridge::Configuration config = getDefaultConfig();
    hbridge::MessagePair config_msgs;

    std::cerr << "Configuring hbridge " << (hbridge_id+1) << std::endl;
    config_msgs = hbd.setConfiguration(hbridge_id, config);
    driver->write(config_msgs.first);
    driver->write(config_msgs.second);

    std::cerr << "Set drive mode " << std::endl;
    msg = hbd.setDriveMode(hbridge::BOARDS_14, hbridge::DM_PWM);
    driver->write(msg);
    
    int msg_cnt = 0;
    unsigned int average = 0;
    unsigned int average_cnt = 0;

    std::cerr << "Start hbridge... " << std::endl;

      while(1) {
        can::Message msg = hbd.setTargetValues(hbridge::BOARDS_14, pwm, pwm, pwm, pwm);

        driver->write(msg);
        //usleep(10000); // 10ms
        usleep(1000); // 1ms

        bool gotmsg = false;
        while(driver->getPendingMessagesCount() > 0) {
          msg = driver->read() ;
          hbd.updateFromCAN(msg);
          gotmsg = true;
        }

        if(gotmsg) {
            msg_cnt++;
            hbridge::BoardState state = hbd.getState(hbridge_id);
            gettimeofday(&tick, 0);
            long diff = (tick.tv_sec*1000000 + tick.tv_usec) - (start.tv_sec*1000000 + start.tv_usec);
            std::cout << diff/1000 << " " << state.current << std::endl;
            average += state.current;
            average_cnt++;
            if (msg_cnt == 20) {
              //std::cout  << diff/1000 << " " << (average/average_cnt) << std::endl;
              average = 0;
              average_cnt = 0;
              msg_cnt = 0;
           }
           
        }
     }
}

