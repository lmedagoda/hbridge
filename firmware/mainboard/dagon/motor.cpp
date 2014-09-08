
extern void assert(int i);

#include "../mainboard/common/arc_packet.h"
#include <motor_maxon/Driver.hpp>
#include <iostream>
#include "../common/timeout.h"
#include "../common/mb_types.h"
#include "../common/mainboardstate.h"
#include "../avalon/avalon_types.h"

motor_maxon::Driver motor_driver[5];

void initMotor(unsigned int id, std::string port){
    int fd = motor_driver[id].openSerialIO(port, 115200);
    motor_driver[id].setFileDescriptor(fd);
    
}
extern avalon_status_t current_status;
extern struct arc_avalon_control_packet curCmd;
extern uint8_t cmdValid;

void initMotors(){
    initMotor(0,"/dev/ttyS1");
    initMotor(1,"/dev/ttyS2");
    initMotor(2,"/dev/ttyS3");
    initMotor(3,"/dev/ttyS4");
}

void setMotor(int id, double value){
    motor_maxon::Command cmd;
    cmd.type = motor_maxon::CONTROL;
    cmd.direction = motor_maxon::CLOCKWISE;
    //cmd.direction = motor_maxon::COUNTER_CLOCKWISE;
    cmd.setpoint = value;
    if(motor_driver[id].isValid()){
        motor_driver[id].write(cmd, base::Time().fromSeconds(2));
    }else{
        std::cerr << "Warning motor id " << id << " is invalid" << std::endl;
    }
}

void setMotor(double value[5]){
    for(int i=0;i<5;i++){
        setMotor(i,value[i]);
    }
}


extern "C" {

void dagon_offState(void){
    //TODO kill here
    //timeout_reset();
}
void dagon_runningState(void){
    /*
    if(timeout_hasTimeout())
    {
        printf("Timout, switching to off\n");
        mbstate_changeState(MAINBOARD_OFF);
        current_status.change_reason = CR_MB_TIMEOUT;
        timeout_reset();
        double value[5];
        for(int i=0;i<5;i++)value[i] = 0;
        setMotor(value);
        return;
    }
    */

    //check actuators
    /*
    if(hbridge_hasActuatorError())
    {
        printf("Actuator error, switching to off\n");
        mbstate_changeState(MAINBOARD_OFF);
        current_status.change_reason = CR_HB_ERROR;
        double value[5];
        for(int i=0;i<5;i++)value[i] = 0;
        setMotor(value);
        return;
    }
    */

    if (!cmdValid)
        return;


    double value[5];
    value[0] = (curCmd.strave-127)/255.0;
    value[1] = (curCmd.dive-127)/255.0;
    value[2] = (curCmd.left-127)/255.0;
    value[3] = (curCmd.right-127)/255.0;
    value[4] = (curCmd.yaw-127)/255.0;
    setMotor(value);

    current_status.change_reason = CR_LEGAL;
    
};

};


