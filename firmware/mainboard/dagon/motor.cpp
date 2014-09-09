
extern void assert(int i);
extern int socked_left;

#include <sys/types.h>
#include <sys/socket.h>
#include "../mainboard/common/arc_packet.h"
#include <motor_maxon/Driver.hpp>
#include <iostream>
#include "../common/timeout.h"
#include "../common/mb_types.h"
#include "../common/mainboardstate.h"
#include "../avalon/avalon_types.h"
#include "remote_msg.h"
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/ioctl.h>

motor_maxon::Driver motor_driver[5];

void initMotor(unsigned int id, std::string port){
    if(!motor_driver[id].isValid()){
        int fd = motor_driver[id].openSerialIO(port, 115200);
        motor_driver[id].setFileDescriptor(fd);
    }
    
}
extern avalon_status_t current_status;
extern struct arc_avalon_control_packet curCmd;
extern uint8_t cmdValid;
extern struct sockaddr_in si_me_left, si_other_left;

void initMotors(){
    initMotor(0,"/dev/ttyS1"); //dive_tail
    initMotor(1,"/dev/ttyS4"); //dive_head
    initMotor(2,"/dev/ttyS5"); //surge_right
    initMotor(3,"/dev/ttyS2"); //sway_tail
}

void closeMotors(){
    for(int id=0;id<4;id++){
        if(motor_driver[id].isValid()){
            motor_driver[id].close();
        }
    }
}

void setMotor(int id, double value){
    motor_maxon::Command cmd;
    cmd.type = motor_maxon::CONTROL;
    if(value >= 0){ 
        cmd.direction = motor_maxon::CLOCKWISE;
    } else{
        cmd.direction = motor_maxon::COUNTER_CLOCKWISE;
    }
    cmd.setpoint = fabs(value);
    //cmd.setpoint = 0;
    if(motor_driver[id].isValid()){
        motor_driver[id].write(cmd, base::Time().fromSeconds(2));
    }else{
        std::cerr << "Warning motor id " << id << " is invalid" << std::endl;
    }
}

void setMotor(double value[5]){
    //printf("Control motor: ");
    for(int i=0;i<4;i++){
        //printf("%+04f ", value[i]*100.0);
        setMotor(i,value[i]);
    }
    //printf("\n");
}


extern "C" {

void sendCommand(uint8_t state, double target){
    if(socked_left <= 0){
        printf("Warning conenction to left invalid\n");
        return;
    }
    remote_msg msg;
    msg.state = state;
    msg.target = target;
    uint8_t buffer[sizeof(msg)+2];
    buffer[0] = '@';
    buffer[sizeof(msg)+1] = '\n';
    memcpy(buffer+1,&msg,sizeof(msg));
    size_t written = sendto(socked_left, buffer, sizeof(msg)+2, 0, (struct sockaddr *)&si_other_left, sizeof(si_other_left));
    
    unsigned char b2[2048];
    unsigned int bytes_available;
    unsigned int slen=sizeof(si_other_left);
    ioctl(socked_left,FIONREAD,&bytes_available);
    if(bytes_available > 2048){
        bytes_available = 2048;
    }
    int read = 0;
    if(bytes_available > 0){
        read = recvfrom(socked_left, b2, bytes_available, 0, (struct sockaddr*)&si_other_left, &slen);
        if(read==-1){
            fprintf(stderr,"Read Failed\n");
            exit(-1);
        }
    }
}

void avalon_autonomousState(void){
    sendCommand(3,0);
    closeMotors();
}

void avalon_offState(void){
    sendCommand(0,0);
    closeMotors();
    //TODO kill here
    //timeout_reset();
}
void avalon_runningState(void){
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

    initMotors();
    double value[5];

//dive_tail
//dive_head
//surge_right
//sway_tail      
//left

    double factor = 2.5;
    value[0] = -(curCmd.dive-127)/255.0;
    value[1] = -(curCmd.dive-127)/255.0;
    value[2] = (0.7 * -(curCmd.right-127)/255.0) + (0.3 * (curCmd.strave-127)/255.0);
    value[3] = -((0.5 * (curCmd.yaw-127)/255.0) - (0.5 * (curCmd.strave-127)/255.0));
    value[4] = (0.7 * (curCmd.left-127)/255.0) - (0.3 * (curCmd.strave-127)/255.0);
    for(int i=0;i<5;i++){
        value[i]*=factor;
    }
    setMotor(value);
    sendCommand(2,value[4]);
    current_status.change_reason = CR_LEGAL;
};

};


