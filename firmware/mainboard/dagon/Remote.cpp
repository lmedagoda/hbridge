#include "/usr/include/assert.h"
#include "remote_msg.h"
#include <base/Timeout.hpp>
#include <iodrivers_base/Driver.hpp>
#include <motor_maxon/Driver.hpp>
#include <base/Timeout.hpp>
#include <iostream>



class Processor : public iodrivers_base::Driver{
    public:
    Processor():
        iodrivers_base::Driver(1024,true),
        timeout(base::Time::fromSeconds(1.0))
    {
        std::cout << "Try to open TCP.." << std::endl;
        //openTCP("192.168.128.20",5001);
        openTCP("127.0.0.1",5001);
        if(!isValid()){
            std::cerr << "Cannot open remote port" << std::endl;
            assert(false);
        }
	//writePacket((uint8_t*)"Hallo",5);
    }

    void reOpen(){
        close();
	openTCP("192.168.128.20",5001);
	//writePacket((uint8_t*)"Hallo",5);
    }

    remote_msg last_msg;
    motor_maxon::Driver motor_driver[5];
    base::Timeout timeout;
    base::Timeout reconnect_timeout;

    bool process(){
	if(reconnect_timeout.elapsed()){
        	if(!isValid()){
        		openUDP("192.168.128.20",4022);
		}
		//writePacket((uint8_t*)"Hallo",5);
		reconnect_timeout.restart();
	}
        uint8_t data[1024];
        if(readPacket(data,1024,base::Time::fromSeconds(1)) ==  sizeof(remote_msg)+2){
            remote_msg *msg = (remote_msg*)(data+1);
            last_msg = *msg;
            timeout.restart();
            return true;
        }
        return false;
    }
    
    void setMotor(int id, double value){
        assert(id == 0);

        motor_maxon::Command cmd;
        cmd.type = motor_maxon::CONTROL;
        if(value >= 0){
		cmd.direction = motor_maxon::CLOCKWISE;
        } else{
		cmd.direction = motor_maxon::COUNTER_CLOCKWISE;
        }
	cmd.setpoint = fabs(value);
        if(motor_driver[id].isValid()){
            motor_driver[id].write(cmd, base::Time().fromSeconds(2));
        }else{
            std::cerr << "Warning motor id " << id << " is invalid" << std::endl;
        }
    }

    void control_system(){
        int id=0;
        if(!motor_driver[id].isValid()){
            system("killall -9 dagon_motors_left");
            int fd = motor_driver[id].openSerialIO("/dev/ttyS2", 115200);
            if(fd >0){
                motor_driver[id].setFileDescriptor(fd);
            }
        }
        if(motor_driver[id].isValid()){
            if(timeout.elapsed()){
                std::cerr << "Attention timeout " << std::endl;
                setMotor(id,0);
            }else{
                setMotor(id,last_msg.target);
            }
        }else{
		printf("Cannot access motor\n");
	}
    }


    void release_control(){
        int id=0;
        if(motor_driver[id].isValid()){
            motor_driver[id].close();
        }
    }
    
    virtual int extractPacket(uint8_t const* buffer, size_t buffer_size) const{
        if(buffer[0] != '@'){
            return -1;
        }
        if(buffer_size < (sizeof(remote_msg)+2)){
                return 0;
        }
        if(buffer[sizeof(remote_msg)+2] != '\n'){
            return -1;
        }
        return sizeof(remote_msg)+2;
    }
};


class Modem : public iodrivers_base::Driver{
    public:    
    Modem():
        iodrivers_base::Driver(1024,true)
    {
        openSerialIO("/dev/ttyS4", 57600);
        if(!isValid()){
            std::cerr << "Cannot open remote port" << std::endl;
            assert(false);
        }
    }

    virtual int extractPacket(uint8_t const* buffer, size_t buffer_size) const{
        return buffer_size;
    }
};



int main(int argc, char **argv){
    std::cout << "Starting ..." << std::endl;
    Processor p;
    std::cout << "TCP Connection established.. " << std::endl;
    Modem modem;
    base::Timeout timeout(base::Time::fromSeconds(1.0));
    while(1){
        try{
            //right to modem
            if ( p.process()){
                modem.writePacket(p.last_msg.modemdata, p.last_msg.modemdata_size);
            }
            //modem to right
            try{
                uint8_t data[1024];
                if(uint8_t size = modem.readPacket(data,1024,0)){
                    std::cout << "Send to right " << size << "Bytes Modemdata" << std::endl;
                    //write without a struct until yet
                    p.writePacket(data, size);
                }
            } catch (iodrivers_base::TimeoutError e){
                //There is no modem it's not so important
                std::cerr << "There is no Modem" << std::endl;
            } 

            switch(p.last_msg.state){
                case 0:
                case 1:
                    {
                        if(timeout.elapsed()){
                            system("killall -9 dagon_motors_left");
                            timeout.restart();
                        }
                        break;
                    }
                case 2:
                    {
                        p.control_system();
                        break;
                    }
                case 3:
                    p.release_control();
                    break;
                    //Do Nothing here
            }
        }catch(...){
            printf("Something went wrong here\n");
            p.reOpen();
        }
        std::cout << "." <<std::flush;
        sleep(0.5);
    }
    return 0;
}
