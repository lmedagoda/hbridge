#include <boost/assert.hpp>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <iostream>
#include <vector>
#include <signal.h>

extern "C" {
#include "drivers/usart.h"
}
using namespace boost::interprocess;

boost::interprocess::message_queue *toRobot;
boost::interprocess::message_queue *fromRobot;

void exit_handler(int sig)
{
    if(!message_queue::remove("from_robot_queue"))
	std::cout << "Warning, could not remove from_robot_queue" << std::endl;
    if(!message_queue::remove("to_robot_queue"))
	std::cout << "Warning, could not remove to_robot_queue" << std::endl;
    
    exit(sig);
}

void USART1_Init(enum USART_MODE mode)
{
    signal(SIGINT, exit_handler);
    fromRobot = new message_queue(
	open_or_create            //only create
	,"from_robot_queue"          //name
	,1024                       //max message number
	,sizeof(char)              //max message size
	);	

    toRobot = NULL;
    int cnt = 0;
    std::cout << "Waiting for pc connection" << std::endl;
    while(!toRobot)
    {
	try {
	    toRobot = new message_queue(open_only ,"to_robot_queue");
	} catch (interprocess_exception &ex) {
	    usleep(10000);
	    cnt++;
	    if(cnt > 100)
	    {
		cnt = 0;
		std::cout << "Waiting for pc connection" << std::endl;
	    }
	}
    }
    std::cout << "Connected" << std::endl;
}

void USART1_DeInit(void)
{
    delete toRobot;
    delete fromRobot;
};

signed int USART1_SendData(const unsigned char *data, const unsigned int size)
{
    unsigned int nrToSend = rand() % (size + 1);
    assert(nrToSend <= size);
    
    try {
	for(unsigned int i = 0; i < nrToSend; i++)
	{
	    if(!fromRobot->try_send(data + i, sizeof(char), 0))
	    {
		std::cout << "Send failed at " << i << std::endl;
		return i;
	    }
	}
    } catch (interprocess_exception &ex) {
	std::cout << "send" << ex.what() << std::endl;
	return -1;
    }
    
    for(int i = 0; i < nrToSend; i++)
	std::cout << data[i] << std::flush;
    
    return nrToSend;
}

signed int USART1_GetData (unsigned char *buffer, const unsigned int buffer_length)
{
    unsigned int nrReveived = rand() % (buffer_length + 1);
    assert(nrReveived <= buffer_length);
    
    unsigned int priority;
    size_t received = 0;
    try {
	if(!toRobot->try_receive(buffer, sizeof(char), received, priority))
	{
	    return 0;
	}
    } catch (interprocess_exception &ex) {
	std::cout << "recv " << ex.what() << std::endl;
	return -1;
    }

//     std::cout << "got " << received << " bytes wanted " << buffer_length << std::endl; 
    return received;
}


void USART3_Init(enum USART_MODE mode)
{
    return;
    signal(SIGINT, exit_handler);
    fromRobot = new message_queue(
	open_or_create            //only create
	,"from_robot_queue"          //name
	,1024                       //max message number
	,sizeof(char)              //max message size
	);	

    toRobot = NULL;
    int cnt = 0;
    std::cout << "Waiting for pc connection" << std::endl;
    while(!toRobot)
    {
	try {
	    toRobot = new message_queue(open_only ,"to_robot_queue");
	} catch (interprocess_exception &ex) {
	    usleep(10000);
	    cnt++;
	    if(cnt > 100)
	    {
		cnt = 0;
		std::cout << "Waiting for pc connection" << std::endl;
	    }
	}
    }
    std::cout << "Connected" << std::endl;
}

void USART3_DeInit(void)
{
    delete toRobot;
    delete fromRobot;
};

signed int USART3_SendData(const unsigned char *data, const unsigned int size)
{
    return size;
    unsigned int nrToSend = rand() % (size + 1);
    assert(nrToSend <= size);
    
    try {
	for(unsigned int i = 0; i < nrToSend; i++)
	{
	    if(!fromRobot->try_send(data + i, sizeof(char), 0))
	    {
		std::cout << "Send failed at " << i << std::endl;
		return i;
	    }
	}
    } catch (interprocess_exception &ex) {
	std::cout << "send" << ex.what() << std::endl;
	return -1;
    }
    
    for(int i = 0; i < nrToSend; i++)
	std::cout << data[i] << std::flush;
    
    return nrToSend;
}

signed int USART3_GetData (unsigned char *buffer, const unsigned int buffer_length)
{
    return 0;
    unsigned int nrReveived = rand() % (buffer_length + 1);
    assert(nrReveived <= buffer_length);
    
    unsigned int priority;
    size_t received = 0;
    try {
	if(!toRobot->try_receive(buffer, sizeof(char), received, priority))
	{
	    return 0;
	}
    } catch (interprocess_exception &ex) {
	std::cout << "recv " << ex.what() << std::endl;
	return -1;
    }

//     std::cout << "got " << received << " bytes wanted " << buffer_length << std::endl; 
    return received;
}


