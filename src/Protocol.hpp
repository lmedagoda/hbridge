#ifndef PROTOCOLL_HPP
#define PROTOCOLL_HPP

#include <canmessage.hh>
#include "Reader.hpp"
#include "Writer.hpp"
#include "Controller.hpp"

#include <vector>
#include <queue>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <map>

namespace hbridge {
    
class CanbusInterface
{
public:
    virtual bool sendCanPacket(const canbus::Message &packet) = 0;
    virtual bool readCanPacket(canbus::Message &packet) = 0;
};
    
class SendQueue
{
public:
    class Entry
    {
    public:
	base::Time sendTime;
	canbus::Message msg;
	int retryCnt;
	boost::function<void (const canbus::Message &)> errorCallback;
	boost::function<void (void)> ackedCallback;
    };
    
    std::queue<Entry> queue;
};

class Protocol
{
    friend class Controller;
    friend class Reader;
    friend class Writer;
public:
    class HbridgeHandle
    {
	friend class Protocol;
    private:
	HbridgeHandle(int id, Protocol *protocol);
	SendQueue queue;
	Reader * reader;
	Writer * writer;
	std::vector<Controller *> controllers;
    public:
	Reader *getReader()
	{
	    return reader;
	};
	
	Writer *getWriter()
	{
	    return writer;
	};
    };
private:
    static Protocol *instance;
    Protocol();
    
    int retryCount;
    base::Time sendTimout;

    std::vector<HbridgeHandle *> handles;
    std::vector<std::vector<Controller *> > controllers;
    std::vector<canbus::Message> sharedCanMessages;
    std::map<unsigned int, bool> markedForSending;
    CanbusInterface *canbus;
    
    int getBoardIdFromMessage(const canbus::Message& msg) const;
    bool isInProtocol(const canbus::Message& msg) const;

    void sendCanPacket(int boardId, const canbus::Message &packet, bool isAcked, boost::function<void (const canbus::Message &)> errorCallback, boost::function<void (void)> ackedCallback = NULL);
    std::vector<Controller *> &getControllers(int id);
    canbus::Message &getSharedMsg(unsigned int canbusId);
    void registerHbridge(int id);
        
public:

    /**
     * This interface needs to be implemented by any
     * canbus driver which should be used with this 
     * hbridge driver.
     * */
    void setCanbusInterface(CanbusInterface *canbus);
    
    /**
     * This function creates and returns a handle
     * to a hbridge. With this handle the hbridge
     * can be accessed.
     * */
    HbridgeHandle *getHbridgeHandle(int id);

    /**
     * This registeres a controller on the driver side. 
     * Note, this method may only be called after all
     * handled for all used hbridges were requested.
     * */
    void registerController(unsigned int controllerId, const hbridge::Controller &ctrl);

    /**
     * Return the Protocol singleton
     * */
    static Protocol *getInstance();
    
    /**
     * Sends out the messages that are shared between multiple
     * hbridges. E.g. setMode and setTargetValue
     * */
    void sendSharedMessages();    
    
    /**
     * Processes all incommin messages. 
     * This function will handle the acking of messages and
     * demultiplexes the messages to the Reader and
     * Controller instances:
     * */
    void processIncommingPackages();
    
    /**
     * Processes the send queues. 
     * This includes resending of noch acked 
     * packets and timeout checks.
     * */
    void processSendQueues();
};

}
#endif // PROTOCOLL_HPP
