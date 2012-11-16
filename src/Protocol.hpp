#ifndef PROTOCOLL_HPP
#define PROTOCOLL_HPP

#include <canmessage.hh>

#include <vector>
#include <queue>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <map>

namespace hbridge {
    
class Controller;
class Protocol;
class Reader;
class Writer;
    
class Packet
{
public:
    int senderId;
    int receiverId;
    int packetId;
    bool broadcastMsg;
    std::vector<uint8_t> data;
};

class BusInterface
{
public:
    virtual bool sendPacket(const Packet &packet) = 0;
    virtual bool readPacket(Packet &packet) = 0;
};

class SendQueue
{
public:
    class Entry
    {
    public:
	base::Time sendTime;
	Packet msg;
	int retryCnt;
	boost::function<void (const Packet &msg)> errorCallback;
	boost::function<void (void)> ackedCallback;
    };
    
    std::queue<Entry> queue;
};

class PacketReveiver
{
public:
    /**
     * Callback function get's called whenever there
     * is a new can message received.
     * */
    virtual void processMsg(const Packet &msg) = 0;
};

class HbridgeHandle
{
    friend class Protocol;
    friend class Writer;
    friend class Reader;
    friend class Controller;
private:
    HbridgeHandle(int id, Protocol *protocol);
    int boardId;
    SendQueue queue;
    Reader * reader;
    Writer * writer;
    Protocol *protocol;
    
    std::vector<PacketReveiver *> msgHandlers;    
    std::vector<Controller *> controllers;
    std::vector<Controller *> &getControllers()
    {
	return controllers;
    };
    
    void registerForMsg(PacketReveiver *reveiver, int packetId);
    void unregisterForMsg(PacketReveiver *reveiver);

    /**
    * 
    * */
    void registerController(hbridge::Controller *ctrl);
public:


    int getBoardId()
    {
	return boardId;
    }
    
    Reader *getReader()
    {
	return reader;
    };
    
    Writer *getWriter()
    {
	return writer;
    };
    
    Protocol *getProtocol() 
    {
	return protocol;
    }

};

class Protocol
{
    friend class Controller;
    friend class Reader;
    friend class Writer;
private:
    static Protocol *instance;
    Protocol();
    
    int retryCount;
    base::Time sendTimout;

    std::vector<HbridgeHandle *> handles;
    std::vector<Packet> sharedMessages;
    std::map<unsigned int, bool> markedForSending;
    BusInterface *bus;
    
    int getBoardIdFromMessage(const canbus::Message& msg) const;
    bool isInProtocol(const canbus::Message& msg) const;

    void sendPacket(int boardId, const Packet &msg, bool isAcked, boost::function<void (const Packet &msg)> errorCallback, boost::function<void (void)> ackedCallback = NULL);
    Packet &getSharedMsg(unsigned int packetId);
    void registerHbridge(int id);
        
public:

    /**
     * This interface needs to be implemented by any
     * canbus driver which should be used with this 
     * hbridge driver.
     * */
    void setBusInterface(BusInterface *bus);
    
    /**
     * This function creates and returns a handle
     * to a hbridge. With this handle the hbridge
     * can be accessed.
     * */
    HbridgeHandle *getHbridgeHandle(int id);

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
