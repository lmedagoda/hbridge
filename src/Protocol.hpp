#ifndef PROTOCOLL_HPP
#define PROTOCOLL_HPP

#include <vector>
#include <queue>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <map>
#include <base/time.h>

namespace firmware
{
    extern "C" {
#include "firmware/common/packets.h"
    }
}

namespace hbridge {
    
class Controller;
class Protocol;
class Reader;
class Writer;
    
class Packet
{
public:
    Packet(): senderId(-1), receiverId(-1), packetId(-1), broadcastMsg(false) 
    {
    };
    unsigned int senderId;
    unsigned int receiverId;
    unsigned int packetId;
    bool broadcastMsg;
    std::vector<uint8_t> data;
};

class BusInterface
{
public:
    virtual bool sendPacket(const Packet &packet) = 0;
    virtual bool readPacket(Packet &packet) = 0;
    virtual uint16_t getMaxPacketSize() = 0;
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
    
    boost::function<void (void)> emptyCallback;
    
    void processAck(const Packet &msg);
};

class PacketReceiver
{
public:
    /**
     * Callback function get's called whenever there
     * is a new can message received.
     * */
    virtual void processMsg(const Packet &msg) = 0;
};

class LowPriorityProtocol
{
private:
    const uint16_t maxPacketSize;
    bool hasHeader;
    uint16_t curSize;
    Packet curMessage;
    BusInterface *bus;
public:
    LowPriorityProtocol(Protocol *proto);
    const Packet *processPackage(const Packet &msg);
    
    void sendPackage(const Packet &msg);    
};

class Protocol
{
    friend class Controller;
    friend class Reader;
    friend class Writer;
    friend class LowPriorityProtocol;
private:

    class Handle
    {
    public:
	Handle(unsigned int id, hbridge::Protocol* proto);
	unsigned int senderId;
	SendQueue queue;
	LowPriorityProtocol lowPrioProtocol;

	std::vector<PacketReceiver *> msgHandlers;
	
	void processMsg(const Packet &msg);
    };
    
    int retryCount;
    base::Time sendTimout;

    std::vector<Handle *> handles;
    std::vector<Packet> sharedMessages;
    std::map<unsigned int, bool> markedForSending;
    BusInterface *bus;
    
    bool isInProtocol(const Packet& msg) const;

    void sendPacket(int boardId, const Packet &msg, bool isAcked, boost::function<void (const Packet &msg)> errorCallback, boost::function<void (void)> ackedCallback = NULL);
    Packet &getSharedMsg(unsigned int packetId);
    void registerReceiver(hbridge::PacketReceiver* recv, unsigned int senderId);
    void setQueueEmptyCallback(boost::function<void (void)> emptyCallback, int senderId);
        
public:
    Protocol(BusInterface *bus);

    void setSendTimeout(const base::Time &timeout)
    {
	sendTimout = timeout;
    }
    
    /**
     * This interface needs to be implemented by any
     * canbus driver which should be used with this 
     * hbridge driver.
     * */
    void setBusInterface(BusInterface *bus);

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
    
    /**
     * This function sends a packet that switches 
     * the motor drivers from only Mainboard mode
     * to Driver mode. This means that the motor 
     * drivers will accept packets from the pc-driver
     * after this message call.
     * 
     * WARNING 	This function must only be used if no
     *		Mainboard is present in you system. 
     * */
    void setDriverAsBusMaster();
};

}
#endif // PROTOCOLL_HPP
