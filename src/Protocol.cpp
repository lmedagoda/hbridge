#include "Protocol.hpp"
#include "../protocol.hpp"
#include "Controller.hpp"
#include "Reader.hpp"
#include "Writer.hpp"

#define HBRIDGE_BOARD_ID(x) ((x + 1) << 5)

namespace hbridge {

HbridgeHandle::HbridgeHandle(int id, Protocol* protocol):boardId(id), protocol(protocol)
{
    controllers.resize(firmware::NUM_CONTROLLERS, NULL);
    reader = new Reader(this);
    writer = new Writer(this);
}

    
Protocol *Protocol::instance;

Protocol* Protocol::getInstance()
{
    if(!instance)
	instance = new Protocol();
    
    return instance;
}

Protocol::Protocol()
{
    handles.resize(BOARD_COUNT);
    
    retryCount = 3;
    sendTimout = base::Time::fromMicroseconds(30);
}

void Protocol::setBusInterface(BusInterface* bus)
{
    this->bus = bus;
}

HbridgeHandle* Protocol::getHbridgeHandle(int id)
{
    if(id < 0 || id > BOARD_COUNT)
	throw std::out_of_range("Error tried to get hbridge handle with invalid hbridge id");

    if(!handles[id])
	handles[id] = new HbridgeHandle(id, this);
	
    return handles[id];
}

void HbridgeHandle::registerForMsg(PacketReveiver* reveiver, int packetId)
{

}


void HbridgeHandle::registerController(hbridge::Controller* ctrl)
{
    if(controllers[ctrl->getControllerId()])
	throw std::out_of_range("Error: There is allready a controller with id X registered");
    
    controllers[ctrl->getControllerId()] = ctrl;
}

Packet& Protocol::getSharedMsg(unsigned int packetId)
{
    std::cout << "Shared id " << packetId << std::endl;
    if(sharedMessages.size() < packetId + 1)
    {
	sharedMessages.resize(packetId + 1);
    }

    sharedMessages[packetId].packetId = packetId;
    
    markedForSending[packetId] = true;
    
    return sharedMessages[packetId];
}

void Protocol::sendSharedMessages()
{
    for(std::map<unsigned int, bool>::iterator it = markedForSending.begin(); it != markedForSending.end();it++)
    {
	bus->sendPacket(sharedMessages[it->first]);
    }
    markedForSending.clear();
}

int Protocol::getBoardIdFromMessage(const canbus::Message& msg) const
{
    int index = ((msg.can_id & 0xE0) >> 5) - 1;
    return index;
}


bool Protocol::isInProtocol(const canbus::Message& msg) const
{
    return true;
}

void Protocol::processIncommingPackages()
{
    Packet msg;
    while(bus->readPacket(msg))
    {
// 	//ignore non prtocoll messages
// 	if(!isInProtocol(msg))
// 	    continue;

	//check if message is a broadcast
	if(msg.broadcastMsg) {
	    //broadcast, inform all readers
	    for(std::vector<HbridgeHandle *>::iterator it = handles.begin(); it != handles.end(); it++)
		(*it)->reader->processMsg(msg);
	} else {
	    //automatic ack handling
	    if(msg.packetId == firmware::PACKET_ID_ACK)
	    {
		const firmware::ackData *adata =
			reinterpret_cast<const firmware::ackData *>(msg.data.data());

		SendQueue &queues(handles[msg.senderId]->queue);
			
		if(queues.queue.empty())
		{
		    std::cout << "Warning got orphaned ack for id " << adata->packetId << std::endl;
		    continue;
		}
		
		
		SendQueue::Entry &entry(queues.queue.front());
		if(adata->packetId == entry.msg.packetId)
		{
		    if(entry.ackedCallback)
			entry.ackedCallback();
		    
		    queues.queue.pop();
		} else
		{
		    std::cout << "Warning got orphaned ack for id " << adata->packetId << " exprected " << entry.msg.packetId << std::endl;		
		}
		continue;
	    }
	    
	    handles[msg.packetId]->reader->processMsg(msg);
	}
    }
}

void Protocol::sendPacket(int boardId, const hbridge::Packet& msg, bool isAcked, boost::function<void (const Packet &msg)> errorCallback, boost::function<void (void)> ackedCallback)
{
    SendQueue::Entry entry;
    entry.msg = msg;
    entry.msg.receiverId = boardId;
    entry.msg.senderId = 0;
    entry.errorCallback = errorCallback;
    entry.ackedCallback = ackedCallback;
    entry.retryCnt = retryCount;
    
    handles[boardId]->queue.queue.push(entry);

}

void Protocol::processSendQueues()
{
    base::Time curTime = base::Time::now();
    for(std::vector<HbridgeHandle *>::iterator it =  handles.begin(); it != handles.end(); it++)
    {
	if(*it == NULL || (*it)->queue.queue.empty())
	    continue;
	
	SendQueue::Entry &curEntry((*it)->queue.queue.front());
	if(curEntry.sendTime == base::Time())
	{
	    bus->sendPacket(curEntry.msg);
	    curEntry.sendTime = curTime;
	    continue;
	}
	
	if(curTime - curEntry.sendTime > sendTimout)
	{
	    if(curEntry.retryCnt <= 0)
	    {
		curEntry.errorCallback(curEntry.msg);
		//kill rest of queue
		while(!(*it)->queue.queue.empty())
		    (*it)->queue.queue.pop();
		continue;
	    }
	    curEntry.retryCnt--;
	    bus->sendPacket(curEntry.msg);
	    curEntry.sendTime = curTime;	    
	}
    }
}



}



