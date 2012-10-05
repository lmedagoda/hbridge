#include "Protocol.hpp"
#include "../protocol.hpp"
#include "Controller.hpp"

#define HBRIDGE_BOARD_ID(x) ((x + 1) << 5)

namespace hbridge {

Protocol::HbridgeHandle::HbridgeHandle(int id, Protocol* protocol)
{
    reader = new Reader(id, protocol);
    writer = new Writer(id, protocol, reader);
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
    controllers.resize(BOARD_COUNT);
    
    retryCount = 3;
    sendTimout = base::Time::fromMicroseconds(30);
}

void Protocol::setCanbusInterface(CanbusInterface* canbus)
{
    this->canbus = canbus;
}

Protocol::HbridgeHandle* Protocol::getHbridgeHandle(int id)
{
    if(id < 0 || id > BOARD_COUNT)
	throw std::out_of_range("Error tried to get hbridge handle with invalid hbridge id");

    if(!handles[id])
	handles[id] = new HbridgeHandle(id, this);
	
    return handles[id];
}


void Protocol::registerController(unsigned int controllerId, const hbridge::Controller& ctrl)
{
    for(unsigned int i = 0; i < handles.size(); i++)
    {
	if(!handles[i])
	    continue;
	
	std::vector<Controller *> &v(handles[i]->controllers);
	if(v.size() < controllerId + 1)
	    v.resize(controllerId + 1, NULL);
	
	if(v[controllerId] != NULL)
	    std::runtime_error("Error ther is allready an controller registered for the given id");
	
	v[controllerId] = ctrl.getCopy();
	v[controllerId]->setReader(handles[i]->reader);
    }
}

std::vector< Controller* >& Protocol::getControllers(int id)
{
    return handles[id]->controllers;
}

canbus::Message& Protocol::getSharedMsg(unsigned int canbusId)
{
    std::cout << "Shared id " << canbusId << std::endl;
    if(sharedCanMessages.size() < canbusId + 1)
    {
	sharedCanMessages.resize(canbusId + 1);
    }

    sharedCanMessages[canbusId].can_id = canbusId;
    
    markedForSending[canbusId] = true;
    
    return sharedCanMessages[canbusId];
}

void Protocol::sendSharedMessages()
{
    for(std::map<unsigned int, bool>::iterator it = markedForSending.begin(); it != markedForSending.end();it++)
    {
	canbus->sendCanPacket(sharedCanMessages[it->first]);
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
    canbus::Message msg;
    while(canbus->readCanPacket(msg))
    {
	//ignore non prtocoll messages
	if(!isInProtocol(msg))
	    continue;

	int index = getBoardIdFromMessage(msg);

	unsigned int msgId = msg.can_id & 0x1f;
	
	//automatic ack handling
	if(msgId == firmware::PACKET_ID_ACK)
	{
	    const firmware::ackData *adata =
		    reinterpret_cast<const firmware::ackData *>(msg.data);

	    SendQueue &queues(handles[index]->queue);
		    
	    if(queues.queue.empty())
	    {
		std::cout << "Warning got orphaned ack for id " << adata->packetId << std::endl;
		continue;
	    }
	    
	    
	    SendQueue::Entry &entry(queues.queue.front());
	    if(adata->packetId == entry.msg.can_id)
	    {
		if(entry.ackedCallback)
		    entry.ackedCallback();
		
		queues.queue.pop();
	    } else
	    {
		std::cout << "Warning got orphaned ack for id " << adata->packetId << " exprected " << entry.msg.can_id << std::endl;		
	    }
	    continue;
	}
	
	//check if message is a broadcast
	if(index == -1) {
	    //broadcast, inform all readers
	    for(std::vector<HbridgeHandle *>::iterator it = handles.begin(); it != handles.end(); it++)
		(*it)->reader->processMsg(msg);
	} else {
	    handles[index]->reader->processMsg(msg);
	}
    }
}

void Protocol::sendCanPacket(int boardId, const canbus::Message &packet, bool isAcked, boost::function<void (const canbus::Message &)> errorCallback, boost::function<void (void)> ackedCallback)
{
    SendQueue::Entry entry;
    entry.msg = packet;
    entry.msg.can_id = HBRIDGE_BOARD_ID(boardId) | entry.msg.can_id;
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
	    canbus->sendCanPacket(curEntry.msg);
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
	    canbus->sendCanPacket(curEntry.msg);
	    curEntry.sendTime = curTime;	    
	}
    }
}



}



