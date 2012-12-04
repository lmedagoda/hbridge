#include "Protocol.hpp"
#include "../protocol.hpp"
#include "Controller.hpp"
#include "Reader.hpp"
#include "Writer.hpp"

canbus::Message msg;
using namespace firmware;

namespace hbridge {

HbridgeHandle::HbridgeHandle(int id, Protocol* protocol):boardId(id), protocol(protocol)
{
    controllers.resize(firmware::NUM_CONTROLLERS, NULL);
    reader = new Reader(this);
    writer = new Writer(this);
    lowPrioProtocol = new LowPriorityProtocol(protocol);
}

Protocol::Protocol(BusInterface *bus) : bus(bus)
{
    handles.resize(BOARD_COUNT);
    
    retryCount = 3;
    sendTimout = base::Time::fromMilliseconds(30);
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
    std::cout << "Protocol : Requested shared message of type " << getPacketName(packetId) << std::endl;
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

bool Protocol::isInProtocol(const Packet& msg) const
{
    return true;
}

void Protocol::processIncommingPackages()
{
    Packet msg;
    while(bus->readPacket(msg))
    {
	//ignore non prtocoll messages
	if(!isInProtocol(msg))
	    continue;

	std::cout << "Protocol : Got incomming packet of type " << getPacketName(msg.packetId);

	//check if message is a broadcast
	if(msg.broadcastMsg) {
	    std::cout << " BROADCAST " << std::endl;
	    //broadcast, inform all readers
	    for(std::vector<HbridgeHandle *>::iterator it = handles.begin(); it != handles.end(); it++)
	    {
		if(*it)
		    (*it)->reader->processMsg(msg);
	    }
	} else {
	    std::cout << " for receiver " << msg.receiverId << std::endl;
	    //automatic ack handling
	    if(msg.packetId == firmware::PACKET_ID_ACK)
	    {
		const firmware::ackData *adata =
			reinterpret_cast<const firmware::ackData *>(msg.data.data());

		SendQueue &queues(handles[msg.senderId]->queue);
			
		if(queues.queue.empty())
		{
		    std::cout << "Warning got orphaned ack for id " << firmware::getPacketName(adata->packetId) << std::endl;
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
		    std::cout << "Warning got orphaned ack for id " << firmware::getPacketName(adata->packetId) << " exprected " << firmware::getPacketName(entry.msg.packetId) << std::endl;
		}
		continue;
	    }
	    
	    handles[msg.senderId]->reader->processMsg(msg);
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
	    if(curEntry.msg.packetId < firmware::PACKET_ID_LOWIDS_START)
		bus->sendPacket(curEntry.msg);
	    else
	    {
		std::cout << "Found low Prio message in send queue with id " << getPacketName(curEntry.msg.packetId) << std::endl;
		(*it)->getLowPriorityProtocol()->sendPackage(curEntry.msg);
	    }
	    
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
	    if(curEntry.msg.packetId < firmware::PACKET_ID_LOWIDS_START)
		bus->sendPacket(curEntry.msg);
	    else
	    {
		std::cout << "Found low Prio message in send queue with id " << getPacketName(curEntry.msg.packetId) << std::endl;
		(*it)->getLowPriorityProtocol()->sendPackage(curEntry.msg);
	    }
	    curEntry.sendTime = curTime;	    
	}
    }
}

LowPriorityProtocol::LowPriorityProtocol(Protocol* proto) : maxPacketSize(proto->bus->getMaxPacketSize()), hasHeader(false), bus(proto->bus)
{

}


const Packet* LowPriorityProtocol::processPackage(const hbridge::Packet& msg)
{
    assert(msg.packetId == firmware::PACKET_LOW_PRIORITY_DATA);
    
    const firmware::LowPrioPacket *lp = reinterpret_cast<const firmware::LowPrioPacket *>(msg.data.data());
    
//     std::cout << "Low Prio: Got packet with id " << getPacketName(msg.packetId)  << msg.packetId << " with type " << lp->type << std::endl;
    const uint16_t dataPerPkg = maxPacketSize - sizeof(firmware::LowPrioPacket);

    switch(lp->type)
    {
	case firmware::TYPE_HEADER:
	{
	    const firmware::LowPrioHeader *header = reinterpret_cast<const firmware::LowPrioHeader *>(msg.data.data() + sizeof(struct firmware::LowPrioPacket));
	    if(hasHeader)
	    {
		std::cout << "Error, got Header and last package was not finished" << std::endl;
	    }
	    hasHeader = true;
	    curMessage.packetId = header->id;
	    curMessage.senderId = msg.senderId;
	    curMessage.receiverId = msg.receiverId;
	    curMessage.broadcastMsg = msg.broadcastMsg;
	    curMessage.data.resize(header->size);
	    curSize = 0;
	}   
	    break;
	case firmware::TYPE_DATA:
	{
	    uint16_t dataPos = lp->sequenceNumber * dataPerPkg;
	    assert(curSize == dataPos);
	    const size_t pkgSize = curMessage.data.size();
	    if(dataPos > pkgSize)
	    {
		std::cout << "Error, sequence number points outside of packet" << std::endl;		
	    }
	    uint16_t toCopy = lp->sequenceNumber < pkgSize / dataPerPkg ? dataPerPkg : pkgSize - dataPos;
	    assert(toCopy == msg.data.size() - sizeof(struct firmware::LowPrioPacket));
	    for(int i = 0; i < toCopy; i++)
	    {
		curMessage.data[dataPos + i] = msg.data[i + sizeof(struct firmware::LowPrioPacket)];
	    }
	    curSize += toCopy;
	    
// 	    std::cout << "Got Packet part " << firmware::getPacketName(curMessage.packetId) << " Seq num " << lp->sequenceNumber << " Size " << msg.data.size() << " cursize " << curSize << std::endl;
	    if(curSize == pkgSize)
	    {
		hasHeader = false;
		return &curMessage;
	    }
	    break;
	}
    }
    return NULL;
    
}

void LowPriorityProtocol::sendPackage(const Packet& msg)
{
    assert(msg.packetId > firmware::PACKET_ID_LOWIDS_START);
    Packet headPkg;
    headPkg.receiverId = msg.receiverId;
    headPkg.senderId = msg.senderId;
    headPkg.broadcastMsg = msg.broadcastMsg;
    headPkg.packetId = firmware::PACKET_LOW_PRIORITY_DATA;
    headPkg.data.resize(sizeof(firmware::LowPrioPacket) + sizeof(firmware::LowPrioHeader));
    firmware::LowPrioPacket *lp = reinterpret_cast<firmware::LowPrioPacket *>(headPkg.data.data());
    firmware::LowPrioHeader *head = reinterpret_cast<firmware::LowPrioHeader *>(headPkg.data.data() + sizeof(struct firmware::LowPrioPacket));
    head->id = (firmware::LOW_PRIORITY_IDs)(msg.packetId);
    head->size = msg.data.size();
//     std::cout << "LowPrio: Sending Header" << std::endl;
    bus->sendPacket(headPkg);
    
//     std::cout << "Low Prio send, size " << msg.data.size() << std::endl;
    
    Packet dataPkg;
    dataPkg.receiverId = msg.receiverId;
    dataPkg.senderId = msg.senderId;
    dataPkg.broadcastMsg = msg.broadcastMsg;
    dataPkg.packetId = firmware::PACKET_LOW_PRIORITY_DATA;
    
    const uint16_t dataPerPkg = maxPacketSize - sizeof(firmware::LowPrioPacket);
    const uint16_t numPackets = msg.data.size() / dataPerPkg + 1;
//     std::cout << "Data per packet " << dataPerPkg << " num packets " << numPackets << std::endl;
    uint16_t dataCnt = 0;
    for(int i = 0; i < numPackets ; i++)
    {
	int toCopy = 0;
	if(i < numPackets -1)
	    toCopy = dataPerPkg;
	else
	    toCopy = msg.data.size() % dataPerPkg;

// 	std::cout << "Packet " << i << " To copy : " << toCopy << std::endl;
	dataPkg.data.resize(toCopy + sizeof(firmware::LowPrioPacket));
	lp = reinterpret_cast<firmware::LowPrioPacket *>(dataPkg.data.data());
	lp->sequenceNumber = i;
	lp->type = firmware::TYPE_DATA;
// 	std::cout << "LowPrio: Sending Data " << i << std::endl;
	for(int j = 0; j < dataPerPkg; j++)
	{
	    dataPkg.data[sizeof(firmware::LowPrioPacket) + j] = msg.data[dataCnt];	
	    dataCnt++;
	}
	
	bus->sendPacket(dataPkg);
    }
}


}



