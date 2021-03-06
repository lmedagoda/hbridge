#include "Protocol.hpp"
#include "Controller.hpp"
#include "Reader.hpp"
#include "Writer.hpp"
#include "../firmware/common/protocol.h"

canbus::Message msg;
using namespace firmware;

namespace hbridge {

Protocol::Handle::Handle(unsigned int id, hbridge::Protocol* proto): senderId(id), lowPrioProtocol(proto)
{

}


Protocol::Protocol(BusInterface *bus) : bus(bus)
{
    retryCount = 3;
    sendTimout = base::Time::fromMilliseconds(30);
}

void Protocol::setBusInterface(BusInterface* bus)
{
    this->bus = bus;
}

void Protocol::registerReceiver(PacketReceiver* recv, unsigned int senderId)
{
    if(handles.size() <= senderId)
	handles.resize(senderId + 1, NULL);
    
    if(!handles[senderId])
	handles[senderId] = new Handle(senderId, this);
    
    handles[senderId]->msgHandlers.push_back(recv);
}

Packet& Protocol::getSharedMsg(unsigned int packetId)
{
//     std::cout << "Protocol : Requested shared message of type " << getPacketName(packetId) << std::endl;
    if(sharedMessages.size() < packetId + 1)
    {
	sharedMessages.resize(packetId + 1);
    }

    Packet &pkg(sharedMessages[packetId]);
    pkg.senderId = SENDER_ID_PC;
    pkg.receiverId = RECEIVER_ID_ALL;
    pkg.packetId = packetId;
    //shared message is allways a broadcast message
    pkg.broadcastMsg = true;
    
    markedForSending[packetId] = true;
    
    return pkg;
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

void SendQueue::processAck(const hbridge::Packet& msg)
{
    const firmware::ackData *adata =
	    reinterpret_cast<const firmware::ackData *>(msg.data.data());

    if(queue.empty())
    {
	std::cout << "Protocol: Warning got orphaned ack for id " << firmware::getPacketName(adata->packetId) << std::endl;
	return;
    }
    
    
    SendQueue::Entry &entry(queue.front());
    if(adata->packetId == entry.msg.packetId)
    {
	if(entry.ackedCallback)
	    entry.ackedCallback();
	
	queue.pop();
    } else
    {
	std::cout << "Protocol: Warning got orphaned ack for id " << firmware::getPacketName(adata->packetId) << " exprected " << firmware::getPacketName(entry.msg.packetId) << std::endl;
    }

}


void Protocol::Handle::processMsg(const hbridge::Packet& msg)
{    
    if((senderId != msg.senderId) && !msg.broadcastMsg){
        std::cout << "Got error state sender id: " << senderId << ", msg.senderID: " << msg.senderId << " Broadcast: " << msg.broadcastMsg << std::endl;
        assert(false);
    }
    
    //automatic ack handling
    if(msg.packetId == firmware::PACKET_ID_ACK)
    {
	queue.processAck(msg);
    }
    else
    {
        const hbridge::Packet *inMsg;
        if(msg.packetId >= firmware::PACKET_ID_LOWIDS_START)
        {
             inMsg = lowPrioProtocol.processPackage(msg);
        }
        else
        {
            inMsg = &msg;
        }
        
        if(inMsg)
        {
            for(std::vector<PacketReceiver *>::iterator handler = msgHandlers.begin(); handler != msgHandlers.end(); handler++)
            {
                (*handler)->processMsg(*inMsg);
            }
        }
    }

}


void Protocol::processIncommingPackages()
{
    Packet msg;
    while(bus->readPacket(msg))
    {
	//ignore non prtocoll messages
	if(!isInProtocol(msg)){
	    std::cout << "Warning Received none protocol message" << std::endl;
	    continue;
        }
	//ignore messages where we don't have a handle for
	if(handles.size() <= msg.senderId || !handles[msg.senderId]){
	    continue;
        }
	
// 	if(msg.packetId != PACKET_ID_STATUS && msg.packetId != PACKET_ID_EXTENDED_STATUS)

	//check if message is a broadcast
	if(msg.broadcastMsg) {
	    //broadcast, inform all readers
	    for(std::vector<Handle *>::iterator it = handles.begin(); it != handles.end(); it++)
	    {
		(*it)->processMsg(msg);
	    }
	}
	else
	{
	    //only process by the correct handler
	    handles[msg.senderId]->processMsg(msg);
	}
    }
}

void Protocol::sendPacket(int boardId, const hbridge::Packet& msg, bool isAcked, boost::function<void (const Packet &msg)> errorCallback, boost::function<void (void)> ackedCallback)
{
    SendQueue::Entry entry;
    entry.msg = msg;
    entry.msg.receiverId = boardId;
    entry.msg.senderId = SENDER_ID_PC;
    entry.errorCallback = errorCallback;
    entry.ackedCallback = ackedCallback;
    entry.retryCnt = retryCount;
    
    handles[boardId]->queue.queue.push(entry);

}

void Protocol::setQueueEmptyCallback(boost::function<void (void)> emptyCallback, int senderId)
{
    handles[senderId]->queue.emptyCallback = emptyCallback;
}

void Protocol::processSendQueues()
{
    base::Time curTime = base::Time::now();
    for(std::vector<Handle *>::iterator it =  handles.begin(); it != handles.end(); it++)
    {
	if(*it == NULL)
	    continue;
	
	if((*it)->queue.queue.empty())   
	{
	    if((*it)->queue.emptyCallback)
		(*it)->queue.emptyCallback();
	    
	    continue;
	}
	
	SendQueue::Entry &curEntry((*it)->queue.queue.front());
	bool doSend = false;
	if(curEntry.sendTime == base::Time())
	{
	    doSend = true;
	}
	
	if(!doSend && (curTime - curEntry.sendTime > sendTimout))
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
	    std::cout << "Timeout, resending " << getPacketName(curEntry.msg.packetId) << " retry " << retryCount - curEntry.retryCnt << " receiver is " << curEntry.msg.receiverId << " cur time " << curTime << " original send time " << curEntry.sendTime << " timeout " << sendTimout <<  std::endl;
	    doSend = true;
	}
	
	if(doSend)
	{
	    if(curEntry.msg.packetId < firmware::PACKET_ID_LOWIDS_START)
		bus->sendPacket(curEntry.msg);
	    else
	    {
		(*it)->lowPrioProtocol.sendPackage(curEntry.msg);
	    }
	    
	    curEntry.sendTime = curTime;
	}
    }
}

void Protocol::setDriverAsBusMaster()
{
    Packet pkg;
    pkg.broadcastMsg = 1;
    pkg.packetId = firmware::PACKET_ID_SET_ALLOWED_SENDER;
    pkg.data.resize(sizeof(setAllowedSenderData));
    pkg.senderId = SENDER_ID_MAINBOARD;
    pkg.receiverId = RECEIVER_ID_ALL;
    
    setAllowedSenderData *sasd = (setAllowedSenderData *) pkg.data.data();
    sasd->onlyMainboard = 0;
    
    //note this message bypasses 
    //the setting of the sender 
    //and therfor directly accesses 
    //the bus.
    bus->sendPacket(pkg);
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
		std::cout << "LowPriorityProtocol: Error, got Header and last package was not finished" << std::endl;
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
	    if(curSize != dataPos)
	    {
		//brocken package, ignore
		hasHeader = false;
		break;
	    }
//	    assert(curSize == dataPos);
	    const size_t pkgSize = curMessage.data.size();
	    if(dataPos > pkgSize)
	    {
		std::cout << "LowPriorityProtocol: Error, sequence number points outside of packet" << std::endl;		
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
	for(int j = 0; j < toCopy; j++)
	{
	    dataPkg.data[sizeof(firmware::LowPrioPacket) + j] = msg.data[dataCnt];	
	    dataCnt++;
	}
	
	bus->sendPacket(dataPkg);
    }
}


}



