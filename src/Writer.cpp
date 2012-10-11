#include "Writer.hpp"
#include "../protocol.hpp"
#include "Protocol.hpp"
#include "Controller.hpp"

namespace hbridge
{

Writer::Writer(int id, Protocol* protocol, Reader *reader): boardId(id), curController(0) ,protocol(protocol), reader(reader)
{

}

    
void Writer::setController(unsigned int controllerId)
{
    if(!reader->isWritable())
	throw std::runtime_error("Error: Tried to write on unconfigured hbridge");
	
    if(curController == protocol->getControllers(boardId)[controllerId])
	return;
    
    curController = protocol->getControllers(boardId)[controllerId];

    int canId = firmware::PACKET_ID_SET_MODE14;
    if(boardId > 3)
	canId = firmware::PACKET_ID_SET_MODE58;
  
    canbus::Message &sharedMsg(protocol->getSharedMsg(canId));
    sharedMsg.size = sizeof(firmware::setModeData);

    firmware::setModeData *data =
	reinterpret_cast<firmware::setModeData *>(sharedMsg.data);

    switch(boardId)
    {
	case 0:
	case 4:
	    data->board1Mode = (firmware::controllerModes) controllerId;
	    break;
	case 1:
	case 5:
	    data->board2Mode = (firmware::controllerModes) controllerId;
	    break;
	case 2:
	case 6:
	    data->board3Mode = (firmware::controllerModes) controllerId;
	    break;
	case 3:
	case 7:
	    data->board4Mode = (firmware::controllerModes) controllerId;
	    break;
    };
    
}

    
void Writer::setTargetValue(double value)
{
    if(!reader->isWritable())
	throw std::runtime_error("Error: Tried to write on unconfigured hbridge");

    int canId = firmware::PACKET_ID_SET_VALUE14;
    if(boardId > 3)
	canId = firmware::PACKET_ID_SET_VALUE58;

    canbus::Message &sharedMsg(protocol->getSharedMsg(canId));
    sharedMsg.size = sizeof(firmware::setValueData);
    
    firmware::setValueData *data =
	reinterpret_cast<firmware::setValueData *>(sharedMsg.data);
    
    unsigned short transportValue = curController->getTargetValue(value);
    
    switch(boardId)
    {
	case 0:
	case 4:
	    data->board1Value = transportValue;
	    break;
	case 1:
	case 5:
	    data->board2Value = transportValue;
	    break;
	case 2:
	case 6:
	    data->board3Value = transportValue;
	    break;
	case 3:
	case 7:
	    data->board4Value = transportValue;
	    break;
    };
}

    
    
    
    
}