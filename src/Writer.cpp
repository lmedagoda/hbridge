#include "Writer.hpp"
#include "Reader.hpp"
#include "../protocol.hpp"
#include "Protocol.hpp"
#include "Controller.hpp"
#define HBRIDGE_BOARD_ID(x) ((x + 1) << 5)

namespace hbridge
{

Writer::Writer(hbridge::HbridgeHandle* handle): curController(0), handle(handle)
{

}

void Writer::setActiveController(Controller* ctrl)
{
    if(!handle->getReader()->isWritable())
	throw std::runtime_error("Error: Tried to write on unconfigured hbridge");

    if(handle->getControllers()[ctrl->getControllerId()] != ctrl)
	throw std::runtime_error("Error: given controller is not registered at handle");
    
    if(curController == ctrl)
	return;

    curController = ctrl;
    
    Packet msg;
    msg.data.resize(sizeof(firmware::setActiveControllerData));

    firmware::setActiveControllerData *data =
	reinterpret_cast<firmware::setActiveControllerData *>(msg.data.data());

    data->controllerId = ctrl->getControllerId();
}

void Writer::setTargetValue(double value)
{
    if(!handle->getReader()->isWritable())
	throw std::runtime_error("Error: Tried to write on unconfigured hbridge");

    if(!curController)
	throw std::runtime_error("Error: No Controller selected");
    
    int packetId = firmware::PACKET_ID_SET_VALUE14;
    if(handle->getBoardId() > 3)
	packetId = firmware::PACKET_ID_SET_VALUE58;

    Packet &sharedMsg(handle->getProtocol()->getSharedMsg(packetId));
    sharedMsg.data.resize(sizeof(firmware::setValueData));
    
    firmware::setValueData *data =
	reinterpret_cast<firmware::setValueData *>(sharedMsg.data.data());
    
    unsigned short transportValue = curController->getTargetValue(value);
    
    switch(handle->getBoardId())
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