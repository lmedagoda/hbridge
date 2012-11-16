#ifndef WRITER_HPP
#define WRITER_HPP
#include <canmessage.hh>
#include <vector>

namespace hbridge
{

class Protocol;
class Reader;
class Controller;
class HbridgeHandle;

class Writer
{
    friend class HbridgeHandle;
private:
    Controller *curController;
    
    friend class Protocol;
    Writer(HbridgeHandle *handle);
    HbridgeHandle *handle;

public:
    void setActiveController(Controller *ctrl);
    void setTargetValue(double value);
};

}
#endif // WRITER_HPP
