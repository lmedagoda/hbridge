#ifndef WRITER_HPP
#define WRITER_HPP
#include <canmessage.hh>
#include <vector>

namespace hbridge
{

class Protocol;
class Reader;
class Controller;

class Writer
{
private:
    int boardId;
    Controller *curController;
    
    friend class Protocol;
    Writer(int id, Protocol *protocol, Reader *reader);
    Protocol *protocol;
    Reader *reader;

public:
    void setController(unsigned int controllerId);
    void setTargetValue(double value);
};

}
#endif // WRITER_HPP
