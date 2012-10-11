#ifndef CAN_STUB_HPP
#define CAN_STUB_HPP

#include <boost/circular_buffer.hpp>
#include <boost/thread.hpp>
#include <canmessage.hh>

extern boost::circular_buffer<canbus::Message> canToHB;
extern boost::circular_buffer<canbus::Message> canFromHB;
extern boost::mutex canMutex;

#endif
