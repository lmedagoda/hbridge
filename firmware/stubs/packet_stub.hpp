#ifndef PACKET_STUB_HPP
#define PACKET_STUB_HPP

#include <boost/circular_buffer.hpp>
#include <boost/thread.hpp>
#include "../../src/Protocol.hpp"


extern boost::circular_buffer<hbridge::Packet> driverToFirmware;
extern boost::circular_buffer<hbridge::Packet> firmwareToDriver;
extern boost::mutex comMutex;

#endif