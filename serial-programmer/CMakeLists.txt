PROJECT(serial-programmer)

cmake_minimum_required(VERSION 2.6)
ADD_DEFINITIONS(-Wall)

ADD_EXECUTABLE(stm32prog stm32prog.cc stm32proto.cc)

TARGET_LINK_LIBRARIES(stm32prog)
