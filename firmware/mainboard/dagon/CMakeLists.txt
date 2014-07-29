set(MD_DIR ${CMAKE_SOURCE_DIR}/../../../firmware/) 
set(MB_DIR ${MD_DIR}/mainboard/)
    
include(FindPkgConfig)
pkg_check_modules(CAN REQUIRED "canbus")


include_directories(
    ${MD_DIR}
    ${MB_DIR}
    ${MD_DIR}/interfaces
    ${MD_DIR}/hbridgeCommon/
    ${MD_DIR}/hbridgeCommon/drivers
    ${MD_DIR}/hbridgeCommon/lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/
    ${MD_DIR}/hbridgeCommon/lib/CMSIS/CM3/CoreSupport/
    ${MD_DIR}/hbridgeCommon/lib/STM32F10x_StdPeriph_Driver/
    ${MD_DIR}/hbridgeCommon/lib/STM32F10x_StdPeriph_Driver/inc/
    ${CAN_INCLUDE_DIRS}
  )

add_definitions(-D USE_STDPERIPH_DRIVER -DSTM32F10X_HD -DSTUBDMODE -std=gnu99 -DNUM_MOTORS=0)

LINK_DIRECTORIES(${CAN_LIBRARY_DIRS})
# SET(CMAKE_C_FLAGS "-ggdb -Wall")
# SET(CMAKE_CXX_FLAGS "-ggdb -Wall")
# 
# add_library(stubs 
# 
# )

add_library(mainboard_sandbox_lib 
    ${MB_DIR}/common/arc_notokendriver.c
    ${MB_DIR}/common/arc_tokendriver.c
    ${MB_DIR}/common/arc_multichannel.c
    ${MD_DIR}/common/packets.c
    ${MD_DIR}/common/protocol.c
    ${MD_DIR}/common/protocol_low_priority.c
    ${MD_DIR}/common/hbridge_cmd.c
    ${MD_DIR}/common/hbridge_cmd2.c
    ${MD_DIR}/hbridgeCommon/protocol_can.c

    ${MB_DIR}/common/arc_driver.c
    ${MB_DIR}/common/arc_packet.c
    ${MB_DIR}/common/timeout.c
    ${MB_DIR}/common/time.c
    ${MB_DIR}/common/arc_ringbuffer.c
    ${MB_DIR}/common/tokenhandling.c
    ${MB_DIR}/common/packethandling.c
    ${MB_DIR}/common/mainboardstate.c
    ${MB_DIR}/dagon/usart_noop_stub.c
    ${MB_DIR}/dagon/can_noop_stub.cpp
    ${MB_DIR}/dagon/gpio_stub.c
    ${MD_DIR}/stubs/various_stubs.cpp
    ${MD_DIR}/stubs/thread_stub.cpp


#     printf_stub.c
#     thread_stub.cpp
#     packet_stub.cpp
)
# target_link_libraries(sandbox_lib pthread rt)
# 
add_executable(test_mb ${MB_DIR}/avalon/main.c ${MB_DIR}/avalon/avalon_can.c ${MB_DIR}/avalon/uwmodem.c)
target_link_libraries(test_mb mainboard_sandbox_lib pthread ${CAN_LIBRARIES})
