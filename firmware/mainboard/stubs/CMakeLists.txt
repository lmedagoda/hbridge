set(MD_DIR ${CMAKE_SOURCE_DIR}/firmware/) 
set(MB_DIR ${CMAKE_SOURCE_DIR}/firmware/mainboard/)

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
  )

add_definitions(-D STM32F10X_MD -D USE_STDPERIPH_DRIVER )

# SET(CMAKE_C_FLAGS "-ggdb -Wall")
# SET(CMAKE_CXX_FLAGS "-ggdb -Wall")
# 
# add_library(stubs 
# 
# )

add_library(mainboard_sandbox_lib 
    ${MD_DIR}/common/packets.c
    ${MD_DIR}/common/protocol.c
    ${MD_DIR}/common/protocol_low_priority.c
    ${MD_DIR}/common/hbridge_cmd.c
    ${MD_DIR}/common/hbridge_cmd2.c

    ${MB_DIR}/common/arc_driver.c
    ${MB_DIR}/common/arc_packet.c
    ${MB_DIR}/common/timeout.c
    ${MB_DIR}/common/time.c
    ${MB_DIR}/common/arc_ringbuffer.c
    ${MB_DIR}/common/tokenhandling.c
    ${MB_DIR}/common/packethandling.c
    ${MB_DIR}/common/mainboardstate.c
    ${MB_DIR}/stubs/usart_noop_stub.c
    ${MB_DIR}/stubs/can_noop_stub.c


#     printf_stub.c
#     thread_stub.cpp
#     packet_stub.cpp
)
# target_link_libraries(sandbox_lib pthread rt)
# 
add_executable(test_mb ${MB_DIR}/asguardv4/main.c)
target_link_libraries(test_mb mainboard_sandbox_lib)
