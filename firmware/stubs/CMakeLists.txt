include_directories(
    ../
    ../interfaces
    ../hbridgeCommon/
    ../hbridgeCommon/lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/
    ../hbridgeCommon/lib/CMSIS/CM3/CoreSupport/
    ../hbridgeCommon/lib/STM32F10x_StdPeriph_Driver/
    ../hbridgeCommon/lib/STM32F10x_StdPeriph_Driver/inc/
  )

add_definitions(-D STM32F10X_MD -D USE_STDPERIPH_DRIVER )

# SET(CMAKE_C_FLAGS "-ggdb -Wall")
# SET(CMAKE_CXX_FLAGS "-ggdb -Wall")
# 
# add_library(stubs 
# 
# )

add_library(sandbox_lib 
    firmware_sandbox.cpp 
    ../common/protocol.c
    ../common/protocol_low_priority.c
    ../common/position_controller.c
    ../common/speed_controller.c
    ../common/controllers.c
    ../common/protocol.c
    ../common/state.c
    ../common/systick.c
    ../common/pid.c
    ../common/encoder.c
    ../common/packets.c
    ../common/temperature_sensor.c
    printf_stub.c
    thread_stub.cpp
    packet_stub.cpp
#    usart_stub.cpp
#    various_stubs.cpp
#     can_stub.cpp
    current_measurement_stub.cpp
    hbridge_stub.cpp
    encoder_stub.cpp
    lm73cimk_stub.cpp
)
target_link_libraries(sandbox_lib pthread rt)

add_executable(test main.cpp)
target_link_libraries(test sandbox_lib)
