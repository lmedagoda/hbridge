include_directories(
    ../
    ../interfaces
  )
# SET(CMAKE_C_FLAGS "-ggdb -Wall")
# SET(CMAKE_CXX_FLAGS "-ggdb -Wall")
# 
add_library(stubs 
    printf_stub.c
    thread_stub.cpp
    packet_stub.cpp
#    usart_stub.cpp
#    various_stubs.cpp
#    can_stub.cpp
    current_measurement_stub.cpp
    hbridge_stub.cpp
    encoder_stub.cpp
    lm73cimk_stub.cpp
)

add_library(sandbox_lib firmware_sandbox.c 
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
    ../common/temperature_sensor.c
)
target_link_libraries(sandbox_lib stubs  pthread rt)

add_executable(test main.cpp)
target_link_libraries(test sandbox_lib)
