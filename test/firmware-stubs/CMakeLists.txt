include_directories(
  ${CMAKE_SOURCE_DIR}/firmware/.
  ${CMAKE_SOURCE_DIR}/firmware/lib/STM32F10x_StdPeriph_Driver/
  ${CMAKE_SOURCE_DIR}/firmware/lib/STM32F10x_StdPeriph_Driver/inc
  ${CMAKE_SOURCE_DIR}/firmware/lib/CMSIS/CM3/CoreSupport/
  ${CMAKE_SOURCE_DIR}/firmware/lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x
)

add_definitions(
  -D STM32F10X_MD -D USE_STDPERIPH_DRIVER -D HSE_VALUE=16000000 -D __SANDBOX__  
  )

add_library(stubs 
    printf_stub.c
    systick_stub.cpp
    usart_stub.cpp
    various_stubs.cpp
    can_stub.cpp
    current_measurement_stub.cpp
    hbridge_stub.cpp
    encoder_stub.cpp
    lm73cimk_stub.cpp
)

add_library(sandbox_lib firmware_sandbox.c 
    ../../firmware/protocol.c
    ../../firmware/position_controller.c
    ../../firmware/speed_controller.c
    ../../firmware/controllers.c
    ../../firmware/protocol.c
    ../../firmware/state.c
    ../../firmware/systick.c
    ../../firmware/drivers/pid.c
    ../../firmware/encoder.c
)
target_link_libraries(sandbox_lib stubs  pthread rt)

