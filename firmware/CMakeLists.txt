project(STM32 C)
cmake_minimum_required(VERSION 2.6)

#needs to be set from outside to work
#set(CMAKE_TOOLCHAIN_FILE  CodeSourcery.cmake)

set(COMPILE_DEFINITIONS_DEBUG    -O3 -ggdb)
set(COMPILE_DEFINITIONS_RELEASE  -O3)

#macro for stripping executables to flashable image
macro(OBJCOPY_FILE EXE_NAME)
 set(FO ${CMAKE_CURRENT_BINARY_DIR}/${EXE_NAME}.bin)
 set(FI ${CMAKE_CURRENT_BINARY_DIR}/${EXE_NAME})
 message(STATUS ${FO})
 add_custom_command(
  OUTPUT "${FO}"
  COMMAND ${CMAKE_OBJCOPY}
  ARGS -O binary ${FI} ${FO}
  DEPENDS ${FI})
 get_filename_component(TGT "${EXE_NAME}" NAME)
 add_custom_target("TargetObjCopy_${TGT}" ALL DEPENDS ${FO} VERBATIM)
 get_directory_property(extra_clean_files ADDITIONAL_MAKE_CLEAN_FILES)
 set_directory_properties(
  PROPERTIES
  ADDITIONAL_MAKE_CLEAN_FILES "${extra_clean_files};${FO}")
 set_source_files_properties("${FO}" PROPERTIES GENERATED TRUE)
endmacro(OBJCOPY_FILE)

#compile asm files with gcc
set_source_files_properties(cortexm3_macro.s PROPERTIES LANGUAGE C)

set(CMAKE_EXE_LINKER_FLAGS "-T ${STM32_SOURCE_DIR}/stm_h103_rom.ld -nostartfiles")

add_definitions(
  -Wall
  -mcpu=cortex-m3 -mthumb -Wall -fno-unwind-tables -fno-common
  -O3 -ggdb
  )

include_directories(
  ${STM32_SOURCE_DIR}/.
  ${STM32_SOURCE_DIR}/inc
  )

add_library(stm32fw 
  src/stm32f10x_adc.c
  src/stm32f10x_dma.c
  src/stm32f10x_iwdg.c
  src/stm32f10x_sdio.c
  src/stm32f10x_bkp.c
  src/stm32f10x_exti.c
  src/stm32f10x_lib.c
  src/stm32f10x_spi.c
  src/stm32f10x_can.c
  src/stm32f10x_flash.c
  src/stm32f10x_nvic.c
  src/stm32f10x_systick.c
  src/stm32f10x_crc.c
  src/stm32f10x_fsmc.c
  src/stm32f10x_pwr.c
  src/stm32f10x_tim.c
  src/stm32f10x_dac.c
  src/stm32f10x_gpio.c
  src/stm32f10x_rcc.c
  src/stm32f10x_usart.c
  src/stm32f10x_dbgmcu.c
  src/stm32f10x_i2c.c
  src/stm32f10x_rtc.c
  src/stm32f10x_wwdg.c
)

add_library(stm32drivers
    assert.c
    i2c.c           
    printf.c  
    usart.c
    can.c
    spi.c
    lm73cimk.c
    pid.c
    cortexm3_macro.s
    stm32f10x_it.c  
    stm32f10x_vector.c
) 

target_link_libraries(stm32drivers stm32fw)

link_directories(
  ${STM32_BINARY_DIR}/fwlib
  )

add_library(hbridge_common
    position_controller.c
    controllers.c
    protocol.c
    state.c
    encoder.c
    encoder_adc.c
    encoder_quadrature.c    
    systick.c
)
target_link_libraries(hbridge_common stm32drivers stm32fw)

add_executable(main
    main.c
    current_measurement.c
    hbridge.c
)

add_executable(pcbv2
    main_pcbv2.c
    hbridgev2.c
    current_measurementv2.c
)

target_link_libraries(main stm32fw hbridge_common stm32fw stm32drivers stm32fw)
objcopy_file(main)

target_link_libraries(pcbv2 stm32fw hbridge_common stm32fw stm32drivers stm32fw)
objcopy_file(pcbv2)

add_custom_target(flash openocd -f ${STM32_SOURCE_DIR}/openocd.cfg 
                  DEPENDS ${FO})