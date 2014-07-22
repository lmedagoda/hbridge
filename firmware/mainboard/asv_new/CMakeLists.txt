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

set(STM32_COMMON_DIR ${STM32_SOURCE_DIR}/../../hbridgeCommon) 
set(MD_COMMON_DIR ${STM32_SOURCE_DIR}/../../common) 
set(MB_COMMON_DIR ${STM32_SOURCE_DIR}/../common) 

#compile asm files with gcc
set_source_files_properties(${STM32_COMMON_DIR}/lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_md.s PROPERTIES LANGUAGE C)
set_source_files_properties(core_cm3.c PROPERTIES LANGUAGE C)

set(CMAKE_EXE_LINKER_FLAGS "-T ${STM32_SOURCE_DIR}/stm_h103_rom.ld -nostartfiles")

add_definitions(
    -Wall
    -mcpu=cortex-m3 -mthumb -Wall -fno-unwind-tables -fno-common -fno-builtin-printf
    -O3 -ggdb
    -D STM32F10X_MD -D USE_STDPERIPH_DRIVER -D HSE_VALUE=16000000  
)


include_directories(
    ${STM32_COMMON_DIR}/.
    ${STM32_COMMON_DIR}/../
    ${STM32_COMMON_DIR}/../interfaces
    ${STM32_COMMON_DIR}/../mainboard/
    ${MD_COMMON_DIR}
    ${MB_COMMON_DIR}
    ${STM32_COMMON_DIR}/drivers/
    ${STM32_COMMON_DIR}/lib/STM32F10x_StdPeriph_Driver/
    ${STM32_COMMON_DIR}/lib/STM32F10x_StdPeriph_Driver/inc
    ${STM32_COMMON_DIR}/lib/CMSIS/CM3/CoreSupport/
    ${STM32_COMMON_DIR}/lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x
)



add_executable(main
    main.c
    ${STM32_COMMON_DIR}/system_clock.c
    ${STM32_COMMON_DIR}/lib/CMSIS/CM3/CoreSupport/core_cm3.c
    ${STM32_COMMON_DIR}/lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_md.s
    ${STM32_COMMON_DIR}/lib/STM32F10x_StdPeriph_Driver/src/misc.c
    ${STM32_COMMON_DIR}/lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_adc.c
    ${STM32_COMMON_DIR}/lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_dma.c
    ${STM32_COMMON_DIR}/lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_iwdg.c
    ${STM32_COMMON_DIR}/lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_sdio.c
    ${STM32_COMMON_DIR}/lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_bkp.c
    ${STM32_COMMON_DIR}/lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_exti.c
    ${STM32_COMMON_DIR}/lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_spi.c
    ${STM32_COMMON_DIR}/lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_can.c
    ${STM32_COMMON_DIR}/lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_flash.c
    ${STM32_COMMON_DIR}/lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_crc.c
    ${STM32_COMMON_DIR}/lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_fsmc.c
    ${STM32_COMMON_DIR}/lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_pwr.c
    ${STM32_COMMON_DIR}/lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c
    ${STM32_COMMON_DIR}/lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_dac.c
    ${STM32_COMMON_DIR}/lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c
    ${STM32_COMMON_DIR}/lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c
    ${STM32_COMMON_DIR}/lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_usart.c
    ${STM32_COMMON_DIR}/lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_dbgmcu.c
    ${STM32_COMMON_DIR}/lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_i2c.c
    ${STM32_COMMON_DIR}/lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_rtc.c
    ${STM32_COMMON_DIR}/lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_wwdg.c
    ${STM32_COMMON_DIR}/drivers/assert.c
    ${STM32_COMMON_DIR}/drivers/i2c.c           
    ${STM32_COMMON_DIR}/drivers/printf.c  
    ${STM32_COMMON_DIR}/drivers/usart.c
    ${STM32_COMMON_DIR}/drivers/can.c
    ${STM32_COMMON_DIR}/drivers/spi.c
    ${STM32_COMMON_DIR}/drivers/lm73cimk.c
    ${STM32_COMMON_DIR}/drivers/tmp100.c
    ${STM32_COMMON_DIR}/encoder_adc.c
    ${STM32_COMMON_DIR}/encoder_quadrature.c    
    ${STM32_COMMON_DIR}/encoder_ichaus.c    
    ${STM32_COMMON_DIR}/thread.c
    ${STM32_COMMON_DIR}/protocol_can.c
    ${MD_COMMON_DIR}/packets.c
    ${MD_COMMON_DIR}/protocol.c
    ${MD_COMMON_DIR}/protocol_low_priority.c
    ${MD_COMMON_DIR}/hbridge_cmd.c
    ${MD_COMMON_DIR}/hbridge_cmd2.c

    ${MB_COMMON_DIR}/arc_driver.c
    ${MB_COMMON_DIR}/arc_packet.c
    ${MB_COMMON_DIR}/timeout.c
    ${MB_COMMON_DIR}/time.c
    ${MB_COMMON_DIR}/arc_ringbuffer.c
    ${MB_COMMON_DIR}/tokenhandling.c
    ${MB_COMMON_DIR}/packethandling.c
    ${MB_COMMON_DIR}/mainboardstate.c
    ${MB_COMMON_DIR}/arc_tokendriver.c
    ${MB_COMMON_DIR}/arc_notokendriver.c
    ${MB_COMMON_DIR}/arc_multichannel.c
    asv_can.c
    uwmodem.c
)

target_link_libraries(main)
objcopy_file(main)


add_custom_target(flash openocd -f ${STM32_SOURCE_DIR}/openocd.cfg 
                  DEPENDS ${FO})
