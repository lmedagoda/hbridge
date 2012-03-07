NAME   = stm103_stk_rom

CC      = arm-none-linux-gnueabi-gcc
LD      = arm-none-linux-gnueabi-ld -v
AR      = arm-none-linux-gnueabi-ar
AS      = arm-none-linux-gnueabi-as
CP      = arm-none-linux-gnueabi-objcopy
OD	= arm-none-linux-gnueabi-objdump

HOST_CC = gcc
HOST_CFLAGS = -Wall

CFLAGS  =  -I ./ -I inc/ -c -fno-common -O1 -g -mcpu=cortex-m3 -mthumb -ggdb -Wall -fno-unwind-tables
AFLAGS  = -ahls -mapcs-32 -o crt.o
LFLAGS  = -L. -Tstm_h103_rom.ld -nostartfiles
CPFLAGS = -Obinary
ODFLAGS	= -S

LIB_OUT = libstm32fw.a

LIB_OBJS = $(sort \
 $(patsubst %.c,%.o,$(wildcard src/*.c)) \
 $(patsubst %.s,%.o,$(wildcard src/*.s)))

OBJS = main.o stm32f10x_it.o stm32f10x_vector.o \
	i2c.o current_measurement.o pid.o usart.o printf.o can.o controllers.o \
	protocol.o spi.o cortexm3_macro.o state.o hbridgev2.o encoder.o \
	lm73cimk.o encoder_adc.o encoder_quadrature.o assert.o position_controller.o

all: bin

flash: all
	@openocd -f openocd.cfg

# lib

$(LIB_OUT): $(LIB_OBJS)
	$(AR) $(ARFLAGS) $@ $(LIB_OBJS)

$(LIB_OBJS): stm32f10x_it.h stm32f10x_conf.h $(wildcard inc/*.h)

clean:
	-rm -f main.list main.out main.bin main1.out main1.bin checksum.c \
	*.o *.a $(LIB_OBJS)

bin: main.bin main.list

main.bin: main.out
	@ echo "...copying"
	$(CP) $(CPFLAGS) $< $@

main.list: main.out
	$(OD) $(ODFLAGS) $< > $@ || rm $@

main1.bin: main1.out
	@ echo "...copying"
	$(CP) $(CPFLAGS) $< $@

main.out: $(OBJS) checksum.o $(LIB_OUT) stm_h103_rom.ld
	@ echo "..linking"
	$(LD) $(LFLAGS) -o $@ $(OBJS) checksum.o -lstm32fw

main1.out: $(OBJS) checksum1.o $(LIB_OUT) stm_h103_rom.ld
	@ echo "..linking"
	$(LD) $(LFLAGS) -o $@ $(OBJS) checksum1.o -lstm32fw

checksum.c: main1.bin gen_checksum
	./gen_checksum < $< > $@ || rm $@

spi.o: spi.c spi.h

pid.o: pid.c pid.h

cortexm3_macro.o: cortexm3_macro.s
	$(CC) $(CFLAGS) cortexm3_macro.s

gen_checksum: gen_checksum.c
	$(HOST_CC) $(HOST_CFLAGS) -o $@ $^

.PHONY: bin clean

#generated using 
# find . -name "*.c" -exec gcc -I inc -I . -MM {} -MQ {} \; | sed -r "s/[ ].+\\.c//" >> Makefile
#and some manual fixups

main.c: inc/stm32f10x_lib.h inc/stm32f10x_map.h stm32f10x_conf.h \
 inc/stm32f10x_type.h inc/stm32f10x_type.h inc/cortexm3_macro.h \
 inc/stm32f10x_adc.h inc/stm32f10x_bkp.h inc/stm32f10x_can.h \
 inc/stm32f10x_crc.h inc/stm32f10x_dac.h inc/stm32f10x_dbgmcu.h \
 inc/stm32f10x_dma.h inc/stm32f10x_exti.h inc/stm32f10x_flash.h \
 inc/stm32f10x_fsmc.h inc/stm32f10x_gpio.h inc/stm32f10x_i2c.h \
 inc/stm32f10x_iwdg.h inc/stm32f10x_nvic.h inc/stm32f10x_pwr.h \
 inc/stm32f10x_rcc.h inc/stm32f10x_rtc.h inc/stm32f10x_sdio.h \
 inc/stm32f10x_spi.h inc/stm32f10x_systick.h inc/stm32f10x_tim.h \
 inc/stm32f10x_usart.h inc/stm32f10x_wwdg.h inc/stm32f10x_gpio.h \
 inc/stm32f10x_tim.h inc/stm32f10x_rcc.h stm32f10x_it.h \
 inc/stm32f10x_lib.h inc/stm32f10x_can.h spi.h inc/stm32f10x_type.h i2c.h \
 inc/stm32f10x_i2c.h pid.h usart.h inc/stm32f10x_usart.h protocol.h can.h \
 state.h controllers.h hbridge.h encoder.h current_measurement.h \
 inc/stm32f10x_adc.h lm73cimk.h printf.h
current_measurement.c: inc/stm32f10x_type.h \
 inc/stm32f10x_adc.h inc/stm32f10x_map.h stm32f10x_conf.h \
 inc/stm32f10x_type.h inc/stm32f10x_type.h inc/cortexm3_macro.h \
 inc/stm32f10x_dma.h inc/stm32f10x_rcc.h inc/stm32f10x_tim.h printf.h \
 current_measurement.h
stm32f10x_it.c: stm32f10x_it.h inc/stm32f10x_lib.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h inc/stm32f10x_adc.h \
 inc/stm32f10x_bkp.h inc/stm32f10x_can.h inc/stm32f10x_crc.h \
 inc/stm32f10x_dac.h inc/stm32f10x_dbgmcu.h inc/stm32f10x_dma.h \
 inc/stm32f10x_exti.h inc/stm32f10x_flash.h inc/stm32f10x_fsmc.h \
 inc/stm32f10x_gpio.h inc/stm32f10x_i2c.h inc/stm32f10x_iwdg.h \
 inc/stm32f10x_nvic.h inc/stm32f10x_pwr.h inc/stm32f10x_rcc.h \
 inc/stm32f10x_rtc.h inc/stm32f10x_sdio.h inc/stm32f10x_spi.h \
 inc/stm32f10x_systick.h inc/stm32f10x_tim.h inc/stm32f10x_usart.h \
 inc/stm32f10x_wwdg.h
src/stm32f10x_i2c.c: inc/stm32f10x_i2c.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h inc/stm32f10x_rcc.h
src/stm32f10x_rtc.c: inc/stm32f10x_rtc.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h
src/stm32f10x_dbgmcu.c: inc/stm32f10x_dbgmcu.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h
src/stm32f10x_wwdg.c: inc/stm32f10x_wwdg.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h inc/stm32f10x_rcc.h
src/stm32f10x_gpio.c: inc/stm32f10x_gpio.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h inc/stm32f10x_rcc.h
src/stm32f10x_spi.c: inc/stm32f10x_spi.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h inc/stm32f10x_rcc.h
src/stm32f10x_systick.c: inc/stm32f10x_systick.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h
src/stm32f10x_iwdg.c: inc/stm32f10x_iwdg.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h
src/stm32f10x_bkp.c: inc/stm32f10x_bkp.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h inc/stm32f10x_rcc.h
src/stm32f10x_rcc.c: inc/stm32f10x_rcc.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h
src/stm32f10x_can.c: inc/stm32f10x_can.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h inc/stm32f10x_rcc.h
src/stm32f10x_dac.c: inc/stm32f10x_dac.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h inc/stm32f10x_rcc.h
src/stm32f10x_flash.c: inc/stm32f10x_flash.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h
src/stm32f10x_adc.c: inc/stm32f10x_adc.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h inc/stm32f10x_rcc.h
src/stm32f10x_sdio.c: inc/stm32f10x_sdio.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h inc/stm32f10x_rcc.h
src/stm32f10x_usart.c: inc/stm32f10x_usart.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h inc/stm32f10x_rcc.h
src/stm32f10x_tim.c: inc/stm32f10x_tim.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h inc/stm32f10x_rcc.h
src/stm32f10x_fsmc.c: inc/stm32f10x_fsmc.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h inc/stm32f10x_rcc.h
src/stm32f10x_exti.c: inc/stm32f10x_exti.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h
src/stm32f10x_crc.c: inc/stm32f10x_crc.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h
src/stm32f10x_nvic.c: inc/stm32f10x_nvic.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h
src/stm32f10x_pwr.c: inc/stm32f10x_pwr.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h inc/stm32f10x_rcc.h
src/stm32f10x_dma.c: inc/stm32f10x_dma.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h inc/stm32f10x_rcc.h
src/stm32f10x_lib.c: inc/stm32f10x_lib.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h inc/stm32f10x_adc.h \
 inc/stm32f10x_bkp.h inc/stm32f10x_can.h inc/stm32f10x_crc.h \
 inc/stm32f10x_dac.h inc/stm32f10x_dbgmcu.h inc/stm32f10x_dma.h \
 inc/stm32f10x_exti.h inc/stm32f10x_flash.h inc/stm32f10x_fsmc.h \
 inc/stm32f10x_gpio.h inc/stm32f10x_i2c.h inc/stm32f10x_iwdg.h \
 inc/stm32f10x_nvic.h inc/stm32f10x_pwr.h inc/stm32f10x_rcc.h \
 inc/stm32f10x_rtc.h inc/stm32f10x_sdio.h inc/stm32f10x_spi.h \
 inc/stm32f10x_systick.h inc/stm32f10x_tim.h inc/stm32f10x_usart.h \
 inc/stm32f10x_wwdg.h
can.c: inc/stm32f10x_can.h inc/stm32f10x_map.h stm32f10x_conf.h \
 inc/stm32f10x_type.h inc/stm32f10x_type.h inc/cortexm3_macro.h \
 inc/stm32f10x_rcc.h inc/stm32f10x_gpio.h can.h inc/stm32f10x_type.h \
 protocol.h printf.h
encoder_adc.c: encoder.h inc/stm32f10x_type.h protocol.h \
 inc/stm32f10x_can.h inc/stm32f10x_map.h stm32f10x_conf.h \
 inc/stm32f10x_type.h inc/stm32f10x_type.h inc/cortexm3_macro.h \
 inc/stm32f10x_adc.h inc/stm32f10x_rcc.h inc/stm32f10x_nvic.h \
 inc/stm32f10x_gpio.h
encoder.c: inc/stm32f10x_tim.h inc/stm32f10x_map.h \
 stm32f10x_conf.h inc/stm32f10x_type.h inc/stm32f10x_type.h \
 inc/cortexm3_macro.h inc/stm32f10x_rcc.h inc/stm32f10x_gpio.h \
 inc/stm32f10x_exti.h inc/stm32f10x_nvic.h printf.h inc/stm32f10x_type.h \
 encoder.h protocol.h inc/stm32f10x_can.h inc/stm32f10x_spi.h \
 encoder_adc.h encoder_quadrature.h
spi.c: inc/stm32f10x_type.h stm32f10x_it.h inc/stm32f10x_lib.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h inc/stm32f10x_adc.h \
 inc/stm32f10x_bkp.h inc/stm32f10x_can.h inc/stm32f10x_crc.h \
 inc/stm32f10x_dac.h inc/stm32f10x_dbgmcu.h inc/stm32f10x_dma.h \
 inc/stm32f10x_exti.h inc/stm32f10x_flash.h inc/stm32f10x_fsmc.h \
 inc/stm32f10x_gpio.h inc/stm32f10x_i2c.h inc/stm32f10x_iwdg.h \
 inc/stm32f10x_nvic.h inc/stm32f10x_pwr.h inc/stm32f10x_rcc.h \
 inc/stm32f10x_rtc.h inc/stm32f10x_sdio.h inc/stm32f10x_spi.h \
 inc/stm32f10x_systick.h inc/stm32f10x_tim.h inc/stm32f10x_usart.h \
 inc/stm32f10x_wwdg.h inc/stm32f10x_gpio.h spi.h
state.c: state.h inc/stm32f10x_type.h protocol.h \
 inc/stm32f10x_can.h inc/stm32f10x_map.h stm32f10x_conf.h \
 inc/stm32f10x_type.h inc/stm32f10x_type.h inc/cortexm3_macro.h printf.h
stm32f10x_vector.c: inc/stm32f10x_lib.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h inc/stm32f10x_adc.h \
 inc/stm32f10x_bkp.h inc/stm32f10x_can.h inc/stm32f10x_crc.h \
 inc/stm32f10x_dac.h inc/stm32f10x_dbgmcu.h inc/stm32f10x_dma.h \
 inc/stm32f10x_exti.h inc/stm32f10x_flash.h inc/stm32f10x_fsmc.h \
 inc/stm32f10x_gpio.h inc/stm32f10x_i2c.h inc/stm32f10x_iwdg.h \
 inc/stm32f10x_nvic.h inc/stm32f10x_pwr.h inc/stm32f10x_rcc.h \
 inc/stm32f10x_rtc.h inc/stm32f10x_sdio.h inc/stm32f10x_spi.h \
 inc/stm32f10x_systick.h inc/stm32f10x_tim.h inc/stm32f10x_usart.h \
 inc/stm32f10x_wwdg.h stm32f10x_it.h
encoder_quadrature.c: encoder.h inc/stm32f10x_type.h \
 protocol.h inc/stm32f10x_can.h inc/stm32f10x_map.h stm32f10x_conf.h \
 inc/stm32f10x_type.h inc/stm32f10x_type.h inc/cortexm3_macro.h \
 inc/stm32f10x_tim.h inc/stm32f10x_rcc.h inc/stm32f10x_nvic.h \
 inc/stm32f10x_gpio.h printf.h
printf.c: usart.h inc/stm32f10x_type.h inc/stm32f10x_usart.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/cortexm3_macro.h
i2c.c: i2c.h inc/stm32f10x_type.h inc/stm32f10x_i2c.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/stm32f10x_type.h inc/cortexm3_macro.h stm32f10x_it.h \
 inc/stm32f10x_lib.h inc/stm32f10x_map.h inc/stm32f10x_adc.h \
 inc/stm32f10x_bkp.h inc/stm32f10x_can.h inc/stm32f10x_crc.h \
 inc/stm32f10x_dac.h inc/stm32f10x_dbgmcu.h inc/stm32f10x_dma.h \
 inc/stm32f10x_exti.h inc/stm32f10x_flash.h inc/stm32f10x_fsmc.h \
 inc/stm32f10x_gpio.h inc/stm32f10x_i2c.h inc/stm32f10x_iwdg.h \
 inc/stm32f10x_nvic.h inc/stm32f10x_pwr.h inc/stm32f10x_rcc.h \
 inc/stm32f10x_rtc.h inc/stm32f10x_sdio.h inc/stm32f10x_spi.h \
 inc/stm32f10x_systick.h inc/stm32f10x_tim.h inc/stm32f10x_usart.h \
 inc/stm32f10x_wwdg.h printf.h
checksum1.c:
controllers.c: pid.h inc/stm32f10x_type.h protocol.h \
 inc/stm32f10x_type.h inc/stm32f10x_can.h inc/stm32f10x_map.h \
 stm32f10x_conf.h inc/stm32f10x_type.h inc/cortexm3_macro.h encoder.h \
 state.h
pid.c: pid.h inc/stm32f10x_type.h stm32f10x_conf.h protocol.h \
 inc/stm32f10x_type.h inc/stm32f10x_can.h inc/stm32f10x_map.h \
 stm32f10x_conf.h inc/stm32f10x_type.h inc/cortexm3_macro.h can.h
usart.c: usart.h inc/stm32f10x_type.h inc/stm32f10x_usart.h \
 inc/stm32f10x_map.h stm32f10x_conf.h inc/stm32f10x_type.h \
 inc/cortexm3_macro.h inc/stm32f10x_usart.h inc/stm32f10x_rcc.h \
 inc/stm32f10x_map.h inc/stm32f10x_gpio.h
protocol.c: protocol.h inc/stm32f10x_type.h \
 inc/stm32f10x_can.h inc/stm32f10x_map.h stm32f10x_conf.h \
 inc/stm32f10x_type.h inc/stm32f10x_type.h inc/cortexm3_macro.h state.h \
 encoder.h inc/stm32f10x_gpio.h printf.h
hbridge.c: inc/stm32f10x_tim.h inc/stm32f10x_map.h \
 stm32f10x_conf.h inc/stm32f10x_type.h inc/stm32f10x_type.h \
 inc/cortexm3_macro.h inc/stm32f10x_rcc.h inc/stm32f10x_adc.h \
 inc/stm32f10x_gpio.h hbridge.h current_measurement.h \
 inc/stm32f10x_type.h
gen_checksum.c:
lm73cimk.c: lm73cimk.h inc/stm32f10x_type.h i2c.h \
 inc/stm32f10x_i2c.h inc/stm32f10x_map.h stm32f10x_conf.h \
 inc/stm32f10x_type.h inc/stm32f10x_type.h inc/cortexm3_macro.h printf.h \
 inc/stm32f10x_rcc.h inc/stm32f10x_nvic.h inc/stm32f10x_gpio.h
