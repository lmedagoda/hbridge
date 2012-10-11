NAME   = stm103_stk_rom

CC      = arm-none-linux-gnueabi-gcc
LD      = arm-none-linux-gnueabi-ld -v
AR      = arm-none-linux-gnueabi-ar
AS      = arm-none-linux-gnueabi-as
CP      = arm-none-linux-gnueabi-objcopy
OD	= arm-none-linux-gnueabi-objdump

HOST_CC = gcc
HOST_CFLAGS = -Wall

CFLAGS  =  -I ./ \
	-I drivers/ \
	-I lib/STM32F10x_StdPeriph_Driver/inc \
	-I lib/STM32F10x_StdPeriph_Driver/ \
	-I lib/CMSIS/CM3/CoreSupport \
	-I lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/ \
	-c -fno-common -O3 -g -mcpu=cortex-m3 -mthumb -ggdb -Wall -fno-unwind-tables \
	-D STM32F10X_MD -D USE_STDPERIPH_DRIVER -D HSE_VALUE=16000000
AFLAGS  = -ahls -mapcs-32 -o crt.o
LFLAGS  = -L. -Tstm_h103_rom.ld -nostartfiles
CPFLAGS = -Obinary
ODFLAGS	= -S

LIB_OUT = libstm32fw.a

LIB_OBJS = $(sort \
 $(patsubst %.c,%.o,$(wildcard lib/STM32F10x_StdPeriph_Driver/src/*.c)) \
 $(patsubst %.c,%.o,$(wildcard lib/CMSIS/CM3/CoreSupport/*.c)) \
 $(patsubst %.c,%.o,$(wildcard lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/*.c)) \
 $(patsubst %.s,%.o,$(wildcard lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_md.s)))

DRIVER_OBJS = $(sort \
 $(patsubst %.c,%.o,$(wildcard drivers/*.c)))

DRIVER_OUT = libstm32drv.a

OBJS = main_pcbv2.o stm32f10x_it.o systick.o\
	current_measurement.o controllers.o \
	protocol.o state.o hbridgev2.o \
	position_controller.o speed_controller.o \
	encoder.o encoder_ichaus.o encoder_quadrature.o encoder_adc.o

all: bin

flash: all
	@openocd -f openocd.cfg

# lib

$(LIB_OUT): $(LIB_OBJS)
	$(AR) $(ARFLAGS) $@ $(LIB_OBJS)

$(LIB_OBJS): stm32f10x_it.h stm32f10x_conf.h $(wildcard lib/STM32F10x_StdPeriph_Driver/inc/*.h)

$(DRIVER_OUT): $(DRIVER_OBJS)
	$(AR) $(ARFLAGS) $@ $(DRIVER_OBJS)

$(DRIVER_OBJS): stm32f10x_it.h stm32f10x_conf.h $(wildcard lib/STM32F10x_StdPeriph_Driver/inc/*.h)

clean:
	-rm -f main.list main.out main.bin main1.out main1.bin \
	*.o *.a $(LIB_OBJS) $(DRIVER_OBJS)

bin: main.bin main.list

main.bin: main.out
	@ echo "...copying"
	$(CP) $(CPFLAGS) $< $@

main.list: main.out
	$(OD) $(ODFLAGS) $< > $@ || rm $@

main.out: $(OBJS) $(LIB_OUT) $(DRIVER_OUT) stm_h103_rom.ld
	@ echo "..linking"
	$(LD) $(LFLAGS) -o $@ $(OBJS) -lstm32fw -lstm32drv -lstm32fw 

.PHONY: bin clean

#generated using 
# find . -name "*.c" -exec gcc -I inc -I . -MM {} -MQ {} \; | sed -r "s/[ ].+\\.c//" >> Makefile
#and some manual fixups

