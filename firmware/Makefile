NAME   = stm103_stk_rom

CC      = arm-none-linux-gnueabi-gcc
LD      = arm-none-linux-gnueabi-ld -v
AR      = arm-none-linux-gnueabi-ar
AS      = arm-none-linux-gnueabi-as
CP      = arm-none-linux-gnueabi-objcopy
OD	= arm-none-linux-gnueabi-objdump

HOST_CC = gcc
HOST_CFLAGS = -Wall

CFLAGS  =  -I ./ -I inc/ -c -fno-common -O1 -g -mcpu=cortex-m3 -mthumb -ggdb -Wall
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
	protocol.o spi.o cortexm3_macro.o state.o hbridge.o encoder.o \
	lm73cimk.o

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
