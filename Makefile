NAME   = stm103_stk_rom

CC      = arm-none-linux-gnueabi-gcc
LD      = arm-none-linux-gnueabi-ld -v
AR      = arm-none-linux-gnueabi-ar
AS      = arm-none-linux-gnueabi-as
CP      = arm-none-linux-gnueabi-objcopy
OD	= arm-none-linux-gnueabi-objdump

CFLAGS  =  -I ./ -I inc/ -I usb-core/ -I usb/ -c -fno-common -O0 -g -mcpu=cortex-m3 -mthumb -ggdb -Wall
AFLAGS  = -ahls -mapcs-32 -o crt.o
LFLAGS  = -L. -Tstm_h103_rom.ld -nostartfiles
CPFLAGS = -Obinary
ODFLAGS	= -S

LIB_OUT = libstm32fw.a

LIB_OBJS = $(sort \
 $(patsubst %.c,%.o,$(wildcard src/*.c)) \
 $(patsubst %.s,%.o,$(wildcard src/*.s)))

USB_CORE_OBJS = $(sort \
 $(patsubst %.c,%.o,$(wildcard usb-core/*.c)))

USB_OBJS = $(sort \
 $(patsubst %.c,%.o,$(wildcard usb/*.c)))


all: bin 

flash: all
	@openocd -f openocd.cfg		

# lib

$(LIB_OUT): $(LIB_OBJS)
	$(AR) $(ARFLAGS) $@ $(LIB_OBJS)

$(LIB_OBJS): $(wildcard *.h) $(wildcard inc/*.h) 


$(USB_CORE_OBJS): $(wildcard usb-core/*.h)

libusb-core.a: $(USB_CORE_OBJS)
	$(AR) $(ARFLAGS) $@ $(USB_CORE_OBJS)

$(USB_OBJS): $(wildcard usb/*.h)

libusb.a: $(USB_OBJS)
	$(AR) $(ARFLAGS) $@ $(USB_OBJS)

clean:
	-rm -f main.list main.out main.bin *.o *.a $(LIB_OBJS) \
	       $(USB_CORE_OBJS) $(USB_OBJS)

bin: main.out
	@ echo "...copying"
	$(CP) $(CPFLAGS) main.out main.bin
	$(OD) $(ODFLAGS) main.out > main.list

main.out: $(LIB_OUT) main.o stm32f10x_vector.o stm32f10x_it.o stm_h103_rom.ld \
	i2c.o md03.o cortexm3_macro.o controller_interface.o dummycontroller.o\
	 spi.o pid.o rc.o usb.o libusb-core.a libusb.a init.o
	@ echo "..linking"
	$(LD) $(LFLAGS) -o main.out main.o stm32f10x_it.o stm32f10x_vector.o \
	controller_interface.o dummycontroller.o init.o i2c.o md03.o pid.o \
	spi.o cortexm3_macro.o rc.o usb.o -lusb -lusb-core -lstm32fw

stm32f10x_vector.o: stm32f10x_vector.c
	$(CC) $(CFLAGS) stm32f10x_vector.c

stm32f10x_it.o: stm32f10x_it.c
	$(CC) $(CFLAGS) stm32f10x_it.c

controller_interface.o: controller_interface.c controller_interface.h
	$(CC) $(CFLAGS) controller_interface.c

dummycontroller.o: dummycontroller.c dummycontroller.h
	$(CC) $(CFLAGS) dummycontroller.c

init.o: init.c
	$(CC) $(CFLAGS) init.c
i2c.o: i2c.c
	$(CC) $(CFLAGS) i2c.c

spi.o: spi.c spi.h
	$(CC) $(CFLAGS) spi.c

rc.o: rc.c rc.h
	$(CC) $(CFLAGS) rc.c

pid.o: pid.c pid.h
	$(CC) $(CFLAGS) pid.c

md03.o: md03.c
	$(CC) $(CFLAGS) md03.c

usb.o: usb.c
	$(CC) $(CFLAGS) usb.c

main.o: main.c
	$(CC) $(CFLAGS) main.c

cortexm3_macro.o: cortexm3_macro.s
	$(CC) $(CFLAGS) cortexm3_macro.s
