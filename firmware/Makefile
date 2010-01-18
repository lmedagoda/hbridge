NAME   = stm103_stk_rom

CC      = arm-none-linux-gnueabi-gcc
LD      = arm-none-linux-gnueabi-ld -v
AR      = arm-none-linux-gnueabi-ar
AS      = arm-none-linux-gnueabi-as
CP      = arm-none-linux-gnueabi-objcopy
OD	= arm-none-linux-gnueabi-objdump

CFLAGS  =  -I ./ -I inc/ -c -fno-common -O1 -g -mcpu=cortex-m3 -mthumb -ggdb -Wall
AFLAGS  = -ahls -mapcs-32 -o crt.o
LFLAGS  = -L. -Tstm_h103_rom.ld -nostartfiles
CPFLAGS = -Obinary
ODFLAGS	= -S

LIB_OUT = libstm32fw.a

LIB_OBJS = $(sort \
 $(patsubst %.c,%.o,$(wildcard src/*.c)) \
 $(patsubst %.s,%.o,$(wildcard src/*.s)))

all: bin 

flash: all
	@openocd -f openocd.cfg		

# lib

$(LIB_OUT): $(LIB_OBJS)
	$(AR) $(ARFLAGS) $@ $(LIB_OBJS)

$(LIB_OBJS): $(wildcard *.h) $(wildcard inc/*.h) 

clean:
	-rm -f main.list main.out main.bin *.o *.a $(LIB_OBJS)

bin: main.out
	@ echo "...copying"
	$(CP) $(CPFLAGS) main.out main.bin
	$(OD) $(ODFLAGS) main.out > main.list

main.out: $(LIB_OUT) main.o stm32f10x_vector.o stm32f10x_it.o stm_h103_rom.ld \
	i2c.o cortexm3_macro.o spi.o pid.o current_measurement.o usart.o printf.o can.o \
	controllers.o protocol.o state.o hbridge.o encoder.o lm73cimk.o
	@ echo "..linking"
	$(LD) $(LFLAGS) -o main.out main.o stm32f10x_it.o stm32f10x_vector.o \
	i2c.o current_measurement.o pid.o usart.o printf.o can.o controllers.o protocol.o \
	spi.o cortexm3_macro.o state.o hbridge.o encoder.o lm73cimk.o -lstm32fw

stm32f10x_vector.o: stm32f10x_vector.c
	$(CC) $(CFLAGS) stm32f10x_vector.c

stm32f10x_it.o: stm32f10x_it.c
	$(CC) $(CFLAGS) stm32f10x_it.c

printf.o: printf.c
	$(CC) $(CFLAGS) printf.c

lm73cimk.o: lm73cimk.c
	$(CC) $(CFLAGS) lm73cimk.c

i2c.o: i2c.c
	$(CC) $(CFLAGS) i2c.c

state.o: state.c
	$(CC) $(CFLAGS) state.c

encoder.o: encoder.c
	$(CC) $(CFLAGS) encoder.c

hbridge.o: hbridge.c
	$(CC) $(CFLAGS) hbridge.c

current_measurement.o: current_measurement.c
	$(CC) $(CFLAGS) current_measurement.c

spi.o: spi.c spi.h
	$(CC) $(CFLAGS) spi.c

pid.o: pid.c pid.h
	$(CC) $(CFLAGS) pid.c

usart.o: usart.c
	$(CC) $(CFLAGS) usart.c

can.o: can.c
	$(CC) $(CFLAGS) can.c

protocol.o: protocol.c
	$(CC) $(CFLAGS) protocol.c

main.o: main.c
	$(CC) $(CFLAGS) main.c

controllers.o: controllers.c
	$(CC) $(CFLAGS) controllers.c

cortexm3_macro.o: cortexm3_macro.s
	$(CC) $(CFLAGS) cortexm3_macro.s
