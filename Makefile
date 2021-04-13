# This Makefile was tested with GNU Make
# Use pkg-config to lookup the proper compiler and linker flags for LCM

CFLAGS=`pkg-config --cflags lcm`
LDFLAGS=`pkg-config --libs lcm`

# target = test
skin_2 = posix
CC := $(shell xeno-config --cc) 
CFLAGS := $(shell xeno-config --skin=$(skin_2) --cflags) $(CFLAGS) 
LDFLAGS := $(shell xeno-config --skin=$(skin_2) --ldflags) $(LDFLAGS) -lpcanfd -lm
ifdef CAN_WRITE
CFLAGS += -DCAN_WRITE 
endif
ifdef DEBUG
CFLAGS += -DDEBUG -g
endif
ifdef WRITE_Q
CFLAGS += -DWRITE_Q
endif

# $(target): $(target).c
# 	$(CC) -o $@ $< $(CFLAGS) $(LDFLAGS)

all: motors_can \ imu_can

motors_can: spi_command_t.o spi_data_t.o motors_can.o
	$(CC) -o $@ $^ $(LDFLAGS)

imu_can: spi_command_t.o spi_data_t.o imu_can.o
	$(CC) -o $@ $^ $(LDFLAGS)

# prevent auto-generated lcm .c/.h files from being deleted
.SECONDARY : exlcm_example_t.c exlcm_example_t.h

%.o: %.c %.h
	$(CC) $(CFLAGS) -c $< 

%.c %.h: ./%.lcm
	lcm-gen -c $<

clean:
	rm -f motors_can imu_can
	rm -f *.o
	rm -f spi_data_t.c spi_data_t.h spi_command_t.h spi_command_t.c
