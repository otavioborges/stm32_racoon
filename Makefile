ARCH ?= arm
CROSS_COMPILE ?= /opt/gcc-arm-none-eabi-7-2018-q2-update/bin/arm-none-eabi-

SRC = $(wildcard src/*.c)
SRC += $(wildcard hal/src/*.c)
SRC += $(wildcard BSP/src/*.c)
SRC += $(wildcard FatFs/src/*.c)

CPPSRC = $(wildcard geardrive/*.cpp)
CPPSRC += $(wildcard geardrive/GZ80/*.cpp)
CPPSRC += $(wildcard src/*.cpp)

SRCASM = $(wildcard src/*.s)
OBJS = $(patsubst %,build/%,$(subst .c,.o, $(SRC)))
OBJS += $(patsubst %,build/%,$(subst .s,.o, $(SRCASM)))
OBJS += $(patsubst %,build/%,$(subst .cpp,.o,$(CPPSRC)))

INCLUDE = -ICMSIS/ -Iinc/ -Ihal/inc/ -IBSP/inc/ -IFatFs/inc

FLAGS = -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard
FLAGS += -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra -g3
CFLAGS  := $(FLAGS) -DSTM32F746xx -DUSE_DHCP -DLWIP_IPV4 -DLWIP_DHCP -std=gnu99 -Wno-missing-prototypes -Wno-missing-declarations -MMD -MP
LDFLAGS := $(FLAGS) -T linker.ld -Xlinker --gc-sections --specs=nosys.specs

all: dir build/bootloader.hex

dir:
	mkdir -p build/src build/hal/src build/BSP/src build/geardrive/GZ80 build/FatFs/src

build/%.o: %.s
	@echo "CC   " $^
	$(CROSS_COMPILE)g++ $(CFLAGS) $(INCLUDE) -c $^ -o $@

build/%.o: %.cpp
	@echo "C++  " $^
	$(CROSS_COMPILE)g++ $(FLAGS) -Iinc/ -Igeardrive/ -Igeardrive/GZ80 -c $^ -o $@

build/%.o: %.c
	@echo "CC   " $^
	$(CROSS_COMPILE)gcc $(CFLAGS) $(INCLUDE) -c $^ -o $@

build/bootloader.elf: $(OBJS)
	@echo "LD   " $^
	$(CROSS_COMPILE)g++ $(LDFLAGS) $^ -o $@

%.hex: %.elf
	$(CROSS_COMPILE)objcopy -O ihex $^ $@

clean:
	rm -f build/src/*.o build/hal/src/*.o build/BSP/src/*.o build/geardrive/*.o build/geardrive/GZ80/*.o build/FatFs/src/*.o build/*.elf build/*.hex build/*.elf
