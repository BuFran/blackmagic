CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

ARCH_FLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16

CFLAGS += -DSTM32F4 -DF4DISCOVERY -I../libopencm3/include \
	-Iplatforms/stm32 -Istm32/include \

LDFLAGS_BOOT := $(LDFLAGS) -lopencm3_stm32f4 -Wl,--defsym,_stack=0x20020000 \
	-Wl,-T,platforms/stm32/f4discovery.ld -nostartfiles -lc -lnosys \
	-mthumb -mcpu=cortex-m4 -Wl,-gc-sections \
	-mfloat-abi=hard -mfpu=fpv4-sp-d16 \
	-L../libopencm3/lib

# start missing !
LDFLAGS = $(LDFLAGS_BOOT)

CFLAGS += $(ARCH_FLAGS)
LDFLAGS += $(ARCH_FLAGS) -Wl,-Map=blackmagic.map
LDFLAGS_BOOT += $(ARCH_FLAGS) -Wl,-Map=blackmagic_dfu.map

VPATH += platforms/stm32

SRC += 	cdcacm.c	\
	platform.c	\
	traceswo.c	\
	usbuart.c	\

# DFU for F4 not tested !
DFUSRC += usbdfu.c	\
	dfucore.c	\
	dfu_f4.c	\

all:	blackmagic.bin blackmagic.hex

blackmagic: blackmagic.elf

host_clean:
	-rm -f blackmagic.elf blackmagic.bin blackmagic.hex blackmagic.map
