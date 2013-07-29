CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

ARCH_FLAGS = -mcpu=cortex-m3 -mthumb

CFLAGS += -DSTM32F1 -DBLACKMAGIC -I../libopencm3/include \
	-Iplatforms/stm32 -Istm32/include \

LDFLAGS_BOOT := $(LDFLAGS) -lopencm3_stm32f1 -Wl,--defsym,_stack=0x20005000 \
	-Wl,-T,platforms/stm32/blackmagic.ld -nostartfiles -lc -lnosys \
	-mthumb -mcpu=cortex-m3 -Wl,-gc-sections \
	-L../libopencm3/lib
LDFLAGS = $(LDFLAGS_BOOT) -Wl,-Ttext=0x8002000

CFLAGS += $(ARCH_FLAGS)
LDFLAGS += $(ARCH_FLAGS) -Wl,-Map=blackmagic.map
LDFLAGS_BOOT += $(ARCH_FLAGS) -Wl,-Map=blackmagic_dfu.map

VPATH += platforms/stm32

SRC += 	cdcacm.c	\
	platform.c	\
	traceswo.c	\
	usbuart.c	\

DFUSRC += usbdfu.c	\
	dfucore.c	\
	dfu_f1.c	\

all:	blackmagic.bin blackmagic.hex blackmagic_dfu.bin blackmagic_dfu.hex

blackmagic: blackmagic.elf

host_clean:
	-rm -f blackmagic.elf blackmagic.bin blackmagic.hex blackmagic.map
	-rm -f blackmagic_dfu.elf blackmagic_dfu.bin blackmagic_dfu.hex blackmagic_dfu.map

