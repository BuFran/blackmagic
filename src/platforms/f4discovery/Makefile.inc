CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

CFLAGS += -Istm32/include -mcpu=cortex-m4 -mthumb \
	-mfloat-abi=hard -mfpu=fpv4-sp-d16 \
	-DSTM32F4 -DF4DISCOVERY -I../libopencm3/include \
	-Iplatforms/stm32

LDFLAGS = -lopencm3_stm32f4 -Wl,--defsym,_stack=0x20006000 \
	-Wl,-T,platforms/stm32/f4discovery.ld -nostartfiles -lc -lnosys \
	-Wl,-Map=mapfile -mthumb -mcpu=cortex-m4 -Wl,-gc-sections \
	-mfloat-abi=hard -mfpu=fpv4-sp-d16 \
	-L../libopencm3/lib

VPATH += platforms/stm32

SRC += 	cdcacm.c	\
	platform.c	\
	traceswo.c	\
	usbuart.c	\

all:	blackmagic.bin

blackmagic.bin:	blackmagic
	$(OBJCOPY) -O binary $^ $@

host_clean:
	-rm blackmagic.bin
