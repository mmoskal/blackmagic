CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

OPT_FLAGS = -Os
CFLAGS += -mcpu=cortex-m3 -mthumb \
	-DSTM32F1 -DDISCOVERY_STLINK -I../libopencm3/include \
	-I platforms/stm32
LDFLAGS_BOOT := $(LDFLAGS) --specs=nano.specs \
	-lopencm3_stm32f1 -Wl,--defsym,_stack=0x20005000 \
	-Wl,-T,platforms/stm32/stlink.ld -nostartfiles -lc \
	-Wl,-Map=mapfile -mthumb -mcpu=cortex-m3 -Wl,-gc-sections \
	-L../libopencm3/lib

ifeq ($(ENABLE_DEBUG), 1)
SYSLDFLAGS = --specs=rdimon.specs
else
SYSLDFLAGS = --specs=nosys.specs
endif

LDFLAGS = $(LDFLAGS_BOOT) -Wl,-Ttext=0x8002000 $(SYSLDFLAGS)
LDFLAGS_UF2 = $(LDFLAGS_BOOT) -Wl,-Ttext=0x8004000 $(SYSLDFLAGS)

VPATH += platforms/stm32

SRC += 	cdcacm.c	\
	usbuart.c 	\
	serialno.c	\
	timing.c	\
	timing_stm32.c	\
	traceswoasync.c	\
	stlink_common.c \

all:	blackmagic.bin blackmagic.uf2


blackmagic.uf2: blackmagic_uf2.bin
	@echo "  UF2     $@"
	python ../scripts/uf2conv.py --base 0x8004000 --output $@ --family 0x5ee21072 --convert $<

blackmagic_uf2: include/version.h $(OBJ)
	@echo "  LD      $@"
	$(Q)$(CC) -o $@ $(OBJ) $(LDFLAGS_UF2)

host_clean:
	-$(Q)$(RM) blackmagic.bin blackmagic.uf2 blackmagic_uf2 blackmagic_uf2.bin

