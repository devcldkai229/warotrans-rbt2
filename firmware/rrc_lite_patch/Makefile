# GCC build of the RRC Lite firmware (FW-25 + WaroTrans patch). Run inside the FW-25 source tree:
#   make -f <this dir>/Makefile PATCH_DIR=<this dir>
PATCH_DIR ?= $(dir $(lastword $(MAKEFILE_LIST)))
TARGET = rrc_lite_warotrans
BUILD = build_gcc

PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size

include sources.mk

MCU = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFS = -DUSE_HAL_DRIVER -DSTM32F407xx
CFLAGS = $(MCU) $(DEFS) $(C_INCLUDES) -O2 -g -std=gnu11 -Wall -Wno-unused-variable -Wno-unused-function \
         -Wno-unused-but-set-variable -Wno-missing-braces -fdata-sections -ffunction-sections -fno-strict-aliasing
ASFLAGS = $(MCU) $(DEFS) -Wall
LDSCRIPT = $(PATCH_DIR)STM32F407VETx_FLASH.ld
LDFLAGS = $(MCU) -specs=nano.specs -specs=nosys.specs -T$(LDSCRIPT) -Wl,-Map=$(BUILD)/$(TARGET).map,--cref \
          -Wl,--gc-sections -Wl,--print-memory-usage -lc -lm -lnosys

OBJECTS = $(addprefix $(BUILD)/,$(C_SOURCES:.c=.o)) $(addprefix $(BUILD)/,$(ASM_SOURCES:.s=.o))

all: $(BUILD)/$(TARGET).bin

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(CP) -O binary -S $< $@

clean:
	rm -rf $(BUILD)

.PHONY: all clean
