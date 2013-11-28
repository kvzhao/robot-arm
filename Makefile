EXECUTABLE= STM32F4_FreeRTOS.elf
BIN_IMAGE = STM32F4_FreeRTOS.bin


CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
ARCH=CM3

LIB_STM = ./libstm
FREERTOS_SRC = $(LIB_STM)/FreeRTOS
FREERTOS_INC = $(FREERTOS_SRC)/include/
FREERTOS_PORT_INC = $(FREERTOS_SRC)/portable/GCC/ARM_$(ARCH)/

TOOLCHAIN_PATH ?= /usr/local/csl/arm-2012.03/arm-none-eabi
C_LIB= $(TOOLCHAIN_PATH)/lib/thumb2

CFLAGS=-g -O2 -mlittle-endian -mthumb
CFLAGS+=-mcpu=cortex-m4
CFLAGS+=-ffreestanding -nostdlib
CFLAGS+= -msoft-float

# to run from FLASH
CFLAGS+=-Wl,-T,stm32_flash.ld

CFLAGS+=-I./

# stm32f4_discovery lib
CFLAGS+=-I./libstm/STM32F4xx_StdPeriph_Driver/inc
CFLAGS+=-I./libstm/STM32F4xx_StdPeriph_Driver/inc/device_support
CFLAGS+=-I./libstm/STM32F4xx_StdPeriph_Driver/inc/core_support

#STM32F4xx_StdPeriph_Driver\inc
CFLAGS+=-I./libstm/STM32F4xx_StdPeriph_Driver/inc

#Utilities
CFLAGS+=-I./libstm/Utilities/STM32F4-Discovery

#include RTOS
CFLAGS+=-I$(FREERTOS_INC)
CFLAGS+=-I$(FREERTOS_PORT_INC)

# LCD 3.2'TFT SSD1289 Driver
CFLAGS+=-I./tft_driver/inc

#Source Files
SRC += system_stm32f4xx.c \
	   startup_stm32f4xx.s\
	   string.c \
	   tft_driver/src/LCD_STM32F4.c \
	   tft_driver/src/fonts.c \
	   $(LIB_STM)/Utilities/STM32F4-Discovery/stm32f4_discovery.c \
	   $(FREERTOS_SRC)/tasks.c $(FREERTOS_SRC)/list.c $(FREERTOS_SRC)/portable/MemMang/heap_1.c \
	   $(FREERTOS_SRC)/portable/GCC/ARM_$(ARCH)/port.c

all: $(BIN_IMAGE)

libstm_build:
	$(MAKE) -C libstm/STM32F4xx_StdPeriph_Driver/build

$(BIN_IMAGE): $(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@

$(EXECUTABLE): main.c $(SRC)
	$(CC) $(CFLAGS) $^ -o $@  -L./libstm/STM32F4xx_StdPeriph_Driver/build \
		-lSTM32F4xx_StdPeriph_Driver -L$(C_LIB)

clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)

flash:
	st-flash write $(BIN_IMAGE) 0x8000000

.PHONY: all clean
