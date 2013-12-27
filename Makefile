
TARGET = STM32F4_FreeRTOS
EXECUTABLE= $(TARGET).elf
BIN_IMAGE = $(TARGET).bin
OBJDIR = obj

ARCH = CM3

LIB_STM = ./libstm
FREERTOS_SRC = $(LIB_STM)/FreeRTOS
FREERTOS_INC = $(FREERTOS_SRC)/include/
FREERTOS_PORT_INC = $(FREERTOS_SRC)/portable/GCC/ARM_$(ARCH)/

# Define programs and commands
TOOLCHAIN = arm-none-eabi
CC        = $(TOOLCHAIN)-gcc
GDB       = $(TOOLCHAIN)-gdb
OBJCOPY   = $(TOOLCHAIN)-objcopy
OBJDUMP   = $(TOOLCHAIN)-objdump
SIZE      = $(TOOLCHAIN)-size
NM        = $(TOOLCHAIN)-nm
OPENOCD   = openocd
DOXYGEN   = doxygen

C_LIB= $(TOOLCHAIN)/lib/thumb2

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
CFLAGS+=-I./inc

#include RTOS
CFLAGS+=-I$(FREERTOS_INC)
CFLAGS+=-I$(FREERTOS_PORT_INC)

#Source Files
SRC += system_stm32f4xx.c \
	   startup_stm32f4xx.s\
	   src/string.c \
	   src/servo.c \
	   $(LIB_STM)/Utilities/STM32F4-Discovery/stm32f4_discovery.c \
	   $(FREERTOS_SRC)/tasks.c \
	   $(FREERTOS_SRC)/list.c  \
	   $(FREERTOS_SRC)/portable/MemMang/heap_1.c \
	   $(FREERTOS_SRC)/portable/GCC/ARM_$(ARCH)/port.c

# Semihost Option
ifeq ($(USE_SEMIHOST),YES)
    CFLAGS  += -DUSE_SEMIHOST
    CC      += -T generic-m-hosted.ld
endif

all: $(BIN_IMAGE) showsize

libstm_build:
	$(MAKE) -C libstm/STM32F4xx_StdPeriph_Driver/build

$(BIN_IMAGE): $(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@

$(EXECUTABLE): main.c $(SRC)
	$(CC) $(CFLAGS) $^ -o $@  -L./libstm/STM32F4xx_StdPeriph_Driver/build \
		-lSTM32F4xx_StdPeriph_Driver -L$(C_LIB)

# Show the final program size
showsize: $(EXECUTABLE)
	@echo
	@$(SIZE) $(TARGET).elf 2>/dev/null

clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)

flash:
	st-flash write $(BIN_IMAGE) 0x8000000

# Create object file directories
#$(shell mkdir -p $(OBJDIR) 2>/dev/null)

# Compile: create object files from C source files
$(OBJDIR)/%.o : %.c
	@echo
	@echo Compiling C: $<
	$(CC) -c $(CPPFLAGS) $(CFLAGS) $(GENDEPFLAGS) $< -o $@ 

# Assemble: create object files from assembler source files
$(OBJDIR)/%.o : %.s
	@echo
	@echo Assembling: $<
	$(CC) -c $(CPPFLAGS) $(ASFLAGS) $< -o $@

.PHONY: all clean
