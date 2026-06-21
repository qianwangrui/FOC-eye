# ------------------------------------------------------------------------------
# Makefile for FOC_eye  (NUCLEO-G431RB / STM32G431RBTx, arm-none-eabi-gcc)
# Usage:
#   make                # build .elf .hex .bin
#   make clean
#   make flash          # st-flash: FULL chip erase — also wipes NVM cal page
#   make flash-keep-cal # STM32CubeProgrammer: erase app pages only, keep page 63
#   make flash-cube     # CubeProgrammer full download (erases per tool settings)
#   make flash-ocd      # OpenOCD program
#   make flash-dap      # OpenOCD + CMSIS-DAP
# ------------------------------------------------------------------------------

TARGET      ?= FOC_eye
BUILD_DIR   ?= build
DEBUG       ?= 1
OPT         ?= -Og

# ----- Toolchain --------------------------------------------------------------
PREFIX      = arm-none-eabi-
CC          = $(PREFIX)gcc
AS          = $(PREFIX)gcc -x assembler-with-cpp
CP          = $(PREFIX)objcopy
SZ          = $(PREFIX)size

HEX         = $(CP) -O ihex
BIN         = $(CP) -O binary -S

# ----- MCU --------------------------------------------------------------------
CPU         = -mcpu=cortex-m4
FPU         = -mfpu=fpv4-sp-d16
FLOAT_ABI   = -mfloat-abi=hard
MCU         = $(CPU) -mthumb $(FPU) $(FLOAT_ABI)

# ----- Defines ----------------------------------------------------------------
C_DEFS = \
  -DSTM32G431xx \

AS_DEFS =

# ----- Includes ---------------------------------------------------------------
C_INCLUDES = \
  -IInc \
  -IAPP \
  -IDrivers/STM32G4xx_HAL_Driver/Inc \
  -IDrivers/STM32G4xx_HAL_Driver/Inc/Legacy \
  -IDrivers/CMSIS/Device/ST/STM32G4xx/Include \
  -IDrivers/CMSIS/Include

AS_INCLUDES =

# ----- Sources ----------------------------------------------------------------
# User application
C_SOURCES = \
  $(wildcard Src/*.c) \
  $(wildcard APP/*.c) \
  $(wildcard User/*.c)

# HAL driver sources (skip *_template.c)
HAL_SRC_DIR = Drivers/STM32G4xx_HAL_Driver/Src
C_SOURCES += $(filter-out %_template.c, $(wildcard $(HAL_SRC_DIR)/*.c))

# Startup
ASM_SOURCES = Startup/startup_stm32g431xx.s

# ----- Linker -----------------------------------------------------------------
LDSCRIPT    = STM32G431RBTX_FLASH.ld
LIBS        = -lc -lm -lnosys
LIBDIR      =
LDFLAGS     = $(MCU) -specs=nano.specs -u _printf_float -T$(LDSCRIPT) $(LIBDIR) $(LIBS) \
              -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
              -Wl,--gc-sections -static \
              -Wl,--start-group -lc -lm -Wl,--end-group

# ----- Compile flags ----------------------------------------------------------
ifeq ($(DEBUG), 1)
DEBUG_FLAGS = -g -gdwarf-2
endif

CFLAGS  = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections $(DEBUG_FLAGS)
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections $(DEBUG_FLAGS)

# ----- Objects ----------------------------------------------------------------
OBJECTS  = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# ----- Rules ------------------------------------------------------------------
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(LDSCRIPT) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

clean:
	-rm -fR $(BUILD_DIR)

flash: $(BUILD_DIR)/$(TARGET).bin
	@echo "NOTE: st-flash mass-erases chip — NVM cal at 0x0801F800 is lost."
	@echo "      Use 'make flash-keep-cal' to preserve calibration page."
	st-flash --reset write $< 0x08000000

# Erase flash pages 0..62 only, program firmware below NVM page 63 (0x0801F800).
flash-keep-cal: $(BUILD_DIR)/$(TARGET).bin
	STM32_Programmer_CLI -c port=SWD -e 0 62 -w $< 0x08000000 -v -rst

flash-cube: $(BUILD_DIR)/$(TARGET).elf
	STM32_Programmer_CLI -c port=SWD -w $< -rst

OPENOCD = /opt/openocd-0.12/bin/openocd

flash-ocd: $(BUILD_DIR)/$(TARGET).elf
	$(OPENOCD) -f interface/stlink.cfg -f target/stm32g4x.cfg -c "program $< verify reset exit"

flash-dap: $(BUILD_DIR)/$(TARGET).elf
	$(OPENOCD) -f interface/cmsis-dap.cfg -f target/stm32g4x.cfg -c "program $< verify reset exit"

# Open serial terminal (DAPLink VCP). Exit: Ctrl-T then Q
SERIAL_PORT ?= /dev/ttyACM0
SERIAL_BAUD ?= 115200
serial:
	tio -b $(SERIAL_BAUD) $(SERIAL_PORT)

-include $(wildcard $(BUILD_DIR)/*.d)

.PHONY: all clean flash flash-keep-cal flash-cube flash-ocd flash-dap serial
