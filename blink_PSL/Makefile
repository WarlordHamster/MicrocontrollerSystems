
# Ahmet Onat 2023

# The files that will be produced have these names:
TARGET = blink_PSL
HEX_FILE=$(TARGET).hex
ELF_FILE=$(TARGET).elf
BIN_FILE=$(TARGET).bin
LST_FILE=$(TARGET).lst
MAP_FILE=$(TARGET).map


# Build directory
BUILD_DIR = build

# Main source directories (Insert your own installation directory):
LIBPATH=/home/tomek/MICRO/MicrocontrollerSystems/Xpresso_SDK
LPC824DIR=$(LIBPATH)/devices/LPC824/

# C sources
C_SOURCES = gpio_led_output.c
C_SOURCES += system_LPC824.c
# drivers from the Processor Support Libraries:
C_SOURCES += fsl_common_arm.c
C_SOURCES += fsl_gpio.c
C_SOURCES += fsl_clock.c
C_SOURCES += fsl_reset.c
C_SOURCES += fsl_swm.c
#C_SOURCES += fsl_power.c

# ASM sources
ASM_SOURCES = startup_LPC824.S


#Tools:
CC = arm-none-eabi-gcc
AS = arm-none-eabi-gcc -x assembler-with-cpp
CP = arm-none-eabi-objcopy
LD = arm-none-eabi-ld
SZ = arm-none-eabi-size


ASM_DEFINES = -DNDEBUG -D__STARTUP_CLEAR_BSS 

ASM_FLAGS  = $(ASM_DEFINES)
ASM_FLAGS += -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft   
ASM_FLAGS += -Wall


C_DEFINES  = -DNDEBUG
C_DEFINES += -DCPU_LPC824 -DCPU_LPC824M201JHI33
C_DEFINES += -DMCUXPRESSO_SDK

C_INCLUDES  = -I.
C_INCLUDES += -I$(LPC824DIR)/
C_INCLUDES += -I$(LPC824DIR)/drivers/
#C_INCLUDES += -I$(LPC824DIR)/utilities/str/
#C_INCLUDES += -I$(LPC824DIR)/utilities/debug_console_lite/
#C_INCLUDES += -I$(LIBPATH)/components/uart/
C_INCLUDES += -I$(LIBPATH)/CMSIS/Core/Include/


C_FLAGS = -mcpu=cortex-m0plus
# The following optimizes the code. It realizes that the delay is not doing
#  anything and deletes that part of the code...
#C_FLAGS  += -O3 -Os 
C_FLAGS += -Wall -mthumb -MMD -MP
C_FLAGS += -fno-common -ffunction-sections -fdata-sections
C_FLAGS += -mapcs
C_FLAGS += -std=gnu99
C_FLAGS += -mfloat-abi=soft
C_FLAGS += -Wall
C_FLAGS += $(C_DEFINES)
C_FLAGS += $(C_INCLUDES) 
C_FLAGS += -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(LST_FILE) 
## Dependency info:
#CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" 

# Add assembly listing of each compiled file to the build dir:
##CFLAGS += -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst))
# -Wa: pass options to assembler.
# -alms specifies:
# -a: turn on listings
# -d: omit debugging directives; n: omit forms processing
# -h: include high-level source
# -l: include assembly


# LD flags, linker script:
LDSCRIPT = LPC824_flash.ld

LDFLAGS  = $(C_FLAGS)
LDFLAGS += --specs=nano.specs --specs=nosys.specs
LDFLAGS += -T$(LDSCRIPT)

LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(MAP_FILE) 


# default action: build all
all: $(BUILD_DIR)/$(ELF_FILE) $(BUILD_DIR)/$(HEX_FILE) $(BUILD_DIR)/$(BIN_FILE)

# LPC *requires* a checksum inserted into the binary. This does it:
#CHECKSUM=/home/onat/elektronik/ARM/LPC/lpcpatchelf/lpcpatchelf
#CHECKSUM_FLAG=-f


#######################################
# Create the list of objects from source codes:
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))

# list of ASM program objects
# CAUTION: Take care with 'S' and 's' extension.
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.S=.o)))

# The paths of source files:
vpath %.c $(LPC824DIR)
vpath %.c $(LPC824DIR)/drivers/


# Rule to compile object files from c source files:
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(C_FLAGS) $< -o $@

# Rule to compile object files from assembly sources.
# CAUTION: take care with 'S' and 's' extension.
$(BUILD_DIR)/%.o: %.S Makefile #| $(BUILD_DIR)
	$(AS) -c $(C_FLAGS) $< -o $@
	echo $(OBJECTS)

$(BUILD_DIR)/$(ELF_FILE): $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@


$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(CP) -O ihex $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(CP) -O binary -S $< $@	

$(BUILD_DIR):
	mkdir $@

# Before flashing, make sure HEX file is up to date:
21flash: $(BUILD_DIR)/$(HEX_FILE)
	lpc21isp -control $(BUILD_DIR)/$(HEX_FILE) /dev/ttyUSB0 115200 12000

clean:
	@rm -f $(OBJECTS) $(OBJECTS:.o=.d) $(OBJECTS:.o=.lst) $(BUILD_DIR)/$(ELF_FILE) $(BUILD_DIR)/$(BIN_FILE) $(BUILD_DIR)/$(LST_FILE) $(BUILD_DIR)/$(MAP_FILE)

# Dependencies
-include $(wildcard $(BUILD_DIR)/*.d)



