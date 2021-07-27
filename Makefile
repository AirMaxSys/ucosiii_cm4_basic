##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [3.10.0-B14] date: [Mon Jul 26 16:21:45 CST 2021]
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = ucosiii_cm4


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Core/Src/main.c \
Core/Src/stm32l4xx_it.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_utils.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_exti.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_gpio.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_pwr.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_tim.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_dma.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usart.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_rcc.c \
Core/Src/system_stm32l4xx.c  \
Utils/print.c	\
RTOS/uC-OS3/Cfg/Template/os_app_hooks.c	\
RTOS/uC-OS3/Ports/ARM-Cortex-M/ARMv7-M/os_cpu_c.c	\
RTOS/uC-OS3/Source/os_cfg_app.c	\
RTOS/uC-OS3/Source/os_core.c	\
RTOS/uC-OS3/Source/os_dbg.c	\
RTOS/uC-OS3/Source/os_flag.c	\
RTOS/uC-OS3/Source/os_mem.c	\
RTOS/uC-OS3/Source/os_msg.c	\
RTOS/uC-OS3/Source/os_mutex.c	\
RTOS/uC-OS3/Source/os_prio.c	\
RTOS/uC-OS3/Source/os_q.c	\
RTOS/uC-OS3/Source/os_sem.c	\
RTOS/uC-OS3/Source/os_stat.c	\
RTOS/uC-OS3/Source/os_task.c	\
RTOS/uC-OS3/Source/os_tick.c	\
RTOS/uC-OS3/Source/os_time.c	\
RTOS/uC-OS3/Source/os_tmr.c	\
RTOS/uC-OS3/Source/os_var.c	\
RTOS/uC-CPU/ARM-Cortex-M4/ARMv7-M/cpu_c.c	\
RTOS/uC-CPU/cpu_core.c	\
RTOS/uC-LIB/lib_ascii.c	\
RTOS/uC-LIB/lib_math.c	\
RTOS/uC-LIB/lib_mem.c	\
RTOS/uC-LIB/lib_str.c

# ASM sources
ASM_SOURCES =  \
startup_stm32l475xx.s	\
RTOS/uC-CPU/ARM-Cortex-M4/ARMv7-M/GNU/cpu_a.s	\
RTOS/uC-OS3/Ports/ARM-Cortex-M/ARMv7-M/GNU/os_cpu_a.s

#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
# Generage .map
NM = $(PREFIX)nm -n -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_FULL_LL_DRIVER \
-DHSE_VALUE=8000000 \
-DHSE_STARTUP_TIMEOUT=100 \
-DLSE_STARTUP_TIMEOUT=5000 \
-DLSE_VALUE=32768 \
-DMSI_VALUE=4000000 \
-DEXTERNALSAI1_CLOCK_VALUE=2097000 \
-DEXTERNALSAI2_CLOCK_VALUE=2097000 \
-DHSI_VALUE=16000000 \
-DLSI_VALUE=32000 \
-DVDD_VALUE=3300 \
-DPREFETCH_ENABLE=0 \
-DINSTRUCTION_CACHE_ENABLE=1 \
-DDATA_CACHE_ENABLE=1 \
-DSTM32L475xx


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-ICore/Inc \
-IDrivers/STM32L4xx_HAL_Driver/Inc \
-IDrivers/CMSIS/Device/ST/STM32L4xx/Include \
-IDrivers/CMSIS/Include \
-IDrivers/CMSIS/Include	\
-IRTOS/uC-CPU	\
-IRTOS/uC-CPU/ARM-Cortex-M4/ARMv7-M/GNU	\
-IRTOS/uC-CPU/Cfg/Template	\
-IRTOS/uC-LIB	\
-IRTOS/uC-LIB/Cfg	\
-IRTOS/uC-OS3/Cfg/Template	\
-IRTOS/uC-OS3/Ports/ARM-Cortex-M/ARMv7-M/GNU	\
-IRTOS/uC-OS3/Source	\
-IUtils


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32L475VETx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin map


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# Generate .map file
#######################################
map:
	$(NM) $(BUILD_DIR)/$(TARGET).elf > $(BUILD_DIR)/System.map

#######################################
# clean up
#######################################
clean:
ifeq ($(OS), Windows_NT)
	-del /Q /f $(BUILD_DIR)
else
	-rm -rf $(BUILD_DIR)/*
endif

#######################################
# OpenOCD device and interface
#######################################
OO_TARGET=OpenOCD/target/stm32f4x.cfg
OO_INTERFACE=OpenOCD/interface/stlink.cfg

#######################################
# download image

# openocd -s search scripts or set
# OPENOCD_SCRIPTS environmet variable
#######################################
dwn:
	openocd	\
		-c "tcl_port disabled"	\
		-c "gdb_port 3333"	\
		-c "telnet_port 4444"	\
		-f "$(OO_TARGET)"	\
		-f "$(OO_INTERFACE)"	\
		-c "program $(BUILD_DIR)/$(TARGET).elf"	\
		-c "reset"	\
		-c "shutdown"

#######################################
# debug
#######################################
debug:
	openocd	\
		-c "tcl_port disabled"	\
		-c "gdb_port 3333"	\
		-c "telnet_port 4444"	\
		-f "$(OO_TARGET)"	\
		-f "$(OO_INTERFACE)"	\
		-c "program $(BUILD_DIR)/$(TARGET).elf"	\
		-c "init"	\
		-c "halt" 
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***