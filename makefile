# Makefile for Motor Control Application

CC = gcc
CFLAGS = -Wall -Wextra -O2 -g
LDFLAGS = -lm -lrt -lpthread

# Build directory
BUILD_DIR = build

# SOEM source directories
SOEM_DIR = SOEM/soem
OSHW_DIR = SOEM/oshw/linux
OSAL_DIR = SOEM/osal
OSAL_LINUX_DIR = SOEM/osal/linux

# Include directories
INCLUDES = -Iinclude -I$(SOEM_DIR) -I$(OSHW_DIR) -I$(OSAL_DIR) -I$(OSAL_LINUX_DIR) -I.

# Build output directories
BUILD_SOEM_DIR = $(BUILD_DIR)/SOEM/soem
BUILD_OSHW_DIR = $(BUILD_DIR)/SOEM/oshw/linux
BUILD_OSAL_DIR = $(BUILD_DIR)/SOEM/osal/linux
BUILD_SRC_DIR = $(BUILD_DIR)/src
BUILD_ROOT_DIR = $(BUILD_DIR)

# SOEM object files
SOEM_OBJ = $(BUILD_SOEM_DIR)/ethercatbase.o \
           $(BUILD_SOEM_DIR)/ethercatcoe.o \
           $(BUILD_SOEM_DIR)/ethercatconfig.o \
           $(BUILD_SOEM_DIR)/ethercatdc.o \
           $(BUILD_SOEM_DIR)/ethercatfoe.o \
           $(BUILD_SOEM_DIR)/ethercatmain.o \
           $(BUILD_SOEM_DIR)/ethercatprint.o \
           $(BUILD_SOEM_DIR)/ethercatsoe.o

# OSHW object files
OSHW_OBJ = $(BUILD_OSHW_DIR)/nicdrv.o \
           $(BUILD_OSHW_DIR)/oshw.o

# OSAL object files
OSAL_OBJ = $(BUILD_OSAL_DIR)/osal.o

# Default to simulator if no target specified
CAN_OBJ = $(BUILD_ROOT_DIR)/can_monitor.o $(BUILD_ROOT_DIR)/can_simulator.o

# Application object files
APP_OBJ = $(BUILD_SRC_DIR)/main.o \
          $(BUILD_SRC_DIR)/ethercat_driver.o \
          $(BUILD_SRC_DIR)/cia402_state.o \
          $(BUILD_SRC_DIR)/motor_control.o \
          $(BUILD_SRC_DIR)/terminal_io.o

# All object files
OBJ = $(SOEM_OBJ) $(OSHW_OBJ) $(OSAL_OBJ) $(CAN_OBJ) $(APP_OBJ)

# Target executable
TARGET = motor_control

# Make options to reduce verbosity
MAKEFLAGS += --no-print-directory

all: create_build_dirs $(TARGET)
	@echo ""
	@echo "================================================================"
	@echo "  Build complete! The program is set to use simulated CAN."
	@echo "  To explicitly choose mode, use:"
	@echo "    make sim  - Use simulated CAN controller"
	@echo "    make real - Use real CAN hardware"
	@echo "================================================================"
	@echo ""

# Create build directories
create_build_dirs:
	@mkdir -p $(BUILD_SOEM_DIR)
	@mkdir -p $(BUILD_OSHW_DIR)
	@mkdir -p $(BUILD_OSAL_DIR)
	@mkdir -p $(BUILD_SRC_DIR)
	@mkdir -p $(BUILD_ROOT_DIR)

# Rule to build all SOEM library files
soem: create_build_dirs $(SOEM_OBJ) $(OSHW_OBJ) $(OSAL_OBJ)

# Compile SOEM
$(BUILD_SOEM_DIR)/%.o: $(SOEM_DIR)/%.c
	@echo "Compiling $<"
	@$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Compile OSHW
$(BUILD_OSHW_DIR)/%.o: $(OSHW_DIR)/%.c
	@echo "Compiling $<"
	@$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Compile OSAL
$(BUILD_OSAL_DIR)/%.o: $(OSAL_LINUX_DIR)/%.c
	@echo "Compiling $<"
	@$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Compile SocketCAN implementation (in src directory)
$(BUILD_SRC_DIR)/socketcan.o: src/socketcan.c include/socketcan.h
	@echo "Compiling $<"
	@$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Compile CAN simulator
$(BUILD_ROOT_DIR)/can_simulator.o: can_simulator.c include/socketcan.h
	@echo "Compiling $<"
	@$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Compile CAN monitor (in root directory)
$(BUILD_ROOT_DIR)/can_monitor.o: can_monitor.c include/can_monitor.h include/socketcan.h
	@echo "Compiling $<"
	@$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Compile application source files
$(BUILD_SRC_DIR)/%.o: src/%.c
	@echo "Compiling $<"
	@$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Link everything together
$(TARGET): $(OBJ)
	@echo "Linking $(TARGET)"
	@$(CC) -o $@ $(OBJ) $(LDFLAGS)

clean:
	@echo "Cleaning build files..."
	@rm -rf $(BUILD_DIR)
	@rm -f $(TARGET)

# Targets for explicitly choosing real or simulated CAN
real: 
	@echo "Building with REAL CAN hardware..."
	@$(MAKE) clean
	@$(MAKE) _real

_real: CAN_OBJ = $(BUILD_ROOT_DIR)/can_monitor.o $(BUILD_SRC_DIR)/socketcan.o
_real: create_build_dirs $(TARGET)
	@echo ""
	@echo "=========================================================="
	@echo "  Build complete! Using REAL CAN hardware."
	@echo "=========================================================="
	@echo ""

sim:
	@echo "Building with SIMULATED CAN..."
	@$(MAKE) clean
	@$(MAKE) _sim

_sim: CAN_OBJ = $(BUILD_ROOT_DIR)/can_monitor.o $(BUILD_ROOT_DIR)/can_simulator.o
_sim: create_build_dirs $(TARGET)
	@echo ""
	@echo "====================================================="
	@echo "  Build complete! Using SIMULATED CAN."
	@echo "====================================================="
	@echo ""

.PHONY: all clean soem create_build_dirs real _real sim _sim