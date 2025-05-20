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

# Application object files
APP_OBJ = $(BUILD_SRC_DIR)/main.o \
          $(BUILD_SRC_DIR)/hardware_io.o \
          $(BUILD_SRC_DIR)/motor_driver.o \
          $(BUILD_SRC_DIR)/can_interface.o

# All object files
OBJ = $(SOEM_OBJ) $(OSHW_OBJ) $(OSAL_OBJ) $(APP_OBJ)

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

# Build SOEM library files
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
	@$(MAKE) CAN_MODE=REAL create_build_dirs $(TARGET)
	@echo ""
	@echo "=========================================================="
	@echo "  Build complete! Using REAL CAN hardware."
	@echo "=========================================================="
	@echo ""

sim:
	@echo "Building with SIMULATED CAN..."
	@$(MAKE) clean
	@$(MAKE) CAN_MODE=SIMULATOR create_build_dirs $(TARGET)
	@echo ""
	@echo "====================================================="
	@echo "  Build complete! Using SIMULATED CAN."
	@echo "====================================================="
	@echo ""

# Define CAN mode for build
ifneq ($(CAN_MODE),)
    CFLAGS += -DCAN_MODE_$(CAN_MODE)
endif

.PHONY: all clean soem create_build_dirs real sim