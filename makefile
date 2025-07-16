CC = gcc
CFLAGS = -Wall -Wextra -O2 -g -DCAN_MODE_REAL
LDFLAGS = -lm -lrt -lpthread

# Build directory
BUILD_DIR = build

# SOEM source directories
SOEM_DIR = SOEM/soem
OSHW_DIR = SOEM/oshw/linux
OSAL_DIR = SOEM/osal
OSAL_LINUX_DIR = SOEM/osal/linux

# inih directory
INIH_DIR = inih

# Include directories
INCLUDES = -Iinclude -I$(SOEM_DIR) -I$(OSHW_DIR) -I$(OSAL_DIR) -I$(OSAL_LINUX_DIR) -I$(INIH_DIR) -I.

# Build output directories
BUILD_SOEM_DIR = $(BUILD_DIR)/SOEM/soem
BUILD_OSHW_DIR = $(BUILD_DIR)/SOEM/oshw/linux
BUILD_OSAL_DIR = $(BUILD_DIR)/SOEM/osal/linux
BUILD_SRC_DIR = $(BUILD_DIR)/src
BUILD_INIH_DIR = $(BUILD_DIR)/inih

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

# inih object files
INIH_OBJ = $(BUILD_INIH_DIR)/ini.o

# Application object files - base set
APP_OBJ = $(BUILD_SRC_DIR)/main.o \
          $(BUILD_SRC_DIR)/hardware_io.o \
          $(BUILD_SRC_DIR)/motor_driver.o \
          $(BUILD_SRC_DIR)/can_interface.o \
          $(BUILD_SRC_DIR)/config.o \
          $(BUILD_SRC_DIR)/data_logger.o \
          $(BUILD_SRC_DIR)/realtime_broadcaster.o

# All object files
OBJ = $(SOEM_OBJ) $(OSHW_OBJ) $(OSAL_OBJ) $(INIH_OBJ) $(APP_OBJ)

# Target executable
TARGET = motor_control

# Make options to reduce verbosity
MAKEFLAGS += --no-print-directory

all: 
	@$(MAKE) clean
	@$(MAKE) create_build_dirs $(TARGET)
	@echo ""
	@echo "==================="
	@echo "  Build complete!  "
	@echo "==================="
	@echo ""

# Simulation mode compilation
sim:
	@$(MAKE) clean
	@$(MAKE) create_build_dirs
	@$(MAKE) CFLAGS="-Wall -Wextra -O2 -g -DCAN_MODE_SIMULATION" keyboard_sim_target
	@echo ""
	@echo "=============================="
	@echo "  Simulation build complete!  "
	@echo "=============================="
	@echo ""

# Special target for simulation that includes keyboard_simulator.o
keyboard_sim_target: $(SOEM_OBJ) $(OSHW_OBJ) $(OSAL_OBJ) $(INIH_OBJ) $(APP_OBJ) $(BUILD_SRC_DIR)/keyboard_simulator.o
	@echo "Linking $(TARGET)"
	@$(CC) -o $(TARGET) $(SOEM_OBJ) $(OSHW_OBJ) $(OSAL_OBJ) $(INIH_OBJ) $(APP_OBJ) $(BUILD_SRC_DIR)/keyboard_simulator.o $(LDFLAGS)

# Create build directories
create_build_dirs:
	@mkdir -p $(BUILD_SOEM_DIR)
	@mkdir -p $(BUILD_OSHW_DIR)
	@mkdir -p $(BUILD_OSAL_DIR)
	@mkdir -p $(BUILD_SRC_DIR)
	@mkdir -p $(BUILD_INIH_DIR)

# Build SOEM library files
soem: create_build_dirs $(SOEM_OBJ) $(OSHW_OBJ) $(OSAL_OBJ)

# Build inih library files
inih: create_build_dirs $(INIH_OBJ)

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

# Compile inih
$(BUILD_INIH_DIR)/%.o: $(INIH_DIR)/%.c
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

real: all

.PHONY: all clean soem inih create_build_dirs real sim keyboard_sim_target