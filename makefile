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

# CAN interface object files
CAN_OBJ = $(BUILD_ROOT_DIR)/can_monitor.o \
          $(BUILD_SRC_DIR)/socketcan.o

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

all: create_build_dirs $(TARGET)

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
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Compile OSHW
$(BUILD_OSHW_DIR)/%.o: $(OSHW_DIR)/%.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Compile OSAL
$(BUILD_OSAL_DIR)/%.o: $(OSAL_LINUX_DIR)/%.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Compile SocketCAN implementation (in src directory)
$(BUILD_SRC_DIR)/socketcan.o: src/socketcan.c include/socketcan.h
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Compile CAN monitor (in root directory)
$(BUILD_ROOT_DIR)/can_monitor.o: can_monitor.c include/can_monitor.h include/socketcan.h
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Compile application source files
$(BUILD_SRC_DIR)/%.o: src/%.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Link everything together
$(TARGET): $(OBJ)
	$(CC) -o $@ $(OBJ) $(LDFLAGS)

clean:
	rm -rf $(BUILD_DIR)
	rm -f $(TARGET)

.PHONY: all clean soem create_build_dirs