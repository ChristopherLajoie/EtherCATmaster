CC = gcc
CFLAGS = -Wall -Wextra -O2 -g
LDFLAGS = -lm -lrt -lpthread

# Python integration - detect Python version and config automatically
PYTHON_VERSION = $(shell python3 --version | cut -d' ' -f2 | cut -d'.' -f1-2)
PYTHON_CFLAGS = $(shell python3-config --includes)
PYTHON_LDFLAGS = $(shell python3-config --ldflags)
PYTHON_LIBS = $(shell python3-config --libs)

# SOEM source directories
SOEM_DIR = SOEM/soem
OSHW_DIR = SOEM/oshw/linux
OSAL_DIR = SOEM/osal
OSAL_LINUX_DIR = SOEM/osal/linux

# Include directories
INCLUDES = -Iinclude -I$(SOEM_DIR) -I$(OSHW_DIR) -I$(OSAL_DIR) -I$(OSAL_LINUX_DIR) -I. $(PYTHON_CFLAGS)

# SOEM object files
SOEM_OBJ = $(SOEM_DIR)/ethercatbase.o \
           $(SOEM_DIR)/ethercatcoe.o \
           $(SOEM_DIR)/ethercatconfig.o \
           $(SOEM_DIR)/ethercatdc.o \
           $(SOEM_DIR)/ethercatfoe.o \
           $(SOEM_DIR)/ethercatmain.o \
           $(SOEM_DIR)/ethercatprint.o \
           $(SOEM_DIR)/ethercatsoe.o

# OSHW object files
OSHW_OBJ = $(OSHW_DIR)/nicdrv.o \
           $(OSHW_DIR)/oshw.o

# OSAL object files
OSAL_OBJ = $(OSAL_LINUX_DIR)/osal.o

# CAN interface object files (kept in root directory)
CAN_OBJ = can_wrapper.o can_monitor.o

# Application object files
APP_OBJ = src/main.o \
          src/ethercat_driver.o \
          src/cia402_state.o \
          src/motor_control.o \
          src/terminal_io.o

# All object files
OBJ = $(SOEM_OBJ) $(OSHW_OBJ) $(OSAL_OBJ) $(CAN_OBJ) $(APP_OBJ)

# Target executable
TARGET = motor_control

# Detect Python version for linking
ifeq ($(shell pkg-config --exists python$(PYTHON_VERSION) && echo yes),yes)
    PYTHON_LINK = $(shell pkg-config --libs python$(PYTHON_VERSION))
else
    # Fallback for systems without pkg-config or specific Python version
    PYTHON_LINK = -lpython$(PYTHON_VERSION)
endif

all: $(TARGET)

# Rule to build all SOEM library files
soem: $(SOEM_OBJ) $(OSHW_OBJ) $(OSAL_OBJ)

# Compile SOEM
$(SOEM_OBJ): %.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Compile OSHW
$(OSHW_OBJ): %.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Compile OSAL
$(OSAL_OBJ): %.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Compile CAN wrapper (in root directory)
can_wrapper.o: can_wrapper.c include/can_wrapper.h
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Compile CAN monitor (in root directory)
can_monitor.o: can_monitor.c include/can_monitor.h
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Compile application source files
src/%.o: src/%.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Check Python interface exists
copy_python_interface:
	@if [ -f can_interface.py ]; then \
		echo "CAN Python interface already exists"; \
	else \
		echo "Error: can_interface.py not found!"; \
		exit 1; \
	fi

# Link everything together
$(TARGET): copy_python_interface $(OBJ)
	$(CC) -o $@ $(OBJ) $(LDFLAGS) $(PYTHON_LDFLAGS) $(PYTHON_LIBS) $(PYTHON_LINK)

# Print Python configuration
python-config:
	@echo "Python version: $(PYTHON_VERSION)"
	@echo "Python CFLAGS: $(PYTHON_CFLAGS)"
	@echo "Python LDFLAGS: $(PYTHON_LDFLAGS)"
	@echo "Python LIBS: $(PYTHON_LIBS)"
	@echo "Python Link: $(PYTHON_LINK)"

clean:
	rm -f $(OBJ) $(TARGET)

.PHONY: all clean soem python-config copy_python_interface