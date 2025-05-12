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
INCLUDES = -I$(SOEM_DIR) -I$(OSHW_DIR) -I$(OSAL_DIR) -I$(OSAL_LINUX_DIR) -I.

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

# Application object files
APP_OBJ = main.o coe_master.o can_wrapper.o can_monitor.o

# All object files
OBJ = $(SOEM_OBJ) $(OSHW_OBJ) $(OSAL_OBJ) $(APP_OBJ)

# Target executable
TARGET = coe_master

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

# Compile the Python wrapper with Python includes
can_wrapper.o: can_wrapper.c can_wrapper.h
	$(CC) $(CFLAGS) $(INCLUDES) $(PYTHON_CFLAGS) -c $< -o $@
    
# Compile CAN monitor
can_monitor.o: can_monitor.c can_monitor.h
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Compile COE master module
coe_master.o: coe_master.c coe_master.h
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Compile main program
main.o: main.c coe_master.h
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Link everything together
$(TARGET): $(OBJ)
	$(CC) -o $@ $^ $(LDFLAGS) $(PYTHON_LINK)

# Print Python configuration
python-config:
	@echo "Python version: $(PYTHON_VERSION)"
	@echo "Python CFLAGS: $(PYTHON_CFLAGS)"
	@echo "Python LDFLAGS: $(PYTHON_LDFLAGS)"
	@echo "Python LIBS: $(PYTHON_LIBS)"
	@echo "Python Link: $(PYTHON_LINK)"

clean:
	rm -f $(OBJ) $(TARGET)

.PHONY: all clean soem python-config