CC = gcc
CFLAGS = -Wall -Wextra -O2 -g
LDFLAGS = -lm -lrt -lpthread

# Python integration - this is the critical fix
PYTHON_CFLAGS = $(shell python3-config --includes)
PYTHON_LDFLAGS = $(shell python3-config --ldflags)
PYTHON_LIBS = $(shell python3-config --libs)

# SOEM source directories
SOEM_DIR = SOEM/soem
OSHW_DIR = SOEM/oshw/linux
OSAL_DIR = SOEM/osal
OSAL_LINUX_DIR = SOEM/osal/linux

# Include directories
INCLUDES = -I$(SOEM_DIR) -I$(OSHW_DIR) -I$(OSAL_DIR) -I$(OSAL_LINUX_DIR)

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

# CAN Python wrapper
CAN_OBJ = can_wrapper.o can_monitor.o

# All object files
OBJ = $(SOEM_OBJ) $(OSHW_OBJ) $(OSAL_OBJ) coe_master.o $(CAN_OBJ)

# Target executable
TARGET = coe_master

all: $(TARGET)

# Compile SOEM
$(SOEM_OBJ): %.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Compile OSHW
$(OSHW_OBJ): %.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Compile OSAL
$(OSAL_OBJ): %.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Compile the CAN Python wrapper
can_wrapper.o: can_wrapper.c
	$(CC) $(CFLAGS) $(INCLUDES) $(PYTHON_CFLAGS) -c $< -o $@
    
can_monitor.o: can_monitor.c can_monitor.h
	$(CC) $(CFLAGS) -c $< -o $@

# Compile the main program
coe_master.o: coe_master.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(TARGET): $(OBJ)
	$(CC) -o $@ $^ $(LDFLAGS) -lpython3.11

clean:
	rm -f $(OBJ) $(TARGET)

.PHONY: all clean