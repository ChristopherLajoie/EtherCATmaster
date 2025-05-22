#ifndef HARDWARE_IO_H
#define HARDWARE_IO_H

#include <termios.h>
#include "common.h"

// Setup terminal for raw input mode
void enable_raw_mode(void);

// Restore terminal to original settings
void disable_raw_mode(void);

// Check if a key was pressed
int kbhit(void);

// Read a character
char readch(void);

// Initialize EtherCAT communication
bool ethercat_init();

// Read drive parameters
void read_drive_parameters();

bool configure_pdo_mappings(int slave);

#endif 