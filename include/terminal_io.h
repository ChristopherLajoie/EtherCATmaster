#ifndef TERMINAL_IO_H
#define TERMINAL_IO_H

#include <termios.h>

// Setup terminal for raw input mode
void enable_raw_mode(void);

// Restore terminal to original settings
void disable_raw_mode(void);

// Check if a key was pressed
int kbhit(void);

// Read a character
char readch(void);

#endif // TERMINAL_IO_H