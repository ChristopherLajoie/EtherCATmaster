#ifndef HARDWARE_IO_H
#define HARDWARE_IO_H

#include <termios.h>
#include "common.h"

void enable_raw_mode(void);
void disable_raw_mode(void);
int kbhit(void);
char readch(void);
bool ethercat_init();
bool configure_pdo_mappings(int slave);
void cia402_decode_statusword(uint16_t statusword);
cia402_state_t get_cia402_state(uint16_t statusword);
const char* get_cia402_state_string(cia402_state_t state);

#endif