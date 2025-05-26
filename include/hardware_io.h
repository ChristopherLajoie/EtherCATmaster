#ifndef HARDWARE_IO_H
#define HARDWARE_IO_H

#include <termios.h>
#include "common.h"

typedef struct
{
    uint16_t code;
    const char* description;
} cia402_state_pair_t;

typedef struct
{
    uint32_t value;
    const char* description;
} pdo_entry_t;

typedef struct
{
    uint16_t index;
    const char* description;
} pdo_config_t;

typedef struct
{
    uint16_t index;
    uint8_t subindex;
    uint8_t size;
    const char* description;
    uint32_t value;
} motion_param_t;

void enable_raw_mode(void);
void disable_raw_mode(void);
int kbhit(void);
char readch(void);
bool ethercat_init();
bool configure_pdo_mappings(int slave);
uint8_t get_cia402_state(uint16_t statusword);
uint16_t get_cia402_state_code(const char* state_str);
const char* get_cia402_state_string(uint16_t state);
uint32_t read_drive_parameter(int slave, uint16_t index, uint8_t subindex, const char* description, const char* unit);
int convert_to_mNm(int16_t raw_torque);
int convert_to_raw(int16_t torque);

#endif