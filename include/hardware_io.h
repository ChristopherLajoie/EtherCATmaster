#ifndef HARDWARE_IO_H
#define HARDWARE_IO_H

#include <termios.h>
#include <math.h>
#include "common.h"

typedef struct
{
    uint8_t motor_i2t_percent;
    float drive_temp_celsius;
    float core_temp_celsius;
    float index_temp_celsius;       // Index temperature (0x2038:01)
    float torque_constant_mNm_per_A;
    float current_actual_A;
    bool data_valid;
} thermal_data_t;

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
bool read_thermal_data(int slave, thermal_data_t* thermal_data);
bool read_torque_constant(int slave, float* torque_constant_mNm_per_A);
void calculate_current_from_torque(thermal_data_t* thermal_data, int32_t torque_actual_mNm, int motor_index);
bool configure_i2t_protection(int slave, uint8_t mode);
int read_i2t_protection_mode(int slave);
bool init_torque_constants(void);
float get_torque_constant(int motor_index);
bool is_torque_constant_valid(int motor_index);

#endif