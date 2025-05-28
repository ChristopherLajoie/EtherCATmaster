#ifndef CONFIG_H
#define CONFIG_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "ini.h"

typedef enum
{
    TYPE_STRING,
    TYPE_INT,
    TYPE_UINT16,
    TYPE_UINT32,
    TYPE_INT16,
    TYPE_UINT8,
    TYPE_FLOAT
} ConfigType;

typedef struct
{
    const char* section;
    const char* name;
    ConfigType type;
    size_t offset;
    size_t max_len;
} ConfigMapping;

typedef struct
{
    /* EtherCAT parameters */
    char interface[32];
    int cycletime;
    int num_motors;

    /* Motion parameters */
    uint16_t max_torque;
    uint32_t max_velocity;
    uint32_t profile_acceleration;
    uint32_t profile_deceleration;
    uint32_t quick_stop_deceleration;
    int16_t motion_profile_type;

    /* Joystick parameters */
    int joystick_min;
    int joystick_max;
    int joystick_center;
    int joystick_deadzone;

    float turn_factor;
    int reverse_left_motor;
    int reverse_right_motor;
} MotorConfig;

extern MotorConfig g_config;

bool load_config(const char* filename);

#endif