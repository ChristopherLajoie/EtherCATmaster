/**
 * @file config.h
 * @brief Configuration management header - NXP Yocto RT version
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

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
    const char *section;
    const char *name;
    ConfigType type;
    size_t offset;
    size_t max_len;
} ConfigMapping;

typedef struct
{
    char interface[64];
    int cycletime;
    int num_motors;

    uint16_t max_torque;
    uint32_t max_velocity;
    uint32_t profile_acceleration;
    uint32_t profile_deceleration;
    uint32_t quick_stop_deceleration;
    int16_t motion_profile_type;

    int joystick_min;
    int joystick_max;
    int joystick_center;
    int joystick_deadzone;

    float turn_factor;
    int reverse_left_motor;
    int reverse_right_motor;

    uint32_t i2t_peak_time_ms; 
} MotorConfig;

extern MotorConfig g_config;

bool load_config(const char *filename);

#endif 