/**
 * @file config.h
 * @brief Configuration management header with logging and HMI support
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
    const char* section;
    const char* name;
    ConfigType type;
    size_t offset;
    size_t max_len;
} ConfigMapping;

typedef struct
{
    // EtherCAT parameters
    char interface[64];
    int cycletime;
    int num_motors;

    // Motion parameters
    uint16_t max_torque;
    uint32_t max_velocity;
    uint32_t profile_acceleration;
    uint32_t profile_deceleration;
    uint32_t quick_stop_deceleration;
    int16_t motion_profile_type;

    // Joystick parameters
    int joystick_min;
    int joystick_max;
    int joystick_center;
    int joystick_deadzone;

    // Differential drive parameters
    float turn_factor;
    int reverse_left_motor;
    int reverse_right_motor;

    // Logging parameters
    int enable_logging;
    char log_file_path[256];
    int log_interval_ms;
    int log_interval_cycles;  // Calculated from log_interval_ms and cycletime

    // Real-time HMI parameters
    int enable_realtime_hmi;
    char hmi_broadcast_ip[16];
    int hmi_broadcast_port;
    int hmi_broadcast_interval_ms;
} MotorConfig;

extern MotorConfig g_config;

/**
 * @brief Load configuration from INI file
 * @param filename Path to the INI file
 * @return true on success, false on failure
 */
bool load_config(const char* filename);

/**
 * @brief Calculate derived configuration values
 */
void calculate_derived_config_values(void);

#endif  // CONFIG_H