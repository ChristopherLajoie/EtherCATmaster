/**
 * @file config.h
 * @brief Configuration handling for motor control
 */
#ifndef CONFIG_H
#define CONFIG_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "ini.h"

/**
 * @brief Configuration parameter types
 */
typedef enum
{
    TYPE_STRING,
    TYPE_INT,
    TYPE_UINT16,
    TYPE_UINT32,
    TYPE_INT16,
    TYPE_UINT8
} ConfigType;

/**
 * @brief Configuration mapping structure
 */
typedef struct
{
    const char* section;
    const char* name;
    ConfigType type;
    size_t offset;
    size_t max_len;
} ConfigMapping;

/**
 * @brief Motor configuration structure
 */
typedef struct
{
    /* EtherCAT parameters */
    char interface[32];
    int cycletime;
    int slave_index;
    int retry_attempts;
    int retry_delay_ms;
    int state_transition_timeout_us;

    /* Status word bits */
    uint16_t sw_ready_to_switch_on_bit;
    uint16_t sw_switched_on_bit;
    uint16_t sw_operation_enabled_bit;
    uint16_t sw_fault_bit;
    uint16_t sw_voltage_enabled_bit;
    uint16_t sw_quick_stop_bit;
    uint16_t sw_switch_on_disabled_bit;
    uint16_t sw_target_reached_bit;
    uint16_t sw_no_communication;
    
    /* Control word bits */
    uint16_t new_velocity_setpoint;
    uint16_t shutdown;
    uint16_t switch_on; /* Note: renamed from switchon */
    uint16_t enable;
    uint16_t disable_voltage;
    uint16_t disable_operation;
    uint16_t fault_reset;

    /* PDO mapping parameters */
    uint16_t rx_pdo_index;
    uint16_t tx_pdo_index;

    /* Motion parameters */
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
} MotorConfig;

/* Global configuration */
extern MotorConfig g_config;

/* Configuration functions */
bool load_config(const char* filename);

#endif /* CONFIG_H */