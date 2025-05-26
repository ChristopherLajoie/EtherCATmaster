/**
 * @file config.c
 * @brief Configuration management for motor control application
 *
 * This file provides functions for loading and parsing configuration settings
 * from INI files into application-specific data structures.
 *
 * Uses the inih library for INI file parsing with a generic mapping mechanism
 * that automatically converts and stores values based on type.
 */

#include "common.h"
#include "config.h"

MotorConfig g_config;

static int config_handler(void* user, const char* section, const char* name, const char* value);

static const ConfigMapping config_map[] = {
    // EtherCAT parameters
    {"ethercat", "interface", TYPE_STRING, offsetof(MotorConfig, interface), sizeof(((MotorConfig*)0)->interface)},
    {"ethercat", "cycletime", TYPE_INT, offsetof(MotorConfig, cycletime), 0},
    {"ethercat", "num_motors", TYPE_INT, offsetof(MotorConfig, num_motors), 0},

    // Status word bits
    {"status_word_bits", "ready_to_switch_on", TYPE_UINT16, offsetof(MotorConfig, sw_ready_to_switch_on_bit), 0},
    {"status_word_bits", "switched_on", TYPE_UINT16, offsetof(MotorConfig, sw_switched_on_bit), 0},
    {"status_word_bits", "operation_enabled", TYPE_UINT16, offsetof(MotorConfig, sw_operation_enabled_bit), 0},
    {"status_word_bits", "fault", TYPE_UINT16, offsetof(MotorConfig, sw_fault_bit), 0},
    {"status_word_bits", "voltage_enabled", TYPE_UINT16, offsetof(MotorConfig, sw_voltage_enabled_bit), 0},
    {"status_word_bits", "quick_stop", TYPE_UINT16, offsetof(MotorConfig, sw_quick_stop_bit), 0},
    {"status_word_bits", "switch_on_disabled", TYPE_UINT16, offsetof(MotorConfig, sw_switch_on_disabled_bit), 0},
    {"status_word_bits", "target_reached", TYPE_UINT16, offsetof(MotorConfig, sw_target_reached_bit), 0},
    {"status_word_bits", "no_communication", TYPE_UINT16, offsetof(MotorConfig, sw_no_communication), 0},

    // Control word bits
    {"control_word_mapping", "new_velocity_setpoint", TYPE_UINT16, offsetof(MotorConfig, new_velocity_setpoint), 0},
    {"control_word_mapping", "shutdown", TYPE_UINT16, offsetof(MotorConfig, shutdown), 0},
    {"control_word_mapping", "switchon", TYPE_UINT16, offsetof(MotorConfig, switch_on), 0},
    {"control_word_mapping", "enable", TYPE_UINT16, offsetof(MotorConfig, enable), 0},
    {"control_word_mapping", "disable_voltage", TYPE_UINT16, offsetof(MotorConfig, disable_voltage), 0},
    {"control_word_mapping", "disable_operation", TYPE_UINT16, offsetof(MotorConfig, disable_operation), 0},
    {"control_word_mapping", "fault_reset", TYPE_UINT16, offsetof(MotorConfig, fault_reset), 0},

    // PDO mapping parameters
    {"pdo_mapping", "rx_pdo_index", TYPE_UINT16, offsetof(MotorConfig, rx_pdo_index), 0},
    {"pdo_mapping", "tx_pdo_index", TYPE_UINT16, offsetof(MotorConfig, tx_pdo_index), 0},

    // Motion parameters
    {"motion_parameters", "max_torque", TYPE_UINT16, offsetof(MotorConfig, max_torque), 0},
    {"motion_parameters", "max_velocity", TYPE_UINT32, offsetof(MotorConfig, max_velocity), 0},
    {"motion_parameters", "profile_acceleration", TYPE_UINT32, offsetof(MotorConfig, profile_acceleration), 0},
    {"motion_parameters", "profile_deceleration", TYPE_UINT32, offsetof(MotorConfig, profile_deceleration), 0},
    {"motion_parameters", "quick_stop_deceleration", TYPE_UINT32, offsetof(MotorConfig, quick_stop_deceleration), 0},
    {"motion_parameters", "motion_profile_type", TYPE_INT16, offsetof(MotorConfig, motion_profile_type), 0},

    // Joystick parameters
    {"joystick", "min_value", TYPE_INT, offsetof(MotorConfig, joystick_min), 0},
    {"joystick", "max_value", TYPE_INT, offsetof(MotorConfig, joystick_max), 0},
    {"joystick", "center_value", TYPE_INT, offsetof(MotorConfig, joystick_center), 0},
    {"joystick", "deadzone", TYPE_INT, offsetof(MotorConfig, joystick_deadzone), 0},

    // Differential drive parameters
    {"differential_drive", "turn_factor", TYPE_FLOAT, offsetof(MotorConfig, turn_factor), 0},
    {"differential_drive", "reverse_left_motor", TYPE_INT, offsetof(MotorConfig, reverse_left_motor), 0},
    {"differential_drive", "reverse_right_motor", TYPE_INT, offsetof(MotorConfig, reverse_right_motor), 0},
};

bool load_config(const char* filename)
{
    if (ini_parse(filename, config_handler, &g_config) < 0)
    {
        return false;
    }

    printf("Configuration loaded from '%s'\n", filename);
    return true;
}

static int config_handler(void* user, const char* section, const char* name, const char* value)
{
    MotorConfig* config = (MotorConfig*)user;

    for (size_t i = 0; i < sizeof(config_map) / sizeof(config_map[0]); i++)
    {
        if (strcmp(section, config_map[i].section) == 0 && strcmp(name, config_map[i].name) == 0)
        {
            void* target = (char*)config + config_map[i].offset;

            switch (config_map[i].type)
            {
                case TYPE_STRING:
                    strncpy((char*)target, value, config_map[i].max_len - 1);
                    ((char*)target)[config_map[i].max_len - 1] = '\0';
                    break;
                case TYPE_INT:
                    *(int*)target = atoi(value);
                    break;
                case TYPE_UINT16:
                    *(uint16_t*)target = (uint16_t)strtol(value, NULL, 0);
                    break;
                case TYPE_UINT32:
                    *(uint32_t*)target = (uint32_t)strtol(value, NULL, 0);
                    break;
                case TYPE_INT16:
                    *(int16_t*)target = (int16_t)atoi(value);
                    break;
                case TYPE_UINT8:
                    *(uint8_t*)target = (uint8_t)strtol(value, NULL, 0);
                    break;
                case TYPE_FLOAT:
                {
                    float val = strtof(value, NULL);
                    if (strcmp(name, "turn_factor") == 0)
                    {
                        if (val < 0.0f || val > 1.0f)
                        {
                            fprintf(stderr, "Invalid turn factor value");
                            return 0;
                        }
                    }
                    *(float*)target = val;
                }
                break;
                default:
                    fprintf(stderr, "Warning: Unknown type for [%s]%s\n", section, name);
                    return 0;
            }

            return 1;
        }
    }

    fprintf(stderr, "Warning: Unknown configuration option [%s]%s\n", section, name);
    return 1;
}