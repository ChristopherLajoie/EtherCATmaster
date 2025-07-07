/**
 * @file config.c
 * @brief Configuration management for motor control application - Updated with HMI support
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

    // Logging parameters
    {"logging", "enable_logging", TYPE_INT, offsetof(MotorConfig, enable_logging), 0},
    {"logging", "log_file_path", TYPE_STRING, offsetof(MotorConfig, log_file_path), sizeof(((MotorConfig*)0)->log_file_path)},
    {"logging", "log_interval_ms", TYPE_INT, offsetof(MotorConfig, log_interval_ms), 0},

    // Real-time HMI parameters
    {"realtime_hmi", "enable_hmi", TYPE_INT, offsetof(MotorConfig, enable_realtime_hmi), 0},
    {"realtime_hmi",
     "broadcast_ip",
     TYPE_STRING,
     offsetof(MotorConfig, hmi_broadcast_ip),
     sizeof(((MotorConfig*)0)->hmi_broadcast_ip)},
    {"realtime_hmi", "broadcast_port", TYPE_INT, offsetof(MotorConfig, hmi_broadcast_port), 0},
    {"realtime_hmi", "broadcast_interval_ms", TYPE_INT, offsetof(MotorConfig, hmi_broadcast_interval_ms), 0},

    // I2t protection parameters
    {"i2t_protection", "peak_time_ms", TYPE_UINT32, offsetof(MotorConfig, i2t_peak_time_ms), 0},
    {"i2t_protection", "thermal_limit", TYPE_UINT32, offsetof(MotorConfig, i2t_thermal_limit), 0},
};

void calculate_derived_config_values(void)
{
    // Calculate log interval in cycles based on cycletime and desired interval in ms
    if (g_config.log_interval_ms > 0 && g_config.cycletime > 0)
    {
        g_config.log_interval_cycles = g_config.log_interval_ms * 1000 / g_config.cycletime;
        if (g_config.log_interval_cycles < 1)
        {
            g_config.log_interval_cycles = 1;
        }
    }
    else
    {
        g_config.log_interval_cycles = 25;  // Default: log every 25 cycles (~100ms at 4ms cycle)
    }
}

bool load_config(const char* filename)
{
    // Set default values for all parameters

    // Logging defaults
    g_config.enable_logging = 0;
    strncpy(g_config.log_file_path, "/tmp/motor_logs", sizeof(g_config.log_file_path) - 1);
    g_config.log_file_path[sizeof(g_config.log_file_path) - 1] = '\0';
    g_config.log_interval_ms = 100;  // Default 100ms logging interval

    // Real-time HMI defaults
    g_config.enable_realtime_hmi = 0;
    strncpy(g_config.hmi_broadcast_ip, "192.168.1.255", sizeof(g_config.hmi_broadcast_ip) - 1);
    g_config.hmi_broadcast_ip[sizeof(g_config.hmi_broadcast_ip) - 1] = '\0';
    g_config.hmi_broadcast_port = 9999;
    g_config.hmi_broadcast_interval_ms = 100;  // Default 10Hz (100ms)

    // I2t protection defaults
    g_config.i2t_peak_time_ms = 5000;      // Default 5 seconds peak time
    g_config.i2t_thermal_limit = 100;      // Default 100% thermal limit

    if (ini_parse(filename, config_handler, &g_config) < 0)
    {
        return false;
    }

    // Calculate derived values
    calculate_derived_config_values();

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
                            fprintf(stderr, "Invalid turn factor value\n");
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