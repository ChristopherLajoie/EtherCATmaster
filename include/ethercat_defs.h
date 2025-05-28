#ifndef ETHERCAT_DEFS_H
#define ETHERCAT_DEFS_H

#include <stdint.h>
#include "ethercat.h"
#include "ethercatprint.h"
#include "ethercatconfig.h"
#include "hardware_io.h"

#define RX_PDO_INDEX 0x1600
#define TX_PDO_INDEX 0x1A00

/* Macro accessors for runtime config */
#define SW_READY_TO_SWITCH_ON_BIT 0x0001
#define SW_SWITCHED_ON_BIT 0x0002
#define SW_OPERATION_ENABLED_BIT 0x0004
#define SW_FAULT_BIT 0x0008
#define SW_VOLTAGE_ENABLED_BIT 0x0010
#define SW_QUICK_STOP_BIT 0x0020
#define SW_SWITCH_ON_DISABLED_BIT 0x0040
#define SW_TARGET_REACHED_BIT 0x0400
#define SW_NO_COMMUNICATION 0x0000

/* Control word macros */
#define CW_NEW_VELOCITY_SETPOINT 0x0010
#define CW_SHUTDOWN 0x0006
#define CW_SWITCHON 0x0007
#define CW_ENABLE 0x000F
#define CW_DISABLEVOLTAGE 0x0000
#define CW_DISABLEOPERATION 0x0007
#define CW_FAULT_RESET 0x0080

/* Motion parameter macros */
#define MAX_TORQUE convert_to_raw(g_config.max_torque)
#define MAX_VELOCITY g_config.max_velocity
#define DEFAULT_PROFILE_ACCEL g_config.profile_acceleration
#define DEFAULT_PROFILE_DECEL g_config.profile_deceleration
#define DEFAULT_QUICK_STOP_DECEL g_config.quick_stop_deceleration
#define DEFAULT_PROFILE_TYPE g_config.motion_profile_type

/* Joystick macros */
#define JOYSTICK_MIN g_config.joystick_min
#define JOYSTICK_MAX g_config.joystick_max
#define JOYSTICK_CENTER g_config.joystick_center
#define JOYSTICK_DEADZONE g_config.joystick_deadzone

#endif