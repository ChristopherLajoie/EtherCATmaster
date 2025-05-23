/**
 * @file ethercat_defs.h
 * @brief EtherCAT and CiA 402 protocol definitions
 */
#ifndef ETHERCAT_DEFS_H
#define ETHERCAT_DEFS_H

#include <stdint.h>
#include "ethercat.h"
#include "ethercatprint.h"
#include "ethercatconfig.h"

/* CiA 402 state machine mask */
#define STATUS_STATE_MASK 0x6F

/* CiA 402 state machine enumeration */
typedef enum
{
    CIA402_STATE_NOT_READY = 0x00,
    CIA402_STATE_SWITCH_ON_DISABLED = 0x40,
    CIA402_STATE_READY_TO_SWITCH_ON = 0x21,
    CIA402_STATE_SWITCHED_ON = 0x23,
    CIA402_STATE_OPERATION_ENABLED = 0x27,
    CIA402_STATE_QUICK_STOP_ACTIVE = 0x07,
    CIA402_STATE_FAULT_REACTION_ACTIVE = 0x0F,
    CIA402_STATE_FAULT = 0x08,
    CIA402_STATE_UNKNOWN = 0xFF
} cia402_state_t;

#define RX_PDO_INDEX g_config.rx_pdo_index
#define TX_PDO_INDEX g_config.tx_pdo_index

/* Macro accessors for runtime config */
#define SW_READY_TO_SWITCH_ON_BIT g_config.sw_ready_to_switch_on_bit
#define SW_SWITCHED_ON_BIT g_config.sw_switched_on_bit
#define SW_OPERATION_ENABLED_BIT g_config.sw_operation_enabled_bit
#define SW_FAULT_BIT g_config.sw_fault_bit
#define SW_VOLTAGE_ENABLED_BIT g_config.sw_voltage_enabled_bit
#define SW_QUICK_STOP_BIT g_config.sw_quick_stop_bit
#define SW_SWITCH_ON_DISABLED_BIT g_config.sw_switch_on_disabled_bit
#define SW_TARGET_REACHED_BIT g_config.sw_target_reached_bit
#define SW_NO_COMMUNICATION g_config.sw_no_communication

/* Control word macros */
#define CW_NEW_VELOCITY_SETPOINT g_config.new_velocity_setpoint
#define CW_SHUTDOWN g_config.shutdown
#define CW_SWITCHON g_config.switch_on
#define CW_ENABLE g_config.enable
#define CW_DISABLEVOLTAGE g_config.disable_voltage
#define CW_DISABLEOPERATION g_config.disable_operation
#define CW_FAULT_RESET g_config.fault_reset

/* Motion parameter macros */
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

#endif /* ETHERCAT_DEFS_H */