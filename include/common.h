#ifndef COMMON_H
#define COMMON_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <signal.h>
#include <fcntl.h>

#include "ethercat.h"
#include "ethercatprint.h"
#include "ethercatconfig.h"

#define OP_MODE_PVM 3
#define OP_MODE_CSV 9

// Controlword bits for profile velocity mode
#define CW_NEW_SETPOINT 0x0010     // Bit 4 - Start the positioning (rising edge 0->1)
#define CW_CHANGE_SET_IMMED 0x0020 // Bit 5 - Change setpoint immediately

// Statusword bits for profile velocity mode
#define SW_SPEED_BIT 0x1000          // Bit 12

#define MAX_VELOCITY 2500

// Joystick parameters
#define JOYSTICK_MIN 4
#define JOYSTICK_MAX 252
#define JOYSTICK_CENTER 128
#define JOYSTICK_DEADZONE 10

// CiA 402 state machine commands (controlword)
#define CW_SHUTDOWN 0x0006
#define CW_SWITCHON 0x0007
#define CW_ENABLE 0x000F
#define CW_ENABLE_VOLTAGE 0x001F
#define CW_QUICKSTOP 0x0002
#define CW_DISABLEVOLTAGE 0x0000
#define CW_DISABLEOPERATION 0x0007
#define CW_FAULT_RESET 0x0080

// CiA 402 statusword bit masks
#define SW_READY_TO_SWITCH_ON_BIT (0x1 << 0)
#define SW_SWITCHED_ON_BIT (0x1 << 1)
#define SW_OPERATION_ENABLED_BIT (0x1 << 2)
#define SW_FAULT_BIT (0x1 << 3)
#define SW_VOLTAGE_ENABLED_BIT (0x1 << 4)
#define SW_QUICK_STOP_BIT (0x1 << 5)
#define SW_SWITCH_ON_DISABLED_BIT (0x1 << 6)
#define SW_TARGET_REACHED_BIT (0x1 << 10)
#define SW_NO_COMMUNICATION 0x0000 

// RxPDO (master to slave)
#pragma pack(push, 1) // no implicit padding
typedef struct
{
    uint16_t controlword;    // 0x6040
    int8_t op_mode;          // 0x6060
    int16_t target_torque;   // 0x6071
    int32_t target_position; // 0x607A
    int32_t target_velocity; // 0x60FF
    int16_t torque_offset;   // 0x60B2
    int32_t tuning_command;  // 0x2701
} rxpdo_t;
#pragma pack(pop)

// TxPDO (slave to master)
#pragma pack(push, 1)
typedef struct
{
    uint16_t statusword;     // 0x6041
    int8_t op_mode_display;  // 0x6061
    int32_t position_actual; // 0x6064
    int16_t velocity_actual; // 0x606C
    int16_t torque_actual;   // 0x6077
} txpdo_t;
#pragma pack(pop)

// Global data structure
typedef struct
{
    char *ifname;
    int cycletime;
    volatile sig_atomic_t run;
    int slave_index;
    char IOmap[4096];
    pthread_t cyclic_thread;
    rxpdo_t *rxpdo;
    txpdo_t *txpdo;
} MotorControl;

// Global instance (declared in main.c)
extern MotorControl g_motor_control;

// Function declarations for ethercat operations
bool ethercat_init(void);

#endif // COMMON_H