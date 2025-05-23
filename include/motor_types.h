#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#include <stdint.h>
#include <signal.h>
#include <pthread.h>

/* RxPDO (master to slave) */
#pragma pack(push, 1) /* no implicit padding */
typedef struct
{
    uint16_t controlword;
    int8_t op_mode;
    int16_t target_torque;
    int32_t target_position;
    int32_t target_velocity;
    int16_t torque_offset;
    int32_t tuning_command;
} rxpdo_t;
#pragma pack(pop)

/* TxPDO (slave to master) */
#pragma pack(push, 1)
typedef struct
{
    uint16_t statusword;
    int8_t op_mode_display;
    int32_t position_actual;
    int16_t velocity_actual;
    int16_t torque_actual;
} txpdo_t;
#pragma pack(pop)

typedef struct
{
    char* ifname;
    int cycletime;
    volatile sig_atomic_t run;
    int slave_index;
    char IOmap[4096];
    pthread_t cyclic_thread;
    rxpdo_t* rxpdo;
    txpdo_t* txpdo;
} MotorControl;

extern MotorControl g_motor_control;

bool ethercat_init(void);

#endif