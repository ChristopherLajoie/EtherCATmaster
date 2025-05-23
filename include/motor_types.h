/**
 * @file motor_types.h
 * @brief Data structures for motor control
 */
#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#include <stdint.h>
#include <signal.h>
#include <pthread.h>

/* RxPDO (master to slave) */
#pragma pack(push, 1) /* no implicit padding */
typedef struct
{
    uint16_t controlword;    /* 0x6040 */
    int8_t op_mode;          /* 0x6060 */
    int16_t target_torque;   /* 0x6071 */
    int32_t target_position; /* 0x607A */
    int32_t target_velocity; /* 0x60FF */
    int16_t torque_offset;   /* 0x60B2 */
    int32_t tuning_command;  /* 0x2701 */
} rxpdo_t;
#pragma pack(pop)

/* TxPDO (slave to master) */
#pragma pack(push, 1)
typedef struct
{
    uint16_t statusword;     /* 0x6041 */
    int8_t op_mode_display;  /* 0x6061 */
    int32_t position_actual; /* 0x6064 */
    int16_t velocity_actual; /* 0x606C */
    int16_t torque_actual;   /* 0x6077 */
} txpdo_t;
#pragma pack(pop)

/* Global motor control structure */
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

/* Global instance */
extern MotorControl g_motor_control;

/* EtherCAT initialization */
bool ethercat_init(void);

#endif /* MOTOR_TYPES_H */