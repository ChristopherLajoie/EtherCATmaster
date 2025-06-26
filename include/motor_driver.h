#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "common.h"
#include "can_interface.h"
#include "realtime_broadcaster.h"

typedef enum
{
    STATE_BOOT,
    STATE_INIT,
    STATE_OPERATIONAL
} motor_state_t;

#define MAX_FAULT_RESET_ATTEMPTS 15
#define MAX_COMM_ERRORS 100

typedef struct
{
    motor_state_t state;
    int init_step;
    int init_wait;
    int fault_reset_attempts;
    int consecutive_comm_errors;
    int32_t current_velocity;
    int32_t target_velocity;
    bool new_setpoint_active;
    uint16_t last_reported_fault_id;
} motor_control_state_t;

typedef struct
{
    int32_t left_velocity;
    int32_t right_velocity;
} differential_velocities_t;

void* motor_control_cyclic_task(void* arg);
differential_velocities_t calculate_differential_drive(int x_axis, int y_axis, int speed_mode);
int32_t map_joystick_to_velocity(int joystick_value, int speed_mode);
void log_motor_status(rxpdo_t* rxpdo[], txpdo_t* txpdo[], differential_velocities_t velocities);

void init_motor_control_state(motor_control_state_t* state);
bool handle_fault_state(rxpdo_t* rxpdo, uint16_t statusword, motor_control_state_t* state, int motor_index);
bool attempt_ethercat_reconnection();

#endif