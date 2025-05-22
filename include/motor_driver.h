#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "common.h"

typedef enum {
    STATE_BOOT,        
    STATE_INIT,        
    STATE_OPERATIONAL 
} motor_state_t;

#define MAX_FAULT_RESET_ATTEMPTS  15   
#define MAX_COMM_ERRORS          100  

typedef struct {
    motor_state_t state;     
    int init_step;           
    int init_wait;          
    int fault_reset_attempts;  
    int consecutive_comm_errors; 
    int32_t current_velocity; 
    int32_t target_velocity;  
    bool new_setpoint_active; 
    bool fault_messages_exhausted;
    bool reconnect_in_progress;
    struct timespec last_reconnect_attempt;
    int status_print_counter;
    int reconnection_attempts; // Add this line
} motor_control_state_t;

void *motor_control_cyclic_task(void *arg);
int32_t map_joystick_to_velocity(int joystick_value, int speed_mode);

void init_motor_control_state(motor_control_state_t *state);
bool handle_fault_state(rxpdo_t *rxpdo, uint16_t statusword, motor_control_state_t *state);
bool transition_to_ready_state(rxpdo_t *rxpdo, uint16_t statusword, motor_control_state_t *state);
bool enable_operation(rxpdo_t *rxpdo, uint16_t statusword, motor_control_state_t *state);
void cia402_decode_statusword(uint16_t statusword);
bool attempt_ethercat_reconnection(motor_control_state_t *state, rxpdo_t *rxpdo);

#endif