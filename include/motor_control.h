#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "common.h"
#include "can_monitor.h"

// Map joystick value to motor velocity
int32_t map_joystick_to_velocity(int joystick_value, int speed_mode);

// Main motor control thread
void *motor_control_cyclic_task(void *arg);

#endif // MOTOR_CONTROL_H