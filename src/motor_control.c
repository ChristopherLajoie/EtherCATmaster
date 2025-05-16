#include "motor_control.h"
#include "cia402_state.h"
#include <time.h>

// Simplified state machine with minimal error handling
void *motor_control_cyclic_task(void *arg)
{
    (void)arg; // Unused parameter

    rxpdo_t *rxpdo = g_motor_control.rxpdo;
    txpdo_t *txpdo = g_motor_control.txpdo;
    int cycletime = g_motor_control.cycletime;
    int64_t cycletime_ns = cycletime * 1000;
    struct timespec ts, tleft;
    int wkc;

    // State for keeping track of initialization
    enum {
        STATE_INIT,
        STATE_OPERATIONAL
    } control_state = STATE_INIT;

    // Initialization counters
    int init_step = 0;
    int init_wait = 0;

    // Time synchronization setup
    clock_gettime(CLOCK_MONOTONIC, &ts);
    ts.tv_nsec += cycletime_ns;
    if (ts.tv_nsec >= 1000000000) {
        ts.tv_sec++;
        ts.tv_nsec -= 1000000000;
    }

    printf("Motor control thread started\n");

    // Main control loop
    while (g_motor_control.run)
    {
        // DC synchronization
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);

        // Calculate next cycle time
        ts.tv_nsec += cycletime_ns;
        if (ts.tv_nsec >= 1000000000) {
            ts.tv_sec++;
            ts.tv_nsec -= 1000000000;
        }

        // Read process data
        wkc = ec_receive_processdata(EC_TIMEOUTRET);

        if (wkc > 0) {
            // Process based on state
            switch (control_state) {
                case STATE_INIT:
                    // Simple initialization sequence
                    switch (init_step) {
                        case 0: // Reset any errors and set shutdown state
                            rxpdo->controlword = CW_FAULT_RESET;
                            if (++init_wait > 50) {
                                init_step = 1;
                                init_wait = 0;
                                printf("Starting motor drive initialization\n");
                            }
                            break;

                        case 1: // Shutdown state
                            rxpdo->controlword = CW_SHUTDOWN;
                            if ((txpdo->statusword & 0x006F) == 0x0021) { // Ready to switch on
                                init_step = 2;
                                printf("Drive is ready to switch on\n");
                            }
                            break;

                        case 2: // Switch on
                            rxpdo->controlword = CW_SWITCHON;
                            if ((txpdo->statusword & 0x006F) == 0x0023) { // Switched on
                                init_step = 3;
                                printf("Drive is switched on\n");
                            }
                            break;

                        case 3: // Set operation mode
                            rxpdo->op_mode = OP_MODE_CSV; // Cyclic Synchronous Velocity
                            if (txpdo->op_mode_display == OP_MODE_CSV || ++init_wait > 100) {
                                init_step = 4;
                                init_wait = 0;
                                printf("Operation mode set\n");
                            }
                            break;

                        case 4: // Enable operation
                            rxpdo->controlword = CW_ENABLE;
                            if ((txpdo->statusword & 0x006F) == 0x0027) { // Operation enabled
                                printf("Drive is now fully operational\n");
                                control_state = STATE_OPERATIONAL;
                            }
                            break;
                    }
                    break;

                case STATE_OPERATIONAL:
                    // Maintain enabled state
                    rxpdo->controlword = CW_ENABLE;

                    // Only process commands if operation is enabled
                    if (txpdo->statusword & SW_OPERATION_ENABLED_BIT) {
                        // Get joystick values and button states
                        int y_axis = get_can_y_axis();    // 4-252, 128 is center
                        int speed_mode = get_can_speed(); // 0 = normal, 1 = reduced speed
                        int estop = get_can_estop();      // 0 = normal, 1 = emergency stop
                        int enable = get_can_enable();    // 0 = disabled, 1 = enabled

                        // Handle e-stop: If e-stop is NOT active, we can move
                        if (!estop) {
                            rxpdo->target_velocity = 0;
                        }
                        // Handle enable: Only move if enabled
                        else if (enable) {
                            // Map joystick Y-axis to velocity
                            int32_t target_velocity = map_joystick_to_velocity(y_axis, speed_mode);

                            // Apply a very conservative ramp to limit acceleration/deceleration
                            static int32_t current_velocity = 0;
                            if (target_velocity > current_velocity) {
                                current_velocity += 1; // Limit to 1 RPM increase per cycle
                            } else if (target_velocity < current_velocity) {
                                current_velocity -= 1; // Limit to 1 RPM decrease per cycle
                            }

                            // Set the final velocity
                            rxpdo->target_velocity = current_velocity;
                        } else {
                            // Disabled state - no movement
                            rxpdo->target_velocity = 0;
                        }

                        // Log values at regular intervals
                        static int log_interval = 0;
                        if (++log_interval >= 250) { // Every second (at 250Hz)
                            // Print status values
                            printf("Joystick: Y=%d | Speed Mode=%d | E-Stop=%d | Enable=%d\n",
                                y_axis, speed_mode, estop, enable);

                            printf("CSV: Target=%d | Actual=%d | Status=0x%04X\n",
                                rxpdo->target_velocity, txpdo->velocity_actual, txpdo->statusword);
                            
                            // Reset log interval
                            log_interval = 0;
                        }
                    } else {
                        // If operation not enabled, try re-enabling
                        printf("Warning: Operation not enabled (0x%04X), reinitializing\n", txpdo->statusword);
                        control_state = STATE_INIT;
                        init_step = 0;
                        init_wait = 0;
                    }
                    break;
            }
        } else {
            // Communication error handling
            static int comm_errors = 0;
            if (++comm_errors % 1000 == 0) {
                printf("Communication error: %d consecutive errors\n", comm_errors);
            }
        }

        // Send process data
        ec_send_processdata();
    }

    // Shutdown sequence
    printf("Shutting down motor\n");
    int shutdown_cycles = 100;
    
    while (shutdown_cycles-- > 0) {
        // DC synchronization
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
        
        // Next cycle time
        ts.tv_nsec += cycletime_ns;
        if (ts.tv_nsec >= 1000000000) {
            ts.tv_sec++;
            ts.tv_nsec -= 1000000000;
        }
        
        // Read process data
        ec_receive_processdata(EC_TIMEOUTRET);
        
        // Stop motor and disable operations
        rxpdo->target_velocity = 0;
        rxpdo->controlword = CW_DISABLEOPERATION;
        
        // Send process data
        ec_send_processdata();
    }

    printf("Motor control thread finished\n");
    return NULL;
}

int32_t map_joystick_to_velocity(int joystick_value, int speed_mode)
{
    // Define deadzone range around center
    int deadzone_min = JOYSTICK_CENTER - JOYSTICK_DEADZONE;
    int deadzone_max = JOYSTICK_CENTER + JOYSTICK_DEADZONE;

    // If joystick is in deadzone (center position ±10), return 0
    if (joystick_value >= deadzone_min && joystick_value <= deadzone_max)
    {
        return 0;
    }

    // Determine max velocity based on speed mode
    int32_t max_rpm = speed_mode ? (MAX_VELOCITY - 50) : (MAX_VELOCITY / 2);

    // Map the joystick value to the velocity range (in RPM)
    int32_t rpm = 0;

    if (joystick_value > deadzone_max)
    {
        // Forward movement (joystick above center + deadzone)
        // Map from (deadzone_max to JOYSTICK_MAX) to (0 to max_rpm)
        rpm = (int32_t)((joystick_value - deadzone_max) * max_rpm / (JOYSTICK_MAX - deadzone_max));
    }
    else if (joystick_value < deadzone_min)
    {
        // Backward movement (joystick below center - deadzone)
        // Map from (JOYSTICK_MIN to deadzone_min) to (-max_rpm to 0)
        rpm = -(int32_t)((deadzone_min - joystick_value) * max_rpm / (deadzone_min - JOYSTICK_MIN));
    }

    return rpm;
}