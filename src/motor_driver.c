#include "motor_driver.h"
#include "hardware_io.h"
#include <time.h>
#include <signal.h>

motor_control_state_t g_motor_state;

void init_motor_control_state(motor_control_state_t* state)
{
    if (state == NULL)
        return;

    memset(state, 0, sizeof(*state));

    state->state = STATE_BOOT;
    clock_gettime(CLOCK_MONOTONIC, &state->last_reconnect_attempt);
}

int32_t map_joystick_to_velocity(int joystick_value, int speed_mode)
{
    int deadzone_min = JOYSTICK_CENTER - JOYSTICK_DEADZONE;
    int deadzone_max = JOYSTICK_CENTER + JOYSTICK_DEADZONE;

    if (joystick_value >= deadzone_min && joystick_value <= deadzone_max)
    {
        return 0;
    }

    int32_t max_rpm = speed_mode ? (MAX_VELOCITY * 0.9) : (MAX_VELOCITY / 2);
    int32_t rpm = 0;

    if (joystick_value > deadzone_max)
    {
        rpm = (int32_t)((joystick_value - deadzone_max) * max_rpm / (JOYSTICK_MAX - deadzone_max));
    }
    else if (joystick_value < deadzone_min)
    {
        rpm = -(int32_t)((deadzone_min - joystick_value) * max_rpm / (deadzone_min - JOYSTICK_MIN));
    }

    return rpm;
}

bool attempt_ethercat_reconnection()
{
    if (ec_slave[g_motor_control.slave_index].islost)
    {
        bool success = ethercat_init();

        if (success)
        {
            printf("EtherCAT reconnection successful!\n");
            ec_slave[g_motor_control.slave_index].islost = FALSE;
            return true;
        }
    }
    return false;
}

bool handle_fault_state(rxpdo_t* rxpdo, uint16_t statusword, motor_control_state_t* state)
{
    if (statusword & SW_FAULT_BIT)
    {
        cia402_state_t current_state = get_cia402_state(statusword);

        if (state->last_reported_fault_id != (statusword & STATUS_STATE_MASK))
        {
            printf("Fault detected (0x%04X): %s\n", statusword, get_cia402_state_string(current_state));

            printf("Starting reset sequence\n");
            state->last_reported_fault_id = statusword & STATUS_STATE_MASK;
            state->fault_reset_attempts = 0;
        }

        static int reset_step = 0;

        switch (reset_step)
        {
            case 0:
                rxpdo->controlword = 0;  // All bits off
                break;

            case 1:
                rxpdo->controlword = CW_FAULT_RESET;  // Fault reset
                break;

            case 2:
                rxpdo->controlword = CW_SHUTDOWN;  // Shutdown
                break;

            case 3:
                rxpdo->controlword = CW_DISABLEVOLTAGE;  // Disable voltage
                break;
        }

        reset_step = (reset_step + 1) % 4;
        state->fault_reset_attempts++;

        // After significant number of cycles, give up and proceed to initialization
        if (state->fault_reset_attempts > 40)
        {
            printf("Reset not succeeding, proceeding to initialization\n");
            state->fault_reset_attempts = 0;
            state->state = STATE_INIT;
            state->init_step = 0;
            state->init_wait = 0;
        }

        return true;
    }

    state->last_reported_fault_id = 0;
    return false;
}

void* motor_control_cyclic_task(void* arg)
{
    (void)arg;

    rxpdo_t* rxpdo = g_motor_control.rxpdo;
    txpdo_t* txpdo = g_motor_control.txpdo;
    int cycletime = g_motor_control.cycletime;
    int64_t cycletime_ns = cycletime * 1000;
    struct timespec ts, tleft;
    int wkc;

    init_motor_control_state(&g_motor_state);

    clock_gettime(CLOCK_MONOTONIC, &ts);
    ts.tv_nsec += cycletime_ns;
    if (ts.tv_nsec >= 1000000000)
    {
        ts.tv_sec++;
        ts.tv_nsec -= 1000000000;
    }

    // Main control loop
    while (g_motor_control.run)
    {
        // DC synchronization
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);

        // Calculate next cycle time
        ts.tv_nsec += cycletime_ns;
        if (ts.tv_nsec >= 1000000000)
        {
            ts.tv_sec++;
            ts.tv_nsec -= 1000000000;
        }

        // Read process data
        wkc = ec_receive_processdata(EC_TIMEOUTRET);

        if (wkc > 0)
        {
            // Reset communication error counter when we receive data
            if (g_motor_state.consecutive_comm_errors > 0)
            {
                g_motor_state.consecutive_comm_errors = 0;
                g_motor_state.reconnection_attempts = 0;  // Reset reconnection attempts counter
            }

            // If previously lost connection was restored
            if (ec_slave[g_motor_control.slave_index].islost)
            {
                ec_slave[g_motor_control.slave_index].islost = FALSE;
                printf("Connection to EtherCAT slave restored, reinitializing...\n");

                g_motor_state.state = STATE_BOOT;
                g_motor_state.init_step = 0;
                g_motor_state.init_wait = 0;
                g_motor_state.fault_reset_attempts = 0;
            }

            // Process based on state
            switch (g_motor_state.state)
            {
                case STATE_BOOT:
                    // Only print status every 50 cycles (about 200ms at 250Hz)
                    if (g_motor_state.status_print_counter++ % 50 == 0)
                    {
                        printf("Status: [");
                        if (txpdo->statusword == SW_NO_COMMUNICATION)
                        {
                            printf("NO_COMM");
                        }
                        else
                        {
                            bool first_flag = true;

                            if (txpdo->statusword & SW_READY_TO_SWITCH_ON_BIT)
                            {
                                printf("READY");
                                first_flag = false;
                            }

                            if (txpdo->statusword & SW_SWITCHED_ON_BIT)
                            {
                                if (!first_flag)
                                    printf(" | ");
                                printf("SWITCHED_ON");
                                first_flag = false;
                            }

                            if (txpdo->statusword & SW_OPERATION_ENABLED_BIT)
                            {
                                if (!first_flag)
                                    printf(" | ");
                                printf("ENABLED");
                                first_flag = false;
                            }

                            if (txpdo->statusword & SW_FAULT_BIT)
                            {
                                if (!first_flag)
                                    printf(" | ");
                                printf("FAULT");
                                first_flag = false;
                            }

                            if (txpdo->statusword & SW_SWITCH_ON_DISABLED_BIT)
                            {
                                if (!first_flag)
                                    printf(" | ");
                                printf("DISABLED");
                                first_flag = false;
                            }
                        }
                        printf("]\n");
                    }

                    // Handle different states specifically
                    if (txpdo->statusword == 0)
                    {
                        // Handle zero statusword (no valid status)
                        static int zero_statusword_count = 0;

                        // Just alternate between fault reset and shutdown
                        rxpdo->controlword = (zero_statusword_count % 2) ? CW_FAULT_RESET : CW_SHUTDOWN;

                        zero_statusword_count++;
                        // After significant time, proceed anyway
                        if (zero_statusword_count > 500)
                        {
                            printf("Proceeding to initialization despite no valid status\n");
                            g_motor_state.state = STATE_INIT;
                            g_motor_state.init_step = 0;
                            g_motor_state.init_wait = 0;
                        }
                    }
                    else if (txpdo->statusword & SW_FAULT_BIT)
                    {
                        // Use the helper function to handle fault state
                        if (handle_fault_state(rxpdo, txpdo->statusword, &g_motor_state))
                        {
                            // If fault handling still in progress, break here
                            break;
                        }

                        // After more attempts, give up and proceed
                        if (++g_motor_state.init_wait > 100)
                        {
                            g_motor_state.fault_reset_attempts = 0;
                            g_motor_state.init_wait = 0;
                            g_motor_state.state = STATE_INIT;
                            printf("Proceeding to initialization despite fault condition\n");
                        }
                    }
                    else if (txpdo->statusword & SW_SWITCH_ON_DISABLED_BIT)
                    {
                        rxpdo->controlword = CW_SHUTDOWN;  // This transitions to "Ready to switch on"
                        if (++g_motor_state.init_wait > 50)
                        {
                            g_motor_state.state = STATE_INIT;
                            g_motor_state.init_step = 0;
                            g_motor_state.init_wait = 0;
                        }
                    }
                    else
                    {
                        g_motor_state.state = STATE_INIT;
                        g_motor_state.init_step = 0;
                        g_motor_state.init_wait = 0;
                    }
                    break;

                case STATE_INIT:
                    // Simple initialization sequence
                    switch (g_motor_state.init_step)
                    {
                        case 0:  // Start with more robust reset sequence
                            // Print the current drive status for debugging
                            if (g_motor_state.init_wait == 0)
                            {
                                // First step: all bits off
                                rxpdo->controlword = 0;
                            }
                            else if (g_motor_state.init_wait == 20)
                            {
                                // Next try fault reset
                                rxpdo->controlword = CW_FAULT_RESET;
                            }
                            else if (g_motor_state.init_wait == 40)
                            {
                                // Then disable voltage (more aggressive than shutdown)
                                rxpdo->controlword = CW_DISABLEVOLTAGE;
                            }
                            else if (g_motor_state.init_wait == 60)
                            {
                                // Another fault reset
                                rxpdo->controlword = CW_FAULT_RESET;
                            }
                            else if (g_motor_state.init_wait == 80)
                            {
                                // Finally try normal shutdown
                                rxpdo->controlword = CW_SHUTDOWN;
                            }

                            if (++g_motor_state.init_wait > 100)
                            {
                                g_motor_state.init_step = 1;
                                g_motor_state.init_wait = 0;
                            }
                            break;

                        case 1:  // Shutdown state - wait for "Ready to switch on"
                            rxpdo->controlword = CW_SHUTDOWN;

                            // Check for "Ready to switch on" state
                            uint8_t state_bits = txpdo->statusword & 0x006F;
                            if (state_bits == 0x0021 || state_bits == 0x0031)
                            {
                                g_motor_state.init_step = 2;
                                printf("Drive is ready to switch on\n");
                            }
                            else if (++g_motor_state.init_wait > 2000)  // Give much more time (8 seconds at 250Hz)
                            {
                                g_motor_state.init_step = 0;
                                g_motor_state.init_wait = 0;
                            }
                            break;

                        case 2:  // Switch on
                            rxpdo->controlword = CW_SWITCHON;
                            if ((txpdo->statusword & 0x006F) == 0x0023)
                            {  // Switched on
                                g_motor_state.init_step = 3;
                                printf("Drive is switched on\n");
                            }
                            break;

                        case 3:  // Set operation mode
                            if (txpdo->op_mode_display == rxpdo->op_mode || ++g_motor_state.init_wait > 100)
                            {
                                g_motor_state.init_step = 4;
                                g_motor_state.init_wait = 0;
                                printf("Operation mode %d set (%s)\n",
                                       rxpdo->op_mode,
                                       rxpdo->op_mode == OP_MODE_CSV ? "Cyclic Synchronous Velocity" : "Profile Velocity Mode");
                            }
                            break;

                        case 4:  // Enable operation
                            rxpdo->controlword = CW_ENABLE;
                            if ((txpdo->statusword & 0x006F) == 0x0027)
                            {  // Operation enabled
                                printf("Drive is now fully operational\n");
                                g_motor_state.state = STATE_OPERATIONAL;
                                g_motor_state.new_setpoint_active = false;
                            }
                            break;
                    }
                    break;

                case STATE_OPERATIONAL:
                    // Maintain enabled state
                    uint16_t controlword = CW_ENABLE;

                    if (!(txpdo->statusword & SW_OPERATION_ENABLED_BIT))
                    {
                        printf("Warning: Operation disabled during operation (0x%04X), reinitializing\n", txpdo->statusword);
                        // cia402_decode_statusword(txpdo->statusword); // Print detailed status
                        g_motor_state.state = STATE_INIT;
                        g_motor_state.init_step = 0;
                        g_motor_state.init_wait = 0;
                        break;  // Don't exit thread, just try to reinitialize
                    }

                    // Only process commands if operation is enabled
                    if (txpdo->statusword & SW_OPERATION_ENABLED_BIT)
                    {
                        // Get joystick values and button states
                        int y_axis = get_can_y_axis();     // 4-252, 128 is center
                        int speed_mode = get_can_speed();  // 0 = normal, 1 = reduced speed
                        int estop = get_can_estop();       // 0 = normal, 1 = emergency stop
                        int enable = get_can_enable();     // 0 = disabled, 1 = enabled

                        // Handle e-stop: If e-stop is NOT active, we can move
                        if (!estop)
                        {
                            g_motor_state.target_velocity = 0;
                        }
                        // Handle enable: Only move if enabled
                        else if (enable)
                        {
                            // Map joystick Y-axis to velocity
                            g_motor_state.target_velocity = map_joystick_to_velocity(y_axis, speed_mode);
                        }
                        else
                        {
                            // Disabled state - no movement
                            rxpdo->target_velocity = 0;
                        }

                        // Mode-specific handling
                        if (rxpdo->op_mode == OP_MODE_CSV)
                        {
                            // CSV Mode - Apply a very conservative ramp
                            if (g_motor_state.target_velocity > g_motor_state.current_velocity)
                            {
                                g_motor_state.current_velocity += 5;
                            }
                            else if (g_motor_state.target_velocity < g_motor_state.current_velocity)
                            {
                                g_motor_state.current_velocity -= 5;
                            }

                            // Set the final velocity
                            rxpdo->target_velocity = g_motor_state.current_velocity;
                        }
                        else if (rxpdo->op_mode == OP_MODE_PVM)
                        {
                            // Check if target velocity has changed
                            if (g_motor_state.target_velocity != g_motor_state.current_velocity)
                            {
                                rxpdo->target_velocity = g_motor_state.target_velocity;
                                g_motor_state.current_velocity = g_motor_state.target_velocity;

                                // Start new setpoint (bit 4)
                                controlword |= CW_NEW_SETPOINT;
                                g_motor_state.new_setpoint_active = true;
                            }
                            else if (g_motor_state.new_setpoint_active)
                            {
                                // Check if target reached
                                if (txpdo->statusword & SW_TARGET_REACHED_BIT)
                                {
                                    // Clear the new setpoint bit once target is reached
                                    controlword &= ~CW_NEW_SETPOINT;
                                    g_motor_state.new_setpoint_active = false;
                                }
                            }
                        }

                        // Set the control word with mode-specific bits
                        rxpdo->controlword = controlword;

                        // Log values at regular intervals
                        static int log_interval = 0;
                        if (++log_interval >= 250)
                        {  // Every second (at 250Hz)
                            // Print status values
                            printf("Joystick: Y=%d | Speed Mode=%d | E-Stop=%d | Enable=%d\n", y_axis, speed_mode, estop, enable);

                            if (rxpdo->op_mode == OP_MODE_CSV)
                            {
                                printf("CSV: Target=%d | Actual=%d | Status=0x%04X\n",
                                       rxpdo->target_velocity,
                                       txpdo->velocity_actual,
                                       txpdo->statusword);
                            }
                            else
                            {
                                printf("PVM: Target=%d | Actual=%d | Status=0x%04X | Target Reached=%d\n",
                                       rxpdo->target_velocity,
                                       txpdo->velocity_actual,
                                       txpdo->statusword,
                                       (txpdo->statusword & SW_TARGET_REACHED_BIT) ? 1 : 0);
                            }

                            // Reset log interval
                            log_interval = 0;
                        }
                    }
                    else
                    {
                        // If operation not enabled, try re-enabling
                        printf("Warning: Operation not enabled (0x%04X), reinitializing\n", txpdo->statusword);
                        g_motor_state.state = STATE_INIT;
                        g_motor_state.init_step = 0;
                        g_motor_state.init_wait = 0;
                    }
                    break;
            }
        }
        else
        {
            // Increment communication error counter
            g_motor_state.consecutive_comm_errors++;

            // Check if we've exceeded the error threshold AND connection isn't already marked as lost
            if (g_motor_state.consecutive_comm_errors >= MAX_COMM_ERRORS && !ec_slave[g_motor_control.slave_index].islost)
            {
                // Mark the slave as lost for the next iteration
                ec_slave[g_motor_control.slave_index].islost = TRUE;
                printf("Connection to EtherCAT slave lost\n");
                clock_gettime(CLOCK_MONOTONIC, &g_motor_state.last_reconnect_attempt);
            }

            // If connection is lost, periodically attempt reconnection
            if (ec_slave[g_motor_control.slave_index].islost)
            {
                struct timespec current_time;
                clock_gettime(CLOCK_MONOTONIC, &current_time);

                // Attempt reconnection every 5 seconds
                if (!g_motor_state.reconnect_in_progress
                    && (current_time.tv_sec - g_motor_state.last_reconnect_attempt.tv_sec) >= 5)
                {
                    // Increment attempt counter and lock the reconnection flag
                    g_motor_state.reconnection_attempts++;
                    g_motor_state.reconnect_in_progress = true;

                    printf("Attempting to recover EtherCAT connection\n");

                    // Attempt reconnection with our simplified function
                    if (attempt_ethercat_reconnection(&g_motor_state, rxpdo))
                    {
                        g_motor_state.consecutive_comm_errors = 0;
                        g_motor_state.state = STATE_BOOT;
                        g_motor_state.init_step = 0;
                        g_motor_state.init_wait = 0;
                        g_motor_state.fault_reset_attempts = 0;
                        g_motor_state.reconnection_attempts = 0;
                    }

                    // Update timestamp and clear flag regardless of outcome
                    g_motor_state.reconnect_in_progress = false;
                    g_motor_state.last_reconnect_attempt = current_time;
                }
            }
        }

        // Send process data (try even during errors to maintain timing)
        ec_send_processdata();
    }

    // Shutdown sequence
    printf("Shutting down motor\n");
    int shutdown_cycles = 100;

    while (shutdown_cycles-- > 0)
    {
        // DC synchronization
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);

        // Next cycle time
        ts.tv_nsec += cycletime_ns;
        if (ts.tv_nsec >= 1000000000)
        {
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