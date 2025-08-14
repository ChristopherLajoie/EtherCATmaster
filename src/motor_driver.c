/**
 * @file motor_driver.c
 * @brief Motor driver implementation - NXP Yocto RT version
 *
 * Adds Quick-Stop → Disable Voltage sequence when enable switch is released
 */

#include "motor_driver.h"
#include "hardware_io.h"
#include "performance_monitor.h" 
#include <time.h>
#include <signal.h>

motor_control_state_t g_motor_state[MAX_MOTORS];
int32_t max_vel = 0;

void init_motor_control_state(motor_control_state_t* state)
{
    if (state == NULL)
        return;

    memset(state, 0, sizeof(*state));
    state->state = STATE_BOOT;
}

int32_t map_joystick_to_velocity(int joystick_value, int speed_mode)
{
    int deadzone_min = JOYSTICK_CENTER - JOYSTICK_DEADZONE;
    int deadzone_max = JOYSTICK_CENTER + JOYSTICK_DEADZONE;

    if (joystick_value >= deadzone_min && joystick_value <= deadzone_max)
    {
        return 0;
    }

    max_vel = speed_mode ? (MAX_VELOCITY * 0.9) : (MAX_VELOCITY / 2);
    int32_t rpm = 0;

    if (joystick_value > deadzone_max)
    {
        rpm = (int32_t)((joystick_value - deadzone_max) * max_vel / (JOYSTICK_MAX - deadzone_max));
    }
    else if (joystick_value < deadzone_min)
    {
        rpm = -(int32_t)((deadzone_min - joystick_value) * max_vel / (deadzone_min - JOYSTICK_MIN));
    }

    return rpm;
}

differential_velocities_t calculate_differential_drive(int x_axis, int y_axis, int speed_mode)
{
    differential_velocities_t result = {0, 0};

    int32_t max_vel = speed_mode ? (MAX_VELOCITY * 0.9f) : (MAX_VELOCITY / 2);
    float speedFactor = speed_mode ? 1.0f : 2.0f;

    float x_norm = 2.0f * (float)(x_axis - JOYSTICK_MIN) / (JOYSTICK_MAX - JOYSTICK_MIN) - 1.0f;
    float y_norm = 2.0f * (float)(y_axis - JOYSTICK_MIN) / (JOYSTICK_MAX - JOYSTICK_MIN) - 1.0f;

    int deadzone_threshold = JOYSTICK_DEADZONE;
    if (abs(x_axis - JOYSTICK_CENTER) < deadzone_threshold)
        x_norm = 0.0f;
    if (abs(y_axis - JOYSTICK_CENTER) < deadzone_threshold)
        y_norm = 0.0f;

    int32_t left_vel = 0;
    int32_t right_vel = 0;

    int32_t turn_component = (int32_t)(x_norm * max_vel / speedFactor);

    if (y_norm >= 0.1f)
    {
        int32_t forward_component = (int32_t)(y_norm * max_vel / speedFactor);

        if (x_norm <= 0.0f)
        {
            right_vel = forward_component;
            left_vel = right_vel + turn_component;
        }
        else
        {
            left_vel = forward_component;
            right_vel = left_vel - turn_component;
        }
    }
    else if (y_norm <= -0.1f)
    {
        int32_t backward_component = (int32_t)(y_norm * max_vel / speedFactor);

        if (x_norm <= 0.0f)
        {
            right_vel = backward_component;
            left_vel = right_vel - turn_component;
        }
        else
        {
            left_vel = backward_component;
            right_vel = left_vel + turn_component;
        }
    }
    else
    {
        left_vel = turn_component;
        right_vel = -turn_component;
    }

    if (left_vel > max_vel)
        left_vel = max_vel;
    if (left_vel < -max_vel)
        left_vel = -max_vel;
    if (right_vel > max_vel)
        right_vel = max_vel;
    if (right_vel < -max_vel)
        right_vel = -max_vel;

    result.left_velocity = left_vel;
    result.right_velocity = right_vel;

    if (g_config.reverse_left_motor)
    {
        result.left_velocity = -result.left_velocity;
    }
    if (g_config.reverse_right_motor)
    {
        result.right_velocity = -result.right_velocity;
    }

    return result;
}

void log_motor_status(rxpdo_t* rxpdo[], txpdo_t* txpdo[], differential_velocities_t velocities)
{
    (void)rxpdo; // Unused parameter

    if (g_motor_control.num_motors >= 2)
    {
        int32_t left_actual = txpdo[LEFT_MOTOR]->velocity_actual;
        int32_t right_actual = txpdo[RIGHT_MOTOR]->velocity_actual;
        int32_t left_torque = convert_to_mNm(txpdo[LEFT_MOTOR]->torque_actual);
        int32_t right_torque = convert_to_mNm(txpdo[RIGHT_MOTOR]->torque_actual);

        int32_t left_target = velocities.left_velocity;
        int32_t right_target = velocities.right_velocity;

        // Apply reverse correction for display
        if (g_config.reverse_left_motor)
        {
            left_actual = -left_actual;
            left_target = -left_target;
            left_torque = -left_torque;
        }
        if (g_config.reverse_right_motor)
        {
            right_actual = -right_actual;
            right_target = -right_target;
            right_torque = -right_torque;
        }

        printf("\nL:%4d/%4d R:%4d/%4d rpm| Torque: L=%4d R=%4d mNm\n",
               left_actual,
               left_target,
               right_actual,
               right_target,
               left_torque,
               right_torque);

        for (int motor = 0; motor < 2; motor++)
        {
            uint16_t current_state = get_cia402_state(txpdo[motor]->statusword);
            const char* state_string = get_cia402_state_string(current_state);
            const char* motor_name = (motor == LEFT_MOTOR) ? "Left " : "Right";

            printf("  %s - Status: %-20s| SW: 0x%04X\n", motor_name, state_string, txpdo[motor]->statusword);
        }
    }
    else if (g_motor_control.num_motors == 1)
    {
        int32_t actual = txpdo[0]->velocity_actual;
        int32_t target = rxpdo[0]->target_velocity;
        int32_t torque = convert_to_mNm(txpdo[0]->torque_actual);

        // Apply reverse correction for display
        if (g_config.reverse_left_motor)
        {
            actual = -actual;
            target = -target;
            torque = -torque;
        }

        printf("Motor: %4d/%4d rpm | Torque: %4d mNm | Differential Test: L=%4d R=%4d rpm\n",
               actual,
               target,
               torque,
               velocities.left_velocity,
               velocities.right_velocity);

        uint16_t current_state = get_cia402_state(txpdo[0]->statusword);
        const char* state_string = get_cia402_state_string(current_state);
        printf("Status: %-20s\n", state_string);
    }
}

bool attempt_ethercat_reconnection()
{
    ec_close(); 

    int max_retries = 20; 
    for (int i = 0; i < max_retries; i++)
    {
        usleep(200000); // 200ms delay between attempts

        printf("  [Attempt %d/%d]\n", i + 1, max_retries);
        if (ethercat_init())
        {
            printf("Recovery successful!\n");
            return true;
        }
    }

    printf("All recovery attempts failed after %d tries.\n", max_retries);
    return false;
}

bool handle_fault_state(rxpdo_t* rxpdo, uint16_t statusword, motor_control_state_t* state, int motor_index)
{
    if (statusword & SW_FAULT_BIT)
    {
        uint16_t current_state = get_cia402_state(statusword);

        if (state->last_reported_fault_id != current_state)
        {
            printf("Motor %d Fault detected: %s\n", motor_index, get_cia402_state_string(current_state));

            printf("  Fault: %s, Ready: %s, Switched On: %s, Op Enabled: %s\n",
                   (statusword & SW_FAULT_BIT) ? "YES" : "NO",
                   (statusword & SW_READY_TO_SWITCH_ON_BIT) ? "YES" : "NO",
                   (statusword & SW_SWITCHED_ON_BIT) ? "YES" : "NO",
                   (statusword & SW_OPERATION_ENABLED_BIT) ? "YES" : "NO");

            // Read the detailed fault codes
            fault_codes_t fault_codes;
            int slave_index = g_motor_control.slave_indices[motor_index];
            if (read_fault_codes(slave_index, &fault_codes))
            {
                printf("CIA-402 Error Code: 0x%04X\n", fault_codes.cia402_error_code);
                printf("Manufacturer Fault String: \"%s\"\n", fault_codes.manufacturer_fault);
            }
            else
            {
                printf("Could not read detailed fault codes\n");
            }

            printf("Starting fault reset sequence\n");
            state->last_reported_fault_id = current_state;
            state->fault_reset_attempts = 0;
        }

        // Keep existing fault reset logic
        static int reset_step = 0;

        switch (reset_step)
        {
            case 0:
                rxpdo->controlword = 0;
                break;
            case 1:
                rxpdo->controlword = CW_FAULT_RESET;
                break;
            case 2:
                rxpdo->controlword = CW_SHUTDOWN;
                break;
            case 3:
                rxpdo->controlword = CW_DISABLEVOLTAGE;
                break;
        }

        reset_step = (reset_step + 1) % 4;
        state->fault_reset_attempts++;

        if (state->fault_reset_attempts > 60)
        {
            printf("Motor %d fault reset unsuccessful after %d attempts, proceeding to init\n",
                   motor_index,
                   state->fault_reset_attempts);
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

    performance_metrics_t perf_metrics;

    int expected_wkc = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    perf_init(&perf_metrics, g_motor_control.cycletime);

    rxpdo_t* rxpdo[MAX_MOTORS];
    txpdo_t* txpdo[MAX_MOTORS];

    for (int i = 0; i < g_motor_control.num_motors; i++)
    {
        rxpdo[i] = g_motor_control.rxpdo[i];
        txpdo[i] = g_motor_control.txpdo[i];
        init_motor_control_state(&g_motor_state[i]);
    }

    g_motor_control.reconnect_in_progress = false;
    g_motor_control.reconnection_attempts = 0;
    clock_gettime(CLOCK_MONOTONIC, &g_motor_control.last_reconnect_attempt);

    int cycletime = g_motor_control.cycletime;
    int64_t cycletime_ns = cycletime * 1000;
    struct timespec ts, tleft;
    int wkc;

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

        perf_cycle_start(&perf_metrics);

        // Calculate next cycle time
        ts.tv_nsec += cycletime_ns;
        if (ts.tv_nsec >= 1000000000)
        {
            ts.tv_sec++;
            ts.tv_nsec -= 1000000000;
        }

        wkc = ec_receive_processdata(EC_TIMEOUTRET);

        perf_receive_complete(&perf_metrics, wkc, expected_wkc);

        if (wkc > 0)
        {
            for (int motor = 0; motor < g_motor_control.num_motors; motor++)
            {
                if (g_motor_state[motor].consecutive_comm_errors > 0)
                {
                    g_motor_state[motor].consecutive_comm_errors = 0;
                }
            }

            if (g_motor_control.reconnection_attempts > 0)
            {
                g_motor_control.reconnection_attempts = 0;
            }

            int x_axis = get_can_x_axis();
            int y_axis = get_can_y_axis();
            int speed_mode = get_can_speed();
            int estop = get_can_estop();
            int enable = get_can_enable();

            differential_velocities_t velocities = calculate_differential_drive(x_axis, y_axis, speed_mode);

            for (int motor = 0; motor < g_motor_control.num_motors; motor++)
            {
                switch (g_motor_state[motor].state)
                {
                    case STATE_BOOT:
                        if (txpdo[motor]->statusword == 0)
                        {
                            static int zero_statusword_count[MAX_MOTORS] = {0};
                            rxpdo[motor]->controlword = CW_DISABLEVOLTAGE;
                            zero_statusword_count[motor]++;

                            // After significant time, proceed anyway
                            if (zero_statusword_count[motor] > 500)
                            {
                                g_motor_state[motor].state = STATE_INIT;
                                g_motor_state[motor].init_step = 0;
                                g_motor_state[motor].init_wait = 0;
                            }
                        }
                        else if (txpdo[motor]->statusword & SW_FAULT_BIT)
                        {
                            if (handle_fault_state(rxpdo[motor], txpdo[motor]->statusword, &g_motor_state[motor], motor))
                            {
                                break;
                            }

                            if (++g_motor_state[motor].init_wait > 100)
                            {
                                g_motor_state[motor].fault_reset_attempts = 0;
                                g_motor_state[motor].init_wait = 0;
                                g_motor_state[motor].state = STATE_INIT;
                            }
                        }
                        else if (txpdo[motor]->statusword & SW_SWITCH_ON_DISABLED_BIT)
                        {
                            rxpdo[motor]->controlword = CW_SHUTDOWN;
                            if (++g_motor_state[motor].init_wait > 50)
                            {
                                g_motor_state[motor].state = STATE_INIT;
                                g_motor_state[motor].init_step = 0;
                                g_motor_state[motor].init_wait = 0;
                            }
                        }
                        else
                        {
                            g_motor_state[motor].state = STATE_INIT;
                            g_motor_state[motor].init_step = 0;
                            g_motor_state[motor].init_wait = 0;
                        }
                        break;

                    case STATE_INIT:
                        switch (g_motor_state[motor].init_step)
                        {
                            // Deep reset state
                            case 0:
                                if (g_motor_state[motor].init_wait == 0)
                                {
                                    rxpdo[motor]->controlword = 0;
                                }
                                else if (g_motor_state[motor].init_wait == 20)
                                {
                                    rxpdo[motor]->controlword = CW_FAULT_RESET;
                                }
                                else if (g_motor_state[motor].init_wait == 40)
                                {
                                    rxpdo[motor]->controlword = CW_DISABLEVOLTAGE;
                                }
                                else if (g_motor_state[motor].init_wait == 60)
                                {
                                    rxpdo[motor]->controlword = CW_FAULT_RESET;
                                }
                                else if (g_motor_state[motor].init_wait == 80)
                                {
                                    rxpdo[motor]->controlword = CW_SHUTDOWN;
                                }

                                if (++g_motor_state[motor].init_wait > 100)
                                {
                                    g_motor_state[motor].init_step = 1;
                                    g_motor_state[motor].init_wait = 0;
                                }
                                break;

                            case 1:
                                rxpdo[motor]->controlword = CW_SHUTDOWN;

                                uint8_t state_bits = get_cia402_state(txpdo[motor]->statusword);

                                if ((state_bits & ~SW_VOLTAGE_ENABLED_BIT) == get_cia402_state_code("Ready To Switch On"))
                                {
                                    g_motor_state[motor].init_step = 2;
                                }
                                else if (++g_motor_state[motor].init_wait > 2000)
                                {
                                    g_motor_state[motor].init_step = 0;
                                    g_motor_state[motor].init_wait = 0;
                                }
                                break;

                            case 2:
                                rxpdo[motor]->controlword = CW_SWITCHON;

                                state_bits = get_cia402_state(txpdo[motor]->statusword);

                                if (state_bits == get_cia402_state_code("Switched On"))
                                {
                                    g_motor_state[motor].init_step = 3;
                                }
                                break;

                            case 3:
                                if (txpdo[motor]->op_mode_display == rxpdo[motor]->op_mode
                                    || ++g_motor_state[motor].init_wait > 500)
                                {
                                    g_motor_state[motor].init_step = 4;
                                    g_motor_state[motor].init_wait = 0;
                                }
                                break;

                            case 4:
                                rxpdo[motor]->controlword = CW_ENABLE;

                                state_bits = get_cia402_state(txpdo[motor]->statusword);

                                if (state_bits == get_cia402_state_code("Operation Enabled"))
                                {
                                    g_motor_state[motor].state = STATE_OPERATIONAL;
                                }
                                break;
                        }
                        break;

                    case STATE_OPERATIONAL:
                    {
                        uint16_t controlword = CW_ENABLE;

                        if (txpdo[motor]->statusword & SW_FAULT_BIT)
                        {
                            printf("Motor %d: Fault detected during operation\n", motor);
                            if (handle_fault_state(rxpdo[motor], txpdo[motor]->statusword, &g_motor_state[motor], motor))
                            {
                                break;  // Exit the state handler, fault is being handled
                            }
                        }

                        // Check for state mismatch
                        if (!(txpdo[motor]->statusword & SW_OPERATION_ENABLED_BIT))
                        {
                            printf("Warning: Motor %d Operation disabled during operation, reinitializing\n", motor);
                            g_motor_state[motor].state = STATE_BOOT;
                            g_motor_state[motor].init_step = 0;
                            g_motor_state[motor].init_wait = 0;
                            break;
                        }

                        if (txpdo[motor]->statusword & SW_OPERATION_ENABLED_BIT)
                        {
                            if (enable)
                            {
                                if (motor == LEFT_MOTOR)
                                {
                                    g_motor_state[motor].target_velocity = velocities.left_velocity;
                                }
                                else if (motor == RIGHT_MOTOR)
                                {
                                    g_motor_state[motor].target_velocity = velocities.right_velocity;
                                }
                            }
                            else if (!enable || !estop)  // estop logic inversed
                            {
                                // Enable switched off
                                rxpdo[motor]->controlword = CW_QUICKSTOP;
                                rxpdo[motor]->target_velocity = 0;

                                if (!(txpdo[motor]->statusword & SW_QUICK_STOP_BIT))
                                {
                                    rxpdo[motor]->controlword = CW_DISABLEVOLTAGE;
                                    g_motor_state[motor].state = STATE_BOOT;
                                    g_motor_state[motor].init_step = 0;
                                    g_motor_state[motor].init_wait = 0;
                                }
                                continue;
                            }

                            if (g_motor_state[motor].target_velocity != g_motor_state[motor].current_velocity)
                            {
                                rxpdo[motor]->target_velocity = g_motor_state[motor].target_velocity;
                                g_motor_state[motor].current_velocity = g_motor_state[motor].target_velocity;

                                controlword |= CW_NEW_VELOCITY_SETPOINT;
                                g_motor_state[motor].new_setpoint_active = true;
                            }
                            else if (g_motor_state[motor].new_setpoint_active)
                            {
                                if (txpdo[motor]->statusword & SW_TARGET_REACHED_BIT)
                                {
                                    controlword &= ~CW_NEW_VELOCITY_SETPOINT;
                                    g_motor_state[motor].new_setpoint_active = false;
                                }
                            }

                            rxpdo[motor]->controlword = controlword;
                        }
                    }
                    break;
                }
            }

            static int log_interval = 0;
            if (++log_interval >= 100)
            {
                log_motor_status(rxpdo, txpdo, velocities);
                perf_log_output(&perf_metrics); 
                log_interval = 0;
            }
        }
        else
        {
            for (int motor = 0; motor < g_motor_control.num_motors; motor++)
            {
                g_motor_state[motor].consecutive_comm_errors++;

                if (g_motor_state[motor].consecutive_comm_errors >= MAX_COMM_ERRORS
                    && !ec_slave[g_motor_control.slave_indices[motor]].islost)
                {
                    ec_slave[g_motor_control.slave_indices[motor]].islost = TRUE;
                    printf("Motor %d Connection lost\n", motor);
                    clock_gettime(CLOCK_MONOTONIC, &g_motor_control.last_reconnect_attempt);
                }
            }

            bool any_lost = false;
            for (int motor = 0; motor < g_motor_control.num_motors; motor++)
            {
                if (ec_slave[g_motor_control.slave_indices[motor]].islost)
                {
                    any_lost = true;
                    break;
                }
            }

            if (any_lost)
            {
                struct timespec current_time;
                clock_gettime(CLOCK_MONOTONIC, &current_time);

                if (!g_motor_control.reconnect_in_progress)
                {
                    g_motor_control.reconnection_attempts++;
                    g_motor_control.reconnect_in_progress = true;

                    if (attempt_ethercat_reconnection())
                    {
                        expected_wkc = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

                        for (int motor = 0; motor < g_motor_control.num_motors; motor++)
                        {
                            g_motor_state[motor].consecutive_comm_errors = 0;
                            g_motor_state[motor].state = STATE_BOOT;
                            g_motor_state[motor].init_step = 0;
                            g_motor_state[motor].init_wait = 0;
                            g_motor_state[motor].fault_reset_attempts = 0;
                        }
                        g_motor_control.reconnection_attempts = 0;
                    }
                    else
                    {
                        printf("EtherCAT reconnection failed\n");
                    }

                    g_motor_control.reconnect_in_progress = false;
                    g_motor_control.last_reconnect_attempt = current_time;
                }
            }
        }

        ec_send_processdata();

        perf_send_complete(&perf_metrics);
    }

    printf("Shutting down motor(s)\n");
    int shutdown_cycles = 200;

    while (shutdown_cycles-- > 0)
    {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);

        // Next cycle time
        ts.tv_nsec += cycletime_ns;
        if (ts.tv_nsec >= 1000000000)
        {
            ts.tv_sec++;
            ts.tv_nsec -= 1000000000;
        }

        ec_receive_processdata(EC_TIMEOUTRET);

        for (int motor = 0; motor < g_motor_control.num_motors; motor++)
        {
            if (shutdown_cycles > 100)
            {
                rxpdo[motor]->target_velocity = 0;
                rxpdo[motor]->controlword = CW_QUICKSTOP;
            }
            else
            {
                rxpdo[motor]->target_velocity = 0;
                rxpdo[motor]->controlword = CW_DISABLEVOLTAGE;
            }
        }

        ec_send_processdata();
    }

    printf("Motor shutdown sequence complete\n");
    return NULL;
}