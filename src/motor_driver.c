/**
 * @file motor_driver.c
 * @brief Implementation of the EtherCAT motor driver state machine and control logic
 * 
 * The main control loop runs in a dedicated real-time thread that processes
 * input data from CAN (joystick/buttons) and communicates with the motor 
 * drive via EtherCAT PDOs at a fixed cycle time.
 * 
 */

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
        if (ethercat_init())
        {
            return true;
        }
    }
    return false;
}

bool handle_fault_state(rxpdo_t* rxpdo, uint16_t statusword, motor_control_state_t* state)
{
    if (statusword & SW_FAULT_BIT)
    {
        uint16_t current_state = get_cia402_state(statusword);

        if (state->last_reported_fault_id != current_state)
        {
            printf("Fault detected: %s\n", get_cia402_state_string(current_state));

            printf("Starting reset sequence\n");
            state->last_reported_fault_id = current_state;
            state->fault_reset_attempts = 0;
        }

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

        wkc = ec_receive_processdata(EC_TIMEOUTRET);

        if (wkc > 0)
        {
            // Reset communication error counter when we receive data
            if (g_motor_state.consecutive_comm_errors > 0)
            {
                g_motor_state.consecutive_comm_errors = 0;
                g_motor_state.reconnection_attempts = 0;
            }

            switch (g_motor_state.state)
            {
                case STATE_BOOT:

                    if (txpdo->statusword == 0)
                    {
                        static int zero_statusword_count = 0;

                        rxpdo->controlword = CW_DISABLEVOLTAGE;

                        zero_statusword_count++;

                        // After significant time, proceed anyway
                        if (zero_statusword_count > 500)
                        {
                            g_motor_state.state = STATE_INIT;
                            g_motor_state.init_step = 0;
                            g_motor_state.init_wait = 0;
                        }
                    }
                    else if (txpdo->statusword & SW_FAULT_BIT)
                    {
                        if (handle_fault_state(rxpdo, txpdo->statusword, &g_motor_state))
                        {
                            break;
                        }

                        if (++g_motor_state.init_wait > 100)
                        {
                            g_motor_state.fault_reset_attempts = 0;
                            g_motor_state.init_wait = 0;
                            g_motor_state.state = STATE_INIT;
                        }
                    }
                    else if (txpdo->statusword & SW_SWITCH_ON_DISABLED_BIT)
                    {
                        rxpdo->controlword = CW_SHUTDOWN;
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

                    switch (g_motor_state.init_step)
                    {
                        // Deep reset state
                        case 0:

                            if (g_motor_state.init_wait == 0)
                            {
                                rxpdo->controlword = 0;
                            }
                            else if (g_motor_state.init_wait == 20)
                            {
                                rxpdo->controlword = CW_FAULT_RESET;
                            }
                            else if (g_motor_state.init_wait == 40)
                            {
                                rxpdo->controlword = CW_DISABLEVOLTAGE;
                            }
                            else if (g_motor_state.init_wait == 60)
                            {
                                rxpdo->controlword = CW_FAULT_RESET;
                            }
                            else if (g_motor_state.init_wait == 80)
                            {
                                rxpdo->controlword = CW_SHUTDOWN;
                            }

                            if (++g_motor_state.init_wait > 100)
                            {
                                g_motor_state.init_step = 1;
                                g_motor_state.init_wait = 0;
                            }
                            break;

                        case 1:
                            rxpdo->controlword = CW_SHUTDOWN;

                            uint8_t state_bits = get_cia402_state(txpdo->statusword);

                            if ((state_bits & ~SW_VOLTAGE_ENABLED_BIT) == get_cia402_state_code("Ready To Switch On"))
                            {
                                g_motor_state.init_step = 2;
                            }
                            else if (++g_motor_state.init_wait > 2000)
                            {
                                g_motor_state.init_step = 0;
                                g_motor_state.init_wait = 0;
                            }
                            break;

                        case 2:
                            rxpdo->controlword = CW_SWITCHON;

                            state_bits = get_cia402_state(txpdo->statusword);

                            if (state_bits == get_cia402_state_code("Switched On"))
                            {
                                g_motor_state.init_step = 3;
                            }
                            break;

                        case 3:
                            if (txpdo->op_mode_display == rxpdo->op_mode || ++g_motor_state.init_wait > 500)
                            {
                                g_motor_state.init_step = 4;
                                g_motor_state.init_wait = 0;
                            }
                            break;

                        case 4:
                            rxpdo->controlword = CW_ENABLE;

                            state_bits = get_cia402_state(txpdo->statusword);

                            if (state_bits == get_cia402_state_code("Operation Enabled"))
                            {
                                g_motor_state.state = STATE_OPERATIONAL;
                                g_motor_state.new_setpoint_active = false;
                            }
                            break;
                    }
                    break;

                case STATE_OPERATIONAL:

                    uint16_t controlword = CW_ENABLE;

                    if (!(txpdo->statusword & SW_OPERATION_ENABLED_BIT))
                    {
                        printf("Warning: Operation disabled during operation, reinitializing\n");

                        g_motor_state.state = STATE_INIT;
                        g_motor_state.init_step = 0;
                        g_motor_state.init_wait = 0;
                        break;
                    }

                    if (txpdo->statusword & SW_OPERATION_ENABLED_BIT)
                    {
                        int y_axis = get_can_y_axis();
                        int speed_mode = get_can_speed();
                        int estop = get_can_estop();
                        int enable = get_can_enable();

                        if (!estop)
                        {
                            g_motor_state.target_velocity = 0;
                        }

                        else if (enable)
                        {
                            g_motor_state.target_velocity = map_joystick_to_velocity(y_axis, speed_mode);
                        }
                        else
                        {
                            rxpdo->target_velocity = 0;
                        }

                        if (g_motor_state.target_velocity != g_motor_state.current_velocity)
                        {
                            rxpdo->target_velocity = g_motor_state.target_velocity;
                            g_motor_state.current_velocity = g_motor_state.target_velocity;

                            controlword |= CW_NEW_VELOCITY_SETPOINT;
                            g_motor_state.new_setpoint_active = true;
                        }
                        else if (g_motor_state.new_setpoint_active)
                        {
                            if (txpdo->statusword & SW_TARGET_REACHED_BIT)
                            {
                                controlword &= ~CW_NEW_VELOCITY_SETPOINT;
                                g_motor_state.new_setpoint_active = false;
                            }
                        }

                        rxpdo->controlword = controlword;

                        static int log_interval = 0;
                        if (++log_interval >= 250)
                        {
                            uint16_t current_state = get_cia402_state(txpdo->statusword);
                            const char* state_string = get_cia402_state_string(current_state);
                            read_drive_parameter(g_motor_control.slave_index, 0x6076, 0x00, "MaxTorque", "raw");
                            printf("Target: %-5d rpm | Velocity: %-5d rpm | Torque: %-5d mNm | Status: %-20s\n",
                                   rxpdo->target_velocity,
                                   txpdo->velocity_actual,
                                   convert_to_mNm(txpdo->torque_actual),
                                   state_string);

                            log_interval = 0;
                        }
                    }
                    break;
            }
        }
        else
        {
            g_motor_state.consecutive_comm_errors++;

            if (g_motor_state.consecutive_comm_errors >= MAX_COMM_ERRORS && !ec_slave[g_motor_control.slave_index].islost)
            {
                ec_slave[g_motor_control.slave_index].islost = TRUE;
                printf("Connection lost\n");
                clock_gettime(CLOCK_MONOTONIC, &g_motor_state.last_reconnect_attempt);
            }

            if (ec_slave[g_motor_control.slave_index].islost)
            {
                struct timespec current_time;
                clock_gettime(CLOCK_MONOTONIC, &current_time);

                if (!g_motor_state.reconnect_in_progress
                    && (current_time.tv_sec - g_motor_state.last_reconnect_attempt.tv_sec) >= 5)
                {
                    g_motor_state.reconnection_attempts++;
                    g_motor_state.reconnect_in_progress = true;

                    printf("Attempting to recover..\n");

                    if (attempt_ethercat_reconnection(&g_motor_state, rxpdo))
                    {
                        g_motor_state.consecutive_comm_errors = 0;
                        g_motor_state.state = STATE_BOOT;
                        g_motor_state.init_step = 0;
                        g_motor_state.init_wait = 0;
                        g_motor_state.fault_reset_attempts = 0;
                        g_motor_state.reconnection_attempts = 0;
                    }

                    g_motor_state.reconnect_in_progress = false;
                    g_motor_state.last_reconnect_attempt = current_time;
                }
            }
        }

        ec_send_processdata();
    }

    printf("Shutting down motor\n");
    int shutdown_cycles = 100;

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

        rxpdo->target_velocity = 0;
        rxpdo->controlword = CW_DISABLEOPERATION;

        ec_send_processdata();
    }

    return NULL;
}