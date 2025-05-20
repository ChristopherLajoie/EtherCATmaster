#include "motor_driver.h"
#include <time.h>

void init_motor_control_state(motor_control_state_t *state)
{
    if (state == NULL)
        return;

    state->state = STATE_BOOT;
    state->init_step = 0;
    state->init_wait = 0;
    state->fault_reset_attempts = 0;
    state->connection_lost = false;
    state->consecutive_comm_errors = 0;
    state->current_velocity = 0;
    state->target_velocity = 0;
    state->new_setpoint_active = false;
    state->fault_messages_exhausted = false;
    state->status_print_counter = 0;
}

bool handle_fault_state(rxpdo_t *rxpdo, uint16_t statusword, motor_control_state_t *state)
{
    if (statusword & SW_FAULT_BIT)
    {
        // Only print the initial message once when we first see a fault
        if (!state->fault_messages_exhausted) {
            // Create a descriptive fault message based on statusword bits
            printf("Fault detected (0x%04X): ", statusword);
            
            uint8_t state_bits = statusword & 0x6F; 
            if (state_bits == 0x08)
                printf("Fault State");
            else if (state_bits == 0x0F)
                printf("Fault Reaction Active");
            else if (state_bits == 0x00)
                printf("Not Ready To Switch On");
            else if (state_bits == 0x40)
                printf("Switch On Disabled with Fault");
            else
                printf("Abnormal State with Fault");
             
            printf("\nStarting reset sequence\n");
            state->fault_messages_exhausted = true;
            state->fault_reset_attempts = 0;
        }
        
        // Apply the different reset commands in sequence
        static int deep_reset_step = 0;
        
        // Apply the different reset commands in sequence
        switch (deep_reset_step) {
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
        
        // Move to next step in cycle
        deep_reset_step = (deep_reset_step + 1) % 4;
        
        // Increment attempt counter
        state->fault_reset_attempts++;
        
        // After significant number of cycles, give up and proceed to initialization
        if (state->fault_reset_attempts > 40) {  // 10 complete cycles (4 steps each)
            printf("Reset not succeeding after %d cycles, proceeding to initialization\n", 
                   state->fault_reset_attempts / 4);
            state->fault_reset_attempts = 0;
            state->fault_messages_exhausted = false;
            state->state = STATE_INIT;
            state->init_step = 0;
            state->init_wait = 0;
        }
        
        return true;
    }

    // Clear the flag if no fault is present
    state->fault_messages_exhausted = false;
    return false;
}

void cia402_decode_statusword(uint16_t statusword)
{
    // Basic State Machine bits
    printf("Status 0x%04X - State Machine: ", statusword);

    // Decode the state machine bits (bits 0, 1, 2, 3, 5, 6)
    uint8_t state_bits = statusword & 0x6F; // Mask for bits 0,1,2,3,5,6

    // State machine according to CiA 402
    if (state_bits == 0x00)
        printf("Not ready to switch on\n");
    else if (state_bits == 0x40)
        printf("Switch on disabled\n");
    else if (state_bits == 0x21)
        printf("Ready to switch on\n");
    else if (state_bits == 0x23)
        printf("Switched on\n");
    else if (state_bits == 0x27)
        printf("Operation enabled\n");
    else if (state_bits == 0x07)
        printf("Quick stop active\n");
    else if (state_bits == 0x0F)
        printf("Fault reaction active\n");
    else if (state_bits == 0x08)
        printf("Fault\n");
    else
        printf("Unknown state (0x%02X)\n", state_bits);

    // Row 1: Bits 0-3
    printf("  0:%-20s  1:%-20s  2:%-20s  3:%-20s\n",
           (statusword & (1 << 0)) ? "Ready to switch on" : "Not ready",
           (statusword & (1 << 1)) ? "Switched on" : "Switched off",
           (statusword & (1 << 2)) ? "Operation enabled" : "Op disabled",
           (statusword & (1 << 3)) ? "Fault present" : "No fault");

    // Row 2: Bits 4-7
    printf("  4:%-20s  5:%-20s  6:%-20s  7:%-20s\n",
           (statusword & (1 << 4)) ? "Voltage enabled" : "Voltage disabled",
           (statusword & (1 << 5)) ? "Quick stop disabled" : "Quick stop enabled",
           (statusword & (1 << 6)) ? "Switch on disabled" : "Switch on enabled",
           (statusword & (1 << 7)) ? "Warning present" : "No warning");
}

void *motor_control_cyclic_task(void *arg)
{
    (void)arg; // Unused parameter

    rxpdo_t *rxpdo = g_motor_control.rxpdo;
    txpdo_t *txpdo = g_motor_control.txpdo;
    int cycletime = g_motor_control.cycletime;
    int64_t cycletime_ns = cycletime * 1000;
    struct timespec ts, tleft;
    int wkc;

    motor_control_state_t motor_state;
    init_motor_control_state(&motor_state);

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
            if (motor_state.consecutive_comm_errors > 0)
            {
                motor_state.consecutive_comm_errors = 0;
            }

            // If previously lost connection was restored
            if (motor_state.connection_lost)
            {
                printf("Connection to EtherCAT slave restored, reinitializing...\n");
                motor_state.connection_lost = false;
                // Reset to boot state to fully reinitialize the drive
                motor_state.state = STATE_BOOT;
                motor_state.init_step = 0;
                motor_state.init_wait = 0;
                motor_state.fault_reset_attempts = 0;
            }

            // Process based on state
            switch (motor_state.state)
            {
            case STATE_BOOT:
                // Only print status every 50 cycles (about 200ms at 250Hz)
                if (motor_state.status_print_counter++ % 50 == 0)
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
                        motor_state.state = STATE_INIT;
                        motor_state.init_step = 0;
                        motor_state.init_wait = 0;
                    }
                }
                else if (txpdo->statusword & SW_FAULT_BIT)
                {
                    // Use the helper function to handle fault state
                    if (handle_fault_state(rxpdo, txpdo->statusword, &motor_state))
                    {
                        // If fault handling still in progress, break here
                        break;
                    }

                    // After more attempts, give up and proceed
                    if (++motor_state.init_wait > 100)
                    {
                        motor_state.fault_reset_attempts = 0;
                        motor_state.init_wait = 0;
                        motor_state.state = STATE_INIT;
                        printf("Proceeding to initialization despite fault condition\n");
                    }
                }
                else if (txpdo->statusword & SW_SWITCH_ON_DISABLED_BIT)
                {

                    rxpdo->controlword = CW_SHUTDOWN; // This transitions to "Ready to switch on"
                    if (++motor_state.init_wait > 50)
                    {
                        motor_state.state = STATE_INIT;
                        motor_state.init_step = 0;
                        motor_state.init_wait = 0;
                    }
                }
                else
                {
                    motor_state.state = STATE_INIT;
                    motor_state.init_step = 0;
                    motor_state.init_wait = 0;
                }
                break;

            case STATE_INIT:
                // Simple initialization sequence
                switch (motor_state.init_step)
                {
                case 0: // Start with a proper sequence of commands to ensure clean state
                    if (motor_state.init_wait == 0)
                    {
                        rxpdo->controlword = 0; // All bits off
                    }
                    else if (motor_state.init_wait == 10)
                    {
                        rxpdo->controlword = CW_FAULT_RESET;
                    }
                    else if (motor_state.init_wait == 20)
                    {
                        rxpdo->controlword = CW_SHUTDOWN;
                    }

                    if (++motor_state.init_wait > 50)
                    {

                        motor_state.init_step = 1;
                        motor_state.init_wait = 0;
                    }
                    break;

                case 1: // Shutdown state - wait for "Ready to switch on"
                    rxpdo->controlword = CW_SHUTDOWN;

                    if ((txpdo->statusword & 0x006F) == 0x0021)
                    {
                        motor_state.init_step = 2;
                        printf("Drive is ready to switch on\n");
                    }
                    else if (++motor_state.init_wait > 200)
                    {
                        // If stuck in this state, try more aggressive reset
                        printf("STATE STUCK: Trying more aggressive reset\n");
                        rxpdo->controlword = CW_DISABLEVOLTAGE;
                        if (motor_state.init_wait > 250)
                        {
                            printf("Resetting init sequence - stuck too long\n");
                            motor_state.init_step = 0;
                            motor_state.init_wait = 0;
                        }
                    }
                    break;

                case 2: // Switch on
                    rxpdo->controlword = CW_SWITCHON;
                    if ((txpdo->statusword & 0x006F) == 0x0023)
                    { // Switched on
                        motor_state.init_step = 3;
                        printf("Drive is switched on\n");
                    }
                    break;

                case 3: // Set operation mode
                    if (txpdo->op_mode_display == rxpdo->op_mode || ++motor_state.init_wait > 100)
                    {
                        motor_state.init_step = 4;
                        motor_state.init_wait = 0;
                        printf("Operation mode %d set (%s)\n",
                               rxpdo->op_mode,
                               rxpdo->op_mode == OP_MODE_CSV ? "Cyclic Synchronous Velocity" : "Profile Velocity Mode");
                    }
                    break;

                case 4: // Enable operation
                    rxpdo->controlword = CW_ENABLE;
                    if ((txpdo->statusword & 0x006F) == 0x0027)
                    { // Operation enabled
                        printf("Drive is now fully operational\n");
                        motor_state.state = STATE_OPERATIONAL;
                        motor_state.new_setpoint_active = false;
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
                    cia402_decode_statusword(txpdo->statusword); // Print detailed status
                    motor_state.state = STATE_INIT;
                    motor_state.init_step = 0;
                    motor_state.init_wait = 0;
                    return NULL; // Exit thread on operation disabled
                }

                // Only process commands if operation is enabled
                if (txpdo->statusword & SW_OPERATION_ENABLED_BIT)
                {
                    // Get joystick values and button states
                    int y_axis = get_can_y_axis();    // 4-252, 128 is center
                    int speed_mode = get_can_speed(); // 0 = normal, 1 = reduced speed
                    int estop = get_can_estop();      // 0 = normal, 1 = emergency stop
                    int enable = get_can_enable();    // 0 = disabled, 1 = enabled

                    // Handle e-stop: If e-stop is NOT active, we can move
                    if (!estop)
                    {
                        motor_state.target_velocity = 0;
                    }
                    // Handle enable: Only move if enabled
                    else if (enable)
                    {
                        // Map joystick Y-axis to velocity
                        motor_state.target_velocity = map_joystick_to_velocity(y_axis, speed_mode);
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
                        if (motor_state.target_velocity > motor_state.current_velocity)
                        {
                            motor_state.current_velocity += 5;
                        }
                        else if (motor_state.target_velocity < motor_state.current_velocity)
                        {
                            motor_state.current_velocity -= 5;
                        }

                        // Set the final velocity
                        rxpdo->target_velocity = motor_state.current_velocity;
                    }
                    else if (rxpdo->op_mode == OP_MODE_PVM)
                    {
                        // Check if target velocity has changed
                        if (motor_state.target_velocity != motor_state.current_velocity)
                        {
                            rxpdo->target_velocity = motor_state.target_velocity;
                            motor_state.current_velocity = motor_state.target_velocity;

                            // Start new setpoint (bit 4)
                            controlword |= CW_NEW_SETPOINT;
                            motor_state.new_setpoint_active = true;
                        }
                        else if (motor_state.new_setpoint_active)
                        {
                            // Check if target reached
                            if (txpdo->statusword & SW_TARGET_REACHED_BIT)
                            {
                                // Clear the new setpoint bit once target is reached
                                controlword &= ~CW_NEW_SETPOINT;
                                motor_state.new_setpoint_active = false;
                            }
                        }
                    }

                    // Set the control word with mode-specific bits
                    rxpdo->controlword = controlword;

                    // Log values at regular intervals
                    static int log_interval = 0;
                    if (++log_interval >= 250)
                    { // Every second (at 250Hz)
                        // Print status values
                        printf("Joystick: Y=%d | Speed Mode=%d | E-Stop=%d | Enable=%d\n",
                               y_axis, speed_mode, estop, enable);

                        if (rxpdo->op_mode == OP_MODE_CSV)
                        {
                            printf("CSV: Target=%d | Actual=%d | Status=0x%04X\n",
                                   rxpdo->target_velocity, txpdo->velocity_actual, txpdo->statusword);
                        }
                        else
                        {
                            printf("PVM: Target=%d | Actual=%d | Status=0x%04X | Target Reached=%d\n",
                                   rxpdo->target_velocity, txpdo->velocity_actual, txpdo->statusword,
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
                    motor_state.state = STATE_INIT;
                    motor_state.init_step = 0;
                    motor_state.init_wait = 0;
                }
                break;
            }
        }
        else
        {
            // Track consecutive communication errors
            motor_state.consecutive_comm_errors++;

            // Check if connection is lost
            if (motor_state.consecutive_comm_errors >= MAX_COMM_ERRORS && !motor_state.connection_lost)
            {
                motor_state.connection_lost = true;
                printf("Connection to EtherCAT slave lost after %d consecutive errors\n",
                       motor_state.consecutive_comm_errors);
            }

            // Only log every 1000 errors to avoid flooding the console
            if (motor_state.consecutive_comm_errors % 1000 == 0)
            {
                printf("Communication error\n");
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
    int32_t max_rpm = speed_mode ? (MAX_VELOCITY * 0.9) : (MAX_VELOCITY / 2);

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