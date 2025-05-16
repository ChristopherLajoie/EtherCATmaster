#include "cia402_state.h"

bool cia402_state_init()
{
    // Initialize drive state variables
    g_motor_control.rxpdo->controlword = 0;
    g_motor_control.rxpdo->op_mode = 0;
    g_motor_control.rxpdo->target_velocity = 0;
    g_motor_control.rxpdo->target_torque = 0;
    g_motor_control.rxpdo->torque_offset = 0;
    
    // Get the initial state
    ec_receive_processdata(EC_TIMEOUTRET);
    
    return true;
}

bool cia402_state_process(uint16_t *controlword, uint16_t statusword, int target_state)
{
    // Current state extraction from statusword
    bool ready_to_switch_on = statusword & SW_READY_TO_SWITCH_ON_BIT;
    bool switched_on = statusword & SW_SWITCHED_ON_BIT;
    bool operation_enabled = statusword & SW_OPERATION_ENABLED_BIT;
    bool fault = statusword & SW_FAULT_BIT;
    bool switch_on_disabled = statusword & SW_SWITCH_ON_DISABLED_BIT;

    // Debug output during initialization
    static bool init_printed = false;
    if (!init_printed && target_state != 0)
    {
        printf("Statusword: 0x%04X [RDY:%d, ON:%d, ENABLED:%d, FAULT:%d, DISABLED:%d]\n",
               statusword, ready_to_switch_on, switched_on, operation_enabled,
               fault, switch_on_disabled);
        if (target_state == 2 && operation_enabled)
            init_printed = true;
    }

    // Special case - if statusword is 0, it's likely we have a communication issue
    if (statusword == 0)
    {
        *controlword = CW_SHUTDOWN;
        return false;
    }

    // Handle fault state first (simplest approach)
    if (fault)
    {
        *controlword = CW_FAULT_RESET;
        return false;
    }

    // Simple state machine for basic functionality
    switch (target_state)
    {
    case 1: // Switch on
        if (switch_on_disabled)
        {
            *controlword = CW_SHUTDOWN;
            return false;
        }
        else if (!ready_to_switch_on)
        {
            *controlword = CW_SHUTDOWN;
            return false;
        }
        else if (!switched_on)
        {
            *controlword = CW_SWITCHON;
            return false;
        }
        else
        {
            printf("State: Switched on achieved\n");
            return true;
        }
        break;

    case 2: // Enable operation
        if (switch_on_disabled)
        {
            *controlword = CW_SHUTDOWN;
            return false;
        }
        else if (!ready_to_switch_on)
        {
            *controlword = CW_SHUTDOWN;
            return false;
        }
        else if (!switched_on)
        {
            *controlword = CW_SWITCHON;
            return false;
        }
        else if (!operation_enabled)
        {
            *controlword = CW_ENABLE;
            return false;
        }
        else
        {
            if (!init_printed)
                printf("State: Operation enabled achieved\n");
            return true;
        }
        break;

    case 0: // Disable operation (for shutdown)
        if (operation_enabled)
        {
            *controlword = CW_DISABLEOPERATION;
            return false;
        }
        else if (switched_on)
        {
            *controlword = CW_SHUTDOWN;
            return false;
        }
        else if (ready_to_switch_on)
        {
            *controlword = CW_DISABLEVOLTAGE;
            return false;
        }
        else
        {
            return true;
        }
        break;

    default:
        return false;
    }

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