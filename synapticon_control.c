#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <signal.h>
#include <termios.h>
#include <fcntl.h>

#include "ethercat.h"
#include "ethercatprint.h"

#define MAX_VELOCITY 2500 

// CiA 402 state machine commands (controlword)
#define CW_SHUTDOWN 0x0006
#define CW_SWITCHON 0x0007
#define CW_ENABLE 0x000F
#define CW_ENABLE_VOLTAGE 0x001F
#define CW_QUICKSTOP 0x0002
#define CW_DISABLEVOLTAGE 0x0000
#define CW_DISABLEOPERATION 0x0007
#define CW_FAULT_RESET 0x0080

// CiA 402 statusword bit masks
#define SW_READY_TO_SWITCH_ON_BIT (0x1 << 0)
#define SW_SWITCHED_ON_BIT (0x1 << 1)
#define SW_OPERATION_ENABLED_BIT (0x1 << 2)
#define SW_FAULT_BIT (0x1 << 3)
#define SW_VOLTAGE_ENABLED_BIT (0x1 << 4)
#define SW_QUICK_STOP_BIT (0x1 << 5)
#define SW_SWITCH_ON_DISABLED_BIT (0x1 << 6)
#define SW_TARGET_REACHED_BIT (0x1 << 10)

// Operation modes
#define OP_MODE_CSV 9 // Cyclic Synchronous Velocity mode

// RxPDO (master to slave)
#pragma pack(push, 1)          // no implicit padding
typedef struct
{
    uint16_t controlword;    // 0x6040
    int8_t   op_mode;        // 0x6060
    int16_t  target_torque;  // 0x6071
    int32_t  target_position;// 0x607A
    int32_t  target_velocity;// 0x60FF
    int16_t  torque_offset;  // 0x60B2
    int32_t  tuning_command; // 0x2701
} rxpdo_t;
#pragma pack(pop)

// TxPDO (slave to master)
#pragma pack(push, 1) 
typedef struct
{
    uint16_t statusword;     // 0x6041
    int8_t op_mode_display;  // 0x6061
    int32_t position_actual; // 0x6064
    int16_t velocity_actual; // 0x606C
    int16_t torque_actual;   // 0x6077
} txpdo_t;
#pragma pack(pop)

// Global variables
char *ifname = "eth0";
int cycletime = 4000; // Cycletime in μs (4ms = 250Hz)
volatile sig_atomic_t run = 1;
int slave_index = 1;
char IOmap[4096];
pthread_t thread1;

uint32_t si_unit_velocity = 0; // 0x60A9
uint32_t feed_numerator = 0;   // 0x6092:1
uint32_t feed_denominator = 1; // 0x6092:2

// Terminal settings for keyboard input
struct termios orig_termios;

// Function prototypes
void signal_handler(int sig);
void print_extended_error(int slave);
bool setup_ethercat();
bool state_machine_control(uint16_t *controlword, uint16_t statusword, int target_state);
void *cyclic_task(void *arg);
void read_drive_parameters();
void cleanup_and_exit();
bool read_error_details(bool print_details);
bool perform_fault_reset(rxpdo_t *rxpdo, txpdo_t *txpdo);
void enable_raw_mode();
void disable_raw_mode();
int kbhit();
char readch();

// Setup terminal for raw input mode
void enable_raw_mode()
{
    tcgetattr(STDIN_FILENO, &orig_termios);
    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON); // Disable echo and canonical mode
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);

    // Set stdin to non-blocking
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

// Restore terminal to original settings
void disable_raw_mode()
{
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);

    // Restore blocking mode
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
}

// Check if a key was pressed
int kbhit()
{
    char ch;
    int nread = read(STDIN_FILENO, &ch, 1);
    if (nread == 1)
    {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

// Read a character
char readch()
{
    char ch;
    read(STDIN_FILENO, &ch, 1);
    return ch;
}

void print_extended_error(int slave)
{
    uint8_t txt[64] = {0};
    int sz = sizeof(txt) - 1;
    if (ec_SDOread(slave, 0x203F, 1, FALSE, &sz, txt, EC_TIMEOUTRXM) > 0)
        printf("Extended error: %s\n", txt);
    else
        printf("Could not read 0x203F:1\n");
}

bool perform_fault_reset(rxpdo_t *rxpdo, txpdo_t *txpdo)
{
    static int reset_sequence_state = 0;
    static int reset_delay_counter = 0;
    bool reset_complete = false;

    // According to CiA 402 standard, fault reset is a bit transition (edge triggered)
    // We need a proper sequence: clear command → set reset bit → wait → clear reset bit
    switch (reset_sequence_state)
    {
    case 0:                          // First, send controlword with reset bit cleared
        rxpdo->controlword = 0x0000; // All bits cleared
        reset_sequence_state = 1;
        reset_delay_counter = 0;
        break;

    case 1: // Wait a bit for the first command to take effect
        reset_delay_counter++;
        if (reset_delay_counter >= 5)
        { // Wait about 20ms (5 cycles at 4ms)
            reset_sequence_state = 2;
        }
        break;

    case 2:                                  // Now send the fault reset command
        rxpdo->controlword = CW_FAULT_RESET; // Set only the fault reset bit (0x0080)
        reset_sequence_state = 3;
        reset_delay_counter = 0;
        break;

    case 3: // Hold the reset command for a while
        reset_delay_counter++;
        if (reset_delay_counter >= 10)
        { // Hold for about 40ms (10 cycles)
            reset_sequence_state = 4;
        }
        break;

    case 4:                          // Clear the reset bit
        rxpdo->controlword = 0x0000; // Clear all bits again
        reset_sequence_state = 5;
        reset_delay_counter = 0;
        break;

    case 5: // Wait and check if fault has cleared
        reset_delay_counter++;
        if (reset_delay_counter >= 5)
        { // Wait about 20ms
            // Check if fault is still present
            if (!(txpdo->statusword & SW_FAULT_BIT))
            {
                // Fault cleared successfully
                reset_complete = true;
            }

            // Reset the sequence to start again if needed
            reset_sequence_state = 0;
        }
        break;
    }

    return reset_complete;
}

void signal_handler(int sig)
{
    printf("\nSignal %d received, stopping program...\n", sig);
    run = 0;
}

void cleanup_and_exit()
{
    printf("Stopping EtherCAT\n");

    pthread_join(thread1, NULL);

    // Close EtherCAT
    ec_close();

    // Restore terminal settings
    disable_raw_mode();

    printf("End program\n");
}

// Function to reinitialize EtherCAT connection
bool reinitialize_ethercat()
{
    printf("\n===== ATTEMPTING TO RECONNECT ETHERCAT =====\n");

    // Close existing connection first
    printf("Closing existing EtherCAT connection\n");
    ec_close();

    // Longer delay to ensure proper network reset and give time for device to power up
    printf("Waiting for EtherCAT devices to come online...\n");
    usleep(2000000); // 2 seconds

    printf("Reinitializing EtherCAT on %s\n", ifname);

    // Reinitialize everything from scratch
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname);

        if (ec_config_init(FALSE) > 0)
        {
            printf("%d slaves found and reconfigured.\n", ec_slavecount);

            // Configure distributed clock
            printf("Reconfiguring DC...\n");
            ec_configdc();

            // Remap process data
            printf("Remapping process data...\n");
            ec_config_map(&IOmap);

            // Set state to SAFE_OP
            ec_slave[0].state = EC_STATE_SAFE_OP;
            ec_writestate(0);
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

            // Set state to OPERATIONAL
            printf("Setting state to OPERATIONAL...\n");
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            ec_writestate(0);

            // Wait for all slaves to reach OP state
            int chk = 40;
            do
            {
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                printf("Reconnection successful - operational state reached for all slaves.\n");
                printf("===== ETHERCAT RECONNECTION COMPLETE =====\n\n");
                return true;
            }
            else
            {
                printf("Reconnection failed - not all slaves reached operational state.\n");
            }
        }
        else
        {
            printf("Reconnection failed - no slaves found!\n");
        }
    }
    else
    {
        printf("Reconnection failed - could not initialize %s\n", ifname);
    }

    printf("===== ETHERCAT RECONNECTION FAILED =====\n\n");
    return false;
}

bool setup_ethercat()
{
    int i, chk;
    uint16_t u16val;
    uint32_t u32val;
    int size;

    // Initialize SOEM, bind socket to ifname
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname);

        // Find and auto-configure slaves
        if (ec_config_init(FALSE) > 0)
        {
            printf("%d slaves found and configured.\n", ec_slavecount);

            // Read device information
            u32val = 0;
            size = sizeof(u32val);
            int ret = ec_SDOread(slave_index, 0x1000, 0, FALSE, &size, &u32val, EC_TIMEOUTRXM);
            if (ret > 0)
            {
                printf("Device type: 0x%08X\n", u32val);
            }
            else
            {
                printf("Failed to read device type\n");
            }

            // Read software version
            char sw_version[32] = {0};
            size = sizeof(sw_version) - 1;
            ret = ec_SDOread(slave_index, 0x100A, 0, FALSE, &size, sw_version, EC_TIMEOUTRXM);
            if (ret > 0)
            {
                printf("Software version: %s\n", sw_version);
            }

            // Try to read error code if any
            u16val = 0;
            size = sizeof(u16val);
            ret = ec_SDOread(slave_index, 0x603F, 0, FALSE, &size, &u16val, EC_TIMEOUTRXM);
            if (ret > 0 && u16val != 0)
            {
                printf("Current error code: 0x%04X\n", u16val);
            }

            // Set state to PRE_OP for configuration
            ec_slave[0].state = EC_STATE_PRE_OP;
            ec_writestate(0);

            read_velocity_scaling();
            //ec_printPDO(slave_index, ec_slave[slave_index].outputs, sizeof(rxpdo_t));

            // Configure distributed clock
            printf("Configuring DC...\n");
            ec_configdc();

            // Read important Drive Parameters
            printf("Reading drive parameters...\n");
            read_drive_parameters();

            ec_config_map(&IOmap);

            // Wait for spacebar press before leaving PRE-OP
            printf("\n\n*********************************************************\n");
            printf("* System is in PRE-OP state                              *\n");
            printf("* Press SPACEBAR to continue to SAFE-OP and OPERATIONAL  *\n");
            printf("*********************************************************\n\n");

            // Enable raw mode for keyboard input
            enable_raw_mode();

            // Wait for spacebar press
            bool spacebar_pressed = false;
            while (!spacebar_pressed && run)
            {
                if (kbhit())
                {
                    char c = readch();
                    if (c == ' ')
                    {
                        spacebar_pressed = true;
                        printf("Spacebar pressed! Continuing to SAFE-OP state...\n");
                    }
                }
                usleep(50000); // 50ms sleep to avoid CPU hogging
            }

            // Exit if program was terminated while waiting
            if (!run)
            {
                return false;
            }

            // Set state to SAFE_OP
            ec_slave[0].state = EC_STATE_SAFE_OP;
            ec_writestate(0);
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

            // Set state to OPERATIONAL
            printf("Setting state to OPERATIONAL...\n");
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            ec_writestate(0);

            // Wait for all slaves to reach OP state
            chk = 40;
            do
            {
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                printf("Operational state reached for all slaves.\n");
                return true;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for (i = 1; i <= ec_slavecount; i++)
                {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode,
                               ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
        }
        else
        {
            printf("No slaves found!\n");
        }
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n", ifname);
    }

    return false;
}

void read_velocity_scaling(void)
{
    // Read SI unit velocity
    int ret, size;
    uint32_t u32;
    size = sizeof(u32);

    ret = ec_SDOread(slave_index, 0x60A9, 0, FALSE, &size, &u32, EC_TIMEOUTRXM);
    if (ret > 0)
    {
        si_unit_velocity = u32;
        printf("SI‑unit velocity (0x60A9): 0x%08X\n", u32);

        // Decode SI unit in detail
        uint8_t unit_type = (u32 >> 16) & 0xFF;
        uint8_t unit_expt = (u32 >> 24) & 0xFF;
        uint8_t prefix_code = u32 & 0xFF;
        printf("  Unit breakdown: Type=0x%02X, Exp=0x%02X, Prefix=0x%02X\n",
               unit_type, unit_expt, prefix_code);

        if (unit_type == 0xB4)
        {
            printf("  Unit type is Angular Velocity\n");
        }
    }

    // Try to read feed constant for additional insights
    size = sizeof(feed_numerator); // Use the global variable directly
    ret = ec_SDOread(slave_index, 0x6092, 1, FALSE, &size, &feed_numerator, EC_TIMEOUTRXM);
    if (ret > 0)
    {
        printf("Feed constant numerator (0x6092:1): %d\n", feed_numerator);
    }

    size = sizeof(feed_denominator); // Use the global variable directly
    ret = ec_SDOread(slave_index, 0x6092, 2, FALSE, &size, &feed_denominator, EC_TIMEOUTRXM);
    if (ret > 0)
    {
        printf("Feed constant denominator (0x6092:2): %d\n", feed_denominator);
    }

    // Read other relevant parameters
    int32_t vel_offset = 0;
    size = sizeof(vel_offset);
    ret = ec_SDOread(slave_index, 0x60B1, 0, FALSE, &size, &vel_offset, EC_TIMEOUTRXM);
    if (ret > 0)
    {
        printf("Velocity offset (0x60B1): %d\n", vel_offset);
    }
}

void read_drive_parameters()
{
    int ret;
    int size;
    uint32_t u32val;
    uint16_t u16val;
    int8_t i8val;
    int32_t i32val;

    // Read maximum motor speed
    u32val = 0;
    size = sizeof(u32val);
    ret = ec_SDOread(slave_index, 0x6080, 0, FALSE, &size, &u32val, EC_TIMEOUTRXM);
    if (ret > 0)
    {
        printf("Max motor speed (0x6080): %d RPM\n", u32val);
    }
    else
    {
        printf("Failed to read max motor speed\n");
    }

    ret = ec_SDOread(slave_index, 0x60A9, 0, FALSE, &size, &u32val, EC_TIMEOUTRXM);
    if (ret > 0)
    {
        printf("Power of Ten is (0x60A9): %d\n", u32val);
    }
    else
    {
        printf("Failed to read\n");
    }

    // Try to read velocity limit
    i32val = 0;
    size = sizeof(i32val);
    ret = ec_SDOread(slave_index, 0x607F, 0, FALSE, &size, &i32val, EC_TIMEOUTRXM);
    if (ret > 0)
    {
        printf("Max profile velocity (0x607F): %d\n", i32val);
        printf("  This is approximately %.2f RPM\n", (float)i32val / 262144.0f);
    }

    // Try to read min/max velocity limit
    i32val = 0;
    size = sizeof(i32val);
    ret = ec_SDOread(slave_index, 0x607E, 1, FALSE, &size, &i32val, EC_TIMEOUTRXM);
    if (ret > 0)
    {
        printf("Min velocity limit (0x607E:1): %d\n", i32val);
    }

    i32val = 0;
    size = sizeof(i32val);
    ret = ec_SDOread(slave_index, 0x607E, 2, FALSE, &size, &i32val, EC_TIMEOUTRXM);
    if (ret > 0)
    {
        printf("Max velocity limit (0x607E:2): %d\n", i32val);
    }

    // Read current limit
    u32val = 0;
    size = sizeof(u32val);
    ret = ec_SDOread(slave_index, 0x6073, 0, FALSE, &size, &u32val, EC_TIMEOUTRXM);
    if (ret > 0)
    {
        printf("Max current (0x6073): %d mA\n", u32val);
    }


    // Read current operation mode
    i8val = 0;
    size = sizeof(i8val);
    ret = ec_SDOread(slave_index, 0x6060, 0, FALSE, &size, &i8val, EC_TIMEOUTRXM);
    if (ret > 0)
    {
        printf("Current mode (0x6060): %d ", i8val);
        switch (i8val)
        {
        case 9:
            printf("(CSV)\n");
            break;
        case 8:
            printf("(CSP)\n");
            break;
        case 10:
            printf("(CST)\n");
            break;
        default:
            printf("(Other)\n");
            break;
        }
    }
}

bool read_error_details(bool print_details)
{
    uint16_t error_code = 0;
    int size = sizeof(error_code);
    int ret = ec_SDOread(slave_index, 0x603F, 0, FALSE, &size, &error_code, EC_TIMEOUTRXM);

    if (ret > 0 && error_code != 0)
    {
        if (print_details)
        {
            printf("Error code: 0x%04X\n", error_code);

            // Specific handling for common error codes
            if (error_code == 0x6320)
            { // Parameter Error
                print_extended_error(slave_index);
                printf("Detected parameter error (0x6320). The motor configuration may be incomplete.\n");
                printf("Consider using OBLAC Drives software to properly configure the motor parameters.\n");

                // Try to read specific missing parameters
                uint16_t specific_error = 0;
                size = sizeof(specific_error);
                ret = ec_SDOread(slave_index, 0x603F, 1, FALSE, &size, &specific_error, EC_TIMEOUTRXM);
                if (ret > 0 && specific_error != 0)
                {
                    printf("Specific parameter error: 0x%04X\n", specific_error);
                }

                // Check for diagnostic registers that might give more details
                uint32_t diagnostic_register = 0;
                size = sizeof(diagnostic_register);

                // Try manufacturer-specific diagnostic registers
                ret = ec_SDOread(slave_index, 0x2000, 0, FALSE, &size, &diagnostic_register, EC_TIMEOUTRXM);
                if (ret > 0)
                {
                    printf("Diagnostic register 0x2000: 0x%08X\n", diagnostic_register);
                }

                ret = ec_SDOread(slave_index, 0x2701, 0, FALSE, &size, &diagnostic_register, EC_TIMEOUTRXM);
                if (ret > 0)
                {
                    printf("Diagnostic register 0x2701: 0x%08X\n", diagnostic_register);
                }
            }
        }
        return true; // Error exists
    }

    if (print_details && ret <= 0)
    {
        printf("Unable to read error code\n");
    }

    return false; // No error
}

/**
 * CiA 402 state machine control
 */
bool state_machine_control(uint16_t *controlword, uint16_t statusword, int target_state)
{
    // Current state extraction from statusword
    bool ready_to_switch_on = statusword & SW_READY_TO_SWITCH_ON_BIT;
    bool switched_on = statusword & SW_SWITCHED_ON_BIT;
    bool operation_enabled = statusword & SW_OPERATION_ENABLED_BIT;
    bool fault = statusword & SW_FAULT_BIT;
    bool switch_on_disabled = statusword & SW_SWITCH_ON_DISABLED_BIT;

    // Debug output - only during normal operation, not during shutdown
    static int last_printed_status = 0;
    if (target_state != 0 || !last_printed_status)
    { // Don't print repetitive status during shutdown
        printf("Statusword: 0x%04X [RDY:%d, ON:%d, ENABLED:%d, FAULT:%d, DISABLED:%d]\n",
               statusword, ready_to_switch_on, switched_on, operation_enabled,
               fault, switch_on_disabled);

        // Track that we've printed status during shutdown
        if (target_state == 0)
        {
            last_printed_status = 1;
        }
    }

    // Special case - if statusword is 0, it's likely we have a communication issue
    if (statusword == 0)
    {
        printf("Initial statusword is zero - normal during startup. Sending reset...\n");
        *controlword = CW_FAULT_RESET;
        return false;
    }

    // Check for specific error code
    static bool error_checked = false;
    if (!error_checked && (statusword & SW_FAULT_BIT))
    {                             // If fault bit is set
        read_error_details(true); // Print error details
        error_checked = true;
    }

    // State machine logic based on CiA 402 state machine
    if (fault)
    {
        // Only print fault message if not in shutdown mode (target_state != 0)
        if (target_state != 0)
        {
            printf("Drive is in FAULT state\n");
        }

        *controlword = CW_FAULT_RESET;

        // Wait a bit after sending fault reset - shorter wait during shutdown
        ec_send_processdata();
        if (target_state != 0)
        {
            usleep(50000); // 50ms delay normally
        }
        else
        {
            usleep(10000); // 10ms delay during shutdown - faster cleanup
        }

        return false;
    }

    switch (target_state)
    {
    case 1: // Switch on
        if (switch_on_disabled)
        {
            printf("State: Switch on disabled -> Shutdown\n");
            *controlword = CW_SHUTDOWN;
            return false;
        }
        else if (!ready_to_switch_on)
        {
            printf("State: Not ready -> Shutdown\n");
            *controlword = CW_SHUTDOWN;
            return false;
        }
        else if (!switched_on)
        {
            printf("State: Ready -> Switch on\n");
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
            printf("State: Switch on disabled -> Shutdown\n");
            *controlword = CW_SHUTDOWN;
            return false;
        }
        else if (!ready_to_switch_on)
        {
            printf("State: Not ready -> Shutdown\n");
            *controlword = CW_SHUTDOWN;
            return false;
        }
        else if (!switched_on)
        {
            printf("State: Ready -> Switch on\n");
            *controlword = CW_SWITCHON;
            return false;
        }
        else if (!operation_enabled)
        {
            printf("State: Switched on -> Enable operation\n");
            *controlword = CW_ENABLE;
            return false;
        }
        else
        {
            printf("State: Operation enabled achieved\n");
            return true;
        }
        break;

    case 0: // Disable operation - for shutdown, minimize output
        // If in fault state during shutdown, we've already set CW_FAULT_RESET above
        if (!fault)
        {
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
                // Only print success message once
                if (!last_printed_status)
                {
                    printf("State: Disabled\n");
                    last_printed_status = 2; // Mark as fully disabled
                }
                return true;
            }
        }
        return false; // Fixed: Added missing return for fault case during shutdown
        break;

    default:
        printf("Invalid target state: %d\n", target_state);
        return false;
    }

    // Fixed: Added default return to fix warning
    return false; // Should never reach here but added for safety
}

/**
 * Cyclic task to process PDOs
 */
void *cyclic_task(void *arg)
{
    (void)arg; // Suppress unused parameter warning

    rxpdo_t *rxpdo;
    txpdo_t *txpdo;
    int joystick_value = 0;
    int wkc;
    int timeout_counter = 0;

    bool continuous_scanning = false;
    int scan_interval_counter = 0;
    const int SCAN_INTERVAL = 500;

    // Connection monitoring
    int comm_errors = 0;
    int reconnection_attempts = 0;
    bool connection_lost = false;
    int consecutive_good_packets = 0;
    const int COMM_ERROR_THRESHOLD = 200;       // If we miss 200 packets, consider connection lost
    const int CONSECUTIVE_GOOD_THRESHOLD = 250; // Need 250 good packets to consider reconnection successful

    // Phases of state machine initialization
    enum
    {
        PHASE_INIT_RESET = 0,       // Initial reset state
        PHASE_FAULT_RECOVERY = 1,   // Special phase for fault recovery
        PHASE_SHUTDOWN = 2,         // Reached shutdown state
        PHASE_SWITCH_ON = 3,        // Reached switch on state
        PHASE_SET_OPMODE = 4,       // Setting operation mode
        PHASE_WAIT_OPMODE_ACK = 5,  // Waiting for operation mode to be acknowledged
        PHASE_ENABLE_OPERATION = 6, // Enabling operation
        PHASE_STABILIZATION = 7,    // Wait for drive to stabilize
        PHASE_OPERATIONAL = 8       // Fully operational
    } init_phase = PHASE_INIT_RESET;

    // Timeout counter for waiting for operation mode acknowledgment
    int opmode_wait_counter = 0;

    // Process data pointers
    rxpdo = (rxpdo_t *)(ec_slave[slave_index].outputs);
    txpdo = (txpdo_t *)(ec_slave[slave_index].inputs);

    // Initialize controlword, opmode, etc.
    rxpdo->controlword = 0;
    rxpdo->op_mode = 0; 
    //rxpdo->target_velocity = 0;
    rxpdo->target_torque = 0;
    rxpdo->torque_offset = 0;

    int drive_initialized = 0;
    int64 cycletime_ns = cycletime * 1000;

    struct timespec ts;
    struct timespec tleft;

    // Stabilization counter
    int stabilization_counter = 0;

    // Counter for fault messages - to reduce output clutter
    int fault_counter = 0;
    int fault_message_modulo = 100; // Only print message every 100 faults
    bool in_fault_state = false;

    // Fault recovery counter
    int fault_recovery_attempts = 0;

    // Shutdown flag
    bool is_shutting_down = false;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    ts.tv_nsec += cycletime_ns;
    if (ts.tv_nsec >= 1000000000)
    {
        ts.tv_sec++;
        ts.tv_nsec -= 1000000000;
    }

    printf("Starting drive initialization with correct sequence...\n");

    // Initial check for fault state
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    if (txpdo->statusword & SW_FAULT_BIT)
    {
        printf("Drive is starting in FAULT state, entering fault recovery sequence\n");
        init_phase = PHASE_FAULT_RECOVERY;
        read_error_details(true);
        in_fault_state = true;
    }

    while (run || is_shutting_down)
    {
        // Check if main program has requested shutdown
        if (!run && !is_shutting_down)
        {
            printf("Cyclic task shutdown initiated...\n");

            // Disable operation before exiting
            rxpdo->target_velocity = 0;
            is_shutting_down = true;
        }

        // Synchronize with DC
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

        // Verify wkc - check if we're getting valid data
        if (wkc <= 0)
        {
            // Communication error - potential disconnection
            comm_errors++;
            consecutive_good_packets = 0;

            // With this (only print every 1000 cycles instead of 100):
            if (comm_errors % 1000 == 0 && comm_errors > 0)
            {
                printf("WARNING: No process data received for %d cycles (~%.1f seconds)\n",
                       comm_errors, comm_errors * cycletime / 1000000.0);
            }

            // If we've accumulated enough errors, consider connection lost
            if (comm_errors >= COMM_ERROR_THRESHOLD)
            {
                // Only print the message once when we first detect the loss
                if (!connection_lost)
                {
                    printf("CRITICAL: Communication with EtherCAT slave lost after %d errors\n", comm_errors);
                    connection_lost = true;
                    reconnection_attempts = 0;
                }

                // Periodic reconnection attempt - try every 1000 cycles (about 4 seconds)
                if (comm_errors % 1000 == 0)
                {
                    printf("Attempting to reestablish connection (attempt %d)...\n", reconnection_attempts + 1);
                    reconnection_attempts++;

                    if (reinitialize_ethercat())
                    {
                        // Reconnection successful
                        printf("EtherCAT connection reestablished!\n");
                        comm_errors = 0;
                        connection_lost = false;

                        // Get fresh pointers after reconnection
                        rxpdo = (rxpdo_t *)(ec_slave[slave_index].outputs);
                        txpdo = (txpdo_t *)(ec_slave[slave_index].inputs);

                        // Reset initialization phase
                        drive_initialized = 0;
                        init_phase = PHASE_INIT_RESET;
                        in_fault_state = false;
                        fault_counter = 0;
                    }
                    else
                    {
                        printf("Still waiting for EtherCAT device to come back online...\n");
                    }
                }
            }

            // If connection is lost, skip the rest of the loop and try again next cycle
            if (connection_lost)
            {

                scan_interval_counter++;
                if (scan_interval_counter >= SCAN_INTERVAL)
                {
                    printf("Scanning for EtherCAT devices...\n");

                    if (reinitialize_ethercat())
                    {
                        // Reconnection successful - reset everything
                        printf("Device found and reconnected successfully!\n");
                        connection_lost = false;
                        continuous_scanning = false;
                        comm_errors = 0;
                    }

                    scan_interval_counter = 0;
                }

                continue;
            }
        }
        else
        {
            // Valid data received
            if (connection_lost)
            {
                // We got a good packet while in lost connection state
                consecutive_good_packets++;

                if (consecutive_good_packets >= CONSECUTIVE_GOOD_THRESHOLD)
                {
                    printf("Connection appears to be stable again after %d good packets\n",
                           consecutive_good_packets);
                    connection_lost = false;
                    comm_errors = 0;

                    // Reset initialization state and start over
                    drive_initialized = 0;
                    init_phase = PHASE_INIT_RESET;
                }
            }
            else
            {
                if (comm_errors > 0)
                {
                    comm_errors = 0;
                    consecutive_good_packets = 0;
                }
            }
        }

        // Special case for shutdown sequence
        if (is_shutting_down)
        {
            // Just execute the state machine to disable operation
            static int shutdown_timeout = 50; // Maximum number of cycles for shutdown

            if (state_machine_control(&rxpdo->controlword, txpdo->statusword, 0))
            {
                // Successfully disabled
                printf("Cyclic task completed\n");
                break; // Exit the main loop
            }

            // Check for shutdown timeout
            shutdown_timeout--;
            if (shutdown_timeout <= 0)
            {
                printf("Shutdown timeout, forcing exit\n");
                break; // Force exit if shutdown takes too long
            }

            ec_send_processdata();
            continue; // Skip to next cycle
        }

        // Handle drive initialization according to the proper sequence
        if (!drive_initialized)
        {
            // Check for fault status
            bool is_fault = (txpdo->statusword & SW_FAULT_BIT);

            // Special case for dedicated fault recovery phase
            if (init_phase == PHASE_FAULT_RECOVERY)
            {
                if (!in_fault_state)
                {
                    // First time entering fault recovery
                    printf("Entering fault recovery sequence\n");
                    in_fault_state = true;
                    fault_counter = 1;
                    fault_recovery_attempts = 0;
                }

                // Try the enhanced fault reset sequence
                bool reset_success = perform_fault_reset(rxpdo, txpdo);

                if (reset_success)
                {
                    printf("Fault successfully cleared! Continuing with initialization\n");
                    in_fault_state = false;
                    init_phase = PHASE_SHUTDOWN; // Move to normal initialization

                    // Small delay after successful reset
                    for (int i = 0; i < 25; i++)
                    { // Wait about 100ms
                        ec_send_processdata();
                        usleep(cycletime);
                        ec_receive_processdata(EC_TIMEOUTRET);
                    }
                }
                else
                {
                    // Reset wasn't successful yet
                    fault_recovery_attempts++;

                    if (fault_recovery_attempts % 100 == 0)
                    {
                        printf("Fault recovery attempt %d - still trying...\n", fault_recovery_attempts);

                        // Try some alternative approaches after enough attempts
                        if (fault_recovery_attempts >= 300)
                        {
                            // Try power cycling the state machine
                            printf("Trying alternative recovery - power cycling control words\n");

                            // Cycle through different control words
                            if ((fault_recovery_attempts / 100) % 3 == 0)
                            {
                                rxpdo->controlword = CW_DISABLEVOLTAGE;
                            }
                            else if ((fault_recovery_attempts / 100) % 3 == 1)
                            {
                                rxpdo->controlword = CW_QUICKSTOP;
                            }
                            else
                            {
                                rxpdo->controlword = CW_FAULT_RESET;
                            }

                            // After many attempts, try switching operation modes
                            if (fault_recovery_attempts >= 500 && fault_recovery_attempts % 200 == 0)
                            {
                                int8_t alt_mode = (fault_recovery_attempts / 200) % 2 == 0 ? 0 : 3; // Try no mode or profile velocity
                                printf("Trying alternative operation mode: %d\n", alt_mode);
                                rxpdo->op_mode = alt_mode;
                            }
                        }
                    }

                    // Timeout after extreme number of attempts
                    if (fault_recovery_attempts > 2000)
                    {
                        printf("CRITICAL: Unable to clear fault after %d attempts.\n",
                               fault_recovery_attempts);
                        printf("You may need to reset the drive using OBLAC software or power cycle it.\n");

                        // Try to continue with regular initialization as a last resort
                        init_phase = PHASE_SHUTDOWN;
                    }
                }
            }
            // Normal initialization phases
            else if (is_fault)
            {
                if (!in_fault_state)
                {
                    // First time entering fault state - print detailed info
                    printf("Drive in FAULT state, entering fault recovery sequence\n");
                    read_error_details(true);
                    in_fault_state = true;
                    init_phase = PHASE_FAULT_RECOVERY; // Switch to dedicated fault handling
                }
            }
            else
            {
                // No fault - proceed with normal initialization
                in_fault_state = false;

                switch (init_phase)
                {
                case PHASE_INIT_RESET:
                    printf("Starting initialization sequence\n");
                    init_phase = PHASE_SHUTDOWN;
                    break;

                case PHASE_SHUTDOWN:
                    // Step 1: Send shutdown command
                    rxpdo->controlword = CW_SHUTDOWN;
                    if ((txpdo->statusword & 0x006F) == 0x0021)
                    { // Ready to switch on
                        printf("Shutdown complete, drive is ready to switch on\n");
                        init_phase = PHASE_SWITCH_ON;
                    }
                    break;

                case PHASE_SWITCH_ON:
                    // Step 2: Send switch on command
                    rxpdo->controlword = CW_SWITCHON;
                    if ((txpdo->statusword & 0x006F) == 0x0023)
                    { // Switched on
                        printf("Switch on complete, drive is switched on\n");
                        init_phase = PHASE_SET_OPMODE;
                    }
                    break;

                case PHASE_SET_OPMODE:

                    // Step 3: Set operation mode BEFORE enabling operation
                    printf("Setting operation mode to CSV (Mode 9)\n");
                    rxpdo->op_mode = OP_MODE_CSV;
                    init_phase = PHASE_WAIT_OPMODE_ACK;
                    opmode_wait_counter = 0;
                    break;

                case PHASE_WAIT_OPMODE_ACK:
                    // Maintain control word while waiting for mode acknowledgment
                    rxpdo->controlword = CW_SWITCHON;

                    // Wait for operation mode to be acknowledged
                    if (txpdo->op_mode_display == OP_MODE_CSV)
                    {
                        printf("Operation mode CSV acknowledged by drive\n");
                        init_phase = PHASE_ENABLE_OPERATION;
                    }
                    else
                    {
                        opmode_wait_counter++;
                        if (opmode_wait_counter >= 100)
                        { // About 400ms timeout
                            printf("WARNING: Operation mode not acknowledged after %d cycles\n",
                                   opmode_wait_counter);
                            printf("Current displayed mode: %d\n", txpdo->op_mode_display);

                            // Retry setting the mode
                            printf("Retrying operation mode setting...\n");
                            init_phase = PHASE_SET_OPMODE;
                        }
                    }
                    break;

                case PHASE_ENABLE_OPERATION:
                    // Step 4: Enable operation AFTER mode is set and acknowledged
                    printf("Enabling operation with op mode %d active\n", txpdo->op_mode_display);
                    rxpdo->controlword = CW_ENABLE;

                    // Check if operation is enabled
                    if ((txpdo->statusword & 0x006F) == 0x0027)
                    { // Operation enabled
                        printf("Operation enabled successfully! Drive is now fully operational\n");
                        init_phase = PHASE_STABILIZATION;
                        stabilization_counter = 0;
                    }
                    break;

                case PHASE_STABILIZATION:

                    //rxpdo->target_velocity = 0; // Ensure zero velocity during stabilization

                    stabilization_counter++;
                    if (stabilization_counter % 50 == 0)
                    {
                        printf("Stabilizing... %d of 500 cycles\n", stabilization_counter);
                    }

                    if (stabilization_counter >= 500)
                    { // About 2 seconds at 250Hz
                        printf("Stabilization complete, transitioning to operational state\n");
                        init_phase = PHASE_OPERATIONAL;
                        drive_initialized = 1;
                    }
                    break;

                case PHASE_OPERATIONAL:
                    // This state shouldn't be reached here, but just in case
                    drive_initialized = 1;
                    break;

                case PHASE_FAULT_RECOVERY:
                    // Handled separately above - this is just to avoid the warning
                    break;
                }
            }

            // Check for timeout during initialization (except during fault recovery)
            if (init_phase != PHASE_FAULT_RECOVERY)
            {
                timeout_counter++;
                if (timeout_counter > 1000)
                { // About 4 seconds at 250Hz
                    printf("TIMEOUT: Drive initialization sequence stalled. Phase: %d\n", init_phase);
                    printf("Statusword: 0x%04X, Op mode display: %d\n",
                           txpdo->statusword, txpdo->op_mode_display);

                    // Check if we're in fault state and need special handling
                    if (txpdo->statusword & SW_FAULT_BIT)
                    {
                        printf("Drive is in fault state, switching to fault recovery\n");
                        init_phase = PHASE_FAULT_RECOVERY;
                    }
                    else
                    {
                        // Try resetting the sequence
                        rxpdo->controlword = 0; // Clear control word first
                        ec_send_processdata();
                        usleep(cycletime * 5);

                        init_phase = PHASE_INIT_RESET;
                    }

                    timeout_counter = 0;
                }
            }
        }
        else
        {
            // Drive is initialized and operational

            // Check for fault status
            bool is_fault = (txpdo->statusword & SW_FAULT_BIT);

            if (is_fault)
            {
                if (!in_fault_state)
                {
                    // First time entering fault state during operation - print detailed info
                    printf("FAULT detected during operation. Attempting reset...\n");
                    read_error_details(true);
                    in_fault_state = true;
                    fault_counter = 1;

                    // Reset target velocity and prepare for reset
                    //rxpdo->target_velocity = 0;
                }
                else
                {
                    // Already in fault state - only print occasionally
                    fault_counter++;
                    if (fault_counter % fault_message_modulo == 0)
                    {
                        printf("Still in FAULT state after %d reset attempts\n", fault_counter);
                    }
                }

                bool reset_success = perform_fault_reset(rxpdo, txpdo);

                if (reset_success)
                {
                    printf("Fault cleared during operation!\n");
                    in_fault_state = false;
                    fault_counter = 0;
                }

                // If fault persists for too long, restart initialization
                if (fault_counter > 500)
                {
                    drive_initialized = 0;
                    init_phase = PHASE_FAULT_RECOVERY;
                    printf("Restarting initialization sequence after persistent faults\n");
                }
            }
            else
            {
               
                in_fault_state = false;
                rxpdo->controlword = CW_ENABLE;

                if (txpdo->statusword & SW_OPERATION_ENABLED_BIT)
                {
                    
                    static int32_t current_velocity = 0;
                    static int log_interval = 0;
                    //rxpdo->target_velocity = (int32_t)(50 * (262144.0));
                    rxpdo->target_velocity = (int32_t)(50);

                    log_interval++;
                    if (log_interval >= 250) { // Every second (at 250Hz)
                        // Print raw status values without averaging
                        printf("CSV: Target=%d | Actual=%d\n",
                            rxpdo->target_velocity,
                               txpdo->velocity_actual);
                        
                        decode_statusword(txpdo->statusword);
                        // Reset log interval
                        log_interval = 0;
                    }
                }

                // Print status information every 250 cycles (about 1 second)
                static int status_counter = 0;
                status_counter++;
                if (status_counter % 250 == 0)
                {
           
                    if (abs(rxpdo->target_velocity) > 2000 && abs(txpdo->velocity_actual) < 100)
                    {
                        // Only warn if we're sending a significant command but seeing very little response
                        printf("WARNING: Motor not responding to velocity commands\n");
                        printf("  Target: %d (≈%.4f RPM), Actual: %d\n",
                               rxpdo->target_velocity,
                               (float)rxpdo->target_velocity / 262144.0f,
                               txpdo->velocity_actual);
                    }
                }
            }
        }

        // Write process data
        ec_send_processdata();
    }

    return NULL;
}

void decode_statusword(uint16_t statusword)
{
    // Basic State Machine bits
    printf("Status 0x%04X - State Machine: ", statusword);
    
    // Decode the state machine bits (bits 0, 1, 2, 3, 5, 6)
    uint8_t state_bits = statusword & 0x6F; // Mask for bits 0,1,2,3,5,6
    
    // State machine according to CiA 402
    if (state_bits == 0x00) printf("Not ready to switch on\n");
    else if (state_bits == 0x40) printf("Switch on disabled\n");
    else if (state_bits == 0x21) printf("Ready to switch on\n");
    else if (state_bits == 0x23) printf("Switched on\n");
    else if (state_bits == 0x27) printf("Operation enabled\n");
    else if (state_bits == 0x07) printf("Quick stop active\n");
    else if (state_bits == 0x0F) printf("Fault reaction active\n");
    else if (state_bits == 0x08) printf("Fault\n");
    else printf("Unknown state (0x%02X)\n", state_bits);
 
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
    
    // Row 3: Bits 8-11
    printf("  8:%-20s  9:%-20s 10:%-20s 11:%-20s\n",
           (statusword & (1 << 8)) ? "Manuf bit set" : "Manuf bit clear",
           (statusword & (1 << 9)) ? "Remote operation" : "No remote",
           (statusword & (1 << 10)) ? "Target reached" : "Target not reached",
           (statusword & (1 << 11)) ? "Internal limit" : "No limit");
    
    // Row 4: Bits 12-15
    printf(" 12:%-20s 13:%-20s 14:%-20s 15:%-20s\n",
           (statusword & (1 << 12)) ? "Following command" : "Vel differs",
           (statusword & (1 << 13)) ? "Following ramp" : "Not following ramp",
           (statusword & (1 << 14)) ? "Manuf bit 14 set" : "Manuf bit 14 clear",
           (statusword & (1 << 15)) ? "Manuf bit 15 set" : "Manuf bit 15 clear");
}

/**
 * Main function
 */
int main(int argc, char *argv[])
{
    printf("SOEM (Simple Open EtherCAT Master)\n");
    printf("Synapticon Actilink-S Motor Control\n");

    // Handle command line parameters
    if (argc > 1)
    {
        ifname = argv[1];
    }

    // Register signal handler with improved handling
    struct sigaction sa;
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;

    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    // Enable terminal raw mode for keyboard input
    enable_raw_mode();

    // Setup EtherCAT
    if (!setup_ethercat())
    {
        printf("Failed to setup EtherCAT. Exiting.\n");
        disable_raw_mode(); // Restore terminal settings
        return 1;
    }

    // Create cyclic task thread
    if (pthread_create(&thread1, NULL, &cyclic_task, NULL) != 0)
    {
        printf("Failed to create cyclic task thread\n");
        ec_close();
        disable_raw_mode(); // Restore terminal settings
        return 1;
    }

    // Wait for task to complete - with periodic checks
    while (run)
    {
        sleep(1); // Check every second instead of tight loop
    }

    // Proper cleanup (will also restore terminal settings)
    cleanup_and_exit();

    return 0;
}