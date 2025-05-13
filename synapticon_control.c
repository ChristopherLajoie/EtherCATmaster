/**
 * EtherCAT Master for Synapticon Actilink-S Motor Control
 * 
 * This program implements an EtherCAT master using the SOEM library
 * to control a Synapticon Actilink-S Integro motor in Cyclic Synchronous
 * Velocity (CSV) mode for joystick velocity control.
 * 
 * Filename: synapticon_control.c
 */

 #include <stdio.h>
 #include <string.h>
 #include <stdlib.h>
 #include <stdbool.h>
 #include <sys/time.h>
 #include <unistd.h>
 #include <pthread.h>
 #include <math.h>
 #include <signal.h>
 
 #include "ethercat.h"
 
 // Joystick simulation values (replace with actual joystick input code)
 #define JOY_MIN_VAL -32768
 #define JOY_MAX_VAL 32767
 #define MAX_VELOCITY 2500  // Maximum velocity in RPM - must be <= max_speed set in 0x6080
 
 // Command rate limiting - only update commands every N cycles
 #define COMMAND_UPDATE_RATE 25   // Only update velocity command every N cycles (25 = 100ms at 4ms cycle)
 
 // CiA 402 state machine commands (controlword)
 #define CW_SHUTDOWN        0x0006
 #define CW_SWITCHON        0x0007
 #define CW_ENABLE          0x001F //used to be 0x000F
 #define CW_QUICKSTOP       0x0002
 #define CW_DISABLEVOLTAGE  0x0000
 #define CW_DISABLEOPERATION 0x0007
 #define CW_FAULT_RESET     0x0080
 
 // CiA 402 statusword bit masks
 #define SW_READY_TO_SWITCH_ON_BIT   (0x1 << 0)
 #define SW_SWITCHED_ON_BIT          (0x1 << 1)
 #define SW_OPERATION_ENABLED_BIT    (0x1 << 2)
 #define SW_FAULT_BIT                (0x1 << 3)
 #define SW_VOLTAGE_ENABLED_BIT      (0x1 << 4)
 #define SW_QUICK_STOP_BIT           (0x1 << 5)
 #define SW_SWITCH_ON_DISABLED_BIT   (0x1 << 6)
 #define SW_TARGET_REACHED_BIT       (0x1 << 10)
 
 // Operation modes
 #define OP_MODE_CSV  9  // Cyclic Synchronous Velocity mode
 
 // PDO mapping offsets based on the provided documentation
 // RxPDO (master to slave)
 typedef struct {
     uint16_t controlword;       // 0x6040
     int8_t op_mode;             // 0x6060
     int16_t target_torque;      // 0x6071
     int32_t target_position;    // 0x607A
     int32_t target_velocity;    // 0x60FF
     int16_t torque_offset;      // 0x60B2
     int32_t tuning_command;     // 0x2701
 } rxpdo_t;
 
 // TxPDO (slave to master)
 typedef struct {
     uint16_t statusword;         // 0x6041
     int8_t op_mode_display;      // 0x6061
     int32_t position_actual;     // 0x6064
     int16_t velocity_actual;     // 0x606C
     int16_t torque_actual;       // 0x6077
 } txpdo_t;
 
 // Global variables
 char *ifname = "eth0";           // Interface name (replace with your actual interface)
 int cycletime = 4000;            // Cycletime in μs (4ms = 250Hz)
 volatile sig_atomic_t run = 1;   // Program run flag - made volatile for proper signal handling
 int slave_index = 1;             // Slave index (assuming first slave is the motor drive)
 char IOmap[4096];                // EtherCAT I/O map
 pthread_t thread1;               // Thread handle for proper cleanup
 
 // Function prototypes
 void signal_handler(int sig);
 bool setup_ethercat();
 int map_joystick_to_velocity(int joy_value);
 bool state_machine_control(uint16_t *controlword, uint16_t statusword, int target_state);
 void* cyclic_task(void *arg);
 void read_drive_parameters();
 void cleanup_and_exit();
 
 /**
  * Signal handler to handle Ctrl+C and other signals
  */
 void signal_handler(int sig) {
     printf("\nSignal %d received, stopping program...\n", sig);
     run = 0;  // Set the flag to exit the main loop
 }

 /**
  * Cleanup resources and exit
  */
 void cleanup_and_exit() {
     printf("Stopping EtherCAT\n");
     
     // Wait for cyclic task to complete gracefully
     pthread_join(thread1, NULL);
     
     // Close EtherCAT
     ec_close();
     
     printf("End program\n");
 }
 
 /**
  * Setup EtherCAT network
  */
 bool setup_ethercat() {
     int i, chk;
     uint16_t u16val;
     uint32_t u32val;
     int size;
     
     // Initialize SOEM, bind socket to ifname
     if (ec_init(ifname)) {
         printf("ec_init on %s succeeded.\n", ifname);
         
         // Find and auto-configure slaves
         if (ec_config_init(FALSE) > 0) {
             printf("%d slaves found and configured.\n", ec_slavecount);
             
             // Read device information
             u32val = 0;
             size = sizeof(u32val);
             int ret = ec_SDOread(slave_index, 0x1000, 0, FALSE, &size, &u32val, EC_TIMEOUTRXM);
             if (ret > 0) {
                 printf("Device type: 0x%08X\n", u32val);
             } else {
                 printf("Failed to read device type\n");
             }
             
             // Read software version
             char sw_version[32] = {0};
             size = sizeof(sw_version) - 1;
             ret = ec_SDOread(slave_index, 0x100A, 0, FALSE, &size, sw_version, EC_TIMEOUTRXM);
             if (ret > 0) {
                 printf("Software version: %s\n", sw_version);
             }
             
             // Try to read error code if any
             u16val = 0;
             size = sizeof(u16val);
             ret = ec_SDOread(slave_index, 0x603F, 0, FALSE, &size, &u16val, EC_TIMEOUTRXM);
             if (ret > 0 && u16val != 0) {
                 printf("Current error code: 0x%04X\n", u16val);
             }
             
             // Set state to PRE_OP for configuration
             ec_slave[0].state = EC_STATE_PRE_OP;
             ec_writestate(0);
             ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
             
             // Configure distributed clock
             printf("Configuring DC...\n");
             ec_configdc();
             
             // Read important Drive Parameters
             printf("Reading drive parameters...\n");
             read_drive_parameters();
             
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
             chk = 40;
             do {
                 ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
             } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
             
             if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
                 printf("Operational state reached for all slaves.\n");
                 return true;
             } else {
                 printf("Not all slaves reached operational state.\n");
                 ec_readstate();
                 for (i = 1; i <= ec_slavecount; i++) {
                     if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
                         printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                                i, ec_slave[i].state, ec_slave[i].ALstatuscode,
                                ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                     }
                 }
             }
         } else {
             printf("No slaves found!\n");
         }
     } else {
         printf("No socket connection on %s\nExcecute as root\n", ifname);
     }
     
     return false;
 }

/**
 * Read important drive parameters for diagnostics
 */
void read_drive_parameters() {
    int ret;
    int size;
    uint32_t u32val;
    uint16_t u16val;
    int8_t i8val;
    
    // Read maximum motor speed
    u32val = 0;
    size = sizeof(u32val);
    ret = ec_SDOread(slave_index, 0x6080, 0, FALSE, &size, &u32val, EC_TIMEOUTRXM);
    if (ret > 0) {
        printf("Max motor speed (0x6080): %d RPM\n", u32val);
    } else {
        printf("Failed to read max motor speed\n");
    }
    
    // Read supported operation modes
    i8val = 0;
    size = sizeof(i8val);
    ret = ec_SDOread(slave_index, 0x6502, 0, FALSE, &size, &i8val, EC_TIMEOUTRXM);
    if (ret > 0) {
        printf("Supported modes (0x6502): 0x%02X\n", i8val);
        printf("  CSV mode supported: %s\n", (i8val & (1 << (OP_MODE_CSV - 1))) ? "Yes" : "No");
    }
    
    // Read current limit 
    u32val = 0;
    size = sizeof(u32val);
    ret = ec_SDOread(slave_index, 0x6073, 0, FALSE, &size, &u32val, EC_TIMEOUTRXM);
    if (ret > 0) {
        printf("Max current (0x6073): %d mA\n", u32val);
    }
    
    // Read motor type 
    u16val = 0;
    size = sizeof(u16val);
    ret = ec_SDOread(slave_index, 0x2020, 0, FALSE, &size, &u16val, EC_TIMEOUTRXM);
    if (ret > 0) {
        printf("Motor type (0x2020): %d ", u16val);
        switch (u16val) {
            case 10: printf("(PMSM)\n"); break;
            case 20: printf("(Stepper)\n"); break;
            default: printf("(Unknown)\n"); break;
        }
    }
    
    // Read current operation mode
    i8val = 0;
    size = sizeof(i8val);
    ret = ec_SDOread(slave_index, 0x6060, 0, FALSE, &size, &i8val, EC_TIMEOUTRXM);
    if (ret > 0) {
        printf("Current mode (0x6060): %d ", i8val);
        switch (i8val) {
            case 9: printf("(CSV)\n"); break;
            case 8: printf("(CSP)\n"); break;
            case 10: printf("(CST)\n"); break;
            default: printf("(Other)\n"); break;
        }
    }
}

 /**
  * Map joystick value to velocity
  */
 int map_joystick_to_velocity(int joy_value) {
     // Dead zone implementation
     if (abs(joy_value) < 3000) {
         return 0;
     }
     
     // Map joystick value to velocity - ensure values stay within limits
     float normalized = (float)joy_value / (float)(joy_value >= 0 ? JOY_MAX_VAL : -JOY_MIN_VAL);
     
     // 80% of max velocity
     int velocity = (int)(normalized * MAX_VELOCITY * 0.8);
     
     // Add some debug info for velocity commands
     static int last_velocity = 0;
     if (velocity != 0 && last_velocity == 0) {
         printf("Sending non-zero velocity command: %d\n", velocity);
     }
     last_velocity = velocity;
     
     return velocity;
 }
 
 /**
  * CiA 402 state machine control
  */
 bool state_machine_control(uint16_t *controlword, uint16_t statusword, int target_state) {
     // Current state extraction from statusword
     bool ready_to_switch_on = statusword & SW_READY_TO_SWITCH_ON_BIT;
     bool switched_on = statusword & SW_SWITCHED_ON_BIT;
     bool operation_enabled = statusword & SW_OPERATION_ENABLED_BIT;
     bool fault = statusword & SW_FAULT_BIT;
     bool switch_on_disabled = statusword & SW_SWITCH_ON_DISABLED_BIT;
     
     // Debug output
     printf("Statusword: 0x%04X [RDY:%d, ON:%d, ENABLED:%d, FAULT:%d, DISABLED:%d]\n", 
            statusword, ready_to_switch_on, switched_on, operation_enabled, 
            fault, switch_on_disabled);
     
     // Special case - if statusword is 0, it's likely we have a communication issue
     if (statusword == 0) {
         printf("ERROR: Statusword is zero. Communication issue detected.\n");
         printf("Attempting to send controlword 0x80 (fault reset)\n");
         *controlword = CW_FAULT_RESET;
         return false;
     }
     
     // Check for specific error code
     static bool error_checked = false;
     if (!error_checked && (statusword & SW_FAULT_BIT)) {  // If fault bit is set
         uint16_t error_code = 0;
         int size = sizeof(error_code);
         int ret = ec_SDOread(slave_index, 0x603F, 0, FALSE, &size, &error_code, EC_TIMEOUTRXM);
         if (ret > 0) {
             printf("Error code: 0x%04X\n", error_code);
             if (error_code == 0x6320) {  // Parameter Error
                 printf("Detected parameter error (0x6320). The motor configuration may be incomplete.\n");
                 printf("Consider using OBLAC Drives software to properly configure the motor parameters.\n");
                 
                 // Try to read specific missing parameters
                 uint16_t specific_error = 0;
                 size = sizeof(specific_error);
                 ret = ec_SDOread(slave_index, 0x603F, 1, FALSE, &size, &specific_error, EC_TIMEOUTRXM);
                 if (ret > 0) {
                     printf("Specific parameter error: 0x%04X\n", specific_error);
                 }
             }
         }
         error_checked = true;
     }
     
     // State machine logic based on CiA 402 state machine
     if (fault) {
         printf("Drive is in FAULT state\n");
         *controlword = CW_FAULT_RESET;
         
         // Wait a bit after sending fault reset
         ec_send_processdata();
         usleep(50000);  // 50ms delay to let fault reset take effect
         
         return false;
     }
     
     switch (target_state) {
         case 1: // Switch on
             if (switch_on_disabled) {
                 printf("State: Switch on disabled -> Shutdown\n");
                 *controlword = CW_SHUTDOWN;
                 return false;
             } else if (!ready_to_switch_on) {
                 printf("State: Not ready -> Shutdown\n");
                 *controlword = CW_SHUTDOWN;
                 return false;
             } else if (!switched_on) {
                 printf("State: Ready -> Switch on\n");
                 *controlword = CW_SWITCHON;
                 return false;
             } else {
                 printf("State: Switched on achieved\n");
                 return true;
             }
             break;
             
         case 2: // Enable operation
             if (switch_on_disabled) {
                 printf("State: Switch on disabled -> Shutdown\n");
                 *controlword = CW_SHUTDOWN;
                 return false;
             } else if (!ready_to_switch_on) {
                 printf("State: Not ready -> Shutdown\n");
                 *controlword = CW_SHUTDOWN;
                 return false;
             } else if (!switched_on) {
                 printf("State: Ready -> Switch on\n");
                 *controlword = CW_SWITCHON;
                 return false;
             } else if (!operation_enabled) {
                 printf("State: Switched on -> Enable operation\n");
                 *controlword = CW_ENABLE;
                 return false;
             } else {
                 printf("State: Operation enabled achieved\n");
                 return true;
             }
             break;
             
         case 0: // Disable operation
             if (operation_enabled) {
                 printf("State: Operation enabled -> Disable operation\n");
                 *controlword = CW_DISABLEOPERATION;
                 return false;
             } else if (switched_on) {
                 printf("State: Switched on -> Shutdown\n");
                 *controlword = CW_SHUTDOWN;
                 return false;
             } else if (ready_to_switch_on) {
                 printf("State: Ready -> Disable voltage\n");
                 *controlword = CW_DISABLEVOLTAGE;
                 return false;
             } else {
                 printf("State: Disabled\n");
                 return true;
             }
             break;
             
         default:
             printf("Invalid target state: %d\n", target_state);
             return false;
     }
 }
 
 /**
  * Cyclic task to process PDOs
  */
 void* cyclic_task(void *arg) {
     (void)arg; // Suppress unused parameter warning
     
     rxpdo_t *rxpdo;
     txpdo_t *txpdo;
     int joystick_value = 0;
     int wkc;
     int timeout_counter = 0;
     
     // Process data pointers
     rxpdo = (rxpdo_t*)(ec_slave[slave_index].outputs);
     txpdo = (txpdo_t*)(ec_slave[slave_index].inputs);
     
     // Initialize controlword, opmode, etc.
     rxpdo->controlword = 0;
     rxpdo->op_mode = OP_MODE_CSV;
     rxpdo->target_velocity = 0;
     rxpdo->target_torque = 0;
     rxpdo->torque_offset = 0;
     
     int drive_initialized = 0;
     int64 cycletime_ns = cycletime * 1000;
     
     struct timespec ts;
     struct timespec tleft;
     
     // Command rate limiting
     int command_counter = 0;
     int last_velocity = 0;
     
     clock_gettime(CLOCK_MONOTONIC, &ts);
     ts.tv_nsec += cycletime_ns;
     if (ts.tv_nsec >= 1000000000) {
         ts.tv_sec++;
         ts.tv_nsec -= 1000000000;
     }
     
     while (run) {
         // Synchronize with DC
         clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
         
         // Calculate next cycle time
         ts.tv_nsec += cycletime_ns;
         if (ts.tv_nsec >= 1000000000) {
             ts.tv_sec++;
             ts.tv_nsec -= 1000000000;
         }
         
         // Read process data
         wkc = ec_receive_processdata(EC_TIMEOUTRET);
         
         // Verify wkc - check if we're getting valid data
         if (wkc <= 0) {
             static int comm_errors = 0;
             comm_errors++;
             if (comm_errors % 100 == 0) {
                 printf("WARNING: No process data received for %d cycles\n", comm_errors);
             }
         }
         
         // Check if the drive is initialized
         if (!drive_initialized) {
             // Initialize the drive state machine
             if (state_machine_control(&rxpdo->controlword, txpdo->statusword, 2)) {
                 drive_initialized = 1;
                 printf("Drive initialized and enabled!\n");
                 
                 // Set operation mode to CSV
                 rxpdo->op_mode = OP_MODE_CSV;
                 
                 // Wait for mode to be accepted
                 for (int i = 0; i < 10; i++) {
                     ec_send_processdata();
                     usleep(cycletime);
                     ec_receive_processdata(EC_TIMEOUTRET);
                     if (txpdo->op_mode_display == OP_MODE_CSV) {
                         printf("CSV mode activated successfully\n");
                         break;
                     }
                 }
                 
                 // Add additional verification of mode
                 int8_t current_mode = 0;
                 int size = sizeof(current_mode);
                 int ret = ec_SDOread(slave_index, 0x6061, 0, FALSE, &size, &current_mode, EC_TIMEOUTRXM);
                 if (ret > 0) {
                     printf("Confirmed operation mode via SDO: %d\n", current_mode);
                     if (current_mode != OP_MODE_CSV) {
                         printf("WARNING: Operation mode mismatch. Expected %d, got %d\n", 
                                OP_MODE_CSV, current_mode);
                     }
                 }
                 
                 // Read velocity factor to understand scaling
                 uint32_t vel_factor_num = 0, vel_factor_denom = 0;
                 size = sizeof(vel_factor_num);
                 ret = ec_SDOread(slave_index, 0x6096, 1, FALSE, &size, &vel_factor_num, EC_TIMEOUTRXM);
                 if (ret > 0) {
                     printf("Velocity factor numerator: %d\n", vel_factor_num);
                 }
                 ret = ec_SDOread(slave_index, 0x6096, 2, FALSE, &size, &vel_factor_denom, EC_TIMEOUTRXM);
                 if (ret > 0) {
                     printf("Velocity factor denominator: %d\n", vel_factor_denom);
                 }
                 
                 // Initial zero velocity command to ensure stable start
                 rxpdo->target_velocity = 0;
                 ec_send_processdata();
                 usleep(cycletime * 5);  // Slightly longer wait for stability
                 
                 // Check if we have a parameter error (0x6320)
                 uint16_t error_code = 0;
                 size = sizeof(error_code);
                 ret = ec_SDOread(slave_index, 0x603F, 0, FALSE, &size, &error_code, EC_TIMEOUTRXM);
                 
                 if (ret > 0 && error_code == 0x6320) {
                     printf("Detected parameter error (0x6320) before sending any velocity commands.\n");
                     printf("This suggests a fundamental configuration issue with the drive.\n");
                     
                     // Try to read specific error details
                     uint16_t specific_error = 0;
                     size = sizeof(specific_error);
                     ret = ec_SDOread(slave_index, 0x603F, 1, FALSE, &size, &specific_error, EC_TIMEOUTRXM);
                     if (ret > 0) {
                         printf("Specific parameter error: 0x%04X\n", specific_error);
                     }
                 }
                 
                 // Wait an additional moment before starting commands
                 usleep(500000);  // 500ms stable wait before sending velocity commands
                 printf("Starting velocity control with reduced command rate...\n");
             } else {
                 // Add timeout mechanism to avoid hanging
                 timeout_counter++;
                 if (timeout_counter > 1000) {  // About 4 seconds at 250Hz
                     printf("TIMEOUT: Drive initialization stalled. Attempting recovery...\n");
                     
                     // Try resetting any faults
                     rxpdo->controlword = CW_FAULT_RESET;
                     ec_send_processdata();
                     usleep(cycletime * 10);
                     
                     // Clear the command and try again
                     rxpdo->controlword = 0;
                     timeout_counter = 0;
                     
                     // Print debugging info
                     printf("PDO data dump - check structure alignment:\n");
                     printf("rxpdo size: %lu, txpdo size: %lu\n", 
                            sizeof(rxpdo_t), sizeof(txpdo_t));
                     printf("rxpdo memory at 0x%p, txpdo memory at 0x%p\n", 
                            (void*)rxpdo, (void*)txpdo);
                 }
             }
         } else {
             // Command rate limiting - only update velocity commands at reduced rate
             command_counter++;
             
             // Only update target velocity at reduced rate
             if (command_counter >= COMMAND_UPDATE_RATE) {
                 command_counter = 0;
                 
                 // Only start velocity commands if we're in a good state
                 if (!(txpdo->statusword & SW_FAULT_BIT)) {
                     // Simulate joystick input (replace with actual joystick code)
                     // Here we're just creating a sine wave for testing - slowed down by 5x
                     static int angle_counter = 0;
                     angle_counter++;
                     joystick_value = (int)(sin((double)angle_counter / 500.0) * JOY_MAX_VAL);
                     
                     // Map joystick value to velocity - update only on command_counter cycles
                     int new_velocity = map_joystick_to_velocity(joystick_value);
                     
                     // Only update if velocity has changed significantly to reduce commands
                     if (abs(new_velocity - last_velocity) > 50) {
                         rxpdo->target_velocity = new_velocity;
                         last_velocity = new_velocity;
                         printf("Updated velocity command: %d\n", new_velocity);
                     }
                 } else {
                     // If in fault state, try to reset
                     printf("FAULT detected during operation. Attempting reset...\n");
                     rxpdo->target_velocity = 0;  // Ensure velocity is zero
                     rxpdo->controlword = CW_FAULT_RESET;
                     last_velocity = 0;
                 }
             }
             
             // Print status information every 250 cycles (about 1 second)
             static int status_counter = 0;
             status_counter++;
             if (status_counter % 250 == 0) {
                 // First, check if controlword is being transmitted correctly
                 printf("Controlword sent: 0x%04X | ", rxpdo->controlword);
                 
                 printf("Joy: %6d | Target Vel: %6d | Actual Vel: %6d | Statusword: 0x%04X | Op Mode: %d\n",
                        joystick_value, rxpdo->target_velocity, txpdo->velocity_actual, 
                        txpdo->statusword, txpdo->op_mode_display);
                 
                 // Check if target velocity is being accepted
                 if (abs(rxpdo->target_velocity) > 0 && abs(txpdo->velocity_actual) < 10) {
                     printf("WARNING: Motor not responding to velocity commands\n");
                     
                     // Query potential error codes (optional)
                     uint16_t error_code = 0;
                     int size = sizeof(error_code);
                     int ret = ec_SDOread(slave_index, 0x603F, 0, FALSE, &size, &error_code, EC_TIMEOUTRXM);
                     if (ret > 0 && error_code != 0) {
                         printf("Error code: 0x%04X\n", error_code);
                         
                         if (error_code == 0x6320) {
                             printf("Parameter error detected. Please use OBLAC Drives to configure basic motor parameters.\n");
                             
                             // Try to read which specific parameter is problematic
                             uint16_t specific_error = 0;
                             size = sizeof(specific_error);
                             ret = ec_SDOread(slave_index, 0x603F, 1, FALSE, &size, &specific_error, EC_TIMEOUTRXM);
                             if (ret > 0) {
                                 printf("Specific parameter missing: 0x%04X\n", specific_error);
                             }
                         }
                     }
                 }
             }
         }
         
         // Write process data
         ec_send_processdata();
     }
     
     printf("Cyclic task shutdown initiated...\n");
     
     // Disable operation before exiting
     rxpdo->target_velocity = 0;
     ec_send_processdata();
     usleep(cycletime);
     ec_receive_processdata(EC_TIMEOUTRET);
     
     // State machine to disable operation
     int shutdown_timeout = 50; // Maximum number of cycles for shutdown
     while (!state_machine_control(&rxpdo->controlword, txpdo->statusword, 0) && 
            shutdown_timeout-- > 0) {
         ec_send_processdata();
         usleep(cycletime);
         ec_receive_processdata(EC_TIMEOUTRET);
     }
     
     printf("Cyclic task completed\n");
     return NULL;
 }
 
 /**
  * Main function
  */
 int main(int argc, char *argv[]) {
     printf("SOEM (Simple Open EtherCAT Master)\n");
     printf("Synapticon Actilink-S Motor Control\n");
     
     // Handle command line parameters
     if (argc > 1) {
         ifname = argv[1];
     }
     
     // Register signal handler with improved handling
     struct sigaction sa;
     sa.sa_handler = signal_handler;
     sigemptyset(&sa.sa_mask);
     sa.sa_flags = 0;
     
     sigaction(SIGINT, &sa, NULL);
     sigaction(SIGTERM, &sa, NULL);
     
     // Setup EtherCAT
     if (!setup_ethercat()) {
         printf("Failed to setup EtherCAT. Exiting.\n");
         return 1;
     }
     
     // Create cyclic task thread
     if (pthread_create(&thread1, NULL, &cyclic_task, NULL) != 0) {
         printf("Failed to create cyclic task thread\n");
         ec_close();
         return 1;
     }
     
     // Wait for task to complete - with periodic checks
     while (run) {
         sleep(1);  // Check every second instead of tight loop
     }
     
     // Proper cleanup
     cleanup_and_exit();
     
     return 0;
 }