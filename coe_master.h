#ifndef COE_MASTER_H
#define COE_MASTER_H

#include <stdint.h>
#include <stdbool.h>
#include "ethercat.h"

/* CiA402 drive states */
typedef enum
{
    STATE_NOT_READY_TO_SWITCH_ON = 0,
    STATE_SWITCH_ON_DISABLED,
    STATE_READY_TO_SWITCH_ON,
    STATE_SWITCHED_ON,
    STATE_OPERATION_ENABLED,
    STATE_QUICK_STOP_ACTIVE,
    STATE_FAULT_REACTION_ACTIVE,
    STATE_FAULT
} DriveState;

/**
 * @brief Main entry point for EtherCAT control system
 * @param ifname Network interface name to use for EtherCAT communication
 */
void coe_master_control(char *ifname);

/**
 * @brief Initialize the EtherCAT master on specified interface
 * @param ifname Network interface name
 * @return 1 if successful, 0 if failed
 */
int init_ethercat(char *ifname);

/**
 * @brief Discover and configure all connected EtherCAT slaves
 * @return 1 if slaves found and configured, 0 if failed
 */
int discover_and_configure_slaves(void);

/**
 * @brief Configure PDO mapping for all slaves
 */
void configure_pdo_mapping(void);

/**
 * @brief Transition all slaves to operational state
 * @return 1 if all slaves reached operational state, 0 if failed
 */
int transition_to_operational(void);

/**
 * @brief Configure drives with SDO parameters
 */
void configure_drives(void);

/**
 * @brief Main control loop for running the EtherCAT system
 */
void run_control_loop(void);

/**
 * @brief Gracefully shutdown all drives
 */
void shutdown_drives(void);

/**
 * @brief Cleanup EtherCAT resources
 */
void cleanup_ethercat(void);

/**
 * @brief Process controller input for differential drive system
 * @param left_motor_velocity Pointer to left motor velocity value
 * @param right_motor_velocity Pointer to right motor velocity value
 * @param cycle_counter Current cycle counter for logging
 */
void process_controller_input(int *left_motor_velocity, int *right_motor_velocity, int cycle_counter);

/**
 * @brief Get the current state of the drive based on statusword
 * @param statusword Drive statusword from CiA402 state machine
 * @return The current drive state
 */
DriveState get_drive_state(uint16_t statusword);

/**
 * @brief Process the CiA402 state machine
 * @param state Current drive state
 * @param enable_status Enable button status
 * @param operation_enabled Pointer to operation enabled flag
 * @param target_velocity Pointer to target velocity
 * @param cycle_counter Current cycle counter
 * @return Control word to send to the drive
 */
uint16_t process_state_machine(DriveState state, int enable_status, int *operation_enabled,
                               int *target_velocity, int cycle_counter);

/**
 * @brief Signal handler for interrupts
 * @param sig Signal number
 */
void signal_handler(int sig);

/**
 * @brief Check for keyboard input (non-blocking)
 * @return 1 if key pressed, 0 if not
 */
int kbhit(void);

/* Global variables that are used across functions */
extern char IOmap[4096];
extern int expectedWKC;
extern volatile int wkc;
extern boolean needlf;
extern boolean inOP;
extern uint8 currentgroup;
extern volatile int run;

#endif /* COE_MASTER_H */