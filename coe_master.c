#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include "ethercat.h"
#include "can_wrapper.h"
#include "can_monitor.h"

char IOmap[4096];
int expectedWKC;
volatile int wkc;
boolean needlf;
boolean inOP;
uint8 currentgroup = 0;
volatile int run = 1;

#define TARGET_VELOCITY 0x60FF
#define ACCELERATION_PROFILE 0x6083
#define DECELERATION_PROFILE 0x6084
#define QUICK_STOP_DECELERATION 0x6085
#define MAX_MOTOR_SPEED 0x6080
#define MAX_TORQUE 0x6072

#define DRIVE_CONTROLWORD 0x6040
#define DRIVE_STATUSWORD 0x6041
#define DRIVE_MODE_OF_OPERATION 0x6060
#define DRIVE_TARGET_POSITION 0x607A
#define DRIVE_ACTUAL_POSITION 0x6064
#define DRIVE_ACTUAL_VELOCITY 0x606C

/* State machine transition commands */
#define COMMAND_SHUTDOWN 0x06
#define COMMAND_SWITCH_ON 0x07
#define COMMAND_DISABLE_VOLTAGE 0x00
#define COMMAND_QUICK_STOP 0x02
#define COMMAND_DISABLE_OPERATION 0x07
#define COMMAND_ENABLE_OPERATION 0x0F
#define COMMAND_FAULT_RESET 0x80

/* Operation modes */
#define OP_MODE_PROFILE_POSITION 1
#define OP_MODE_PROFILE_VELOCITY 3
#define OP_MODE_PROFILE_TORQUE 4
#define OP_MODE_CSP 8
#define OP_MODE_CSV 9
#define OP_MODE_CST 10

/* Default velocity profile parameters */
#define DEFAULT_VELOCITY 1000
#define DEFAULT_ACCELERATION 10000
#define DEFAULT_DECELERATION 10000
#define DEFAULT_QUICK_STOP_DEC 20000

/* PDO mapping offsets (may need adjustment) */
#define OFFSET_CONTROLWORD 0       /* Byte offset for controlword in output PDO */
#define OFFSET_MODE_OF_OPERATION 2 /* Byte offset for operation mode in output PDO */
#define OFFSET_TARGET_VELOCITY 5   /* Byte offset for target velocity in output PDO */

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

void signal_handler(int sig)
{
    run = 0;
    printf("\nCaught signal %d, shutting down\n", sig);

    static int count = 0;
    if (++count > 1)
    {
        printf("Forcing exit...\n");
        _exit(1);
    }
}

/* Non-blocking keyboard input check */
int kbhit()
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

/* Get the current state of the drive based on statusword */
DriveState get_drive_state(uint16_t statusword)
{

    if ((statusword & 0x4F) == 0x00)
        return STATE_NOT_READY_TO_SWITCH_ON;
    if ((statusword & 0x4F) == 0x40)
        return STATE_SWITCH_ON_DISABLED;
    if ((statusword & 0x6F) == 0x21)
        return STATE_READY_TO_SWITCH_ON;
    if ((statusword & 0x6F) == 0x23)
        return STATE_SWITCHED_ON;
    if ((statusword & 0x6F) == 0x27)
        return STATE_OPERATION_ENABLED;
    if ((statusword & 0x6F) == 0x07)
        return STATE_QUICK_STOP_ACTIVE;
    if ((statusword & 0x4F) == 0x0F)
        return STATE_FAULT_REACTION_ACTIVE;
    if ((statusword & 0x4F) == 0x08)
        return STATE_FAULT;

    return STATE_FAULT;
}

uint16_t process_state_machine(DriveState state, int enable_status, int *operation_enabled, int *target_velocity, int cycle_counter)
{
    uint16_t controlword = 0;

    switch (state)
    {
    case STATE_NOT_READY_TO_SWITCH_ON:
        /* Wait for drive to become ready */
        controlword = 0x00;
        *operation_enabled = 0;
        *target_velocity = 0;
        break;

    case STATE_SWITCH_ON_DISABLED:
        /* Transition to Ready To Switch On */
        controlword = COMMAND_SHUTDOWN;
        *operation_enabled = 0;
        *target_velocity = 0;
        break;

    case STATE_READY_TO_SWITCH_ON:
        /* Transition to Switched On */
        controlword = COMMAND_SWITCH_ON;
        *operation_enabled = 0;
        *target_velocity = 0;
        break;

    case STATE_SWITCHED_ON:
        /* Transition to Operation Enabled only if enable button is active */
        if (enable_status)
        {
            controlword = COMMAND_ENABLE_OPERATION;
        }
        else
        {
            controlword = COMMAND_SWITCH_ON; // Stay in Switched On state
        }
        *operation_enabled = 0;
        *target_velocity = 0;
        break;

    case STATE_OPERATION_ENABLED:
        /* Already in Operation Enabled */
        controlword = COMMAND_ENABLE_OPERATION;
        *operation_enabled = 1;

        if (enable_status)
        {
            /* When enabled, gradually ramp up velocity - oscillate direction every 5 seconds */
            if (cycle_counter % 5000 < 2500)
            {
                *target_velocity = DEFAULT_VELOCITY; // Forward
            }
            else
            {
                *target_velocity = -DEFAULT_VELOCITY; // Reverse
            }
        }
        else
        {
            /* When disabled, stop */
            *target_velocity = 0;
            /* Disable operation */
            controlword = COMMAND_DISABLE_OPERATION;
        }
        break;

    case STATE_QUICK_STOP_ACTIVE:
        /* Handle quick stop */
        controlword = COMMAND_ENABLE_OPERATION; /* Try to re-enable after quick stop */
        *operation_enabled = 0;
        *target_velocity = 0;
        break;

    case STATE_FAULT_REACTION_ACTIVE:
        /* Wait for fault reaction to complete */
        controlword = 0x00;
        *operation_enabled = 0;
        *target_velocity = 0;
        break;

    case STATE_FAULT:
        /* Reset fault */
        controlword = COMMAND_FAULT_RESET;
        *operation_enabled = 0;
        *target_velocity = 0;
        break;

    default:
        /* Default to fault reset if status is unknown */
        controlword = COMMAND_FAULT_RESET;
        *operation_enabled = 0;
        *target_velocity = 0;
        break;
    }

    return controlword;
}

void simpletest(char *ifname)
{
    int chk, slc;

    /* Install signal handler for Ctrl+C */
    signal(SIGINT, signal_handler);

    printf("Starting EtherCAT master\n");
    printf("Press any key to stop the program\n");

    /* Initialize SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("Initialized on %s\n", ifname);

        /* Find and auto-configure slaves */
        if (ec_config_init(FALSE) > 0)
        {
            printf("%d slaves found and configured.\n", ec_slavecount);

            /* Verify if all drives are detected correctly */
            for (slc = 1; slc <= ec_slavecount; slc++)
            {
                printf("Slave %d - Name: %s, Output size: %dbits, Input size: %dbits, State: %d\n",
                       slc, ec_slave[slc].name, ec_slave[slc].Obits, ec_slave[slc].Ibits,
                       ec_slave[slc].state);
            }

            /* REMOVE FOR REAL SLAVE : Configure PDO mappings explicitly */
            for (slc = 1; slc <= ec_slavecount; slc++)
            {
                printf("Configuring PDOs for slave %d\n", slc);

                // Set SM2 and SM3 PDO mappings explicitly
                ec_slave[slc].SMtype[2] = 3; // SM2 is for outputs
                ec_slave[slc].SMtype[3] = 4; // SM3 is for inputs

                // Output PDO mappings
                ec_slave[slc].Obits = 280;
                ec_slave[slc].Ibits = 376;

                // Update output and input lengths in bytes
                ec_slave[slc].Obytes = 35;
                ec_slave[slc].Ibytes = 47;

                printf("Manual PDO config: Obits=%d, Ibits=%d, Obytes=%d, Ibytes=%d\n",
                       ec_slave[slc].Obits, ec_slave[slc].Ibits,
                       ec_slave[slc].Obytes, ec_slave[slc].Ibytes);
            }
            /* REMOVE FOR REAL SLAVE : Configure PDO mappings explicitly */

            ec_config_map(&IOmap);
            ec_configdc();

            printf("Slaves mapped, state to SAFE_OP.\n");
            /* Wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

            for (slc = 1; slc <= ec_slavecount; slc++)
            {
                printf("Slave %d - State: %d\n", slc, ec_slave[slc].state);
            }

            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);

            printf("Request operational state for all slaves\n");
            ec_slave[0].state = EC_STATE_OPERATIONAL;

            /* Request OP state for all slaves */
            ec_writestate(0);

            /* Wait for all slaves to reach OP state */
            chk = 40;
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                printf("Operational state reached for all slaves.\n");
                inOP = TRUE;

                uint16_t statusword = 0;
                uint16_t controlword = 0;
                int32_t velocity_actual = 0;
                DriveState drive_state;

                printf("Starting CiA402 state machine...\n");

                for (slc = 1; slc <= ec_slavecount; slc++)
                {
                    /* Configure profile velocity parameters via SDO before enabling the drive */
                    int32_t acceleration = DEFAULT_ACCELERATION;
                    int32_t deceleration = DEFAULT_DECELERATION;
                    int32_t quick_stop_dec = DEFAULT_QUICK_STOP_DEC;
                    uint32_t max_velocity = DEFAULT_VELOCITY * 2; // Set max to twice our target for safety

                    /* Write acceleration profile */
                    int size = sizeof(acceleration);
                    int ret = ec_SDOwrite(slc, ACCELERATION_PROFILE, 0, FALSE, size, &acceleration, EC_TIMEOUTRXM);
                    printf("SDO write acceleration profile result: %d\n", ret);

                    /* Write deceleration profile */
                    size = sizeof(deceleration);
                    ret = ec_SDOwrite(slc, DECELERATION_PROFILE, 0, FALSE, size, &deceleration, EC_TIMEOUTRXM);
                    printf("SDO write deceleration profile result: %d\n", ret);

                    /* Write quick stop deceleration */
                    size = sizeof(quick_stop_dec);
                    ret = ec_SDOwrite(slc, QUICK_STOP_DECELERATION, 0, FALSE, size, &quick_stop_dec, EC_TIMEOUTRXM);
                    printf("SDO write quick stop deceleration result: %d\n", ret);

                    /* Write maximum motor speed */
                    size = sizeof(max_velocity);
                    ret = ec_SDOwrite(slc, MAX_MOTOR_SPEED, 0, FALSE, size, &max_velocity, EC_TIMEOUTRXM);
                    printf("SDO write max motor speed result: %d\n", ret);

                    /* Initialize outputs with initial values */
                    memset(ec_slave[slc].outputs, 0, ec_slave[slc].Obytes);

                    // Set initial controlword to Reset Fault (bit 7)
                    *(uint16_t *)(ec_slave[slc].outputs + OFFSET_CONTROLWORD) = COMMAND_FAULT_RESET;

                    // Set profile velocity mode (3)
                    *(int8_t *)(ec_slave[slc].outputs + OFFSET_MODE_OF_OPERATION) = OP_MODE_PROFILE_VELOCITY;

                    // Set initial target velocity to 0
                    *(int32_t *)(ec_slave[slc].outputs + OFFSET_TARGET_VELOCITY) = 0;

                    // Send data
                    ec_send_processdata();
                    wkc = ec_receive_processdata(EC_TIMEOUTRET);

                    printf("Initial send/receive WKC: %d\n", wkc);
                    osal_usleep(100000); // 100ms delay to allow fault reset

                    // Now send SHUTDOWN command to start state machine
                    *(uint16_t *)(ec_slave[slc].outputs + OFFSET_CONTROLWORD) = COMMAND_SHUTDOWN;
                    ec_send_processdata();
                    wkc = ec_receive_processdata(EC_TIMEOUTRET);
                    osal_usleep(100000); // 100ms delay
                }

                /* Try reading via SDO for diagnostic purposes */
                for (slc = 1; slc <= ec_slavecount; slc++)
                {
                    uint16_t statusword_sdo = 0;
                    int size = sizeof(statusword_sdo);
                    int ret = ec_SDOread(slc, DRIVE_STATUSWORD, 0, FALSE, &size, &statusword_sdo, EC_TIMEOUTRXM);
                    printf("SDO read statusword result: %d, value: 0x%04X\n", ret, statusword_sdo);

                    int8_t opmode = 0;
                    size = sizeof(opmode);
                    ret = ec_SDOread(slc, 0x6061, 0, FALSE, &size, &opmode, EC_TIMEOUTRXM);
                    printf("SDO read mode of operation display result: %d, value: %d\n", ret, opmode);

                    printf("Check if we are in profile velocity mode (expect 3): %d\n", opmode);
                }

                /* Main loop - stay in OP state until user input or error */
                int cycle_counter = 0;
                int target_velocity = 0;
                int enable_status = 0;
                int operation_enabled = 0;
                int prev_state = (DriveState)(-1);

                printf("Starting main control loop...\n");
                printf("Press any key to stop the program or Ctrl+C to exit.\n");

                /* Run until user input or signal */
                while (run && !kbhit())
                {
                    /* Start processing PDO data */
                    ec_send_processdata();
                    wkc = ec_receive_processdata(EC_TIMEOUTRET);

                    enable_status = get_can_enable_safe();

                    /* Periodically print CAN status for monitoring */
                    if (cycle_counter % 5000 == 0)
                    { /* Every 5 seconds at 1ms cycle time */
                        print_can_status();
                    }

                    /* Check if we received the expected working counter */
                    if (wkc >= expectedWKC)
                    {
                        /* Process each slave */
                        for (slc = 1; slc <= ec_slavecount; slc++)
                        {
                            /* Read statusword */
                            statusword = *(uint16_t *)(ec_slave[slc].inputs);

                            /* Get the current drive state */
                            drive_state = get_drive_state(statusword);

                            /* Log state transitions */
                            if ((int)prev_state != (int)drive_state)
                            {
                                const char *state_names[] = {
                                    "Not Ready To Switch On",
                                    "Switch On Disabled",
                                    "Ready To Switch On",
                                    "Switched On",
                                    "Operation Enabled",
                                    "Quick Stop Active",
                                    "Fault Reaction Active",
                                    "Fault"};
                                printf("Drive state changed: %s -> %s\n",
                                       (prev_state >= 0) ? state_names[prev_state] : "Unknown",
                                       state_names[drive_state]);
                                prev_state = drive_state;
                            }

                            /* Read actual velocity (assuming it's at offset 4) */
                            /* Exact offset might need adjustment based on PDO mapping */
                            velocity_actual = *(int32_t *)(ec_slave[slc].inputs + 4);

                            /* Debug output every 1000 cycles (1 second) */
                            if (cycle_counter % 1000 == 0)
                            {
                                printf("Slave %d: State: %d, Statusword: 0x%04X, Actual Velocity: %d, Target: %d, Enable: %d\n",
                                       slc, drive_state, statusword, velocity_actual, target_velocity, enable_status);
                            }

                            /* Process the CiA402 state machine */
                            controlword = process_state_machine(drive_state, enable_status,
                                                                &operation_enabled, &target_velocity,
                                                                cycle_counter);

                            /* Set velocity in PDO */
                            *(int32_t *)(ec_slave[slc].outputs + OFFSET_TARGET_VELOCITY) = target_velocity;

                            /* Maintain profile velocity mode */
                            *(int8_t *)(ec_slave[slc].outputs + OFFSET_MODE_OF_OPERATION) = OP_MODE_PROFILE_VELOCITY;

                            /* Write controlword */
                            *(uint16_t *)(ec_slave[slc].outputs + OFFSET_CONTROLWORD) = controlword;
                        }
                    }
                    else
                    {
                        /* Working counter error - log every 1000 cycles to avoid flooding */
                        if (cycle_counter % 1000 == 0)
                        {
                            printf("Working counter error - expected: %d, got: %d\n", expectedWKC, wkc);
                        }

                        /* Check if we've had continuous errors for a long time */
                        if (cycle_counter % 10000 == 0)
                        {
                            printf("Too many working counter errors, consider stopping\n");
                        }
                    }

                    /* Increment cycle counter */
                    cycle_counter++;

                    /* Sleep for 1ms */
                    osal_usleep(1000);
                }

                /* Graceful shutdown - stop motion and disable operation */
                printf("Shutting down...\n");
                for (slc = 1; slc <= ec_slavecount; slc++)
                {
                    /* Set velocity to zero */
                    *(int32_t *)(ec_slave[slc].outputs + OFFSET_TARGET_VELOCITY) = 0;

                    /* Send velocity zero command */
                    ec_send_processdata();
                    wkc = ec_receive_processdata(EC_TIMEOUTRET);
                    osal_usleep(500000); // 500ms to allow deceleration

                    /* Disable operation */
                    *(uint16_t *)(ec_slave[slc].outputs + OFFSET_CONTROLWORD) = COMMAND_DISABLE_OPERATION;
                    ec_send_processdata();
                    wkc = ec_receive_processdata(EC_TIMEOUTRET);
                    osal_usleep(100000); // 100ms delay

                    /* Disable voltage */
                    *(uint16_t *)(ec_slave[slc].outputs + OFFSET_CONTROLWORD) = COMMAND_DISABLE_VOLTAGE;
                    ec_send_processdata();
                    wkc = ec_receive_processdata(EC_TIMEOUTRET);
                }

                inOP = FALSE;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for (slc = 1; slc <= ec_slavecount; slc++)
                {
                    if (ec_slave[slc].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               slc, ec_slave[slc].state, ec_slave[slc].ALstatuscode, ec_ALstatuscode2string(ec_slave[slc].ALstatuscode));
                    }
                }
            }

            printf("Request safe operational state for all slaves\n");
            ec_slave[0].state = EC_STATE_SAFE_OP;
            /* request SAFE_OP state for all slaves */
            ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }

        printf("End simple test, close socket\n");
        /* Stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n", ifname);
    }
}

int main(int argc, char *argv[])
{
    signal(SIGINT, signal_handler);

    if (!initialize_python_can())
    {
        printf("Failed to initialize Python CAN interface, continuing without it\n");
    }
    else
    {
        acquire_gil();
        set_yellow_bat_led(1);
        release_gil();

        if (!init_can_monitor())
        {
            printf("Warning: CAN monitoring disabled\n");
        }
    }

    if (argc > 1)
    {
        simpletest(argv[1]);
    }
    else
    {
        printf("Usage: %s <network interface>\nExample: %s eth0\n", argv[0], argv[0]);
    }

    printf("End program\n");

    acquire_gil();
    set_yellow_bat_led(0);
    release_gil();

    stop_can_monitor();
    _exit(0);

    return 0;
}