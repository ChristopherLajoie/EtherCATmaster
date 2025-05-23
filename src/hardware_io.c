/**
 * @file hardware_io.c
 * @brief Hardware interface for EtherCAT communication and terminal I/O
 *
 * This file provides functions for EtherCAT initialization, PDO mapping
 * configuration, and terminal raw mode handling for keyboard input.
 */

#include "common.h"
#include "hardware_io.h"
#include "config.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#define MAX_STATE_TRANSITION_RETRIES 3
#define STATE_CHECK_ITERATIONS 40
#define STATE_CHECK_TIMEOUT_US 100000
#define STATE_CHECK_DELAY_US 50000

static struct termios orig_termios;

/**
 * @brief Get the CiA 402 state from a statusword
 * @param statusword Status word from the drive
 * @return The CiA 402 state
 */
cia402_state_t get_cia402_state(uint16_t statusword)
{
    uint8_t state_bits = statusword & STATUS_STATE_MASK;

    switch (state_bits)
    {
        case 0x00:
            return CIA402_STATE_NOT_READY;
        case 0x40:
            return CIA402_STATE_SWITCH_ON_DISABLED;
        case 0x21:
            return CIA402_STATE_READY_TO_SWITCH_ON;
        case 0x23:
            return CIA402_STATE_SWITCHED_ON;
        case 0x27:
            return CIA402_STATE_OPERATION_ENABLED;
        case 0x07:
            return CIA402_STATE_QUICK_STOP_ACTIVE;
        case 0x0F:
            return CIA402_STATE_FAULT_REACTION_ACTIVE;
        case 0x08:
            return CIA402_STATE_FAULT;
        default:
            return CIA402_STATE_UNKNOWN;
    }
}

/**
 * @brief Get a human-readable string for a CiA 402 state
 * @param state The CiA 402 state
 * @return String representation of the state
 */
const char* get_cia402_state_string(cia402_state_t state)
{
    switch (state)
    {
        case CIA402_STATE_NOT_READY:
            return "Not Ready To Switch On";
        case CIA402_STATE_SWITCH_ON_DISABLED:
            return "Switch On Disabled";
        case CIA402_STATE_READY_TO_SWITCH_ON:
            return "Ready To Switch On";
        case CIA402_STATE_SWITCHED_ON:
            return "Switched On";
        case CIA402_STATE_OPERATION_ENABLED:
            return "Operation Enabled";
        case CIA402_STATE_QUICK_STOP_ACTIVE:
            return "Quick Stop Active";
        case CIA402_STATE_FAULT_REACTION_ACTIVE:
            return "Fault Reaction Active";
        case CIA402_STATE_FAULT:
            return "Fault";
        case CIA402_STATE_UNKNOWN:
        default:
            return "Unknown State";
    }
}

/**
 * @brief Decode and print the CiA 402 statusword bit by bit
 * @param statusword Status word from the drive
 */
void cia402_decode_statusword(uint16_t statusword)
{
    cia402_state_t current_state = get_cia402_state(statusword);

    printf("Status 0x%04X - State Machine: %s\n", statusword, get_cia402_state_string(current_state));

    printf("  0:%-20s  1:%-20s  2:%-20s  3:%-20s\n",
           (statusword & SW_READY_TO_SWITCH_ON_BIT) ? "Ready to switch on" : "Not ready",
           (statusword & SW_SWITCHED_ON_BIT) ? "Switched on" : "Switched off",
           (statusword & SW_OPERATION_ENABLED_BIT) ? "Operation enabled" : "Op disabled",
           (statusword & SW_FAULT_BIT) ? "Fault present" : "No fault");

    printf("  4:%-20s  5:%-20s  6:%-20s  7:%-20s\n",
           (statusword & SW_VOLTAGE_ENABLED_BIT) ? "Voltage enabled" : "Voltage disabled",
           (statusword & SW_QUICK_STOP_BIT) ? "Quick stop disabled" : "Quick stop enabled",
           (statusword & SW_SWITCH_ON_DISABLED_BIT) ? "Switch on disabled" : "Switch on enabled",
           (statusword & (1 << 7)) ? "Warning present" : "No warning");
}

/**
 * @brief Configure RxPDO mappings for a slave
 * @param slave Slave index to configure
 */
static bool configure_rxpdo_mappings(int slave)
{
    int ret;
    uint8_t rxpdo_count = 0;
    uint32_t rxpdo_mapping;
    uint8_t zero = 0;

    /* Disable RxPDO mapping to configure it */
    ret = ec_SDOwrite(slave, RX_PDO_INDEX, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to clear RxPDO mapping: %d\n", ret);
        return false;
    }

    /* Map control word (16 bits) */
    rxpdo_mapping = 0x60400010; /* Index:Subindex:BitLength */
    ret = ec_SDOwrite(slave, RX_PDO_INDEX, ++rxpdo_count, FALSE, sizeof(rxpdo_mapping), &rxpdo_mapping, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to map Controlword: %d\n", ret);
        return false;
    }

    /* Map modes of operation (8 bits) */
    rxpdo_mapping = 0x60600008;
    ret = ec_SDOwrite(slave, RX_PDO_INDEX, ++rxpdo_count, FALSE, sizeof(rxpdo_mapping), &rxpdo_mapping, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to map Modes of operation: %d\n", ret);
        return false;
    }

    /* Map target torque (16 bits) */
    rxpdo_mapping = 0x60710010;
    ret = ec_SDOwrite(slave, RX_PDO_INDEX, ++rxpdo_count, FALSE, sizeof(rxpdo_mapping), &rxpdo_mapping, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to map Target torque: %d\n", ret);
        return false;
    }

    /* Map target position (32 bits) */
    rxpdo_mapping = 0x607A0020;
    ret = ec_SDOwrite(slave, RX_PDO_INDEX, ++rxpdo_count, FALSE, sizeof(rxpdo_mapping), &rxpdo_mapping, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to map Target position: %d\n", ret);
        return false;
    }

    /* Map target velocity (32 bits) */
    rxpdo_mapping = 0x60FF0020;
    ret = ec_SDOwrite(slave, RX_PDO_INDEX, ++rxpdo_count, FALSE, sizeof(rxpdo_mapping), &rxpdo_mapping, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to map Target velocity: %d\n", ret);
        return false;
    }

    /* Map torque offset (16 bits) */
    rxpdo_mapping = 0x60B20010;
    ret = ec_SDOwrite(slave, RX_PDO_INDEX, ++rxpdo_count, FALSE, sizeof(rxpdo_mapping), &rxpdo_mapping, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to map Torque offset: %d\n", ret);
        return false;
    }

    /* Map tuning command (32 bits) - Synapticon-specific extension */
    rxpdo_mapping = 0x27010020;
    ret = ec_SDOwrite(slave, RX_PDO_INDEX, ++rxpdo_count, FALSE, sizeof(rxpdo_mapping), &rxpdo_mapping, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to map Tuning command: %d\n", ret);
        return false;
    }

    /* Set the number of configured RxPDO entries */
    ret = ec_SDOwrite(slave, RX_PDO_INDEX, 0, FALSE, sizeof(rxpdo_count), &rxpdo_count, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to set RxPDO count: %d\n", ret);
        return false;
    }

    return true;
}

/**
 * @brief Configure TxPDO mappings for a slave
 * @param slave Slave index to configure
 */
static bool configure_txpdo_mappings(int slave)
{
    int ret;
    uint8_t txpdo_count = 0;
    uint32_t txpdo_mapping;
    uint8_t zero = 0;

    /* Disable TxPDO mapping to configure it */
    ret = ec_SDOwrite(slave, TX_PDO_INDEX, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to clear TxPDO mapping: %d\n", ret);
        return false;
    }

    /* Map status word (16 bits) */
    txpdo_mapping = 0x60410010;
    ret = ec_SDOwrite(slave, TX_PDO_INDEX, ++txpdo_count, FALSE, sizeof(txpdo_mapping), &txpdo_mapping, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to map Statusword: %d\n", ret);
        return false;
    }

    /* Map modes of operation display (8 bits) */
    txpdo_mapping = 0x60610008;
    ret = ec_SDOwrite(slave, TX_PDO_INDEX, ++txpdo_count, FALSE, sizeof(txpdo_mapping), &txpdo_mapping, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to map Modes of operation display: %d\n", ret);
        return false;
    }

    /* Map position actual value (32 bits) */
    txpdo_mapping = 0x60640020;
    ret = ec_SDOwrite(slave, TX_PDO_INDEX, ++txpdo_count, FALSE, sizeof(txpdo_mapping), &txpdo_mapping, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to map Position actual value: %d\n", ret);
        return false;
    }

    /* Map velocity actual value (16 bits) */
    txpdo_mapping = 0x606C0010; /* Using 16-bit to match txpdo_t struct */
    ret = ec_SDOwrite(slave, TX_PDO_INDEX, ++txpdo_count, FALSE, sizeof(txpdo_mapping), &txpdo_mapping, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to map Velocity actual value: %d\n", ret);
        return false;
    }

    /* Map torque actual value (16 bits) */
    txpdo_mapping = 0x60770010;
    ret = ec_SDOwrite(slave, TX_PDO_INDEX, ++txpdo_count, FALSE, sizeof(txpdo_mapping), &txpdo_mapping, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to map Torque actual value: %d\n", ret);
        return false;
    }

    /* Set the number of configured TxPDO entries */
    ret = ec_SDOwrite(slave, TX_PDO_INDEX, 0, FALSE, sizeof(txpdo_count), &txpdo_count, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to set TxPDO count: %d\n", ret);
        return false;
    }

    return true;
}

/**
 * @brief Configure motion parameters
 * @param slave Slave index to configure
 * @return true if successful, false on error
 */
static bool configure_motion_parameters(int slave)
{
    int ret;

    /* Profile velocity (0 = use drive's internal profile generator) */
    uint32_t profile_velocity = 0;
    ret = ec_SDOwrite(slave, 0x6081, 0, FALSE, sizeof(profile_velocity), &profile_velocity, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to set profile velocity: %d\n", ret);
        return false;
    }

    /* Max motor speed */
    uint32_t max_motor_speed = MAX_VELOCITY;
    ret = ec_SDOwrite(slave, 0x6080, 0, FALSE, sizeof(max_motor_speed), &max_motor_speed, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to set max motor speed: %d\n", ret);
        return false;
    }

    /* Profile acceleration */
    uint32_t profile_acceleration = DEFAULT_PROFILE_ACCEL;
    ret = ec_SDOwrite(slave, 0x6083, 0, FALSE, sizeof(profile_acceleration), &profile_acceleration, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to set profile acceleration: %d\n", ret);
        return false;
    }

    /* Profile deceleration */
    uint32_t profile_deceleration = DEFAULT_PROFILE_DECEL;
    ret = ec_SDOwrite(slave, 0x6084, 0, FALSE, sizeof(profile_deceleration), &profile_deceleration, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to set profile deceleration: %d\n", ret);
        return false;
    }

    /* Quick stop deceleration */
    uint32_t quick_stop_decel = DEFAULT_QUICK_STOP_DECEL;
    ret = ec_SDOwrite(slave, 0x6085, 0, FALSE, sizeof(quick_stop_decel), &quick_stop_decel, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to set quick stop deceleration: %d\n", ret);
        return false;
    }

    /* Motion profile type (0 = Linear ramps) */
    int16_t motion_profile_type = DEFAULT_PROFILE_TYPE;
    ret = ec_SDOwrite(slave, 0x6086, 0, FALSE, sizeof(motion_profile_type), &motion_profile_type, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to set motion profile type: %d\n", ret);
        return false;
    }

    return true;
}

/**
 * @brief Configure PDO mappings for EtherCAT slave
 * @param slave Slave index to configure
 * @return true if successful, false on error
 */
bool configure_pdo_mappings(int slave)
{
    int ret;
    uint8_t zero = 0;
    uint8_t num_pdos = 1;

    /* Disable PDO assignments during configuration */
    ret = ec_SDOwrite(slave, 0x1C12, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to disable RxPDO assignments: %d\n", ret);
        return false;
    }

    ret = ec_SDOwrite(slave, 0x1C13, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to disable TxPDO assignments: %d\n", ret);
        return false;
    }

    /* Configure RxPDO mappings */
    if (!configure_rxpdo_mappings(slave))
    {
        return false;
    }

    /* Configure TxPDO mappings */
    if (!configure_txpdo_mappings(slave))
    {
        return false;
    }

    /* Assign PDO mappings */
    ret = ec_SDOwrite(slave, 0x1C12, 0, FALSE, sizeof(num_pdos), &num_pdos, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to set RxPDO assignment count: %d\n", ret);
        return false;
    }

    ret = ec_SDOwrite(slave, 0x1C13, 0, FALSE, sizeof(num_pdos), &num_pdos, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fprintf(stderr, "Failed to set TxPDO assignment count: %d\n", ret);
        return false;
    }

    /* Configure motion parameters */
    if (!configure_motion_parameters(slave))
    {
        return false;
    }

    return true;
}

/**
 * @brief Wait for EtherCAT slave to reach PRE_OP state with retry
 * @return true if successful, false on error
 */
static bool wait_for_preop_state(void)
{
    int retry_count = 0;
    int chk;
    bool preop_reached = false;

    while (!preop_reached && retry_count < MAX_STATE_TRANSITION_RETRIES)
    {
        /* Try to reach PRE_OP state with adequate timeout */
        chk = STATE_CHECK_ITERATIONS;
        do
        {
            ec_statecheck(0, EC_STATE_PRE_OP, STATE_CHECK_TIMEOUT_US);
            if (ec_slave[0].state == EC_STATE_PRE_OP)
            {
                preop_reached = true;
                break;
            }
            usleep(STATE_CHECK_DELAY_US);
        }
        while (chk--);

        if (!preop_reached)
        {
            printf("PRE_OP state retry %d/%d\n", retry_count + 1, MAX_STATE_TRANSITION_RETRIES);
            /* Force state transition again */
            ec_slave[0].state = EC_STATE_PRE_OP;
            ec_writestate(0);
            retry_count++;
        }
    }

    if (!preop_reached)
    {
        fprintf(stderr, "Error: Failed to reach PRE_OP state after %d attempts\n", MAX_STATE_TRANSITION_RETRIES);
        return false;
    }

    return true;
}

/**
 * @brief Read basic drive parameters
 * @param slave Slave index to read from
 */
static void read_drive_parameters(int slave)
{
    uint32_t u32val = 0;
    int size = sizeof(u32val);
    int ret;

    ret = ec_SDOread(slave, 0x6080, 0, FALSE, &size, &u32val, EC_TIMEOUTRXM);
    if (ret > 0)
    {
        printf("Max motor speed: %d RPM\n", u32val);
    }
    else
    {
        fprintf(stderr, "Failed to read max motor speed\n");
    }

    ret = ec_SDOread(slave, 0x6073, 0, FALSE, &size, &u32val, EC_TIMEOUTRXM);
    if (ret > 0)
    {
        printf("Max current: %d mA\n", u32val);
    }
    else
    {
        fprintf(stderr, "Failed to read max current\n");
    }
}

/**
 * @brief Initialize EtherCAT communication
 * @return true if successful, false on error
 */
bool ethercat_init(void)
{
    int chk;
    int slave;

    /* Initialize SOEM, bind socket to ifname */
    if (!ec_init(g_motor_control.ifname))
    {
        fprintf(stderr, "No socket connection on %s\nExecute as root\n", g_motor_control.ifname);
        return false;
    }

    /* Find and auto-configure slaves */
    if (ec_config_init(FALSE) <= 0)
    {
        fprintf(stderr, "No slaves found!\n");
        ec_close();
        return false;
    }

    printf("%d slaves found and configured.\n", ec_slavecount);
    slave = g_motor_control.slave_index;

    /* Set state to PRE_OP for configuration */
    ec_slave[0].state = EC_STATE_PRE_OP;
    ec_writestate(0);

    /* Wait for PRE_OP state with retry */
    if (!wait_for_preop_state())
    {
        ec_close();
        return false;
    }

    /* Configure PDO mappings */
    if (!configure_pdo_mappings(slave))
    {
        fprintf(stderr, "Failed to configure PDO mappings\n");
        ec_close();
        return false;
    }

    /* Configure distributed clock */
    ec_configdc();

    /* Map the PDOs */
    ec_config_map(&g_motor_control.IOmap);

    /* Set state to SAFE_OP */
    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_writestate(0);
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

    /* Read drive parameters */
    read_drive_parameters(slave);

    /* Set state to OPERATIONAL */
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_writestate(0);

    /* Wait for all slaves to reach OP state */
    chk = STATE_CHECK_ITERATIONS;
    do
    {
        ec_statecheck(0, EC_STATE_OPERATIONAL, STATE_CHECK_TIMEOUT_US / 2);
    }
    while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

    if (ec_slave[0].state != EC_STATE_OPERATIONAL)
    {
        fprintf(stderr, "Not all slaves reached operational state.\n");
        ec_close();
        return false;
    }

    printf("Operational state reached for all slaves.\n");

    /* Set up process data pointers */
    g_motor_control.rxpdo = (rxpdo_t*)(ec_slave[g_motor_control.slave_index].outputs);
    g_motor_control.txpdo = (txpdo_t*)(ec_slave[g_motor_control.slave_index].inputs);

    return true;
}

/**
 * @brief Enable terminal raw mode for keyboard input
 */
void enable_raw_mode(void)
{
    tcgetattr(STDIN_FILENO, &orig_termios);
    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON); /* Disable echo and canonical mode */
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);

    /* Set stdin to non-blocking */
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

/**
 * @brief Disable terminal raw mode, restoring original settings
 */
void disable_raw_mode(void)
{
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);

    /* Restore blocking mode */
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
}

/**
 * @brief Check if keyboard input is available
 * @return 1 if a key is available, 0 otherwise
 */
int kbhit(void)
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

/**
 * @brief Read a character from keyboard
 * @return The character read
 */
char readch(void)
{
    char ch;
    read(STDIN_FILENO, &ch, 1);
    return ch;
}