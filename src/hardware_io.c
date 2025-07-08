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

#define STATUS_STATE_MASK 0x6F

#define MAX_STATE_TRANSITION_RETRIES 3
#define STATE_CHECK_ITERATIONS 40
#define STATE_CHECK_TIMEOUT_US 100000
#define STATE_CHECK_DELAY_US 50000

static struct termios orig_termios;

static const cia402_state_pair_t cia402_states[] = {{0x00, "Not Ready To Switch On"},
                                                    {0x40, "Switch On Disabled"},
                                                    {0x21, "Ready To Switch On"},
                                                    {0x23, "Switched On"},
                                                    {0x27, "Operation Enabled"},
                                                    {0x07, "Quick Stop Active"},
                                                    {0x0F, "Fault Reaction Active"},
                                                    {0x08, "Fault"},
                                                    {0xFF, "Unknown State"}};

#define CIA402_NUM_STATES (sizeof(cia402_states) / sizeof(cia402_states[0]))

static float g_torque_constants[MAX_MOTORS] = {0};
static bool g_torque_constants_valid[MAX_MOTORS] = {false};

uint8_t get_cia402_state(uint16_t statusword)
{
    uint16_t state_bits = statusword & 0x6F;

    if ((state_bits & 0x4F) == 0x00)
        return 0x00;  // Not Ready to Switch On
    if ((state_bits & 0x4F) == 0x40)
        return 0x40;  // Switch On Disabled
    if ((state_bits & 0x6F) == 0x21)
        return 0x21;  // Ready to Switch On
    if ((state_bits & 0x6F) == 0x23)
        return 0x23;  // Switched On
    if ((state_bits & 0x6F) == 0x27)
        return 0x27;  // Operation Enabled
    if ((state_bits & 0x6F) == 0x07)
        return 0x07;  // Quick Stop Active
    if ((state_bits & 0x4F) == 0x0F)
        return 0x0F;  // Fault Reaction Active
    if ((state_bits & 0x4F) == 0x08)
        return 0x08;  // Fault

    printf("Unknown CIA-402 state: statusword=0x%04X, state_bits=0x%02X\n", statusword, state_bits);
    return 0xFF;
}

const char* get_cia402_state_string(uint16_t state)
{
    for (size_t i = 0; i < CIA402_NUM_STATES - 1; i++)
    {
        if (cia402_states[i].code == state)
        {
            return cia402_states[i].description;
        }
    }

    return cia402_states[CIA402_NUM_STATES - 1].description;
}

uint16_t get_cia402_state_code(const char* state_str)
{
    if (state_str == NULL)
    {
        return 0xFF;
    }

    for (size_t i = 0; i < CIA402_NUM_STATES; i++)
    {
        if (strcmp(cia402_states[i].description, state_str) == 0)
        {
            return cia402_states[i].code;
        }
    }

    return 0xFF;
}

uint32_t read_drive_parameter(int slave, uint16_t index, uint8_t subindex, const char* description, const char* unit)
{
    uint32_t value = 0;
    int size = sizeof(value);
    int ret;

    ret = ec_SDOread(slave, index, subindex, FALSE, &size, &value, EC_TIMEOUTRXM);

    if (ret > 0)
    {
        printf("%s: %d %s\n", description, value, unit ? unit : "");
    }

    return value;
}

bool read_thermal_data(int slave, thermal_data_t* thermal_data)
{
    int ret;
    int size;
    uint32_t temp_value;
    float raw_temp_data;

    // Read Motor thermal utilisation (I²t) - 0x200A:3
    size = sizeof(uint8_t);
    ret = ec_SDOread(slave, 0x200A, 3, FALSE, &size, &thermal_data->motor_i2t_percent, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        thermal_data->data_valid = false;
        return false;
    }

    // Read Drive-module temperature - 0x2031:1 (in m°C)
    size = sizeof(uint32_t);
    ret = ec_SDOread(slave, 0x2031, 1, FALSE, &size, &temp_value, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        thermal_data->data_valid = false;
        return false;
    }
    thermal_data->drive_temp_celsius = (int32_t)temp_value / 1000.0f;

    // Read Core-board temperature - 0x2030:1 (in m°C)
    size = sizeof(uint32_t);
    ret = ec_SDOread(slave, 0x2030, 1, FALSE, &size, &temp_value, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        thermal_data->data_valid = false;
        return false;
    }
    thermal_data->core_temp_celsius = (int32_t)temp_value / 1000.0f;

    // Read Index temperature - 0x2038:1 (in m°C)
    //size = sizeof(float);
    size = sizeof(uint32_t);
    ret = 1;
    // ret = ec_SDOread(slave, 0x2038, 1, FALSE, &size, &temp_float_value, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        thermal_data->data_valid = false;
        return false;
    }
    // printf("Temperature = %f\n", temp_float_value);

    /*
    // ret = ec_SDOread(slave, 0x2038, 11, FALSE, &size, &temp_float_value, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        thermal_data->data_valid = false;
        return false;
    }
    // printf("upper = %f\n", temp_float_value);
    ret = ec_SDOread(slave, 0x2038, 12, FALSE, &size, &raw_temp_data, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        thermal_data->data_valid = false;
        return false;
    }
    // temp_float_value = 0.0f;
    printf("lower = %f\n", raw_temp_data);
    // thermal_data->index_temp_celsius = (float)temp_float_value;
    */
    thermal_data->current_actual_A = 0.0f;
    thermal_data->data_valid = true;

    return true;
}

static bool configure_rxpdo_mappings(int slave)
{
    int ret;
    uint8_t rxpdo_count = 0;
    uint8_t zero = 0;

    /* SAME ORDER HAS IN MOTOR_TYPES.H */
    static const pdo_entry_t rx_mappings[] = {{0x60400010, "Control word"},
                                              {0x60600008, "Modes of operation"},
                                              {0x60710010, "Target torque"},
                                              {0x607A0020, "Target position"},
                                              {0x60FF0020, "Target velocity"},
                                              {0x60B20010, "Torque offset"},
                                              {0x27010020, "Tuning command (Synapticon)"}};

    // Disable RxPDO mapping to configure it
    ret = ec_SDOwrite(slave, RX_PDO_INDEX, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        return false;
    }

    for (size_t i = 0; i < sizeof(rx_mappings) / sizeof(rx_mappings[0]); i++)
    {
        rxpdo_count++;
        ret = ec_SDOwrite(slave,
                          RX_PDO_INDEX,
                          rxpdo_count,
                          FALSE,
                          sizeof(rx_mappings[i].value),
                          &rx_mappings[i].value,
                          EC_TIMEOUTRXM);
        if (ret <= 0)
        {
            printf("Failed to configure RxPDO mapping %u (0x%08X)\n", rxpdo_count, rx_mappings[i].value);
            return false;
        }
    }

    ret = ec_SDOwrite(slave, RX_PDO_INDEX, 0, FALSE, sizeof(rxpdo_count), &rxpdo_count, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        return false;
    }

    return true;
}

static bool configure_txpdo_mappings(int slave)
{
    int ret;
    uint8_t txpdo_count = 0;
    uint8_t zero = 0;

    /* SAME ORDER HAS IN MOTOR_TYPES.H */
    static const pdo_entry_t tx_mappings[] = {{0x60410010, "Status word"},
                                              {0x60610008, "Modes of operation display"},
                                              {0x60640020, "Position actual value"},
                                              {0x606C0010, "Velocity actual value"},
                                              {0x60770010, "Torque actual value"}};

    // Disable TxPDO mapping to configure it
    ret = ec_SDOwrite(slave, TX_PDO_INDEX, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        return false;
    }

    for (size_t i = 0; i < sizeof(tx_mappings) / sizeof(tx_mappings[0]); i++)
    {
        txpdo_count++;
        ret = ec_SDOwrite(slave,
                          TX_PDO_INDEX,
                          txpdo_count,
                          FALSE,
                          sizeof(tx_mappings[i].value),
                          &tx_mappings[i].value,
                          EC_TIMEOUTRXM);
        if (ret <= 0)
        {
            printf("Failed to configure TxPDO mapping %u (0x%08X)\n", txpdo_count, tx_mappings[i].value);
            return false;
        }
    }

    ret = ec_SDOwrite(slave, TX_PDO_INDEX, 0, FALSE, sizeof(txpdo_count), &txpdo_count, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        return false;
    }

    return true;
}

static bool configure_motion_and_thermal_parameters(int slave)
{
    int ret;

    const motion_param_t params[] = {{0x6080, 0, sizeof(uint32_t), "Max motor speed", MAX_VELOCITY},
                                     {0x6072, 0, sizeof(uint16_t), "Max torque", MAX_TORQUE},
                                     {0x6083, 0, sizeof(uint32_t), "Profile acceleration", DEFAULT_PROFILE_ACCEL},
                                     {0x6084, 0, sizeof(uint32_t), "Profile deceleration", DEFAULT_PROFILE_DECEL},
                                     {0x6085, 0, sizeof(uint32_t), "Quick stop deceleration", DEFAULT_QUICK_STOP_DECEL},
                                     {0x6086, 0, sizeof(int16_t), "Motion profile type", DEFAULT_PROFILE_TYPE},
                                     {0x200A, 2, sizeof(uint32_t), "I2t peak time (ms)", I2T_PEAK_TIME_MS},
                                     {0x2038, 2, sizeof(uint8_t), "Internal analog input", 0},
                                     //{0x2038, 11, sizeof(float), "I2t thermal limit upper", 100.0f}};
                                     {0x2038, 12, sizeof(float), "I2t thermal limit lower", 0.0f}};

    for (size_t i = 0; i < sizeof(params) / sizeof(params[0]); i++)
    {
        ret = ec_SDOwrite(slave, params[i].index, params[i].subindex, FALSE, params[i].size, &params[i].value, EC_TIMEOUTRXM);

        if (ret <= 0)
        {
            fprintf(stderr, "Failed to set %s\n", params[i].description);
            return false;
        }
    }

    return true;
}

bool configure_pdo_mappings(int slave)
{
    int ret;
    uint8_t zero = 0;
    uint8_t num_pdos = 1;

    const pdo_config_t pdo_configs[] = {{0x1C12, "RxPDO assignment"}, {0x1C13, "TxPDO assignment"}};

    // Disable PDO assignments
    for (size_t i = 0; i < sizeof(pdo_configs) / sizeof(pdo_configs[0]); i++)
    {
        ret = ec_SDOwrite(slave, pdo_configs[i].index, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM);
        if (ret <= 0)
        {
            return false;
        }
    }

    if (!configure_rxpdo_mappings(slave))
    {
        return false;
    }

    if (!configure_txpdo_mappings(slave))
    {
        return false;
    }

    for (size_t i = 0; i < sizeof(pdo_configs) / sizeof(pdo_configs[0]); i++)
    {
        ret = ec_SDOwrite(slave, pdo_configs[i].index, 0, FALSE, sizeof(num_pdos), &num_pdos, EC_TIMEOUTRXM);
        if (ret <= 0)
        {
            return false;
        }
    }

    if (!configure_motion_and_thermal_parameters(slave))
    {
        return false;
    }

    if (!configure_i2t_protection(slave, 0))
    {
        return false;
    }
    return true;
}

static bool wait_for_preop_state(void)
{
    int retry_count = 0;
    int chk;
    bool preop_reached = false;

    while (!preop_reached && retry_count < MAX_STATE_TRANSITION_RETRIES)
    {
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
            ec_slave[0].state = EC_STATE_PRE_OP;
            ec_writestate(0);
            retry_count++;
        }
    }

    if (!preop_reached)
    {
        fprintf(stderr, "Failed to reach PRE_OP\n");
        return false;
    }

    return true;
}

bool ethercat_init(void)
{
    int chk;

    /* Initialize SOEM, bind socket to ifname */
    if (!ec_init(g_motor_control.ifname))
    {
        fprintf(stderr, "No socket connection on %s\nExecute as root\n", g_motor_control.ifname);
        return false;
    }

    /* Find and auto-configure slaves */
    if (ec_config_init(FALSE) < g_motor_control.num_motors)
    {
        fprintf(stderr, "One or more slaves missing\n");
        ec_close();
        return false;
    }

    printf("%d slave(s) found and configured.\n", ec_slavecount);

    ec_slave[0].state = EC_STATE_PRE_OP;
    ec_writestate(0);

    if (!wait_for_preop_state())
    {
        ec_close();
        return false;
    }

    for (int i = 0; i < g_motor_control.num_motors; i++)
    {
        int slave_index = g_motor_control.slave_indices[i];
        if (!configure_pdo_mappings(slave_index))
        {
            ec_close();
            return false;
        }
    }

    ec_configdc();
    ec_config_map(&g_motor_control.IOmap);

    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_writestate(0);

    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_writestate(0);

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
    for (int i = 0; i < g_motor_control.num_motors; i++)
    {
        int slave_index = g_motor_control.slave_indices[i];
        g_motor_control.rxpdo[i] = (rxpdo_t*)(ec_slave[slave_index].outputs);
        g_motor_control.txpdo[i] = (txpdo_t*)(ec_slave[slave_index].inputs);
    }
    if (!init_torque_constants())
    {
        printf("Warning: Some torque constants could not be read\n");
    }

    return true;
}

/**
 * @brief Enable terminal raw mode for keyboard input
 */
void enable_raw_mode(void)
{
    tcgetattr(STDIN_FILENO, &orig_termios);
    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON);
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);

    /* Set stdin to non-blocking */
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

void disable_raw_mode(void)
{
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);

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

/**
 * @brief Convert torque value from raw format to mNm
 * @return The torque value in mNm
 */
int convert_to_mNm(int16_t raw_torque)
{
    return (raw_torque * 3300) / 1000;
}

int convert_to_raw(int16_t torque)
{
    return (torque * 1000) / 3300;
}

/**
 * @brief Read torque constant via SDO
 */
bool read_torque_constant(int slave, float* torque_constant_mNm_per_A)
{
    int ret;
    int size;
    uint32_t torque_constant_uNm_per_A;

    // Read Torque constant - 0x2003:2 (in µNm/A)
    size = sizeof(uint32_t);
    ret = ec_SDOread(slave, 0x2003, 2, FALSE, &size, &torque_constant_uNm_per_A, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        return false;
    }

    // Convert from µNm/A to mNm/A (divide by 1000)
    *torque_constant_mNm_per_A = (int32_t)torque_constant_uNm_per_A / 1000.0f;

    return true;
}

/**
 * @brief Calculate current from torque using torque constant
 */
void calculate_current_from_torque(thermal_data_t* thermal_data, int32_t torque_actual_mNm, int motor_index)
{
    if (is_torque_constant_valid(motor_index))
    {
        float torque_constant = get_torque_constant(motor_index);
        if (torque_constant > 0.0f)
        {
            // Current (A) = Torque (mNm) / Torque Constant (mNm/A)
            thermal_data->current_actual_A = fabsf((float)torque_actual_mNm / torque_constant);
        }
        else
        {
            thermal_data->current_actual_A = 0.0f;
        }
    }
    else
    {
        thermal_data->current_actual_A = 0.0f;
    }
}

/**
 * @brief Read current I2t protection mode
 */
int read_i2t_protection_mode(int slave)
{
    int ret;
    int size;
    uint8_t i2t_mode;

    size = sizeof(uint8_t);
    ret = ec_SDOread(slave, 0x200A, 1, FALSE, &size, &i2t_mode, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        return -1;
    }

    const char* mode_descriptions[] = {"Disabled",
                                       "Current Limitation",
                                       "Error Protection",
                                       "Current Limitation (compatibility)",
                                       "Error Protection (compatibility)"};

    if (i2t_mode <= 4)
    {
        printf("Motor %d I2t mode: %d (%s)\n", slave, i2t_mode, mode_descriptions[i2t_mode]);
    }
    else
    {
        printf("Motor %d I2t mode: %d (Unknown)\n", slave, i2t_mode);
    }

    return i2t_mode;
}

/**
 * @brief Configure I2t protection mode
 */
bool configure_i2t_protection(int slave, uint8_t i2t_mode)
{
    int ret;
    ret = ec_SDOwrite(slave, 0x200A, 1, FALSE, sizeof(i2t_mode), &i2t_mode, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        printf("Failed to configure I2t protection mode for slave %d\n", slave);
        return false;
    }
    return true;
}

bool read_fault_codes(int slave, fault_codes_t* fault_codes)
{
    int ret, size;

    if (!fault_codes)
        return false;

    fault_codes->data_valid = true;
    memset(fault_codes->manufacturer_fault, 0, sizeof(fault_codes->manufacturer_fault));

    // Read CIA-402 Error Code (0x603F) - this is still uint16_t
    size = sizeof(uint16_t);
    ret = ec_SDOread(slave, 0x603F, 0, FALSE, &size, &fault_codes->cia402_error_code, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        fault_codes->cia402_error_code = 0;
        fault_codes->data_valid = false;
    }

    // Read Manufacturer Fault Code (0x203F) - CORRECTED: This is STRING(8)
    size = 8;  // 8 bytes for STRING(8)
    ret = ec_SDOread(slave, 0x203F, 1, FALSE, &size, fault_codes->manufacturer_fault, EC_TIMEOUTRXM);
    if (ret <= 0)
    {
        strncpy(fault_codes->manufacturer_fault, "UNKNOWN", 8);
        fault_codes->data_valid = false;
    }

    // Ensure null termination
    fault_codes->manufacturer_fault[8] = '\0';

    return fault_codes->data_valid;
}
bool init_torque_constants(void)
{
    bool all_success = true;

    for (int i = 0; i < g_motor_control.num_motors; i++)
    {
        int slave_index = g_motor_control.slave_indices[i];
        if (read_torque_constant(slave_index, &g_torque_constants[i]))
        {
            g_torque_constants_valid[i] = true;
        }
        else
        {
            g_torque_constants_valid[i] = false;
            g_torque_constants[i] = 0.0f;
            printf("Warning: Failed to read torque constant for motor %d\n", slave_index);
            all_success = false;
        }
    }

    return all_success;
}

float get_torque_constant(int motor_index)
{
    if (motor_index >= 0 && motor_index < MAX_MOTORS && g_torque_constants_valid[motor_index])
    {
        return g_torque_constants[motor_index];
    }
    return 0.0f;
}

bool is_torque_constant_valid(int motor_index)
{
    return (motor_index >= 0 && motor_index < MAX_MOTORS && g_torque_constants_valid[motor_index]);
}