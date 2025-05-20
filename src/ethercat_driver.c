#include "common.h"
#include "terminal_io.h"

bool ethercat_init()
{
    int chk;
    int ret;
    int slave;

    // Initialize SOEM, bind socket to ifname
    if (ec_init(g_motor_control.ifname))
    {
        printf("ec_init on %s succeeded.\n", g_motor_control.ifname);

        // Find and auto-configure slaves
        if (ec_config_init(FALSE) > 0)
        {
            printf("%d slaves found and configured.\n", ec_slavecount);

            slave = g_motor_control.slave_index;

            // Set state to PRE_OP for configuration
            ec_slave[0].state = EC_STATE_PRE_OP;
            ec_writestate(0);

            // More robust PRE_OP state checking with retry
            int retry_count = 0;
            const int MAX_RETRIES = 3;
            bool preop_reached = false;

            while (!preop_reached && retry_count < MAX_RETRIES)
            {
                // Try to reach PRE_OP state with adequate timeout
                chk = 40;
                do
                {
                    ec_statecheck(0, EC_STATE_PRE_OP, 100000); // Increased timeout to 100ms
                    if (ec_slave[0].state == EC_STATE_PRE_OP)
                    {
                        preop_reached = true;
                        break;
                    }
                    usleep(50000); // Additional delay between checks
                } while (chk--);

                if (!preop_reached)
                {
                    printf("PRE_OP state retry %d/%d\n", retry_count + 1, MAX_RETRIES);
                    // Force state transition again
                    ec_slave[0].state = EC_STATE_PRE_OP;
                    ec_writestate(0);
                    retry_count++;
                }
            }

            if (!preop_reached)
            {
                printf("Error: Failed to reach PRE_OP state after %d attempts\n", MAX_RETRIES);
                return false;
            }

            printf("PRE_OP state reached successfully\n");

            uint32_t rxpdo_mapping;
            uint8_t rxpdo_count = 0;
            uint32_t txpdo_mapping;
            uint8_t txpdo_count = 0;
            uint16_t pdo_index;
            uint8_t num_pdos = 1;

            uint8_t zero = 0;

            // Disable RxPDO assignments
            ret = ec_SDOwrite(slave, 0x1C12, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to disable RxPDO assignments: %d\n", ret);

            // Disable TxPDO assignments
            ret = ec_SDOwrite(slave, 0x1C13, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to disable TxPDO assignments: %d\n", ret);

            // Disable RxPDO mapping to configure it
            ret = ec_SDOwrite(slave, 0x1600, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to clear RxPDO mapping: %d\n", ret);

            // 0x6040:0 Controlword (16 bits)
            rxpdo_mapping = 0x60400010; // Index:Subindex:BitLength
            ret = ec_SDOwrite(slave, 0x1600, ++rxpdo_count, FALSE, sizeof(rxpdo_mapping), &rxpdo_mapping, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to map Controlword: %d\n", ret);

            // 0x6060:0 Modes of operation (8 bits)
            rxpdo_mapping = 0x60600008;
            ret = ec_SDOwrite(slave, 0x1600, ++rxpdo_count, FALSE, sizeof(rxpdo_mapping), &rxpdo_mapping, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to map Modes of operation: %d\n", ret);

            // 0x6071:0 Target torque (16 bits)
            rxpdo_mapping = 0x60710010;
            ret = ec_SDOwrite(slave, 0x1600, ++rxpdo_count, FALSE, sizeof(rxpdo_mapping), &rxpdo_mapping, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to map Target torque: %d\n", ret);

            // 0x607A:0 Target position (32 bits)
            rxpdo_mapping = 0x607A0020;
            ret = ec_SDOwrite(slave, 0x1600, ++rxpdo_count, FALSE, sizeof(rxpdo_mapping), &rxpdo_mapping, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to map Target position: %d\n", ret);

            // 0x60FF:0 Target velocity (32 bits)
            rxpdo_mapping = 0x60FF0020;
            ret = ec_SDOwrite(slave, 0x1600, ++rxpdo_count, FALSE, sizeof(rxpdo_mapping), &rxpdo_mapping, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to map Target velocity: %d\n", ret);

            // 0x60B2:0 Torque offset (16 bits)
            rxpdo_mapping = 0x60B20010;
            ret = ec_SDOwrite(slave, 0x1600, ++rxpdo_count, FALSE, sizeof(rxpdo_mapping), &rxpdo_mapping, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to map Torque offset: %d\n", ret);

            // 0x2701:0 Tuning command (32 bits) - This appears to be a Synapticon-specific extension
            rxpdo_mapping = 0x27010020;
            ret = ec_SDOwrite(slave, 0x1600, ++rxpdo_count, FALSE, sizeof(rxpdo_mapping), &rxpdo_mapping, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to map Tuning command: %d\n", ret);

            // Set the number of configured RxPDO entries
            ret = ec_SDOwrite(slave, 0x1600, 0, FALSE, sizeof(rxpdo_count), &rxpdo_count, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to set RxPDO count: %d\n", ret);

            // Disable TxPDO mapping to configure it
            ret = ec_SDOwrite(slave, 0x1A00, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to clear TxPDO mapping: %d\n", ret);

            // 0x6041:0 Statusword (16 bits)
            txpdo_mapping = 0x60410010;
            ret = ec_SDOwrite(slave, 0x1A00, ++txpdo_count, FALSE, sizeof(txpdo_mapping), &txpdo_mapping, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to map Statusword: %d\n", ret);

            // 0x6061:0 Modes of operation display (8 bits)
            txpdo_mapping = 0x60610008;
            ret = ec_SDOwrite(slave, 0x1A00, ++txpdo_count, FALSE, sizeof(txpdo_mapping), &txpdo_mapping, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to map Modes of operation display: %d\n", ret);

            // 0x6064:0 Position actual value (32 bits)
            txpdo_mapping = 0x60640020;
            ret = ec_SDOwrite(slave, 0x1A00, ++txpdo_count, FALSE, sizeof(txpdo_mapping), &txpdo_mapping, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to map Position actual value: %d\n", ret);

            // 0x606C:0 Velocity actual value (16 bits) - Note: Documentation shows 32 bits but your struct has 16 bits
            txpdo_mapping = 0x606C0010; // Using 16-bit to match your txpdo_t struct
            ret = ec_SDOwrite(slave, 0x1A00, ++txpdo_count, FALSE, sizeof(txpdo_mapping), &txpdo_mapping, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to map Velocity actual value: %d\n", ret);

            // 0x6077:0 Torque actual value (16 bits)
            txpdo_mapping = 0x60770010;
            ret = ec_SDOwrite(slave, 0x1A00, ++txpdo_count, FALSE, sizeof(txpdo_mapping), &txpdo_mapping, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to map Torque actual value: %d\n", ret);

            // Set the number of configured TxPDO entries
            ret = ec_SDOwrite(slave, 0x1A00, 0, FALSE, sizeof(txpdo_count), &txpdo_count, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to set TxPDO count: %d\n", ret);

            // Assign RxPDO mapping
            pdo_index = 0x1600; // Index of the first RxPDO mapping
            ret = ec_SDOwrite(slave, 0x1C12, 0, FALSE, sizeof(num_pdos), &num_pdos, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to set RxPDO assignment count: %d\n", ret);
            // ret = ec_SDOwrite(slave, 0x1C12, 1, FALSE, sizeof(pdo_index), &pdo_index, EC_TIMEOUTRXM);
            // if (ret <= 0) printf("Failed to assign RxPDO: %d\n", ret);

            // Assign TxPDO mapping
            pdo_index = 0x1A00; // Index of the first TxPDO mapping
            ret = ec_SDOwrite(slave, 0x1C13, 0, FALSE, sizeof(num_pdos), &num_pdos, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to set TxPDO assignment count: %d\n", ret);
            // ret = ec_SDOwrite(slave, 0x1C13, 1, FALSE, sizeof(pdo_index), &pdo_index, EC_TIMEOUTRXM);
            // if (ret <= 0) printf("Failed to assign TxPDO: %d\n", ret);

            uint32_t profile_velocity = 0;
            ret = ec_SDOwrite(slave, 0x6081, 0, FALSE, sizeof(profile_velocity), &profile_velocity, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to set profile velocity: %d\n", ret);

            uint32_t max_motor_speed = MAX_VELOCITY;
            ret = ec_SDOwrite(slave, 0x6080, 0, FALSE, sizeof(max_motor_speed), &max_motor_speed, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to set max motor speed: %d\n", ret);

            uint32_t profile_acceleration = 3000; // Adjust as needed
            ret = ec_SDOwrite(slave, 0x6083, 0, FALSE, sizeof(profile_acceleration), &profile_acceleration, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to set profile acceleration: %d\n", ret);

            uint32_t profile_deceleration = 3000; // Adjust as needed
            ret = ec_SDOwrite(slave, 0x6084, 0, FALSE, sizeof(profile_deceleration), &profile_deceleration, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to set profile deceleration: %d\n", ret);

            uint32_t quick_stop_decel = 3000;
            ret = ec_SDOwrite(slave, 0x6085, 0, FALSE, sizeof(quick_stop_decel), &quick_stop_decel, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to set quick stop deceleration: %d\n", ret);

            // Motion profile type (0 = Linear ramps)
            int16_t motion_profile_type = 0;
            ret = ec_SDOwrite(slave, 0x6086, 0, FALSE, sizeof(motion_profile_type), &motion_profile_type, EC_TIMEOUTRXM);
            if (ret <= 0)
                printf("Failed to set motion profile type: %d\n", ret);

            // Configure distributed clock
            printf("Configuring DC...\n");
            ec_configdc();

            // Map the PDOs
            ec_config_map(&g_motor_control.IOmap);

            // Set state to SAFE_OP
            ec_slave[0].state = EC_STATE_SAFE_OP;
            ec_writestate(0);
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

            // Read some basic drive info
            printf("Reading drive parameters...\n");
            uint32_t u32val = 0;
            int size = sizeof(u32val);
            int ret;

            ret = ec_SDOread(g_motor_control.slave_index, 0x6080, 0, FALSE, &size, &u32val, EC_TIMEOUTRXM);
            if (ret > 0)
                printf("Max motor speed: %d RPM\n", u32val);

            ret = ec_SDOread(g_motor_control.slave_index, 0x6073, 0, FALSE, &size, &u32val, EC_TIMEOUTRXM);
            if (ret > 0)
                printf("Max current: %d mA\n", u32val);

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

                // Set up process data pointers
                g_motor_control.rxpdo = (rxpdo_t *)(ec_slave[g_motor_control.slave_index].outputs);
                g_motor_control.txpdo = (txpdo_t *)(ec_slave[g_motor_control.slave_index].inputs);

                return true;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
            }
        }
        else
        {
            /*NO SLAVES FOUND*/
        }
    }
    else
    {
        printf("No socket connection on %s\nExecute as root\n", g_motor_control.ifname);
    }

    return false;
}