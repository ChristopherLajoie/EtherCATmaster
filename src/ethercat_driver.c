#include "common.h"
#include "terminal_io.h"

bool ethercat_init() 
{
    int i, chk;
    
    // Initialize SOEM, bind socket to ifname
    if (ec_init(g_motor_control.ifname))
    {
        printf("ec_init on %s succeeded.\n", g_motor_control.ifname);

        // Find and auto-configure slaves
        if (ec_config_init(FALSE) > 0)
        {
            printf("%d slaves found and configured.\n", ec_slavecount);

            // Set state to PRE_OP for configuration
            ec_slave[0].state = EC_STATE_PRE_OP;
            ec_writestate(0);

            // Configure distributed clock
            printf("Configuring DC...\n");
            ec_configdc();

            // Map the PDOs
            ec_config_map(&g_motor_control.IOmap);

            // Wait for spacebar press before leaving PRE-OP
            printf("\n\n*********************************************************\n");
            printf("* System is in PRE-OP state                              *\n");
            printf("* Press SPACEBAR to continue to SAFE-OP and OPERATIONAL  *\n");
            printf("*********************************************************\n\n");

            // Wait for spacebar press
            bool spacebar_pressed = false;
            while (!spacebar_pressed && g_motor_control.run)
            {
                if (kbhit())
                {
                    char c = readch();
                    if (c == ' ')
                    {
                        spacebar_pressed = true;
                        printf("Continuing to SAFE-OP state...\n");
                    }
                }
                usleep(50000); // 50ms sleep to avoid CPU hogging
            }

            // Exit if program was terminated while waiting
            if (!g_motor_control.run)
            {
                return false;
            }

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
            if (ret > 0) printf("Max motor speed: %d RPM\n", u32val);
            
            ret = ec_SDOread(g_motor_control.slave_index, 0x6073, 0, FALSE, &size, &u32val, EC_TIMEOUTRXM);
            if (ret > 0) printf("Max current: %d mA\n", u32val);

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
            printf("No slaves found!\n");
        }
    }
    else
    {
        printf("No socket connection on %s\nExecute as root\n", g_motor_control.ifname);
    }

    return false;
}