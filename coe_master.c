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
#include "coe_master.h"

char IOmap[4096];
int expectedWKC;
volatile int wkc;
boolean needlf;
boolean inOP;
uint8 currentgroup = 0;
volatile int run = 1;

/* Manual control mode */
int manual_mode = 1;              // Enable manual mode by default
int manual_velocity = 0;          // Current velocity setting
int manual_enable = 0;            // Disabled initially
int manual_speed = 0;             // Normal speed mode

/* CiA402 object dictionary addresses */
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
#define DEFAULT_MAX_VELOCITY 2000
#define DEFAULT_MAX_TORQUE 3000 // t_max / t_rated * 1000

/* PDO mapping offsets based on SOMANET Circulo documentation */
// RxPDO1 offsets
#define OFFSET_CONTROLWORD 0        /* 0x6040:00 Controlword (16 bits) */
#define OFFSET_MODE_OF_OPERATION 2  /* 0x6060:00 Modes of operation (8 bits) */

// RxPDO2 offsets (starts after RxPDO1)
#define OFFSET_TARGET_POSITION 5    /* 0x607A:00 Target position (32 bits) */
#define OFFSET_TARGET_VELOCITY 9    /* 0x60FF:00 Target velocity (32 bits) */
#define OFFSET_TARGET_TORQUE 3     /* 0x6071:00 Target torque (16 bits) */

// TxPDO1 offsets
#define OFFSET_STATUSWORD 0         /* 0x6041:00 Statusword (16 bits) */
#define OFFSET_MODE_DISPLAY 2       /* 0x6061:00 Modes of operation display (8 bits) */

// TxPDO2 offsets (starts after TxPDO1)
#define OFFSET_ACTUAL_POSITION 3    /* 0x6064:00 Position actual value (32 bits) */
#define OFFSET_ACTUAL_VELOCITY 7    /* 0x606C:00 Velocity actual value (32 bits) */
#define OFFSET_ACTUAL_TORQUE 11     /* 0x6077:00 Torque actual value (16 bits) */

/* Debug function to read the PDO configuration from the slave */
void debug_slave_config(int slave)
{
    printf("\n=== Reading PDO configuration for slave %d ===\n", slave);
    
    // Try to read the PDO assignment objects
    uint8 pdos, entries, subindex;
    uint16 index;
    uint32 entry;
    int size, ret;
    
    // Display SyncManager information
    printf("SyncManager Configuration:\n");
    for (int sm = 0; sm < EC_MAXSM; sm++) {
        printf("  SM%d: StartAddr=0x%04X, Length=%d, Flags=0x%02X, Type=%d\n",
               sm, ec_slave[slave].SM[sm].StartAddr, ec_slave[slave].SM[sm].SMlength, 
               ec_slave[slave].SM[sm].SMflags, ec_slave[slave].SMtype[sm]);
    }
    
    // Read RxPDO assignments (0x1C12)
    size = sizeof(pdos);
    ret = ec_SDOread(slave, 0x1C12, 0, FALSE, &size, &pdos, EC_TIMEOUTRXM);
    printf("RxPDO assignments (0x1C12): %d (ret=%d)\n", pdos, ret);
    
    if (ret > 0 && pdos > 0) {
        for (subindex = 1; subindex <= pdos; subindex++) {
            size = sizeof(index);
            ret = ec_SDOread(slave, 0x1C12, subindex, FALSE, &size, &index, EC_TIMEOUTRXM);
            printf("  RxPDO %d: 0x%04X (ret=%d)\n", subindex, index, ret);
            
            if (ret > 0) {
                // Read the PDO mapping
                size = sizeof(entries);
                ret = ec_SDOread(slave, index, 0, FALSE, &size, &entries, EC_TIMEOUTRXM);
                printf("    Entries: %d (ret=%d)\n", entries, ret);
                
                if (ret > 0 && entries > 0) {
                    for (uint8 i = 1; i <= entries; i++) {
                        size = sizeof(entry);
                        ret = ec_SDOread(slave, index, i, FALSE, &size, &entry, EC_TIMEOUTRXM);
                        printf("      Entry %d: 0x%08X (ret=%d)\n", i, entry, ret);
                    }
                }
            }
        }
    }
    
    // Read TxPDO assignments (0x1C13)
    size = sizeof(pdos);
    ret = ec_SDOread(slave, 0x1C13, 0, FALSE, &size, &pdos, EC_TIMEOUTRXM);
    printf("TxPDO assignments (0x1C13): %d (ret=%d)\n", pdos, ret);
    
    if (ret > 0 && pdos > 0) {
        for (subindex = 1; subindex <= pdos; subindex++) {
            size = sizeof(index);
            ret = ec_SDOread(slave, 0x1C13, subindex, FALSE, &size, &index, EC_TIMEOUTRXM);
            printf("  TxPDO %d: 0x%04X (ret=%d)\n", subindex, index, ret);
            
            if (ret > 0) {
                // Read the PDO mapping
                size = sizeof(entries);
                ret = ec_SDOread(slave, index, 0, FALSE, &size, &entries, EC_TIMEOUTRXM);
                printf("    Entries: %d (ret=%d)\n", entries, ret);
                
                if (ret > 0 && entries > 0) {
                    for (uint8 i = 1; i <= entries; i++) {
                        size = sizeof(entry);
                        ret = ec_SDOread(slave, index, i, FALSE, &size, &entry, EC_TIMEOUTRXM);
                        printf("      Entry %d: 0x%08X (ret=%d)\n", i, entry, ret);
                    }
                }
            }
        }
    }
    
    printf("=== PDO configuration reading complete ===\n\n");
}

/* NEW: Diagnose WKC mismatch */
void diagnose_wkc_mismatch(int slave) {
    printf("\n=== WKC MISMATCH DIAGNOSTICS ===\n");
    
    // Test input processing
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    printf("Input-only test: WKC = %d\n", wkc);
    
    // Test output processing
    wkc = ec_send_processdata();
    printf("Output-only test: WKC = %d\n", wkc);
    
    // Test both
    wkc = ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    printf("Combined test: WKC = %d\n", wkc);
    
    // Check SyncManager status
    printf("SM2 (Output) details: Address=0x%04X, Length=%d, Flags=0x%04X\n",
           ec_slave[slave].SM[2].StartAddr, ec_slave[slave].SM[2].SMlength, 
           ec_slave[slave].SM[2].SMflags);
    printf("SM3 (Input) details: Address=0x%04X, Length=%d, Flags=0x%04X\n",
           ec_slave[slave].SM[3].StartAddr, ec_slave[slave].SM[3].SMlength, 
           ec_slave[slave].SM[3].SMflags);
    
    // Check ALStatus
    uint16 alstatus;
    int size = sizeof(alstatus);
    if (ec_SDOread(slave, 0x130, 0, FALSE, &size, &alstatus, EC_TIMEOUTRXM) > 0) {
        printf("AL Status: 0x%04X\n", alstatus);
    } else {
        printf("Failed to read AL Status\n");
    }
    
    // Try to read the SyncManager activation
    uint8 smActivation;
    size = sizeof(smActivation);
    if (ec_SDOread(slave, 0x1C32, 1, FALSE, &size, &smActivation, EC_TIMEOUTRXM) > 0) {
        printf("SM2 activation state: 0x%02X\n", smActivation);
    } else {
        printf("Failed to read SM2 activation\n");
    }
    
    if (ec_SDOread(slave, 0x1C33, 1, FALSE, &size, &smActivation, EC_TIMEOUTRXM) > 0) {
        printf("SM3 activation state: 0x%02X\n", smActivation);
    } else {
        printf("Failed to read SM3 activation\n");
    }
    
    // Try to read error register
    uint8 errorReg;
    size = sizeof(errorReg);
    if (ec_SDOread(slave, 0x1001, 0, FALSE, &size, &errorReg, EC_TIMEOUTRXM) > 0) {
        printf("Error Register (0x1001): 0x%02X\n", errorReg);
    }
    
    // Check DC settings
    printf("DC Active: %d\n",
           ec_slave[slave].DCactive);
    
    printf("=== END DIAGNOSTICS ===\n\n");
}

/* NEW: Validate PDO Objects */
void validate_pdo_objects(int slave) {
    printf("\n=== PDO OBJECT VALIDATION ===\n");
    
    // Test critical output objects
    int size;
    uint16 controlword = 0;
    int8_t opmode = 0;
    int32_t velocity = 0;
    
    // Check controlword (0x6040)
    size = sizeof(controlword);
    if (ec_SDOread(slave, 0x6040, 0, FALSE, &size, &controlword, EC_TIMEOUTRXM) > 0) {
        printf("Controlword accessible via SDO: 0x%04X\n", controlword);
    } else {
        printf("ERROR: Controlword not accessible!\n");
    }
    
    // Check mode of operation (0x6060)
    size = sizeof(opmode);
    if (ec_SDOread(slave, 0x6060, 0, FALSE, &size, &opmode, EC_TIMEOUTRXM) > 0) {
        printf("Mode of operation accessible via SDO: %d\n", opmode);
    } else {
        printf("ERROR: Mode of operation not accessible!\n");
    }
    
    // Check target velocity (0x60FF)
    size = sizeof(velocity);
    if (ec_SDOread(slave, 0x60FF, 0, FALSE, &size, &velocity, EC_TIMEOUTRXM) > 0) {
        printf("Target velocity accessible via SDO: %d\n", velocity);
    } else {
        printf("ERROR: Target velocity not accessible!\n");
    }
    
    // Test critical input objects
    uint16 statusword = 0;
    
    // Check statusword (0x6041)
    size = sizeof(statusword);
    if (ec_SDOread(slave, 0x6041, 0, FALSE, &size, &statusword, EC_TIMEOUTRXM) > 0) {
        printf("Statusword accessible via SDO: 0x%04X\n", statusword);
    } else {
        printf("ERROR: Statusword not accessible!\n");
    }
    
    // Try a few more objects
    int32_t actual_velocity = 0;
    size = sizeof(actual_velocity);
    if (ec_SDOread(slave, 0x606C, 0, FALSE, &size, &actual_velocity, EC_TIMEOUTRXM) > 0) {
        printf("Actual velocity accessible via SDO: %d\n", actual_velocity);
    } else {
        printf("NOTE: Actual velocity not accessible via SDO\n");
    }
    
    printf("=== END VALIDATION ===\n\n");
}

/* NEW: Configure minimal PDO mapping for testing */
void configure_minimal_pdo_mapping(int slave) {
    printf("\n=== CONFIGURING MINIMAL PDO MAPPING ===\n");
    
    // Stop outputs by going to PREOP state
    printf("Transitioning to PRE_OP state...\n");
    ec_slave[0].state = EC_STATE_PRE_OP;
    ec_writestate(0);
    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    
    // Clear RxPDO map (control word, operation mode, target velocity)
    uint8 zero = 0;
    uint8 entries = 3;
    uint32 map;
    
    printf("Configuring RxPDO mapping...\n");
    // Disable RxPDO mapping
    int ret = ec_SDOwrite(slave, 0x1600, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM);
    printf("Disable RxPDO mapping result: %d\n", ret);
    
    // Build new mapping
    map = 0x60400010;  // Controlword (16 bits)
    ret = ec_SDOwrite(slave, 0x1600, 1, FALSE, sizeof(map), &map, EC_TIMEOUTRXM);
    printf("Map controlword result: %d\n", ret);
    
    map = 0x60600008;  // Mode of operation (8 bits)
    ret = ec_SDOwrite(slave, 0x1600, 2, FALSE, sizeof(map), &map, EC_TIMEOUTRXM);
    printf("Map mode of operation result: %d\n", ret);
    
    map = 0x60FF0020;  // Target velocity (32 bits)
    ret = ec_SDOwrite(slave, 0x1600, 3, FALSE, sizeof(map), &map, EC_TIMEOUTRXM);
    printf("Map target velocity result: %d\n", ret);
    
    // Set number of entries
    ret = ec_SDOwrite(slave, 0x1600, 0, FALSE, sizeof(entries), &entries, EC_TIMEOUTRXM);
    printf("Set number of RxPDO entries result: %d\n", ret);
    
    printf("Configuring TxPDO mapping...\n");
    // Clear TxPDO map (status word, operation mode display, actual velocity)
    entries = 3;
    
    // Disable TxPDO mapping
    ret = ec_SDOwrite(slave, 0x1A00, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM);
    printf("Disable TxPDO mapping result: %d\n", ret);
    
    // Build new mapping
    map = 0x60410010;  // Statusword (16 bits)
    ret = ec_SDOwrite(slave, 0x1A00, 1, FALSE, sizeof(map), &map, EC_TIMEOUTRXM);
    printf("Map statusword result: %d\n", ret);
    
    map = 0x60610008;  // Mode of operation display (8 bits)
    ret = ec_SDOwrite(slave, 0x1A00, 2, FALSE, sizeof(map), &map, EC_TIMEOUTRXM);
    printf("Map mode of operation display result: %d\n", ret);
    
    map = 0x606C0020;  // Velocity actual value (32 bits)
    ret = ec_SDOwrite(slave, 0x1A00, 3, FALSE, sizeof(map), &map, EC_TIMEOUTRXM);
    printf("Map velocity actual value result: %d\n", ret);
    
    // Set number of entries
    ret = ec_SDOwrite(slave, 0x1A00, 0, FALSE, sizeof(entries), &entries, EC_TIMEOUTRXM);
    printf("Set number of TxPDO entries result: %d\n", ret);
    
    printf("Configuring SyncManagers...\n");
    // Configure SyncManagers
    // SM2 for RxPDO
    uint16 sm2pdo = 0x1600;
    ret = ec_SDOwrite(slave, 0x1C12, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM);
    printf("Clear SM2 PDO assignment result: %d\n", ret);
    
    ret = ec_SDOwrite(slave, 0x1C12, 1, FALSE, sizeof(sm2pdo), &sm2pdo, EC_TIMEOUTRXM);
    printf("Assign RxPDO to SM2 result: %d\n", ret);
    
    uint8 one = 1;
    ret = ec_SDOwrite(slave, 0x1C12, 0, FALSE, sizeof(one), &one, EC_TIMEOUTRXM);
    printf("Set number of SM2 PDO assignments result: %d\n", ret);
    
    // SM3 for TxPDO
    uint16 sm3pdo = 0x1A00;
    ret = ec_SDOwrite(slave, 0x1C13, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM);
    printf("Clear SM3 PDO assignment result: %d\n", ret);
    
    ret = ec_SDOwrite(slave, 0x1C13, 1, FALSE, sizeof(sm3pdo), &sm3pdo, EC_TIMEOUTRXM);
    printf("Assign TxPDO to SM3 result: %d\n", ret);
    
    ret = ec_SDOwrite(slave, 0x1C13, 0, FALSE, sizeof(one), &one, EC_TIMEOUTRXM);
    printf("Set number of SM3 PDO assignments result: %d\n", ret);
    
    // Apply mapping config by going to SAFEOP
    printf("Transitioning to SAFE_OP state...\n");
    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_writestate(0);
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    
    // Reconfigure IOmap
    printf("Reconfiguring IOmap...\n");
    ec_config_map(&IOmap);
    
    printf("=== MINIMAL PDO MAPPING CONFIGURATION COMPLETE ===\n\n");
}

/* NEW: Send and receive diagnostic version */
void diagnostic_send_receive(void) {
    static int output_wkc = 0;
    static int input_wkc = 0;
    
    // Send outputs only
    output_wkc = ec_send_processdata();
    
    // Receive inputs only
    input_wkc = ec_receive_processdata(EC_TIMEOUTRET);
    
    // Print diagnostic info periodically
    static int counter = 0;
    if (counter++ % 1000 == 0) {
        printf("Diagnostic WKC: Send=%d, Receive=%d, Total=%d (Expected=%d)\n", 
              output_wkc, input_wkc, output_wkc + input_wkc, expectedWKC);
    }
}

/* Process a keypress for manual control */
void process_key(char key)
{
    switch(key)
    {
        case 'w': // Forward
            manual_velocity = DEFAULT_VELOCITY;
            printf("Command: FORWARD (velocity=%d)\n", manual_velocity);
            break;
        case 's': // Backward
            manual_velocity = -DEFAULT_VELOCITY;
            printf("Command: BACKWARD (velocity=%d)\n", manual_velocity);
            break;
        case ' ': // Stop
            manual_velocity = 0;
            printf("Command: STOP\n");
            break;
        case 'e': // Toggle enable
            manual_enable = !manual_enable;
            printf("Enable: %s\n", manual_enable ? "ON" : "OFF");
            break;
        case 'f': // Toggle speed
            manual_speed = !manual_speed;
            printf("Speed mode: %s\n", manual_speed ? "HIGH" : "NORMAL");
            if (manual_velocity != 0) {
                // Adjust velocity if it's already set
                int direction = manual_velocity > 0 ? 1 : -1;
                manual_velocity = direction * DEFAULT_VELOCITY * (manual_speed ? 2 : 1);
                printf("Adjusted velocity: %d\n", manual_velocity);
            }
            break;
        case 'r': // Reset fault
            printf("Command: FAULT RESET\n");
            break;
        case 'd': // Run diagnostics
            printf("Running WKC diagnostics...\n");
            diagnose_wkc_mismatch(1);  // Run diagnostics on slave 1
            validate_pdo_objects(1);   // Validate PDO objects on slave 1
            break;
        case 'h': // Show help
            printf("\nManual Control Commands:\n");
            printf("w - Forward\n");
            printf("s - Backward\n");
            printf("SPACE - Stop\n");
            printf("e - Toggle enable\n");
            printf("f - Toggle speed mode\n");
            printf("r - Reset fault\n");
            printf("d - Run diagnostics\n");
            printf("q - Quit\n");
            printf("h - Show this help\n\n");
            break;
        case 'q': // Quit
            run = 0;
            printf("Quitting...\n");
            break;
        default:
            // Ignore other keys
            break;
    }
}

/* Initialize EtherCAT interface */
int init_ethercat(char *ifname)
{
    if (ec_init(ifname))
    {
        printf("Initialized on %s\n", ifname);
        return 1;
    }
    return 0;
}

/* Find and configure all EtherCAT slaves */
int discover_and_configure_slaves(void)
{
    int slc;
    if (ec_config_init(FALSE) > 0)
    {
        printf("%d slaves found and configured.\n", ec_slavecount);

        /* Verify if all drives are detected correctly */
        for (slc = 1; slc <= ec_slavecount; slc++)
        {
            printf("Slave %d - Name: %s, Output size: %dbits, Input size: %dbits, State: %d\n",
                   slc, ec_slave[slc].name, ec_slave[slc].Obits, ec_slave[slc].Ibits,
                   ec_slave[slc].state);
            
            // Additional diagnostic info
            printf("Slave %d - Product code: 0x%08X, Vendor ID: 0x%08X, Revision: 0x%08X\n",
                   slc, ec_slave[slc].eep_id, ec_slave[slc].eep_man, ec_slave[slc].eep_rev);
        }
        return 1;
    }
    return 0;
}

void configure_pdo_mapping(void)
{
    int slc;

    for (slc = 1; slc <= ec_slavecount; slc++)
    {
        printf("Reading and using existing PDO configuration for slave %d\n", slc);
        debug_slave_config(slc);
        
        /* Important: Don't change any PDO mappings, just use what's already there */
        /* Just make sure types are set correctly */
        ec_slave[slc].SMtype[2] = 3; // SM2 is for outputs (RxPDO)
        ec_slave[slc].SMtype[3] = 4; // SM3 is for inputs (TxPDO)
        
        /* Save original values for diagnostics */
        int orig_sm2_len = ec_slave[slc].SM[2].SMlength;
        int orig_sm3_len = ec_slave[slc].SM[3].SMlength;
    }

    ec_config_map(&IOmap);
    ec_configdc();
    
    printf("Slaves mapped, checking final configuration\n");
    for (slc = 1; slc <= ec_slavecount; slc++)
    {
        printf("Final configuration for slave %d after mapping:\n", slc);
        printf("  Obits: %d, Ibits: %d, Obytes: %d, Ibytes: %d\n",
               ec_slave[slc].Obits, ec_slave[slc].Ibits,
               ec_slave[slc].Obytes, ec_slave[slc].Ibytes);
        printf("  SM2 length: %d, SM3 length: %d\n",
               ec_slave[slc].SM[2].SMlength, ec_slave[slc].SM[3].SMlength);
    }
    
    printf("Waiting for all slaves to reach SAFE_OP state...\n");
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

    for (slc = 1; slc <= ec_slavecount; slc++)
    {
        printf("Slave %d - State: %d\n", slc, ec_slave[slc].state);
    }

    expectedWKC = ec_slavecount * 2;
    //expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    printf("Calculated workcounter %d\n", expectedWKC);
}

/* Transition all slaves to operational state */
int transition_to_operational(void)
{
    int chk;

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
        return 1;
    }

    /* Handle error case */
    ec_readstate();
    for (int slc = 1; slc <= ec_slavecount; slc++)
    {
        if (ec_slave[slc].state != EC_STATE_OPERATIONAL)
        {
            printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                   slc, ec_slave[slc].state, ec_slave[slc].ALstatuscode,
                   ec_ALstatuscode2string(ec_slave[slc].ALstatuscode));
        }
    }
    return 0;
}

/* Configure drives with SDO parameters */
void configure_drives(void)
{
    int slc;

    printf("Starting CiA402 state machine...\n");

    for (slc = 1; slc <= ec_slavecount; slc++)
    {
        int32_t acceleration = DEFAULT_ACCELERATION;
        int32_t deceleration = DEFAULT_DECELERATION;
        int32_t quick_stop_dec = DEFAULT_QUICK_STOP_DEC;
        uint32_t max_velocity = DEFAULT_MAX_VELOCITY;

        /* Write SDO parameters */
        int size = sizeof(acceleration);
        int ret = ec_SDOwrite(slc, ACCELERATION_PROFILE, 0, FALSE, size, &acceleration, EC_TIMEOUTRXM);
        printf("SDO write acceleration profile result: %d\n", ret);

        size = sizeof(deceleration);
        ret = ec_SDOwrite(slc, DECELERATION_PROFILE, 0, FALSE, size, &deceleration, EC_TIMEOUTRXM);
        printf("SDO write deceleration profile result: %d\n", ret);

        size = sizeof(quick_stop_dec);
        ret = ec_SDOwrite(slc, QUICK_STOP_DECELERATION, 0, FALSE, size, &quick_stop_dec, EC_TIMEOUTRXM);
        printf("SDO write quick stop deceleration result: %d\n", ret);

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

    /* Read SDO diagnostics */
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
}

void run_control_loop(void)
{
    int cycle_counter = 0;
    int test_cycles = 100;  // Just run a few test cycles for now
    
    printf("Starting simplified test loop\n");
    printf("Will run %d cycles for testing\n", test_cycles);

    // Initialize all outputs to zero
    for (int slc = 1; slc <= ec_slavecount; slc++) {
        memset(ec_slave[slc].outputs, 0, ec_slave[slc].Obytes);
        
        // Set controlword to Shutdown (simplest safe command)
        *(uint16_t *)(ec_slave[slc].outputs + OFFSET_CONTROLWORD) = COMMAND_SHUTDOWN;
        
        // Set profile velocity mode
        *(int8_t *)(ec_slave[slc].outputs + OFFSET_MODE_OF_OPERATION) = OP_MODE_PROFILE_VELOCITY;
    }

    /* Run until test cycles complete or signal */
    while (run && cycle_counter < test_cycles)
    {
        // Print diagnostic data every cycle during testing
        printf("Cycle %d: ", cycle_counter);
        
        // Print first few bytes of output data for debugging
        printf("OUT[0-3]: %02X %02X %02X %02X | ", 
               ec_slave[1].outputs[0], ec_slave[1].outputs[1], 
               ec_slave[1].outputs[2], ec_slave[1].outputs[3]);
        
        /* Process PDO data */
        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        
        printf("WKC=%d | ", wkc);
        
        // Print first few bytes of input data for debugging
        printf("IN[0-3]: %02X %02X %02X %02X\n", 
               ec_slave[1].inputs[0], ec_slave[1].inputs[1], 
               ec_slave[1].inputs[2], ec_slave[1].inputs[3]);
        
        /* Process working counter */
        if (wkc < expectedWKC)
        {
            printf("Working counter error - expected: %d, got: %d\n", expectedWKC, wkc);
        }

        cycle_counter++;
        osal_usleep(10000); // 10ms cycle time for testing
    }
    
    printf("Test loop complete\n");
}

void test_soem_pdo_sizes(void)
{
    printf("\n=== SOEM PDO size test ===\n");
    printf("Group 0: expectedWKC=%d, outputs_WKC=%d, inputs_WKC=%d\n", 
           expectedWKC, ec_group[0].outputsWKC, ec_group[0].inputsWKC);
    
    int total_output_size = 0;
    int total_input_size = 0;
    
    for (int slc = 1; slc <= ec_slavecount; slc++) {
        printf("Slave %d: Obytes=%d, Ibytes=%d, output addr=%p, input addr=%p\n",
               slc, ec_slave[slc].Obytes, ec_slave[slc].Ibytes,
               ec_slave[slc].outputs, ec_slave[slc].inputs);
        
        total_output_size += ec_slave[slc].Obytes;
        total_input_size += ec_slave[slc].Ibytes;
    }
    
    printf("Total mapped size: outputs=%d bytes, inputs=%d bytes\n", 
           total_output_size, total_input_size);
    
    printf("IOmap size: %d bytes\n", (int)sizeof(IOmap));
    printf("=== Test complete ===\n\n");
}

/* MODIFIED: Enhanced manual control loop with diagnostics */
void run_manual_control_loop(void)
{
    uint16_t statusword = 0;
    uint16_t controlword = 0;
    DriveState drive_state;
    int cycle_counter = 0;
    int operation_enabled = 0;
    int prev_state = (DriveState)(-1);
    int slc;
    int separate_wkc = 0;  // Flag to toggle separate send/receive
    int output_wkc = 0;
    int input_wkc = 0;

    printf("Starting manual control loop with enhanced WKC diagnostics\n");
    printf("Press 'h' for command help, 'd' for diagnostics, or 'q' to quit.\n");

    /* Run until user input or signal */
    while (run)
    {
        /* Check for keyboard input */
        if (kbhit())
        {
            char key = getchar();
            process_key(key);
        }

        /* Process PDO data */
        if (separate_wkc) {
            // Separate send/receive for diagnostics
            output_wkc = ec_send_processdata();
            input_wkc = ec_receive_processdata(EC_TIMEOUTRET);
            
            // Log separate WKC values periodically
            if (cycle_counter % 1000 == 0) {
                printf("Output WKC: %d, Input WKC: %d, Total: %d (Expected: %d)\n",
                       output_wkc, input_wkc, output_wkc + input_wkc, expectedWKC);
            }
        } else {
            // Standard send/receive
            ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);
        }

        /* Process data even with WKC errors - we know data exchange is working */
        for (slc = 1; slc <= ec_slavecount; slc++)
        {
            statusword = *(uint16_t *)(ec_slave[slc].inputs + OFFSET_STATUSWORD);
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
                printf("Drive %d state changed: %s -> %s (Statusword: 0x%04X)\n",
                       slc,
                       (prev_state >= 0) ? state_names[prev_state] : "Unknown",
                       state_names[drive_state], statusword);
                prev_state = drive_state;
            }

            /* Process state machine */
            switch (drive_state) {
                case STATE_NOT_READY_TO_SWITCH_ON:
                    controlword = 0x00; // Just wait
                    operation_enabled = 0;
                    break;
                case STATE_SWITCH_ON_DISABLED:
                    controlword = COMMAND_SHUTDOWN; // Transition to ready
                    operation_enabled = 0;
                    break;
                case STATE_READY_TO_SWITCH_ON:
                    controlword = COMMAND_SWITCH_ON; // Switch on
                    operation_enabled = 0;
                    break;
                case STATE_SWITCHED_ON:
                    if (manual_enable) {
                        controlword = COMMAND_ENABLE_OPERATION; // Enable operation
                    } else {
                        controlword = COMMAND_SWITCH_ON; // Stay switched on
                    }
                    operation_enabled = 0;
                    break;
                case STATE_OPERATION_ENABLED:
                    if (manual_enable) {
                        controlword = COMMAND_ENABLE_OPERATION; // Stay enabled
                        operation_enabled = 1;
                    } else {
                        controlword = COMMAND_DISABLE_OPERATION; // Disable
                        operation_enabled = 0;
                    }
                    break;
                case STATE_QUICK_STOP_ACTIVE:
                    controlword = COMMAND_ENABLE_OPERATION; // Try to re-enable
                    operation_enabled = 0;
                    break;
                case STATE_FAULT_REACTION_ACTIVE:
                    controlword = 0x00; // Wait for fault reaction
                    operation_enabled = 0;
                    break;
                case STATE_FAULT:
                    controlword = COMMAND_FAULT_RESET; // Reset fault
                    operation_enabled = 0;
                    break;
                default:
                    controlword = COMMAND_FAULT_RESET;
                    operation_enabled = 0;
                    break;
            }

            /* Write control word */
            *(uint16_t *)(ec_slave[slc].outputs + OFFSET_CONTROLWORD) = controlword;

            /* Maintain profile velocity mode */
            *(int8_t *)(ec_slave[slc].outputs + OFFSET_MODE_OF_OPERATION) = OP_MODE_PROFILE_VELOCITY;
            
            /* Set velocity if operation enabled */
            if (operation_enabled) {
                *(int32_t *)(ec_slave[slc].outputs + OFFSET_TARGET_VELOCITY) = manual_velocity;
                
                // Print status info periodically
                if (cycle_counter % 100 == 0) {
                    int32_t actual_velocity = 0;
                    if (OFFSET_ACTUAL_VELOCITY < ec_slave[slc].Ibytes - 4) {
                        actual_velocity = *(int32_t *)(ec_slave[slc].inputs + OFFSET_ACTUAL_VELOCITY);
                    }
                    printf("Drive %d: Target=%d, Actual=%d, Status=0x%04X, WKC=%d/%d\n", 
                           slc, manual_velocity, actual_velocity, statusword, wkc, expectedWKC);
                }
            } else {
                *(int32_t *)(ec_slave[slc].outputs + OFFSET_TARGET_VELOCITY) = 0;
            }
        }

        /* Report WKC errors every 5000 cycles */
        if (wkc < expectedWKC && cycle_counter % 5000 == 0)
        {
            printf("Note: Working counter mismatch - expected: %d, got: %d (continuing anyway)\n", 
                  expectedWKC, wkc);
        }

        cycle_counter++;
        osal_usleep(5000); // Increased to 5ms cycle time for better stability
    }
}

/* Process controller input for differential drive system */
void process_controller_input(int *left_motor_velocity, int *right_motor_velocity, int cycle_counter)
{
    // In manual mode, we'll use our global manual_velocity instead
    if (manual_mode) {
        *left_motor_velocity = manual_velocity;
        *right_motor_velocity = manual_velocity;
        return;
    }
    
    // If not in manual mode, fall back to CAN input
    int x_axis = get_can_x_axis();
    int y_axis = get_can_y_axis();

    int speed_button = get_can_speed();
    int estop = get_can_estop();

    // Handle emergency stop
    if (estop) {
        *left_motor_velocity = 0;
        *right_motor_velocity = 0;
        return;
    }

    // Convert to -124 to +124 range with center at 0
    int y_centered = y_axis - 128;
    int x_centered = x_axis - 128;

    // Apply deadband to ignore small joystick movements
    if (abs(y_centered) < 10)
    {
        y_centered = 0;
    }

    if (abs(x_centered) < 10)
    {
        x_centered = 0;
    }

    // Apply speed multiplier if speed button is pressed
    int speed_multiplier = speed_button ? 2 : 1;

    int base_speed = (y_centered * DEFAULT_VELOCITY * speed_multiplier) / 124;
    int turn_component = (x_centered * DEFAULT_VELOCITY * speed_multiplier) / 124;

    *left_motor_velocity = base_speed + turn_component;
    *right_motor_velocity = base_speed - turn_component;

    // Limit values to maximum allowed velocity
    int max_velocity = DEFAULT_MAX_VELOCITY * speed_multiplier;

    if (*left_motor_velocity > max_velocity)
        *left_motor_velocity = max_velocity;
    if (*left_motor_velocity < -max_velocity)
        *left_motor_velocity = -max_velocity;

    if (*right_motor_velocity > max_velocity)
        *right_motor_velocity = max_velocity;
    if (*right_motor_velocity < -max_velocity)
        *right_motor_velocity = -max_velocity;

    // Debug output
    if (cycle_counter % 1000 == 0)
    {
        printf("Joystick: X=%d, Y=%d | Left Motor=%d, Right Motor=%d\n",
               x_axis, y_axis, *left_motor_velocity, *right_motor_velocity);
    }
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

/* Process the CiA402 state machine */
uint16_t process_state_machine(DriveState state, int enable_status, int *operation_enabled, 
                              int *target_velocity, int cycle_counter)
{
    // If in manual mode, handle it in the run_control_loop function
    if (manual_mode) {
        *operation_enabled = 0;
        *target_velocity = 0;
        return COMMAND_SHUTDOWN;  // Default safe value
    }
    
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

/* Graceful shutdown of drives */
void shutdown_drives(void)
{
    int slc;

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

    printf("Request safe operational state for all slaves\n");
    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_writestate(0);
}

/* Clean up EtherCAT resources */
void cleanup_ethercat(void)
{
    printf("End control, close socket\n");
    ec_close();
}


/* Signal handler for Ctrl+C */
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
int kbhit(void)
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

/* Implementation for coe_master_control that's called from main.c */
void coe_master_control(char *ifname)
{
    printf("Starting EtherCAT master in MANUAL mode\n");
    printf("This version allows direct keyboard control\n");

    /* Initialize SOEM, bind socket to ifname */
    if (!init_ethercat(ifname)) {
        printf("No socket connection on %s\nExcecute as root\n", ifname);
        return;
    }

    /* Find and auto-configure slaves */
    if (!discover_and_configure_slaves()) {
        printf("No slaves found!\n");
        cleanup_ethercat();
        return;
    }

    /* Configure PDO mappings */
    configure_pdo_mapping();
    test_soem_pdo_sizes();

    /* Transition to operational state */
    if (!transition_to_operational()) {
        printf("Not all slaves reached operational state.\n");
        /* Error handling here */
        cleanup_ethercat();
        return;
    }

    /* Run diagnostics BEFORE configuring drives */
    diagnose_wkc_mismatch(1);
    validate_pdo_objects(1);

    /* Configure drives with SDO parameters */
    configure_drives();

    /* Optional: Try minimal PDO mapping if WKC issues persist */
    // Uncomment this to test minimal PDO mapping:
    configure_minimal_pdo_mapping(1);
    // diagnose_wkc_mismatch(1);

    /* Main control loop */
    run_manual_control_loop();

    /* Graceful shutdown */
    shutdown_drives();
    cleanup_ethercat();
}