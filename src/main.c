#include "common.h"
#include "terminal_io.h"
#include "can_monitor.h"
#include "socketcan.h"
#include "motor_control.h"

// Global instance
MotorControl g_motor_control = {
    .ifname = "eth0",
    .cycletime = 4000, // 4ms cycle time (250Hz)
    .run = 1,
    .slave_index = 1
};

// Signal handler
void signal_handler(int sig)
{
    printf("\nSignal %d received, stopping program...\n", sig);
    g_motor_control.run = 0;
}

int main(void)
{
    printf("SOEM (Simple Open EtherCAT Master)\n");
    printf("Synapticon Motor Control\n");
    
    // Register signal handler
    struct sigaction sa;
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    
    // Enable terminal raw mode for keyboard input
    enable_raw_mode();
    
    // Initialize CAN monitor
    if (!init_can_monitor()) {
        printf("Failed to initialize CAN monitor\n");
        disable_raw_mode();
        return 1;
    }
    
    printf("CAN monitor initialized\n");
    
    bool ethercat_initialized = false;
    int retry_count = 0;
    const int MAX_RETRIES = 20; 
    const int RETRY_DELAY_SEC = 3;
    
    printf("Attempting to initialize EtherCAT on %s...\n", g_motor_control.ifname);
    
    while (!ethercat_initialized && g_motor_control.run && retry_count < MAX_RETRIES) {
        if (retry_count > 0) {
            printf("Retry attempt %d/%d...\n", retry_count + 1, MAX_RETRIES);
        }
        
        ethercat_initialized = ethercat_init();
        
        if (!ethercat_initialized) {
            retry_count++;
            
            if (retry_count >= MAX_RETRIES) {
                printf("Maximum retries (%d) exceeded. Could not find any EtherCAT slaves.\n", MAX_RETRIES);
                break;
            }
            
            printf("No EtherCAT slaves found. Will retry in %d seconds (Press Ctrl+C to abort)\n", 
                   RETRY_DELAY_SEC);
        }
    }
    
    if (!ethercat_initialized) {
        printf("Failed to setup EtherCAT. Exiting.\n");
        stop_can_monitor();
        disable_raw_mode();
        return 1;
    }
    
    printf("EtherCAT initialized successfully\n");
    
    // Default to PVM mode
    g_motor_control.rxpdo->op_mode = OP_MODE_PVM;
    
    // Create cyclic task thread
    if (pthread_create(&g_motor_control.cyclic_thread, NULL, motor_control_cyclic_task, NULL) != 0) {
        printf("Failed to create cyclic task thread\n");
        ec_close();
        stop_can_monitor();
        disable_raw_mode();
        return 1;
    }
    
    printf("Motor control running. Press Ctrl+C to exit.\n");
    
    // Main loop - just monitor status
    while (g_motor_control.run) {
        sleep(1);
        
        // Print CAN status periodically
        static int status_counter = 0;
        if (status_counter++ % 5 == 0) {
            print_can_status();
        }
    }
    
    // Wait for cyclic task to finish
    pthread_join(g_motor_control.cyclic_thread, NULL);
    
    // Cleanup
    ec_close();
    stop_can_monitor();
    disable_raw_mode();
    
    printf("Program terminated successfully\n");
    return 0;
}