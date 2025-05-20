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
    .slave_index = 1};

// Signal handler
void signal_handler(int sig)
{
    printf("\nSignal %d received, stopping program...\n", sig);
    g_motor_control.run = 0;
}

int main(void)
{   
    bool ethercat_initialized = false;
    int retry_count = 0;
    const int MAX_RETRIES = 20;
    const int RETRY_DELAY_SEC = 3;
    
    printf("SOEM (Simple Open EtherCAT Master)\n");
    printf("Synapticon Actilink-S Control\n");

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
    if (!init_can_monitor())
    {
        printf("Failed to initialize CAN monitor\n");
        disable_raw_mode();
        return 1;
    }

    printf("Attempting to initialize EtherCAT on %s...\n", g_motor_control.ifname);

    while (!ethercat_initialized && g_motor_control.run && retry_count < MAX_RETRIES)
    {
        ethercat_initialized = ethercat_init();

        if (!ethercat_initialized)
        {
            retry_count++;

            if (retry_count >= MAX_RETRIES)
            {
                printf("Could not find any EtherCAT slaves.\n");
                break;
            }

            printf("No EtherCAT slaves found. Will retry in %d seconds (Press Ctrl+C to abort)\n",
                   RETRY_DELAY_SEC);

            for (int i = 0; i < RETRY_DELAY_SEC * 10 && g_motor_control.run; i++)
            {
                usleep(100000); 
            }
        }
    }

    if (!ethercat_initialized)
    {
        printf("Failed to setup EtherCAT. Exiting.\n");
        stop_can_monitor();
        disable_raw_mode();
        return 1;
    }

    g_motor_control.rxpdo->op_mode = OP_MODE_PVM;

    if (pthread_create(&g_motor_control.cyclic_thread, NULL, motor_control_cyclic_task, NULL) != 0)
    {
        printf("Failed to create cyclic task thread\n");
        ec_close();
        stop_can_monitor();
        disable_raw_mode();
        return 1;
    }

    // Main loop - just monitor status
    while (g_motor_control.run)
    {
        usleep(100000);
        //print_can_status();
    }

    pthread_join(g_motor_control.cyclic_thread, NULL);

    // Cleanup
    ec_close();
    stop_can_monitor();
    disable_raw_mode();

    printf("Program terminated successfully\n");
    return 0;
}