#include "common.h"
#include "terminal_io.h"
#include "can_monitor.h"
#include "can_wrapper.h"
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

int main(int argc, char *argv[])
{
    printf("SOEM (Simple Open EtherCAT Master)\n");
    printf("Synapticon Motor Control - Simplified Version\n");

    // Handle command line parameters
    if (argc > 1)
    {
        g_motor_control.ifname = argv[1];
    }

    // Register signal handler
    struct sigaction sa;
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;

    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    // Enable terminal raw mode for keyboard input
    enable_raw_mode();

    // Initialize Python CAN interface
    if (!initialize_python_can())
    {
        printf("Failed to initialize Python CAN interface\n");
        disable_raw_mode();
        return 1;
    }

    // Initialize CAN monitor
    if (!init_can_monitor())
    {
        printf("Failed to initialize CAN monitor\n");
        disable_raw_mode();
        return 1;
    }

    printf("CAN monitor initialized\n");

    // Initialize EtherCAT and get process data pointers
    if (!ethercat_init())
    {
        printf("Failed to setup EtherCAT. Exiting.\n");
        stop_can_monitor();
        disable_raw_mode();
        return 1;
    }

    // Create cyclic task thread
    if (pthread_create(&g_motor_control.cyclic_thread, NULL, motor_control_cyclic_task, NULL) != 0)
    {
        printf("Failed to create cyclic task thread\n");
        ec_close();
        stop_can_monitor();
        disable_raw_mode();
        return 1;
    }

    printf("Motor control running. Press Ctrl+C to exit.\n");

    // Main loop - just monitor status
    while (g_motor_control.run)
    {
        sleep(1);
        
        // Print CAN status periodically
        static int status_counter = 0;
        if (status_counter++ % 5 == 0)
        {
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