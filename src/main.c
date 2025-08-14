/**
 * @file main.c
 * @brief Main program for EtherCAT motor control system - NXP Yocto RT version
 *
 * Initializes EtherCAT communication with Synapticon
 * Actilink-S servo drives and controls them using CAN interface.
 */

#include "common.h"
#include "hardware_io.h"
#include "can_interface.h"
#include "motor_driver.h"
#include "config.h"

#define SLEEP_INTERVAL_US 1000000 // 1 second

MotorControl g_motor_control = {.ifname = "eth1",
                                .cycletime = 4000,
                                .run = 1,
                                .num_motors = 2,
                                .slave_indices = {1, 2},
                                .reconnect_in_progress = false,
                                .reconnection_attempts = 0};

static void signal_handler(int sig)
{
    static volatile sig_atomic_t signal_count = 0;
    signal_count++;

    printf("\nSignal %d received, stopping program... (%d)\n", sig, signal_count);

    if (signal_count == 1)
    {
        g_motor_control.run = 0;
    }
    else
    {
        printf("Forcing exit...\n");
        exit(EXIT_FAILURE);
    }
}

static int setup_signal_handlers(void)
{
    struct sigaction sa;

    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;

    if (sigaction(SIGINT, &sa, NULL) != 0)
    {
        perror("Failed to set SIGINT handler");
        return -1;
    }

    if (sigaction(SIGTERM, &sa, NULL) != 0)
    {
        perror("Failed to set SIGTERM handler");
        return -1;
    }

    return 0;
}

static bool initialize_ethercat(void)
{
    return ethercat_init();
}

static void cleanup_resources(void)
{
    ec_close();
    stop_can_interface();
}

int main(int argc, char *argv[])
{
    const char *config_filename;

    fprintf(stderr,"Build v1.1\n");

    // Check for command-line argument for config file path
    if (argc > 1)
    {
        config_filename = argv[1];
    }
    else
    {
        // If no argument, print usage and exit
        fprintf(stderr, "Error: Configuration file not specified.\n");
        fprintf(stderr, "Usage: %s <path_to_config_file>\n", argv[0]);
        return EXIT_FAILURE;
    }

    // Load configuration from the specified file
    if (!load_config(config_filename))
    {
        fprintf(stderr, "Failed to load configuration from '%s'\n", config_filename);
        return EXIT_FAILURE;
    }

    /* Update motor control with config values */
    g_motor_control.ifname = g_config.interface;
    g_motor_control.cycletime = g_config.cycletime;
    g_motor_control.num_motors = g_config.num_motors;

    g_motor_control.slave_indices[LEFT_MOTOR] = 1;
    g_motor_control.slave_indices[RIGHT_MOTOR] = 2;

    if (setup_signal_handlers() != 0)
    {
        return EXIT_FAILURE;
    }

    if (!init_can_interface())
    {
        return EXIT_FAILURE;
    }

    if (!initialize_ethercat())
    {
        cleanup_resources();
        return EXIT_FAILURE;
    }

    /* Configure operation mode to PVM */
    for (int i = 0; i < g_motor_control.num_motors; i++)
    {
        g_motor_control.rxpdo[i]->op_mode = 3;
    }

    if (pthread_create(&g_motor_control.cyclic_thread, NULL, motor_control_cyclic_task, NULL) != 0)
    {
        fprintf(stderr, "Failed to create cyclic task thread\n");
        cleanup_resources();
        return EXIT_FAILURE;
    }

    setvbuf(stdout, NULL, _IONBF, 0);

    while (g_motor_control.run)
    {
        usleep(SLEEP_INTERVAL_US);
    }

    pthread_join(g_motor_control.cyclic_thread, NULL);

    cleanup_resources();

    printf("Program terminated successfully\n");
    return EXIT_SUCCESS;
}