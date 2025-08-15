/**
 * @file main.c
 * @brief Main program for Master - NXP Yocto RT version
 */

#include "common.h"
#include "hardware_io.h"
#include "can_interface.h"
#include "motor_driver.h"
#include "config.h"

#define MAIN_SLEEP_INTERVAL_US 1000000

// UPDATE THESE PARAMETERS IN CONFIG FILE NOT HERE
MotorControl g_motor_control = {.ifname = "eth0",
                                .cycletime = 10000,
                                .run = 1,
                                .num_motors = 2,
                                .slave_indices = {1, 2},
                                .reconnect_in_progress = false,
                                .reconnection_attempts = 0};

static void signal_handler(/*int sig*/)
{
    static volatile sig_atomic_t signal_count = 0;
    signal_count++;

    printf("\nStopping program..\n");

    if (signal_count == 1)
    {
        g_motor_control.run = 0;
    }
    else
    {
        printf("Forcing exit..\n");
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

static void cleanup_resources(void)
{
    ec_close();
    stop_can_interface();
}

int main(int argc, char *argv[])
{
    const char *config_filename;

    fprintf(stderr, "Build IMX91 V1.0\n");

    if (argc > 1)
    {
        config_filename = argv[1];
    }
    else
    {
        fprintf(stderr, "Error: Configuration file not specified.\n");
        return EXIT_FAILURE;
    }

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

    if (!ethercat_init())
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

    /* Set stdout to unbuffered mode for immediate output */
    setvbuf(stdout, NULL, _IONBF, 0);

    while (g_motor_control.run)
    {
        usleep(MAIN_SLEEP_INTERVAL_US);
    }

    pthread_join(g_motor_control.cyclic_thread, NULL);

    cleanup_resources();

    return EXIT_SUCCESS;
}