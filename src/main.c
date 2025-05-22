/**
 * @file main.c
 * @brief Main program for EtherCAT motor control system
 *
 * This program initializes an EtherCAT communication with a Synapticon
 * Actilink-S servo drive and controls it using a CAN interface.
 */

#include "common.h"
#include "hardware_io.h"
#include "can_interface.h"
#include "motor_driver.h"

/* Configuration constants */
#define MAX_ETHERCAT_RETRIES 20
#define ETHERCAT_RETRY_DELAY_SEC 3
#define SLEEP_INTERVAL_US 100000

/* Global motor control structure */
MotorControl g_motor_control = {.ifname = "eth0", .cycletime = 4000, .run = 1, .slave_index = 1};

/**
 * @brief Signal handler for graceful termination
 * @param sig Signal number
 */
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

/**
 * @brief Initialize signal handlers
 */
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

/**
 * @brief Initialize EtherCAT with retry logic
 */
static bool initialize_ethercat(void)
{
    int retry_count = 0;

    printf("Attempting to initialize EtherCAT on %s...\n", g_motor_control.ifname);

    while (g_motor_control.run && retry_count < MAX_ETHERCAT_RETRIES)
    {
        if (ethercat_init())
        {
            printf("EtherCAT initialized successfully.\n");
            return true;
        }

        retry_count++;

        if (retry_count >= MAX_ETHERCAT_RETRIES)
        {
            printf("Failed to find EtherCAT slaves.\n");
            break;
        }

        for (int i = 0; i < ETHERCAT_RETRY_DELAY_SEC * 10 && g_motor_control.run; i++)
        {
            usleep(SLEEP_INTERVAL_US / 10);
        }
    }

    return false;
}

/**
 * @brief Clean up resources before exit
 */
static void cleanup_resources(void)
{
    ec_close();
    stop_can_interface();
    disable_raw_mode();
}

/**
 * @brief Main program entry point
 */
int main(void)
{
    printf("SOEM (Simple Open EtherCAT Master)\n");
    printf("Synapticon Actilink-S Control\n");

    if (setup_signal_handlers() != 0)
    {
        return EXIT_FAILURE;
    }

    enable_raw_mode();

    if (!init_can_interface())
    {
        disable_raw_mode();
        return EXIT_FAILURE;
    }

    if (!initialize_ethercat())
    {
        cleanup_resources();
        return EXIT_FAILURE;
    }

    /* Configure operation mode */
    g_motor_control.rxpdo->op_mode = OP_MODE_PVM;

    if (pthread_create(&g_motor_control.cyclic_thread, NULL, motor_control_cyclic_task, NULL) != 0)
    {
        fprintf(stderr, "Failed to create cyclic task thread\n");
        cleanup_resources();
        return EXIT_FAILURE;
    }

    /* Main monitoring loop */
    while (g_motor_control.run)
    {
        usleep(SLEEP_INTERVAL_US);
        /* print_can_status(); */
    }

    pthread_join(g_motor_control.cyclic_thread, NULL);

    cleanup_resources();

    printf("Program terminated successfully\n");
    return EXIT_SUCCESS;
}