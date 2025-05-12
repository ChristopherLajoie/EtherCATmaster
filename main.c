#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include "coe_master.h"
#include "can_wrapper.h"
#include "can_monitor.h"

int main(int argc, char *argv[])
{
    signal(SIGINT, signal_handler);

    if (!initialize_python_can())
    {
        printf("Failed to initialize Python CAN interface, continuing without it\n");
    }
    else
    {
        acquire_gil();
        set_yellow_bat_led(1);
        release_gil();

        if (!init_can_monitor())
        {
            printf("Warning: CAN monitoring disabled\n");
        }
    }

    if (argc > 1)
    {
        coe_master_control(argv[1]);
    }
    else
    {
        printf("Usage: %s <network interface>\nExample: %s eth0\n", argv[0], argv[0]);
    }

    printf("End program\n");

    acquire_gil();
    set_yellow_bat_led(0);
    release_gil();

    stop_can_monitor();
    _exit(0);

    return 0;
}