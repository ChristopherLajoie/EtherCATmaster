#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include "coe_master.h"
#include "can_wrapper.h"
#include "can_monitor.h"

int main(int argc, char *argv[])
{
    // We don't need to set the signal handler here since it's done in coe_master_control
    
    printf("Starting in manual control mode (bypassing CAN interface)\n");

    // Skip CAN initialization to simplify troubleshooting
    // Leave it for later when EtherCAT communication is working

    if (argc > 1)
    {
        coe_master_control(argv[1]);
    }
    else
    {
        printf("Usage: %s <network interface>\nExample: %s eth0\n", argv[0], argv[0]);
    }

    printf("End program\n");
    _exit(0);

    return 0;
}