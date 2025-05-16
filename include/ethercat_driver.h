#ifndef ETHERCAT_DRIVER_H
#define ETHERCAT_DRIVER_H

#include "common.h"

// Initialize EtherCAT communication
bool ethercat_init();

// Read drive parameters
void read_drive_parameters();

// Clean up EtherCAT resources
void ethercat_cleanup();

#endif // ETHERCAT_DRIVER_H