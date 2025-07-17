// Add these declarations to realtime_broadcaster.h

/**
 * @file realtime_broadcaster.h
 * @brief Real-time data broadcasting for web HMI
 */

#ifndef REALTIME_BROADCASTER_H
#define REALTIME_BROADCASTER_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include "motor_types.h"
#include "hardware_io.h"  // For thermal_data_t

typedef struct
{
    bool enabled;
    int socket_fd;
    int broadcast_interval_cycles;
    int cycle_counter;
    char broadcast_ip[16];  // Primary broadcast IP (for backward compatibility)
    int broadcast_port;
    // Support for multiple broadcast addresses
    char broadcast_addresses[4][16];  // Support up to 4 broadcast addresses
    int num_broadcast_addresses;
} RealtimeBroadcaster;

extern RealtimeBroadcaster g_broadcaster;

// GLOBAL thermal data - accessible to all modules
extern thermal_data_t g_thermal_data[MAX_MOTORS];
extern pthread_mutex_t g_thermal_mutex;

/**
 * @brief Initialize the real-time data broadcaster
 * @return true on success, false on failure
 */
bool init_realtime_broadcaster(void);

/**
 * @brief Cleanup the broadcaster
 */
void cleanup_realtime_broadcaster(void);

/**
 * @brief Broadcast motor data (call from control loop)
 * @param rxpdo Array of RxPDO pointers
 * @param txpdo Array of TxPDO pointers
 * @param num_motors Number of motors
 */
void broadcast_motor_data(txpdo_t* txpdo[], int num_motors);

#endif