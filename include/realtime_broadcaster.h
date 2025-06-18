/**
 * @file realtime_broadcaster.h
 * @brief Real-time data broadcasting for web HMI
 */

#ifndef REALTIME_BROADCASTER_H
#define REALTIME_BROADCASTER_H

#include <stdint.h>
#include <stdbool.h>
#include "motor_types.h"

typedef struct {
    bool enabled;
    int socket_fd;
    int broadcast_interval_cycles;
    int cycle_counter;
    char broadcast_ip[16];
    int broadcast_port;
} RealtimeBroadcaster;

extern RealtimeBroadcaster g_broadcaster;

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