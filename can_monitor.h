#ifndef CAN_MONITOR_H
#define CAN_MONITOR_H

#include <pthread.h>
#include <time.h>
#include <stdint.h>

/* Structure to hold CAN variables */
typedef struct {
    int enable;             /* Enable button status (0/1) */
    int x_axis;             /* Joystick X-axis value (0-255) */
    int y_axis;             /* Joystick Y-axis value (0-255) */
    int horn;               /* Horn button status (0/1) */
    int estop;              /* Emergency stop status (0/1) */
    int speed;              /* Speed button status (0/1) */
    /* Add other CAN variables as needed */
    
    pthread_mutex_t mutex;        /* Mutex for thread-safe access */
    int error_count;              /* Count of read errors */
    int update_count;             /* Count of successful updates */
    struct timespec last_update;  /* Timestamp of last successful update */
    int monitoring_active;        /* Flag to indicate if monitoring is active */
} CANVariables;

/* Global variable, accessible from EtherCAT master */
extern CANVariables can_vars;

/**
 * Initialize the CAN monitor thread
 * 
 * @return 1 on success, 0 on failure
 */
int init_can_monitor(void);

/**
 * Stop the CAN monitor thread
 */
void stop_can_monitor(void);

/**
 * Get the enable button status safely
 * 
 * @return The enable button status (0 or 1), or default value if data is stale
 */
int get_can_enable(void);

/**
 * Get the X-axis value safely
 * 
 * @return The X-axis value (0-255), or default value if data is stale
 */
int get_can_x_axis(void);

/**
 * Get the Y-axis value safely
 * 
 * @return The Y-axis value (0-255), or default value if data is stale
 */
int get_can_y_axis(void);

/**
 * Get the horn button status safely
 * 
 * @return The horn button status (0 or 1), or default value if data is stale
 */
int get_can_horn(void);

/**
 * Get the emergency stop status safely
 * 
 * @return The emergency stop status (0 or 1), or default value if data is stale
 */
int get_can_estop(void);

/**
 * Get the speed button status safely
 * 
 * @return The speed button status (0 or 1), or default value if data is stale
 */
int get_can_speed(void);

/**
 * Print current CAN status for debugging
 */
void print_can_status(void);

#endif /* CAN_MONITOR_H */