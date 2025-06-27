/**
 * @file data_logger.h
 * @brief Data logging module for motor control system
 *
 * Provides CSV logging functionality for motor velocity and torque data
 * with configurable logging intervals and file management.
 */

#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include "common.h"
#include "motor_types.h"
#include <stdio.h>

typedef struct
{
    bool logging_enabled;
    char log_file_path[256];
    FILE* log_file;

    bool fault_logging_enabled;
    FILE* fault_log_file;
    char fault_log_path[512];
} DataLogger;

extern DataLogger g_data_logger;

/**
 * @brief Initialize the data logging system
 * @return true on success, false on failure
 */
bool init_data_logger(void);

/**
 * @brief Cleanup and close the data logging system
 */
void cleanup_data_logger(void);

/**
 * @brief Log motor data to CSV file
 * @param rxpdo Array of RxPDO pointers
 * @param txpdo Array of TxPDO pointers
 * @param num_motors Number of motors to log
 */
void log_motor_data(txpdo_t* txpdo[], int num_motors);

/**
 * @brief Create CSV header based on number of motors
 * @param num_motors Number of motors
 */
void write_csv_header(void);

/**
 * @brief Initialize fault logging (separate from regular data logging)
 * @return true on success, false on failure
 */
bool init_fault_logger(void);

/**
 * @brief Log a motor fault event
 * @param motor_index Motor index (0=left, 1=right)
 * @param fault_code CIA-402 fault state code
 * @param fault_description Human readable fault description
 */
void log_motor_fault(int motor_index, uint16_t fault_code, const char* fault_description);

/**
 * @brief Cleanup fault logging
 */
void cleanup_fault_logger(void);

#endif