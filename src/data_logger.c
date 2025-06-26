/**
 * @file data_logger.c - Updated version with thermal data logging
 * @brief Implementation of data logging functionality with thermal monitoring
 */

#include "data_logger.h"
#include "hardware_io.h"
#include <time.h>
#include <sys/stat.h>
#include <errno.h>

DataLogger g_data_logger = {0};

/**
 * @brief Generate a unique filename that doesn't exist
 * @param base_path Base path for the log file
 * @param filename Output buffer for the generated filename
 * @param filename_size Size of the filename buffer
 * @return true on success, false on failure
 */
static bool generate_unique_filename(const char* base_path, char* filename, size_t filename_size)
{
    time_t now;
    struct tm* tm_info;
    char timestamp[64];
    char temp_filename[512];
    int counter = 0;
    
    time(&now);
    tm_info = localtime(&now);
    
    // Create base timestamp (YYYY-MM-DD_HH-MM-SS)
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d_%H-%M-%S", tm_info);
    
    // Try the basic filename first
    snprintf(temp_filename, sizeof(temp_filename), "%s/motor_data_%s.csv", 
             base_path, timestamp);
    
    // If file doesn't exist, use this name
    if (access(temp_filename, F_OK) != 0) {
        size_t len = strlen(temp_filename);
        if (len >= filename_size) {
            fprintf(stderr, "Error: Filename too long\n");
            return false;
        }
        strcpy(filename, temp_filename);
        return true;
    }
    
    // If file exists, add a counter
    do {
        counter++;
        snprintf(temp_filename, sizeof(temp_filename), "%s/motor_data_%s_%03d.csv", 
                 base_path, timestamp, counter);
        
        if (counter > 999) {
            fprintf(stderr, "Error: Too many log files with same timestamp\n");
            return false;
        }
    } while (access(temp_filename, F_OK) == 0);
    
    size_t len = strlen(temp_filename);
    if (len >= filename_size) {
        fprintf(stderr, "Error: Filename too long\n");
        return false;
    }
    strcpy(filename, temp_filename);
    return true;
}

bool init_data_logger(void)
{
    if (!g_config.enable_logging) {
        g_data_logger.logging_enabled = false;
        printf("Data logging disabled in configuration\n");
        return true;
    }

    // Create log directory if it doesn't exist    
    struct stat st = {0};
    if (stat(g_config.log_file_path, &st) == -1) {
        if (mkdir(g_config.log_file_path, 0755) != 0 && errno != EEXIST) {
            fprintf(stderr, "Failed to create log directory: %s - %s\n", g_config.log_file_path, strerror(errno));
            return false;
        }
    }

    // Generate unique filename
    char filename[512];
    if (!generate_unique_filename(g_config.log_file_path, filename, sizeof(filename))) {
        fprintf(stderr, "Failed to generate unique filename\n");
        return false;
    }

    g_data_logger.log_file = fopen(filename, "w");
    if (!g_data_logger.log_file) {
        fprintf(stderr, "Failed to open log file: %s - %s\n", filename, strerror(errno));
        return false;
    }

    g_data_logger.logging_enabled = true;

    printf("Data logging initialized: %s\n", filename);
    
    // Write CSV header
    write_csv_header();
    
    return true;
}

void cleanup_data_logger(void)
{
    if (g_data_logger.log_file) {
        fclose(g_data_logger.log_file);
        g_data_logger.log_file = NULL;
        printf("Data logging file closed\n");
    }
    g_data_logger.logging_enabled = false;
}

void write_csv_header(void)
{
    if (!g_data_logger.log_file) return;

    // Enhanced CSV header with thermal data and current
    fprintf(g_data_logger.log_file, "Timestamp,Motor,Actual_Velocity_RPM,Torque_mNm,Current_A,I2t_Percent,Drive_Temp_C,Core_Temp_C,Torque_Constant_mNm_per_A,Thermal_Valid\n");
    fflush(g_data_logger.log_file);
}

void log_motor_data(txpdo_t* txpdo[], int num_motors)
{
    if (!g_data_logger.logging_enabled || !g_data_logger.log_file) {
        return;
    }

    // Get high precision timestamp with milliseconds
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    
    // Format timestamp as: YYYY-MM-DD HH:MM:SS.milliseconds
    struct tm* tm_info = localtime(&ts.tv_sec);
    char timestamp[64];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", tm_info);
    
    // Add milliseconds
    char full_timestamp[80];
    snprintf(full_timestamp, sizeof(full_timestamp), "%s.%03ld", 
             timestamp, ts.tv_nsec / 1000000);

    // Read thermal data for all motors once per log cycle
    static thermal_data_t thermal_data[MAX_MOTORS] = {0};
    for (int motor = 0; motor < num_motors; motor++) {
        int slave_index = g_motor_control.slave_indices[motor];
        if (!read_thermal_data(slave_index, &thermal_data[motor])) {
            // If thermal read fails, mark as invalid but continue logging
            thermal_data[motor].data_valid = false;
            thermal_data[motor].motor_i2t_percent = 0;
            thermal_data[motor].drive_temp_celsius = 0.0f;
            thermal_data[motor].core_temp_celsius = 0.0f;
            thermal_data[motor].current_actual_A = 0.0f;
            thermal_data[motor].torque_constant_mNm_per_A = 0.0f;
        }
    }

    // Log data for each motor
    for (int motor = 0; motor < num_motors; motor++) {
        const char* motor_name;
        
        if (num_motors == 1) {
            motor_name = "Motor";
        } else {
            motor_name = (motor == LEFT_MOTOR) ? "Left_Motor" : "Right_Motor";
        }

        int32_t actual_velocity = txpdo[motor]->velocity_actual;
        int32_t torque_mNm = convert_to_mNm(txpdo[motor]->torque_actual);
        
        // Apply reverse correction for logging - show logical robot direction
        // not physical motor direction
        if (motor == LEFT_MOTOR && g_config.reverse_left_motor) {
            actual_velocity = -actual_velocity;
            torque_mNm = -torque_mNm;
        }
        if (motor == RIGHT_MOTOR && g_config.reverse_right_motor) {
            actual_velocity = -actual_velocity;
            torque_mNm = -torque_mNm;
        }

        // Calculate current from torque
        calculate_current_from_torque(&thermal_data[motor], torque_mNm);

        // Write CSV row with thermal data and current
        fprintf(g_data_logger.log_file, "%s,%s,%d,%d,%.3f,%d,%.1f,%.1f,%.3f,%s\n",
                full_timestamp,
                motor_name,
                actual_velocity,
                torque_mNm,
                thermal_data[motor].current_actual_A,
                thermal_data[motor].motor_i2t_percent,
                thermal_data[motor].drive_temp_celsius,
                thermal_data[motor].core_temp_celsius,
                thermal_data[motor].torque_constant_mNm_per_A,
                thermal_data[motor].data_valid ? "true" : "false");
    }
    
    fflush(g_data_logger.log_file);  // Ensure data is written immediately
}