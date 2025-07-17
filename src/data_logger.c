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
    snprintf(temp_filename, sizeof(temp_filename), "%s/motor_data_%s.csv", base_path, timestamp);

    // If file doesn't exist, use this name
    if (access(temp_filename, F_OK) != 0)
    {
        size_t len = strlen(temp_filename);
        if (len >= filename_size)
        {
            fprintf(stderr, "Error: Filename too long\n");
            return false;
        }
        strcpy(filename, temp_filename);
        return true;
    }

    // If file exists, add a counter
    do
    {
        counter++;
        snprintf(temp_filename, sizeof(temp_filename), "%s/motor_data_%s_%03d.csv", base_path, timestamp, counter);

        if (counter > 999)
        {
            fprintf(stderr, "Error: Too many log files with same timestamp\n");
            return false;
        }
    }
    while (access(temp_filename, F_OK) == 0);

    size_t len = strlen(temp_filename);
    if (len >= filename_size)
    {
        fprintf(stderr, "Error: Filename too long\n");
        return false;
    }
    strcpy(filename, temp_filename);
    return true;
}

bool init_data_logger(void)
{
    if (!g_config.enable_logging)
    {
        g_data_logger.logging_enabled = false;
        printf("Data logging disabled in configuration\n");
        return true;
    }

    // Create log directory if it doesn't exist
    struct stat st = {0};
    if (stat(g_config.log_file_path, &st) == -1)
    {
        if (mkdir(g_config.log_file_path, 0755) != 0 && errno != EEXIST)
        {
            fprintf(stderr, "Failed to create log directory: %s - %s\n", g_config.log_file_path, strerror(errno));
            return false;
        }
    }

    // Generate unique filename
    char filename[512];
    if (!generate_unique_filename(g_config.log_file_path, filename, sizeof(filename)))
    {
        fprintf(stderr, "Failed to generate unique filename\n");
        return false;
    }

    g_data_logger.log_file = fopen(filename, "w");
    if (!g_data_logger.log_file)
    {
        fprintf(stderr, "Failed to open log file: %s - %s\n", filename, strerror(errno));
        return false;
    }

    g_data_logger.logging_enabled = true;

    write_csv_header();

    return true;
}

void cleanup_data_logger(void)
{
    if (g_data_logger.log_file)
    {
        fclose(g_data_logger.log_file);
        g_data_logger.log_file = NULL;
    }
    g_data_logger.logging_enabled = false;

    cleanup_fault_logger();
}

void write_csv_header(void)
{
    if (!g_data_logger.log_file)
        return;

    fprintf(g_data_logger.log_file,
            "Timestamp,Motor,Actual_Velocity_RPM,Torque_mNm,Current_A,I2t_Percent,Drive_Temp_C,Core_Temp_C,Index_Temp_C,Thermal_Valid\n");
    fflush(g_data_logger.log_file);
}

void log_motor_data(txpdo_t* txpdo[], int num_motors)
{
    if (!g_data_logger.logging_enabled || !g_data_logger.log_file)
    {
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
    snprintf(full_timestamp, sizeof(full_timestamp), "%s.%03ld", timestamp, ts.tv_nsec / 1000000);

    extern thermal_data_t g_thermal_data[MAX_MOTORS];
    extern pthread_mutex_t g_thermal_mutex;
    
    static thermal_data_t thermal_data[MAX_MOTORS] = {0};
    
    // Copy thermal data safely
    pthread_mutex_lock(&g_thermal_mutex);
    for (int motor = 0; motor < num_motors && motor < MAX_MOTORS; motor++)
    {
        thermal_data[motor] = g_thermal_data[motor];
    }
    pthread_mutex_unlock(&g_thermal_mutex);

    for (int motor = 0; motor < num_motors; motor++)
    {
        const char* motor_name;

        if (num_motors == 1)
        {
            motor_name = "Motor";
        }
        else
        {
            motor_name = (motor == LEFT_MOTOR) ? "Left_Motor" : "Right_Motor";
        }

        int32_t actual_velocity = txpdo[motor]->velocity_actual;
        int32_t torque_mNm = convert_to_mNm(txpdo[motor]->torque_actual);

        // Apply reverse correction for logging
        if (motor == LEFT_MOTOR && g_config.reverse_left_motor)
        {
            actual_velocity = -actual_velocity;
            torque_mNm = -torque_mNm;
        }
        if (motor == RIGHT_MOTOR && g_config.reverse_right_motor)
        {
            actual_velocity = -actual_velocity;
            torque_mNm = -torque_mNm;
        }

        calculate_current_from_torque(&thermal_data[motor], torque_mNm, motor);

        fprintf(g_data_logger.log_file,
                "%s,%s,%d,%d,%.3f,%d,%.1f,%.1f,%.1f,%s\n",
                full_timestamp,
                motor_name,
                actual_velocity,
                torque_mNm,
                thermal_data[motor].current_actual_A,
                thermal_data[motor].motor_i2t_percent,
                thermal_data[motor].drive_temp_celsius,
                thermal_data[motor].core_temp_celsius,
                thermal_data[motor].index_temp_celsius,
                thermal_data[motor].data_valid ? "true" : "false");
    }

    fflush(g_data_logger.log_file);  // Ensure data is written immediately
}

bool init_fault_logger(void)
{
    if (!g_config.enable_logging)
    {
        g_data_logger.fault_logging_enabled = false;
        return true;  
    }

    // Create fault log filename
    time_t now;
    struct tm* tm_info;
    char timestamp[64];

    time(&now);
    tm_info = localtime(&now);
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d_%H-%M-%S", tm_info);

    snprintf(g_data_logger.fault_log_path,
             sizeof(g_data_logger.fault_log_path),
             "%s/motor_faults_%s.csv",
             g_config.log_file_path,
             timestamp);

    struct stat st = {0};
    if (stat(g_config.log_file_path, &st) == -1)
    {
        if (mkdir(g_config.log_file_path, 0755) != 0 && errno != EEXIST)
        {
            fprintf(stderr, "Failed to create fault log directory: %s - %s\n", g_config.log_file_path, strerror(errno));
            return false;
        }
    }

    g_data_logger.fault_log_file = fopen(g_data_logger.fault_log_path, "w");
    if (!g_data_logger.fault_log_file)
    {
        fprintf(stderr, "Failed to open fault log file: %s - %s\n", g_data_logger.fault_log_path, strerror(errno));
        return false;
    }

    g_data_logger.fault_logging_enabled = true;

    fprintf(g_data_logger.fault_log_file, "Timestamp,Motor,Fault_Code_Hex,Fault_Description\n");
    fflush(g_data_logger.fault_log_file);

    printf("Fault logging initialized: %s\n", g_data_logger.fault_log_path);
    return true;
}

void log_motor_fault(int motor_index, uint16_t fault_code, const char* fault_description)
{
    if (!g_data_logger.fault_logging_enabled || !g_data_logger.fault_log_file)
    {
        return;
    }

    // Get high precision timestamp
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    struct tm* tm_info = localtime(&ts.tv_sec);
    char timestamp[64];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", tm_info);

    // Add milliseconds
    char full_timestamp[80];
    snprintf(full_timestamp, sizeof(full_timestamp), "%s.%03ld", timestamp, ts.tv_nsec / 1000000);

    // Determine motor name
    const char* motor_name;
    if (g_motor_control.num_motors == 1)
    {
        motor_name = "Motor";
    }
    else
    {
        motor_name = (motor_index == LEFT_MOTOR) ? "Left_Motor" : "Right_Motor";
    }

    // Write fault to CSV
    fprintf(g_data_logger.fault_log_file, "%s,%s,0x%04X,%s\n", full_timestamp, motor_name, fault_code, fault_description);

    fflush(g_data_logger.fault_log_file);  // Ensure immediate write

    printf("FAULT LOGGED: %s - Motor %s - Code 0x%04X - %s\n", full_timestamp, motor_name, fault_code, fault_description);
}

void cleanup_fault_logger(void)
{
    if (g_data_logger.fault_log_file)
    {
        fclose(g_data_logger.fault_log_file);
        g_data_logger.fault_log_file = NULL;
    }
    g_data_logger.fault_logging_enabled = false;
}