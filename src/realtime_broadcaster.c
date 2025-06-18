/**
 * @file realtime_broadcaster.c
 * @brief Real-time data broadcasting implementation with thermal monitoring
 */

#include "realtime_broadcaster.h"
#include "hardware_io.h"
#include "config.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <stdio.h>

RealtimeBroadcaster g_broadcaster = {0};

bool init_realtime_broadcaster(void)
{
    // Check if broadcasting is enabled in config
    if (!g_config.enable_realtime_hmi) {
        g_broadcaster.enabled = false;
        printf("Real-time HMI broadcasting disabled in configuration\n");
        return true;
    }

    // Create UDP socket
    g_broadcaster.socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_broadcaster.socket_fd < 0) {
        perror("Failed to create UDP socket");
        return false;
    }

    // Enable broadcast
    int broadcast_enable = 1;
    if (setsockopt(g_broadcaster.socket_fd, SOL_SOCKET, SO_BROADCAST, 
                   &broadcast_enable, sizeof(broadcast_enable)) < 0) {
        perror("Failed to enable broadcast");
        close(g_broadcaster.socket_fd);
        return false;
    }

    // Set up broadcast parameters
    strncpy(g_broadcaster.broadcast_ip, g_config.hmi_broadcast_ip, sizeof(g_broadcaster.broadcast_ip) - 1);
    g_broadcaster.broadcast_ip[sizeof(g_broadcaster.broadcast_ip) - 1] = '\0'; 
    g_broadcaster.broadcast_port = g_config.hmi_broadcast_port;
    
    // Debug the data types and values
    printf("Debug: broadcast_interval_ms = %d (type size: %zu)\n", 
           g_config.hmi_broadcast_interval_ms, sizeof(g_config.hmi_broadcast_interval_ms));
    printf("Debug: cycletime = %d (type size: %zu)\n", 
           g_config.cycletime, sizeof(g_config.cycletime));
    
    // Calculate broadcast interval with explicit casting and safety checks
    int interval_ms = g_config.hmi_broadcast_interval_ms;
    int cycle_us = g_config.cycletime;
    
    if (cycle_us <= 0) {
        printf("Error: Invalid cycletime %d\n", cycle_us);
        cycle_us = 4000; // Default to 4ms
    }
    
    if (interval_ms <= 0) {
        printf("Error: Invalid broadcast interval %d\n", interval_ms);
        interval_ms = 100; // Default to 100ms
    }
    
    // Calculate: interval_ms * 1000 / cycle_us
    long interval_us = (long)interval_ms * 1000L;
    g_broadcaster.broadcast_interval_cycles = (int)(interval_us / cycle_us);
    
    // Safety check - minimum 1 cycle, maximum reasonable value
    if (g_broadcaster.broadcast_interval_cycles < 1) {
        printf("Warning: Calculated cycles was %d, setting to 25 (100ms default)\n", 
               g_broadcaster.broadcast_interval_cycles);
        g_broadcaster.broadcast_interval_cycles = 25; // Default 100ms at 4ms cycle
    }
    
    printf("Debug: calculated broadcast_interval_cycles = %d\n", g_broadcaster.broadcast_interval_cycles);
    printf("Debug: actual broadcast rate = %.2f Hz\n", 
           1000.0 / (g_broadcaster.broadcast_interval_cycles * cycle_us / 1000.0));
    
    g_broadcaster.cycle_counter = 0;
    g_broadcaster.enabled = true;

    printf("Real-time HMI broadcaster initialized\n");
    printf("  Broadcast to: %s:%d\n", g_broadcaster.broadcast_ip, g_broadcaster.broadcast_port);
    printf("  Update rate: %.1f Hz\n", 1000.0 / interval_ms);
    
    return true;
}

void cleanup_realtime_broadcaster(void)
{
    if (g_broadcaster.enabled && g_broadcaster.socket_fd >= 0) {
        close(g_broadcaster.socket_fd);
        g_broadcaster.socket_fd = -1;
        printf("Real-time broadcaster stopped\n");
    }
    g_broadcaster.enabled = false;
}

void broadcast_motor_data(txpdo_t* txpdo[], int num_motors)
{
    if (!g_broadcaster.enabled) {
        return;
    }

    // Check if it's time to broadcast
    g_broadcaster.cycle_counter++;
    
    if (g_broadcaster.cycle_counter < g_broadcaster.broadcast_interval_cycles) {
        return;
    }
    g_broadcaster.cycle_counter = 0;

    // Read thermal data every 10th broadcast (once per second at 10Hz)
    static int thermal_counter = 0;
    static thermal_data_t thermal_data[MAX_MOTORS] = {0};
    
    if (++thermal_counter >= 10) {
        thermal_counter = 0;
        for (int i = 0; i < num_motors; i++) {
            int slave_index = g_motor_control.slave_indices[i];
            if (!read_thermal_data(slave_index, &thermal_data[i])) {
                printf("Warning: Failed to read thermal data for motor %d\n", i);
                thermal_data[i].data_valid = false;
            }
        }
    }

    // Get current timestamp
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    
    // Format timestamp as ISO 8601
    struct tm* tm_info = localtime(&ts.tv_sec);
    char timestamp[32];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", tm_info);
    
    // Add milliseconds
    char full_timestamp[40];
    snprintf(full_timestamp, sizeof(full_timestamp), "%s.%03ldZ", 
             timestamp, ts.tv_nsec / 1000000);

    // Build JSON message
    char json_buffer[1024];
    
    if (num_motors == 1) {
        // Single motor
        int32_t velocity = txpdo[0]->velocity_actual;
        int32_t torque = convert_to_mNm(txpdo[0]->torque_actual);
        
        // Apply reverse correction for logical direction
        if (g_config.reverse_left_motor) {
            velocity = -velocity;
            torque = -torque;
        }
        
        snprintf(json_buffer, sizeof(json_buffer),
            "{"
            "\"timestamp\":\"%s\","
            "\"motor\":{\"velocity\":%d,\"torque\":%d},"
            "\"thermal\":{\"i2t\":%d,\"drive_temp\":%.1f,\"core_temp\":%.1f,\"valid\":%s}"
            "}",
            full_timestamp, velocity, torque,
            thermal_data[0].motor_i2t_percent,
            thermal_data[0].drive_temp_celsius,
            thermal_data[0].core_temp_celsius,
            thermal_data[0].data_valid ? "true" : "false");
            
    } else if (num_motors >= 2) {
        // Dual motor
        int32_t left_velocity = txpdo[LEFT_MOTOR]->velocity_actual;
        int32_t right_velocity = txpdo[RIGHT_MOTOR]->velocity_actual;
        int32_t left_torque = convert_to_mNm(txpdo[LEFT_MOTOR]->torque_actual);
        int32_t right_torque = convert_to_mNm(txpdo[RIGHT_MOTOR]->torque_actual);
        
        // Apply reverse correction for logical direction
        if (g_config.reverse_left_motor) {
            left_velocity = -left_velocity;
            left_torque = -left_torque;
        }
        if (g_config.reverse_right_motor) {
            right_velocity = -right_velocity;
            right_torque = -right_torque;
        }
        
        snprintf(json_buffer, sizeof(json_buffer),
            "{"
            "\"timestamp\":\"%s\","
            "\"left_motor\":{\"velocity\":%d,\"torque\":%d},"
            "\"right_motor\":{\"velocity\":%d,\"torque\":%d},"
            "\"thermal\":{"
                "\"left\":{\"i2t\":%d,\"drive_temp\":%.1f,\"core_temp\":%.1f},"
                "\"right\":{\"i2t\":%d,\"drive_temp\":%.1f,\"core_temp\":%.1f},"
                "\"valid\":%s"
            "}"
            "}",
            full_timestamp,
            left_velocity, left_torque, right_velocity, right_torque,
            thermal_data[LEFT_MOTOR].motor_i2t_percent, 
            thermal_data[LEFT_MOTOR].drive_temp_celsius, 
            thermal_data[LEFT_MOTOR].core_temp_celsius,
            thermal_data[RIGHT_MOTOR].motor_i2t_percent, 
            thermal_data[RIGHT_MOTOR].drive_temp_celsius, 
            thermal_data[RIGHT_MOTOR].core_temp_celsius,
            (thermal_data[LEFT_MOTOR].data_valid && thermal_data[RIGHT_MOTOR].data_valid) ? "true" : "false");
    }

    // Send UDP broadcast
    struct sockaddr_in broadcast_addr;
    memset(&broadcast_addr, 0, sizeof(broadcast_addr));
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(g_broadcaster.broadcast_port);
    inet_pton(AF_INET, g_broadcaster.broadcast_ip, &broadcast_addr.sin_addr);

    ssize_t sent = sendto(g_broadcaster.socket_fd, json_buffer, strlen(json_buffer), 0,
                         (struct sockaddr*)&broadcast_addr, sizeof(broadcast_addr));
    
    if (sent < 0) {
        static int error_count = 0;
        if (++error_count % 100 == 1) { // Log every 100th error
            perror("UDP broadcast failed");
        }
    }
}