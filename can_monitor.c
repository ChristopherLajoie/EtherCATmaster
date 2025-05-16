#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <pthread.h>
#include "can_monitor.h"
#include "socketcan.h"  // Changed from "can_wrapper.h"

CANVariables can_vars;

static volatile int keep_running = 1;

static pthread_t monitor_thread_id;

#define MAX_DATA_AGE_MS 500

#define CAN_POLL_INTERVAL_MS 50

static int is_data_stale(void)
{
    struct timespec now, diff;
    long diff_ms;

    clock_gettime(CLOCK_MONOTONIC, &now);

    diff.tv_sec = now.tv_sec - can_vars.last_update.tv_sec;
    diff.tv_nsec = now.tv_nsec - can_vars.last_update.tv_nsec;
    if (diff.tv_nsec < 0)
    {
        diff.tv_sec--;
        diff.tv_nsec += 1000000000;
    }

    diff_ms = (diff.tv_sec * 1000) + (diff.tv_nsec / 1000000);

    return (diff_ms > MAX_DATA_AGE_MS);
}

void *can_monitor_thread(void *arg)
{
    (void)arg;

    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

    int temp_enable, temp_x, temp_y, temp_horn, temp_estop, temp_speed;
    int valid_reading;

    while (keep_running)
    {
        // Fetch all data in one batch with proper error handling
        // No need for GIL acquisition/release since we're no longer using Python
        temp_enable = get_enable_button();
        temp_x = get_x_axis();
        temp_y = get_y_axis();
        temp_horn = get_horn_button();
        temp_estop = get_estop_button();
        temp_speed = get_speed_button();

        valid_reading = 1;

        // Verify readings
        if (temp_enable != 0 && temp_enable != 1)
        {
            valid_reading = 0;
        }

        if (valid_reading)
        {
            /* Lock mutex before updating shared data */
            pthread_mutex_lock(&can_vars.mutex);

            can_vars.enable = temp_enable;
            can_vars.x_axis = temp_x;
            can_vars.y_axis = temp_y;
            can_vars.horn = temp_horn;
            can_vars.estop = temp_estop;
            can_vars.speed = temp_speed;

            can_vars.update_count++;
            clock_gettime(CLOCK_MONOTONIC, &can_vars.last_update);

            /* Unlock mutex */
            pthread_mutex_unlock(&can_vars.mutex);
        }
        else
        {
            pthread_mutex_lock(&can_vars.mutex);
            can_vars.error_count++;
            pthread_mutex_unlock(&can_vars.mutex);

            static int error_log_count = 0;
            if (error_log_count++ % 100 == 0)
            {
                printf("CAN read error #%d: enable=%d, x=%d, y=%d\n",
                       can_vars.error_count, temp_enable, temp_x, temp_y);
            }
        }
        
        pthread_testcancel();

        /* Sleep to avoid excessive CPU usage */
        usleep(CAN_POLL_INTERVAL_MS * 1000);
    }

    return NULL;
}

int init_can_monitor(void)
{
    pthread_mutex_init(&can_vars.mutex, NULL);

    memset(&can_vars, 0, sizeof(CANVariables));
    can_vars.enable = 1; /* Default to enabled */
    clock_gettime(CLOCK_MONOTONIC, &can_vars.last_update);
    can_vars.monitoring_active = 0;

    // Initialize SocketCAN
    if (!initialize_can_bus("can0")) {
        printf("Failed to initialize CAN bus\n");
        return 0;
    }

    keep_running = 1;

    if (pthread_create(&monitor_thread_id, NULL, can_monitor_thread, NULL) != 0)
    {
        printf("Failed to create CAN monitor thread\n");
        shutdown_can_bus();
        return 0;
    }

    can_vars.monitoring_active = 1;

    return 1;
}

void stop_can_monitor(void)
{
    if (!can_vars.monitoring_active)
    {
        return;
    }

    keep_running = 0;

    usleep(300000); // 300ms wait

    pthread_cancel(monitor_thread_id);
    pthread_join(monitor_thread_id, NULL);

    pthread_mutex_destroy(&can_vars.mutex);
    can_vars.monitoring_active = 0;
    
    // Shutdown CAN bus
    shutdown_can_bus();
}

// The rest of the functions remain unchanged
int get_can_enable(void)
{
    int result;

    pthread_mutex_lock(&can_vars.mutex);

    if (is_data_stale())
    {
        result = 1;

        static int stale_warnings = 0;
        if (stale_warnings++ % 1000 == 0)
        {
            printf("Warning: CAN enable data is stale\n");
        }
    }
    else
    {
        result = can_vars.enable;
    }

    pthread_mutex_unlock(&can_vars.mutex);
    return result;
}

int get_can_x_axis(void)
{
    int result;

    pthread_mutex_lock(&can_vars.mutex);

    if (is_data_stale())
    {
        result = 128; /* Default to center position */
    }
    else
    {
        result = can_vars.x_axis;
    }

    pthread_mutex_unlock(&can_vars.mutex);
    return result;
}

int get_can_y_axis(void)
{
    int result;

    pthread_mutex_lock(&can_vars.mutex);

    if (is_data_stale())
    {
        result = 128; /* Default to center position */
    }
    else
    {
        result = can_vars.y_axis;
    }

    pthread_mutex_unlock(&can_vars.mutex);
    return result;
}

int get_can_horn(void)
{
    int result;

    pthread_mutex_lock(&can_vars.mutex);

    if (is_data_stale())
    {
        result = 0; /* Default to not pressed */
    }
    else
    {
        result = can_vars.horn;
    }

    pthread_mutex_unlock(&can_vars.mutex);
    return result;
}

int get_can_estop(void)
{
    int result;

    pthread_mutex_lock(&can_vars.mutex);

    if (is_data_stale())
    {
        result = 0; /* Default to not engaged */
    }
    else
    {
        result = can_vars.estop;
    }

    pthread_mutex_unlock(&can_vars.mutex);
    return result;
}

int get_can_speed(void)
{
    int result;

    pthread_mutex_lock(&can_vars.mutex);

    if (is_data_stale())
    {
        result = 0; /* Default to normal speed */
    }
    else
    {
        result = can_vars.speed;
    }

    pthread_mutex_unlock(&can_vars.mutex);
    return result;
}

void print_can_status(void)
{
    struct timespec now, diff;
    long diff_ms;

    pthread_mutex_lock(&can_vars.mutex);

    clock_gettime(CLOCK_MONOTONIC, &now);

    diff.tv_sec = now.tv_sec - can_vars.last_update.tv_sec;
    diff.tv_nsec = now.tv_nsec - can_vars.last_update.tv_nsec;
    if (diff.tv_nsec < 0)
    {
        diff.tv_sec--;
        diff.tv_nsec += 1000000000;
    }

    diff_ms = (diff.tv_sec * 1000) + (diff.tv_nsec / 1000000);

    printf("CAN Status - Updates: %d, Errors: %d, Last Update: %ld ms ago\n",
           can_vars.update_count, can_vars.error_count, diff_ms);

    printf("Enable: %d, X-Axis: %d, Y-Axis: %d, Horn: %d, E-Stop: %d, Speed: %d\n",
           can_vars.enable, can_vars.x_axis, can_vars.y_axis,
           can_vars.horn, can_vars.estop, can_vars.speed);

    pthread_mutex_unlock(&can_vars.mutex);
}