#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>

#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <poll.h>

#include "can_interface.h"

#ifdef CAN_MODE_SIMULATION
typedef struct
{
    uint8_t moves_data[8];
    uint8_t buttons_data[8];
    struct timespec last_moves_update;
    struct timespec last_buttons_update;
} sim_can_data_t;

static sim_can_data_t sim_can_data = {0};
static pthread_mutex_t sim_can_mutex = PTHREAD_MUTEX_INITIALIZER;

// Initialize simulation CAN data with default values
static void init_sim_can_data(void) {
    pthread_mutex_lock(&sim_can_mutex);
    // Initialize MOVES data (center position)
    sim_can_data.moves_data[0] = 128; // Y center
    sim_can_data.moves_data[1] = 128; // X center
    memset(&sim_can_data.moves_data[2], 0, 6); // Clear remaining bytes
    
    // Initialize BUTTONS data 
    memset(sim_can_data.buttons_data, 0, 8);
    // Set bit 6 (enable) to 1 - start enabled
    sim_can_data.buttons_data[0] |= (1 << 6); 
    // Set bit 63 (e-stop) to 1 - always not stopped (inverted logic)
    sim_can_data.buttons_data[7] |= (1 << 7);  
    
    clock_gettime(CLOCK_MONOTONIC, &sim_can_data.last_moves_update);
    clock_gettime(CLOCK_MONOTONIC, &sim_can_data.last_buttons_update);
    pthread_mutex_unlock(&sim_can_mutex);
}

#endif

CANVariables can_vars;
static volatile int keep_running = 1;
static pthread_t monitor_thread_id;

#define MAX_DATA_AGE_MS 500
#define CAN_POLL_INTERVAL_MS 50

#define BUTTONS_ID 0x18A
#define MOVES_ID 0x28A
#define LED_ID 0x30A

static void* can_monitor_thread(void* arg);
static int is_data_stale(void);

static int can_socket = -1;

int initialize_can_bus(const char* interface)
{
#ifdef CAN_MODE_SIMULATION
    (void)interface;
    can_socket = 1;  // Dummy valid socket

    init_sim_can_data();
    printf("CAN bus initialized in simulation mode\n");
    return 1;
#else
    struct sockaddr_can addr;
    struct ifreq ifr;

    // Create socket
    can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket < 0)
    {
        perror("Error creating CAN socket");
        return 0;
    }

    // Specify CAN interface
    strcpy(ifr.ifr_name, interface);
    if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("Error getting interface index");
        close(can_socket);
        can_socket = -1;
        return 0;
    }

    // Bind socket to CAN interface
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        perror("Error binding CAN socket");
        close(can_socket);
        can_socket = -1;
        return 0;
    }

    return 1;
#endif
}

void shutdown_can_bus(void)
{
    if (can_socket >= 0)
    {
        close(can_socket);
        can_socket = -1;
    }
}

int send_can_message(int can_id, uint8_t* data, int data_length)
{
#ifdef CAN_MODE_SIMULATION
    // In simulation mode, store the data locally for retrieval by receive functions
    if (can_id == MOVES_ID && data_length == 8)
    {
        pthread_mutex_lock(&sim_can_mutex);
        memcpy(sim_can_data.moves_data, data, 8);
        clock_gettime(CLOCK_MONOTONIC, &sim_can_data.last_moves_update);
        pthread_mutex_unlock(&sim_can_mutex);
        return 1;
    }
    else if (can_id == BUTTONS_ID && data_length == 8)
    {
        pthread_mutex_lock(&sim_can_mutex);
        memcpy(sim_can_data.buttons_data, data, 8);
        clock_gettime(CLOCK_MONOTONIC, &sim_can_data.last_buttons_update);
        pthread_mutex_unlock(&sim_can_mutex);
        return 1;
    }
    return 1;  // Return success for other messages
#else
    struct can_frame frame;

    if (can_socket < 0)
    {
        fprintf(stderr, "CAN bus not initialized\n");
        return 0;
    }

    if (data_length > 8)
    {
        fprintf(stderr, "Data length exceeds maximum (8 bytes)\n");
        return 0;
    }

    // Prepare CAN frame
    memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id;
    frame.can_dlc = data_length;
    memcpy(frame.data, data, data_length);

    // Send frame
    if (write(can_socket, &frame, sizeof(frame)) != sizeof(frame))
    {
        perror("Error sending CAN frame");
        return 0;
    }

    return 1;
#endif
}

int receive_can_message(int can_id, uint8_t* data, int* data_length, int timeout_ms)
{
#ifdef CAN_MODE_SIMULATION
    // In simulation mode, retrieve data from local storage

    (void)timeout_ms;
    pthread_mutex_lock(&sim_can_mutex);

    if (can_id == MOVES_ID)
    {
        memcpy(data, sim_can_data.moves_data, 8);
        *data_length = 8;
        pthread_mutex_unlock(&sim_can_mutex);
        return 1;
    }
    else if (can_id == BUTTONS_ID)
    {
        memcpy(data, sim_can_data.buttons_data, 8);
        *data_length = 8;
        pthread_mutex_unlock(&sim_can_mutex);
        return 1;
    }

    pthread_mutex_unlock(&sim_can_mutex);
    return 0;  // Message not available
#else
    struct can_frame frame;
    struct can_filter filter;
    struct pollfd pfd;
    int ret;

    if (can_socket < 0)
    {
        fprintf(stderr, "CAN bus not initialized\n");
        return 0;
    }

    // Set filter to only receive frames with specified ID
    filter.can_id = can_id;
    filter.can_mask = CAN_SFF_MASK;
    if (setsockopt(can_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0)
    {
        perror("Error setting CAN filter");
        return 0;
    }

    // Wait for frame using poll
    pfd.fd = can_socket;
    pfd.events = POLLIN;
    ret = poll(&pfd, 1, timeout_ms);

    if (ret < 0)
    {
        perror("Error polling CAN socket");
        return 0;
    }
    if (ret == 0)
    {
        // Timeout
        return 0;
    }

    // Read frame
    ret = read(can_socket, &frame, sizeof(frame));
    if (ret < 0)
    {
        perror("Error reading CAN frame");
        return 0;
    }

    if (frame.can_id != (canid_t)can_id)
    {
        // Not the expected frame
        return 0;
    }

    // Copy data
    *data_length = frame.can_dlc;
    memcpy(data, frame.data, frame.can_dlc);

    return 1;
#endif
}

// Bit extraction helpers
static int extract_bit(uint8_t* data, int bit_position)
{
    int byte_index = bit_position / 8;
    int bit_index = bit_position % 8;
    return (data[byte_index] >> bit_index) & 1;
}

// Button getter implementations for real hardware
int get_button_value(int bit_position)
{
    uint8_t data[8];
    int data_length;

    if (receive_can_message(BUTTONS_ID, data, &data_length, 1000))
    {
        return extract_bit(data, bit_position);
    }
    return -1;
}

int get_enable_button(void)
{
    return get_button_value(6);  // Enable bit (bit 6) from BUTTONS block
}

int get_speed_button(void)
{
    return get_button_value(14);  // Speed bit (bit 14) from BUTTONS block
}

int get_horn_button(void)
{
    return get_button_value(16);  // Horn bit (bit 16) from BUTTONS block
}

int get_estop_button(void)
{
    return get_button_value(63);  // Estop bit (bit 63) from BUTTONS block
}

int get_y_axis(void)
{
    uint8_t data[8];
    int data_length;

    if (receive_can_message(MOVES_ID, data, &data_length, 1000))
    {
        return data[0];  // Y_Axis value (bits 0-7) from MOVES block
    }
    return -1;
}

int get_x_axis(void)
{
    uint8_t data[8];
    int data_length;

    if (receive_can_message(MOVES_ID, data, &data_length, 1000))
    {
        return data[1];  // X_Axis value (bits 8-15) from MOVES block
    }
    return -1;
}

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

// Monitor thread that continuously updates CAN values
static void* can_monitor_thread(void* arg)
{
    (void)arg;

    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

    int temp_enable, temp_x, temp_y, temp_horn, temp_estop, temp_speed;
    int valid_reading;

    while (keep_running)
    {
        // Fetch all data in one batch with proper error handling
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
                printf("CAN read error #%d: enable=%d, x=%d, y=%d\n", can_vars.error_count, temp_enable, temp_x, temp_y);
            }
        }

        pthread_testcancel();

        /* Sleep to avoid excessive CPU usage */
        usleep(CAN_POLL_INTERVAL_MS * 1000);
    }

    return NULL;
}

// Public interface functions

int init_can_interface(void)
{
    pthread_mutex_init(&can_vars.mutex, NULL);

    memset(&can_vars, 0, sizeof(CANVariables));
    can_vars.enable = 1; /* Default to enabled */
    clock_gettime(CLOCK_MONOTONIC, &can_vars.last_update);
    can_vars.monitoring_active = 0;

    if (!initialize_can_bus("can0"))
    {
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

void stop_can_interface(void)
{
    if (!can_vars.monitoring_active)
    {
        return;
    }

    keep_running = 0;
    usleep(300000);  // 300ms wait

    pthread_cancel(monitor_thread_id);
    pthread_join(monitor_thread_id, NULL);

    pthread_mutex_destroy(&can_vars.mutex);
    can_vars.monitoring_active = 0;

    // Shutdown CAN bus
    shutdown_can_bus();
}

// LED control functions
int set_led(int bit_position, int state)
{
    uint8_t data[8] = {0};

    if (state)
    {
        data[bit_position / 8] |= (1 << (bit_position % 8));
    }

    return send_can_message(LED_ID, data, 8);
}

int set_yellow_bat_led(int state)
{
    return set_led(0, state);  // Yellow_Bat_Led (bit 0) in LED block
}

int set_red_bat_led(int state)
{
    return set_led(1, state);  // Red_Bat_Led (bit 1) in LED block
}

int set_overload_led(int state)
{
    return set_led(2, state);  // Overload_Led (bit 2) in LED block
}

int set_aux_led(int state)
{
    return set_led(3, state);  // Aux_Led (bit 3) in LED block
}

// High-level interface functions - same for both real and simulation
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
           can_vars.update_count,
           can_vars.error_count,
           diff_ms);

    printf("Enable: %d, X-Axis: %d, Y-Axis: %d, Horn: %d, E-Stop: %d, Speed: %d\n",
           can_vars.enable,
           can_vars.x_axis,
           can_vars.y_axis,
           can_vars.horn,
           can_vars.estop,
           can_vars.speed);

    pthread_mutex_unlock(&can_vars.mutex);
}