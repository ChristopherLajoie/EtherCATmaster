
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

#ifndef CAN_MODE_SIMULATOR
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <poll.h>
#endif

#include "can_interface.h"

// Global variables for CAN state
CANVariables can_vars;
static volatile int keep_running = 1;
static pthread_t monitor_thread_id;

// Timeouts and intervals
#define MAX_DATA_AGE_MS 500
#define CAN_POLL_INTERVAL_MS 50

// CAN message IDs
#define BUTTONS_ID 0x18A
#define MOVES_ID 0x28A
#define LED_ID 0x30A

// Forward declarations for internal functions
static void* can_monitor_thread(void* arg);
static int is_data_stale(void);

#ifdef CAN_MODE_SIMULATOR
//==============================================================================
// SIMULATOR IMPLEMENTATION
//==============================================================================

// Structure to hold simulated CAN data
typedef struct
{
    uint8_t enable_button;
    uint8_t speed_button;
    uint8_t horn_button;
    uint8_t can_enable_button;
    uint8_t estop_button;
    uint8_t x_axis;
    uint8_t y_axis;

    // For auto-spring back behavior
    struct timeval last_x_press;
    struct timeval last_y_press;
    int x_direction;  // -1 = left, 0 = none, 1 = right
    int y_direction;  // -1 = down, 0 = none, 1 = up

    pthread_mutex_t mutex;           // Mutex for thread-safe access
    int simulation_active;           // Flag to indicate if simulation is active
    pthread_t keyboard_thread_id;    // Thread for keyboard input
    pthread_t springback_thread_id;  // Thread for spring-back behavior
} CANSimulator;

static CANSimulator can_sim;
static struct termios orig_termios;

// Function prototypes for simulator
static void sim_enable_raw_mode(void);
static void sim_disable_raw_mode(void);
static int sim_kbhit(void);
static char sim_readch(void);
static void print_simulator_status(void);
static void* springback_thread(void* arg);
static void* keyboard_input_thread(void* arg);

// Set terminal to raw mode to read keystrokes without waiting for Enter
static void sim_enable_raw_mode(void)
{
    tcgetattr(STDIN_FILENO, &orig_termios);
    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON);  // Disable echo and canonical mode
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);

    // Set stdin to non-blocking
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

static void sim_disable_raw_mode(void)
{
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);

    // Restore blocking mode
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
}

// Function to read keyboard without blocking
static int sim_kbhit(void)
{
    char ch;
    int nread = read(STDIN_FILENO, &ch, 1);
    if (nread == 1)
    {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

static char sim_readch(void)
{
    char ch;
    read(STDIN_FILENO, &ch, 1);
    return ch;
}

// Print current controller state
static void print_simulator_status(void)
{
    pthread_mutex_lock(&can_sim.mutex);

    printf("\033[2J\033[H");  // Clear screen and move cursor to top
    printf("=== CAN SIMULATOR STATUS ===\n");
    printf("Enable Button (1):    %s\n", can_sim.enable_button ? "ON" : "OFF");
    printf("Speed Button (2):     %s\n", can_sim.speed_button ? "ON" : "OFF");
    printf("Horn Button (3):      %s\n", can_sim.horn_button ? "ON" : "OFF");
    printf("CAN Enable Button (4): %s\n", can_sim.can_enable_button ? "ON" : "OFF");
    printf("Emergency Stop (5):   %s\n", can_sim.estop_button ? "ON" : "OFF");
    printf("X-Axis:              %d\n", can_sim.x_axis);
    printf("Y-Axis:              %d\n", can_sim.y_axis);
    printf("\n=== KEYBOARD CONTROLS ===\n");
    printf("1-5: Toggle buttons   6: Center joystick\n");
    printf("Arrow Keys: Control Joystick (auto-centers after release)\n");
    printf("P: Show this status\n");

    pthread_mutex_unlock(&can_sim.mutex);
}

// Joystick spring-back thread function
static void* springback_thread(void* arg)
{
    (void)arg;  // Unused parameter

    struct timeval now;
    long diff_ms_x, diff_ms_y;
    const int SPRING_TIMEOUT_MS = 200;  // Time without keypresses to trigger spring-back
    const int SPRING_STEP = 10;         // How quickly to return to center

    while (keep_running)
    {
        pthread_mutex_lock(&can_sim.mutex);

        gettimeofday(&now, NULL);

        // Calculate time since last X press
        diff_ms_x = ((now.tv_sec - can_sim.last_x_press.tv_sec) * 1000) + ((now.tv_usec - can_sim.last_x_press.tv_usec) / 1000);

        // Calculate time since last Y press
        diff_ms_y = ((now.tv_sec - can_sim.last_y_press.tv_sec) * 1000) + ((now.tv_usec - can_sim.last_y_press.tv_usec) / 1000);

        // Spring back X-axis
        if (diff_ms_x > SPRING_TIMEOUT_MS && can_sim.x_axis != 128)
        {
            if (can_sim.x_axis < 128)
            {
                can_sim.x_axis += SPRING_STEP;
                if (can_sim.x_axis > 128)
                    can_sim.x_axis = 128;
            }
            else
            {
                can_sim.x_axis -= SPRING_STEP;
                if (can_sim.x_axis < 128)
                    can_sim.x_axis = 128;
            }
            can_sim.x_direction = 0;  // Reset direction once centered
        }

        // Spring back Y-axis
        if (diff_ms_y > SPRING_TIMEOUT_MS && can_sim.y_axis != 128)
        {
            if (can_sim.y_axis < 128)
            {
                can_sim.y_axis += SPRING_STEP;
                if (can_sim.y_axis > 128)
                    can_sim.y_axis = 128;
            }
            else
            {
                can_sim.y_axis -= SPRING_STEP;
                if (can_sim.y_axis < 128)
                    can_sim.y_axis = 128;
            }
            can_sim.y_direction = 0;  // Reset direction once centered
        }

        pthread_mutex_unlock(&can_sim.mutex);

        // Sleep to reduce CPU usage
        usleep(50000);  // 50ms
    }

    return NULL;
}

// Keyboard input thread for simulation control
static void* keyboard_input_thread(void* arg)
{
    (void)arg;  // Unused

    sim_enable_raw_mode();

    // Initialize last press time
    gettimeofday(&can_sim.last_x_press, NULL);
    gettimeofday(&can_sim.last_y_press, NULL);

    while (keep_running)
    {
        if (sim_kbhit())
        {
            char c = sim_readch();

            pthread_mutex_lock(&can_sim.mutex);

            switch (c)
            {
                case '1':  // Toggle Enable button
                    can_sim.enable_button = !can_sim.enable_button;
                    printf("Enable button: %s\n", can_sim.enable_button ? "ON" : "OFF");
                    break;

                case '2':  // Toggle Speed button
                    can_sim.speed_button = !can_sim.speed_button;
                    printf("Speed button: %s\n", can_sim.speed_button ? "ON" : "OFF");
                    break;

                case '3':  // Toggle Horn button
                    can_sim.horn_button = !can_sim.horn_button;
                    printf("Horn button: %s\n", can_sim.horn_button ? "ON" : "OFF");
                    break;

                case '4':  // Toggle CAN Enable button
                    can_sim.can_enable_button = !can_sim.can_enable_button;
                    printf("CAN Enable button: %s\n", can_sim.can_enable_button ? "ON" : "OFF");
                    break;

                case '5':  // Toggle Emergency stop
                    can_sim.estop_button = !can_sim.estop_button;
                    printf("Emergency Stop: %s\n", can_sim.estop_button ? "ON" : "OFF");
                    break;

                case '6':  // Center joystick
                    can_sim.x_axis = 128;
                    can_sim.y_axis = 128;
                    printf("Joystick centered\n");
                    break;

                case 27:  // Arrow keys (ESC [ A/B/C/D sequence)
                    if (sim_readch() == '[')
                    {
                        char dir = sim_readch();

                        switch (dir)
                        {
                            case 'A':  // Up - Increase Y axis (forward)
                                can_sim.y_axis = (can_sim.y_axis <= 235) ? can_sim.y_axis + 20 : 255;
                                can_sim.y_direction = 1;
                                gettimeofday(&can_sim.last_y_press, NULL);
                                break;

                            case 'B':  // Down - Decrease Y axis (backward)
                                can_sim.y_axis = (can_sim.y_axis >= 20) ? can_sim.y_axis - 20 : 0;
                                can_sim.y_direction = -1;
                                gettimeofday(&can_sim.last_y_press, NULL);
                                break;

                            case 'C':  // Right - Increase X axis
                                can_sim.x_axis = (can_sim.x_axis <= 235) ? can_sim.x_axis + 20 : 255;
                                can_sim.x_direction = 1;
                                gettimeofday(&can_sim.last_x_press, NULL);
                                break;

                            case 'D':  // Left - Decrease X axis
                                can_sim.x_axis = (can_sim.x_axis >= 20) ? can_sim.x_axis - 20 : 0;
                                can_sim.x_direction = -1;
                                gettimeofday(&can_sim.last_x_press, NULL);
                                break;
                        }
                    }
                    break;

                case 'p':  // Print status
                case 'P':
                    pthread_mutex_unlock(&can_sim.mutex);
                    print_simulator_status();
                    pthread_mutex_lock(&can_sim.mutex);
                    break;
            }

            pthread_mutex_unlock(&can_sim.mutex);
        }

        // Sleep to avoid excessive CPU usage
        usleep(50000);  // 50ms
    }

    sim_disable_raw_mode();
    return NULL;
}

// Simulator implementation of CAN functions
int initialize_can_bus(const char* interface)
{
    printf("Initializing simulated CAN bus on interface: %s\n", interface);

    // Initialize the simulator data structure
    memset(&can_sim, 0, sizeof(CANSimulator));
    pthread_mutex_init(&can_sim.mutex, NULL);

    // Set default values
    can_sim.enable_button = 1;  // Default to enabled
    can_sim.speed_button = 0;
    can_sim.horn_button = 0;
    can_sim.can_enable_button = 1;
    can_sim.estop_button = 1;  // Set to 1 to work with the motor_control.c logic
    can_sim.x_axis = 128;      // Center position
    can_sim.y_axis = 128;      // Center position

    // Initialize spring-back tracking
    gettimeofday(&can_sim.last_x_press, NULL);
    gettimeofday(&can_sim.last_y_press, NULL);
    can_sim.x_direction = 0;
    can_sim.y_direction = 0;

    // Create keyboard input thread
    keep_running = 1;
    if (pthread_create(&can_sim.keyboard_thread_id, NULL, keyboard_input_thread, NULL) != 0)
    {
        printf("Failed to create keyboard input thread\n");
        pthread_mutex_destroy(&can_sim.mutex);
        return 0;
    }

    // Create spring-back thread
    if (pthread_create(&can_sim.springback_thread_id, NULL, springback_thread, NULL) != 0)
    {
        printf("Failed to create springback thread\n");
        pthread_cancel(can_sim.keyboard_thread_id);
        pthread_join(can_sim.keyboard_thread_id, NULL);
        pthread_mutex_destroy(&can_sim.mutex);
        return 0;
    }

    can_sim.simulation_active = 1;

    return 1;
}

void shutdown_can_bus(void)
{
    if (!can_sim.simulation_active)
    {
        return;
    }

    printf("Shutting down simulated CAN bus\n");

    keep_running = 0;
    usleep(300000);  // 300ms wait

    pthread_cancel(can_sim.keyboard_thread_id);
    pthread_join(can_sim.keyboard_thread_id, NULL);

    pthread_cancel(can_sim.springback_thread_id);
    pthread_join(can_sim.springback_thread_id, NULL);

    pthread_mutex_destroy(&can_sim.mutex);
    can_sim.simulation_active = 0;
}

int send_can_message(int can_id, uint8_t* data, int data_length)
{
    (void)can_id;       // Prevent unused parameter warning
    (void)data;         // Prevent unused parameter warning
    (void)data_length;  // Prevent unused parameter warning
    return 1;
}

int receive_can_message(int can_id, uint8_t* data, int* data_length, int timeout_ms)
{
    (void)can_id;      // Prevent unused parameter warning
    (void)timeout_ms;  // Prevent unused parameter warning

    memset(data, 0, 8);
    *data_length = 8;
    return 1;
}

// Button and axis getter functions for simulator
int get_enable_button(void)
{
    int result;
    pthread_mutex_lock(&can_sim.mutex);
    result = can_sim.enable_button;
    pthread_mutex_unlock(&can_sim.mutex);
    return result;
}

int get_speed_button(void)
{
    int result;
    pthread_mutex_lock(&can_sim.mutex);
    result = can_sim.speed_button;
    pthread_mutex_unlock(&can_sim.mutex);
    return result;
}

int get_horn_button(void)
{
    int result;
    pthread_mutex_lock(&can_sim.mutex);
    result = can_sim.horn_button;
    pthread_mutex_unlock(&can_sim.mutex);
    return result;
}

int get_estop_button(void)
{
    int result;
    pthread_mutex_lock(&can_sim.mutex);
    result = can_sim.estop_button;
    pthread_mutex_unlock(&can_sim.mutex);
    return result;
}

int get_y_axis(void)
{
    int result;
    pthread_mutex_lock(&can_sim.mutex);
    result = can_sim.y_axis;
    pthread_mutex_unlock(&can_sim.mutex);
    return result;
}

int get_x_axis(void)
{
    int result;
    pthread_mutex_lock(&can_sim.mutex);
    result = can_sim.x_axis;
    pthread_mutex_unlock(&can_sim.mutex);
    return result;
}

#else
//==============================================================================
// REAL HARDWARE IMPLEMENTATION
//==============================================================================

static int can_socket = -1;

int initialize_can_bus(const char* interface)
{
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
}

int receive_can_message(int can_id, uint8_t* data, int* data_length, int timeout_ms)
{
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

    if (frame.can_id != can_id)
    {
        // Not the expected frame
        return 0;
    }

    // Copy data
    *data_length = frame.can_dlc;
    memcpy(data, frame.data, frame.can_dlc);

    return 1;
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

#endif

//==============================================================================
// COMMON IMPLEMENTATION (WORKS WITH BOTH REAL AND SIMULATED CAN)
//==============================================================================

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

    // Initialize CAN bus
#ifdef CAN_MODE_SIMULATOR
    if (!initialize_can_bus("sim0"))
    {
        printf("Failed to initialize simulated CAN bus\n");
        return 0;
    }
    printf("Using SIMULATED CAN interface\n");
#else
    if (!initialize_can_bus("can0"))
    {
        printf("Failed to initialize CAN bus\n");
        return 0;
    }
    printf("Using REAL CAN hardware\n");
#endif

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