#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <poll.h>
#include <errno.h>
#include "socketcan.h"

static int can_socket = -1;

int initialize_can_bus(const char *interface)
{
    struct sockaddr_can addr;
    struct ifreq ifr;

    // Create socket
    can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket < 0) {
        perror("Error creating CAN socket");
        return 0;
    }

    // Specify CAN interface
    strcpy(ifr.ifr_name, interface);
    if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0) {
        perror("Error getting interface index");
        close(can_socket);
        can_socket = -1;
        return 0;
    }

    // Bind socket to CAN interface
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error binding CAN socket");
        close(can_socket);
        can_socket = -1;
        return 0;
    }

    return 1;
}

void shutdown_can_bus(void)
{
    if (can_socket >= 0) {
        close(can_socket);
        can_socket = -1;
    }
}

int send_can_message(int can_id, uint8_t *data, int data_length)
{
    struct can_frame frame;

    if (can_socket < 0) {
        fprintf(stderr, "CAN bus not initialized\n");
        return 0;
    }

    if (data_length > 8) {
        fprintf(stderr, "Data length exceeds maximum (8 bytes)\n");
        return 0;
    }

    // Prepare CAN frame
    memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id;
    frame.can_dlc = data_length;
    memcpy(frame.data, data, data_length);

    // Send frame
    if (write(can_socket, &frame, sizeof(frame)) != sizeof(frame)) {
        perror("Error sending CAN frame");
        return 0;
    }

    return 1;
}

int receive_can_message(int can_id, uint8_t *data, int *data_length, int timeout_ms)
{
    struct can_frame frame;
    struct can_filter filter;
    struct pollfd pfd;
    int ret;

    if (can_socket < 0) {
        fprintf(stderr, "CAN bus not initialized\n");
        return 0;
    }

    // Set filter to only receive frames with specified ID
    filter.can_id = can_id;
    filter.can_mask = CAN_SFF_MASK;
    if (setsockopt(can_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
        perror("Error setting CAN filter");
        return 0;
    }

    // Wait for frame using poll
    pfd.fd = can_socket;
    pfd.events = POLLIN;
    ret = poll(&pfd, 1, timeout_ms);

    if (ret < 0) {
        perror("Error polling CAN socket");
        return 0;
    }
    if (ret == 0) {
        // Timeout
        return 0;
    }

    // Read frame
    ret = read(can_socket, &frame, sizeof(frame));
    if (ret < 0) {
        perror("Error reading CAN frame");
        return 0;
    }

    if (frame.can_id != can_id) {
        // Not the expected frame
        return 0;
    }

    // Copy data
    *data_length = frame.can_dlc;
    memcpy(data, frame.data, frame.can_dlc);

    return 1;
}

// Bit extraction helpers
static int extract_bit(uint8_t *data, int bit_position)
{
    int byte_index = bit_position / 8;
    int bit_index = bit_position % 8;
    return (data[byte_index] >> bit_index) & 1;
}

// BUTTONS Block Functions (0x18A)
int get_button_value(int bit_position)
{
    uint8_t data[8];
    int data_length;

    if (receive_can_message(BUTTONS_ID, data, &data_length, 1000)) {
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

int get_can_enable_button(void) 
{
    return get_button_value(59);  // CAN_Enable bit (bit 59) from BUTTONS block
}

int get_estop_button(void) 
{
    return get_button_value(63);  // Estop bit (bit 63) from BUTTONS block
}

// MOVES Block Functions (0x28A)
int get_y_axis(void) 
{
    uint8_t data[8];
    int data_length;

    if (receive_can_message(MOVES_ID, data, &data_length, 1000)) {
        return data[0];  // Y_Axis value (bits 0-7) from MOVES block
    }
    return -1;
}

int get_x_axis(void) 
{
    uint8_t data[8];
    int data_length;

    if (receive_can_message(MOVES_ID, data, &data_length, 1000)) {
        return data[1];  // X_Axis value (bits 8-15) from MOVES block
    }
    return -1;
}

// LED Block Functions (0x30A)
int set_led(int bit_position, int state)
{
    uint8_t data[8] = {0};

    if (state) {
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