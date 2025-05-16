#ifndef SOCKETCAN_H
#define SOCKETCAN_H

#include <stdint.h>

// CAN message IDs
#define BUTTONS_ID 0x18A
#define MOVES_ID 0x28A
#define LED_ID 0x30A
#define MONITOR_ID 0x40B

// Initialize and cleanup
int initialize_can_bus(const char *interface);
void shutdown_can_bus(void);

// Individual button functions - each returns 1 (on) or 0 (off), -1 on error
int get_enable_button(void);
int get_speed_button(void);
int get_horn_button(void);
int get_can_enable_button(void);
int get_estop_button(void);

// Individual joystick axis functions - each returns 0-255 value, -1 on error
int get_x_axis(void);
int get_y_axis(void);

// Individual LED control functions - each returns 1 on success, 0 on failure
int set_yellow_bat_led(int state);
int set_red_bat_led(int state);
int set_overload_led(int state);
int set_aux_led(int state);

// Utility functions
int send_can_message(int can_id, uint8_t *data, int data_length);
int receive_can_message(int can_id, uint8_t *data, int *data_length, int timeout_ms);

#endif /* SOCKETCAN_H */