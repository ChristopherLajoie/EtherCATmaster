#ifndef CAN_WRAPPER_H
#define CAN_WRAPPER_H

// Initialize and cleanup
int initialize_python_can(void);

// GIL handling functions
void acquire_gil(void);
void release_gil(void);

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

#endif /* CAN_WRAPPER_H */