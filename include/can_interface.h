#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

#include <stdint.h>
#include <pthread.h>
#include <time.h>

typedef struct
{
    pthread_mutex_t mutex;
    struct timespec last_update;
    int monitoring_active;
    int update_count;
    int error_count;
    int enable;
    int x_axis;
    int y_axis;
    int horn;
    int estop;
    int speed;
} CANVariables;

extern CANVariables can_vars;

int init_can_interface(void);
void stop_can_interface(void);

int get_can_enable(void);
int get_can_x_axis(void);
int get_can_y_axis(void);
int get_can_horn(void);
int get_can_estop(void);
int get_can_speed(void);
void print_can_status(void);

int set_yellow_bat_led(int state);
int set_red_bat_led(int state);
int set_overload_led(int state);
int set_aux_led(int state);

#endif