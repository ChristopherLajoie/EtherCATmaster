/**
 * @file keyboard_simulator.c
 * @brief Implementation of keyboard-based CAN message simulator
 * 
 * Provides a virtual CAN device that simulates joystick and button inputs
 * using keyboard controls. Sends CAN messages that are compatible with
 * the existing CAN monitoring infrastructure.
 */

#include "keyboard_simulator.h"
#include "can_interface.h"
#include "hardware_io.h"
#include "common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

// Global state for keyboard simulator
static KeyboardSimState sim_state;

// Forward declarations
static void* keyboard_simulator_thread(void* arg);
static void process_keyboard_input(void);
static void send_moves_message(void);
static void send_buttons_message(void);
static void set_button_bit(uint8_t* data, int bit_position, int state);

bool init_keyboard_simulator(void)
{
    // Initialize state
    memset(&sim_state, 0, sizeof(KeyboardSimState));
    sim_state.x_position = JOYSTICK_CENTER;
    sim_state.y_position = JOYSTICK_CENTER;
    sim_state.enable_state = 1;  // Start enabled
    sim_state.speed_state = 0;
    sim_state.running = 1;

    // Create the keyboard monitoring thread
    if (pthread_create(&sim_state.thread_id, NULL, keyboard_simulator_thread, NULL) != 0) {
        fprintf(stderr, "Failed to create keyboard simulator thread\n");
        return false;
    }

    printf("\nControls: WASD=movement, Space=enable, Q=speed, R=reset, Ctrl+C=exit\n");
    printf("Initial state: Enable=ON, Speed=NORMAL, Position=CENTER\n");
    return true;
}

void stop_keyboard_simulator(void)
{
    if (sim_state.running) {
        sim_state.running = 0;
        pthread_join(sim_state.thread_id, NULL);
    }
}

static void* keyboard_simulator_thread(void* arg)
{
    (void)arg;
    
    struct timespec sleep_time;
    sleep_time.tv_sec = 0;
    sleep_time.tv_nsec = CAN_POLL_INTERVAL_MS * 1000000L; // Convert ms to ns

    while (sim_state.running) {
        // Process keyboard input
        process_keyboard_input();
        
        // Send CAN messages
        send_moves_message();
        send_buttons_message();
        
        // Sleep for polling interval
        nanosleep(&sleep_time, NULL);
    }

    return NULL;
}

static void process_keyboard_input(void)
{
    char ch;
    static int prev_space_state = 0;
    static int prev_shift_state = 0;
    
    // Process all available key presses
    while (kbhit()) {
        ch = readch();
        
        switch (ch) {
            case 'w':
            case 'W':
                // Increase Y-axis (max 255)
                if (sim_state.y_position <= 255 - JOYSTICK_RAMP_RATE) {
                    sim_state.y_position += JOYSTICK_RAMP_RATE;
                } else {
                    sim_state.y_position = 255;
                }
                break;
                
            case 's':
            case 'S':
                // Decrease Y-axis (min 0)
                if (sim_state.y_position >= JOYSTICK_RAMP_RATE) {
                    sim_state.y_position -= JOYSTICK_RAMP_RATE;
                } else {
                    sim_state.y_position = 0;
                }
                break;
                
            case 'a':
            case 'A':
                // Decrease X-axis (min 0)
                if (sim_state.x_position >= JOYSTICK_RAMP_RATE) {
                    sim_state.x_position -= JOYSTICK_RAMP_RATE;
                } else {
                    sim_state.x_position = 0;
                }
                break;
                
            case 'd':
            case 'D':
                // Increase X-axis (max 255)
                if (sim_state.x_position <= 255 - JOYSTICK_RAMP_RATE) {
                    sim_state.x_position += JOYSTICK_RAMP_RATE;
                } else {
                    sim_state.x_position = 255;
                }
                break;
                
            case ' ':
                // Space bar: Toggle enable button (0→1→0)
                if (!prev_space_state) {
                    sim_state.enable_state = !sim_state.enable_state;
                    printf("Enable: %s\n", sim_state.enable_state ? "ON" : "OFF");
                }
                prev_space_state = 1;
                break;
                
            case 'r':
            case 'R':
                // Reset both axes to center
                sim_state.x_position = JOYSTICK_CENTER;
                sim_state.y_position = JOYSTICK_CENTER;
                printf("Position reset to center\n");
                break;
                
            case 'q':
            case 'Q':
                // Q key as shift substitute for better compatibility
                if (!prev_shift_state) {
                    sim_state.speed_state = !sim_state.speed_state;
                    printf("Speed mode: %s\n", sim_state.speed_state ? "HIGH" : "NORMAL");
                }
                prev_shift_state = 1;
                break;
                
            default:
                // Reset previous key states for non-special keys
                prev_space_state = 0;
                prev_shift_state = 0;
                break;
        }
    }
    
    // Reset key states if no keys pressed
    if (!kbhit()) {
        prev_space_state = 0;
        prev_shift_state = 0;
    }
}

static void send_moves_message(void)
{
    uint8_t data[8];
    
    // Initialize data buffer
    memset(data, 0, sizeof(data));
    
    // MOVES_ID message format:
    // data[0] = y_position (0-255)
    // data[1] = x_position (0-255)
    // data[2-7] = 0 (unused)
    data[0] = sim_state.y_position;
    data[1] = sim_state.x_position;
    
    // Send the CAN message
    send_can_message(MOVES_ID, data, 8);
}

static void send_buttons_message(void)
{
    uint8_t data[8];
    
    // Initialize data buffer
    memset(data, 0, sizeof(data));
    
    // Bit 63: e-stop (always 1 - inverted logic, 1 = not stopped)
    set_button_bit(data, 6, sim_state.enable_state);
    set_button_bit(data, 14, sim_state.speed_state);
    set_button_bit(data, 63, 1); 
    
    // Send the CAN message
    send_can_message(BUTTONS_ID, data, 8);
}

static void set_button_bit(uint8_t* data, int bit_position, int state)
{
    int byte_index = bit_position / 8;
    int bit_index = bit_position % 8;
    
    if (byte_index < 8) { // Ensure we don't exceed buffer bounds
        if (state) {
            data[byte_index] |= (1 << bit_index);
        } else {
            data[byte_index] &= ~(1 << bit_index);
        }
    }
}