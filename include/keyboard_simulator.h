/**
 * @file keyboard_simulator.h
 * @brief Header file for keyboard-based CAN message simulator
 * 
 * Provides a virtual CAN device that simulates joystick and button inputs
 * using keyboard controls. Sends CAN messages that are compatible with
 * the existing CAN monitoring infrastructure.
 */

#ifndef KEYBOARD_SIMULATOR_H
#define KEYBOARD_SIMULATOR_H

#include <stdint.h>
#include <pthread.h>
#include <stdbool.h>

// Configuration defines
#define JOYSTICK_RAMP_RATE 5        // Units per polling cycle
#define CAN_POLL_INTERVAL_MS 50     // Same as real CAN
// Note: JOYSTICK_CENTER is defined in ethercat_defs.h

// CAN message IDs (must match can_interface.c)
#define BUTTONS_ID 0x18A
#define MOVES_ID 0x28A

// State structure for keyboard simulator
typedef struct {
    uint8_t x_position;         // 0-255, init 128
    uint8_t y_position;         // 0-255, init 128
    uint8_t enable_state;       // 0 or 1
    uint8_t speed_state;        // 0 or 1
    volatile int running;       // Thread control
    pthread_t thread_id;
} KeyboardSimState;

/**
 * @brief Initialize the keyboard simulator
 * @return true on success, false on failure
 */
bool init_keyboard_simulator(void);

/**
 * @brief Stop the keyboard simulator and cleanup resources
 */
void stop_keyboard_simulator(void);

#endif // KEYBOARD_SIMULATOR_H