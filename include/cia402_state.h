#ifndef CIA402_STATE_H
#define CIA402_STATE_H

#include "common.h"

// Initialize CiA 402 state machine
bool cia402_state_init();

// Process CiA 402 state machine
bool cia402_state_process(uint16_t *controlword, uint16_t statusword, int target_state);

// Decode and print statusword information
void cia402_decode_statusword(uint16_t statusword);

#endif // CIA402_STATE_H