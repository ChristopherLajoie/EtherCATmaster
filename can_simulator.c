/**
 * can_simulator.c
 * 
 * Simulated CAN controller for testing without real hardware.
 * This file replaces socketcan.c when simulation is needed.
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include <unistd.h>
 #include <pthread.h>
 #include <time.h>
 #include <termios.h>
 #include <fcntl.h>
 #include <sys/time.h>
 #include "socketcan.h"
 
 // Structure to hold simulated CAN data
 typedef struct {
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
     int x_direction; // -1 = left, 0 = none, 1 = right
     int y_direction; // -1 = down, 0 = none, 1 = up
     
     pthread_mutex_t mutex;          // Mutex for thread-safe access
     int simulation_active;          // Flag to indicate if simulation is active
     pthread_t keyboard_thread_id;   // Thread for keyboard input
     pthread_t springback_thread_id; // Thread for spring-back behavior
 } CANSimulator;
 
 static CANSimulator can_sim;
 static volatile int keep_running = 1;
 
 // Function to handle keyboard input for simulation control
 static struct termios orig_termios;
 
 // Set terminal to raw mode to read keystrokes without waiting for Enter
 static void sim_enable_raw_mode() {
     tcgetattr(STDIN_FILENO, &orig_termios);
     struct termios raw = orig_termios;
     raw.c_lflag &= ~(ECHO | ICANON); // Disable echo and canonical mode
     tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
     
     // Set stdin to non-blocking
     int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
     fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
 }
 
 static void sim_disable_raw_mode() {
     tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
     
     // Restore blocking mode
     int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
     fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
 }
 
 /**
  * Set simulated CAN values directly - this function is called from main.c
  * 
  * @param enable Enable button status (0/1)
  * @param x_axis X-axis value (0-255)
  * @param y_axis Y-axis value (0-255)
  * @param horn Horn button status (0/1)
  * @param estop Emergency stop status (0/1)
  * @param speed Speed button status (0/1)
  */
 void set_simulated_values(int enable, int x_axis, int y_axis, int horn, int estop, int speed) {
     pthread_mutex_lock(&can_sim.mutex);
     
     can_sim.enable_button = enable;
     can_sim.x_axis = x_axis;
     can_sim.y_axis = y_axis;
     can_sim.horn_button = horn;
     can_sim.estop_button = estop;
     can_sim.speed_button = speed;
     
     pthread_mutex_unlock(&can_sim.mutex);
 }
 
 /**
  * Initialize the CAN simulator - this function is called from main.c
  * 
  * @return 1 on success, 0 on failure
  */
 int init_can_simulator(void) {
     return initialize_can_bus("sim0");
 }
 
 /**
  * Stop the CAN simulator - this function is called from main.c
  */
 void stop_can_simulator(void) {
     shutdown_can_bus();
 }
 
 // Function to read keyboard without blocking
 static int sim_kbhit() {
     char ch;
     int nread = read(STDIN_FILENO, &ch, 1);
     if (nread == 1) {
         ungetc(ch, stdin);
         return 1;
     }
     return 0;
 }
 
 static char sim_readch() {
     char ch;
     read(STDIN_FILENO, &ch, 1);
     return ch;
 }
 
 // Print current controller state
 static void print_simulator_status() {
     pthread_mutex_lock(&can_sim.mutex);
     
     printf("\033[2J\033[H"); // Clear screen and move cursor to top
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
 static void *springback_thread(void *arg) {
     (void)arg; // Unused parameter
     
     struct timeval now;
     long diff_ms_x, diff_ms_y;
     const int SPRING_TIMEOUT_MS = 200; // Time without keypresses to trigger spring-back
     const int SPRING_STEP = 10;        // How quickly to return to center
     
     while (keep_running) {
         pthread_mutex_lock(&can_sim.mutex);
         
         gettimeofday(&now, NULL);
         
         // Calculate time since last X press
         diff_ms_x = ((now.tv_sec - can_sim.last_x_press.tv_sec) * 1000) + 
                    ((now.tv_usec - can_sim.last_x_press.tv_usec) / 1000);
                    
         // Calculate time since last Y press
         diff_ms_y = ((now.tv_sec - can_sim.last_y_press.tv_sec) * 1000) + 
                    ((now.tv_usec - can_sim.last_y_press.tv_usec) / 1000);
         
         // Spring back X-axis
         if (diff_ms_x > SPRING_TIMEOUT_MS && can_sim.x_axis != 128) {
             if (can_sim.x_axis < 128) {
                 can_sim.x_axis += SPRING_STEP;
                 if (can_sim.x_axis > 128) can_sim.x_axis = 128;
             } else {
                 can_sim.x_axis -= SPRING_STEP;
                 if (can_sim.x_axis < 128) can_sim.x_axis = 128;
             }
             can_sim.x_direction = 0; // Reset direction once centered
         }
         
         // Spring back Y-axis
         if (diff_ms_y > SPRING_TIMEOUT_MS && can_sim.y_axis != 128) {
             if (can_sim.y_axis < 128) {
                 can_sim.y_axis += SPRING_STEP;
                 if (can_sim.y_axis > 128) can_sim.y_axis = 128;
             } else {
                 can_sim.y_axis -= SPRING_STEP;
                 if (can_sim.y_axis < 128) can_sim.y_axis = 128;
             }
             can_sim.y_direction = 0; // Reset direction once centered
         }
         
         pthread_mutex_unlock(&can_sim.mutex);
         
         // Sleep to reduce CPU usage
         usleep(50000); // 50ms
     }
     
     return NULL;
 }
 
 // Keyboard input thread for simulation control
 static void *keyboard_input_thread(void *arg) {
     (void)arg; // Unused
     
     sim_enable_raw_mode();
     printf("CAN Simulator active. Press 'P' for controls, 'Q' to exit.\n");
     
     // Initialize last press time
     gettimeofday(&can_sim.last_x_press, NULL);
     gettimeofday(&can_sim.last_y_press, NULL);
     
     while (keep_running) {
         if (sim_kbhit()) {
             char c = sim_readch();
             
             pthread_mutex_lock(&can_sim.mutex);
             
             switch (c) {
                 case '1': // Toggle Enable button
                     can_sim.enable_button = !can_sim.enable_button;
                     printf("Enable button: %s\n", can_sim.enable_button ? "ON" : "OFF");
                     break;
                     
                 case '2': // Toggle Speed button
                     can_sim.speed_button = !can_sim.speed_button;
                     printf("Speed button: %s\n", can_sim.speed_button ? "ON" : "OFF");
                     break;
                     
                 case '3': // Toggle Horn button
                     can_sim.horn_button = !can_sim.horn_button;
                     printf("Horn button: %s\n", can_sim.horn_button ? "ON" : "OFF");
                     break;
                     
                 case '4': // Toggle CAN Enable button
                     can_sim.can_enable_button = !can_sim.can_enable_button;
                     printf("CAN Enable button: %s\n", can_sim.can_enable_button ? "ON" : "OFF");
                     break;
                     
                 case '5': // Toggle Emergency stop
                     can_sim.estop_button = !can_sim.estop_button;
                     printf("Emergency Stop: %s\n", can_sim.estop_button ? "ON" : "OFF");
                     break;
                     
                 case '6': // Center joystick
                     can_sim.x_axis = 128;
                     can_sim.y_axis = 128;
                     printf("Joystick centered\n");
                     break;
                     
                 case 27: // Arrow keys (ESC [ A/B/C/D sequence)
                     if (sim_readch() == '[') {
                         char dir = sim_readch();
                         
                         switch (dir) {
                             case 'A': // Up - Increase Y axis (forward)
                                 can_sim.y_axis = (can_sim.y_axis <= 235) ? can_sim.y_axis + 20 : 255;
                                 can_sim.y_direction = 1;
                                 gettimeofday(&can_sim.last_y_press, NULL);
                                 break;
                                 
                             case 'B': // Down - Decrease Y axis (backward)
                                 can_sim.y_axis = (can_sim.y_axis >= 20) ? can_sim.y_axis - 20 : 0;
                                 can_sim.y_direction = -1;
                                 gettimeofday(&can_sim.last_y_press, NULL);
                                 break;
                                 
                             case 'C': // Right - Increase X axis
                                 can_sim.x_axis = (can_sim.x_axis <= 235) ? can_sim.x_axis + 20 : 255;
                                 can_sim.x_direction = 1;
                                 gettimeofday(&can_sim.last_x_press, NULL);
                                 break;
                                 
                             case 'D': // Left - Decrease X axis
                                 can_sim.x_axis = (can_sim.x_axis >= 20) ? can_sim.x_axis - 20 : 0;
                                 can_sim.x_direction = -1;
                                 gettimeofday(&can_sim.last_x_press, NULL);
                                 break;
                         }
                     }
                     break;
                     
                 case 'p': // Print status
                 case 'P':
                     pthread_mutex_unlock(&can_sim.mutex);
                     print_simulator_status();
                     pthread_mutex_lock(&can_sim.mutex);
                     break;
             }
             
             pthread_mutex_unlock(&can_sim.mutex);
         }
         
         // Sleep to avoid excessive CPU usage
         usleep(50000); // 50ms
     }
     
     sim_disable_raw_mode();
     return NULL;
 }
 
 // Implementation of socketcan.h functions with simulation
 
 /**
  * Initialize the CAN bus
  */
 int initialize_can_bus(const char *interface) {
     printf("Initializing simulated CAN bus on interface: %s\n", interface);
     
     // Initialize the simulator data structure
     memset(&can_sim, 0, sizeof(CANSimulator));
     pthread_mutex_init(&can_sim.mutex, NULL);
     
     // Set default values
     can_sim.enable_button = 1;      // Default to enabled
     can_sim.speed_button = 0;
     can_sim.horn_button = 0;
     can_sim.can_enable_button = 1;
     can_sim.estop_button = 1;       // Set to 1 to work with the motor_control.c logic
     can_sim.x_axis = 128;           // Center position
     can_sim.y_axis = 128;           // Center position
     
     // Initialize spring-back tracking
     gettimeofday(&can_sim.last_x_press, NULL);
     gettimeofday(&can_sim.last_y_press, NULL);
     can_sim.x_direction = 0;
     can_sim.y_direction = 0;
     
     // Create keyboard input thread
     keep_running = 1;
     if (pthread_create(&can_sim.keyboard_thread_id, NULL, keyboard_input_thread, NULL) != 0) {
         printf("Failed to create keyboard input thread\n");
         pthread_mutex_destroy(&can_sim.mutex);
         return 0;
     }
     
     // Create spring-back thread
     if (pthread_create(&can_sim.springback_thread_id, NULL, springback_thread, NULL) != 0) {
         printf("Failed to create springback thread\n");
         pthread_cancel(can_sim.keyboard_thread_id);
         pthread_join(can_sim.keyboard_thread_id, NULL);
         pthread_mutex_destroy(&can_sim.mutex);
         return 0;
     }
     
     can_sim.simulation_active = 1;
     
     return 1;
 }
 
 /**
  * Shutdown the CAN bus
  */
 void shutdown_can_bus(void) {
     if (!can_sim.simulation_active) {
         return;
     }
     
     printf("Shutting down simulated CAN bus\n");
     
     keep_running = 0;
     usleep(300000); // 300ms wait
     
     pthread_cancel(can_sim.keyboard_thread_id);
     pthread_join(can_sim.keyboard_thread_id, NULL);
     
     pthread_cancel(can_sim.springback_thread_id);
     pthread_join(can_sim.springback_thread_id, NULL);
     
     pthread_mutex_destroy(&can_sim.mutex);
     can_sim.simulation_active = 0;
 }
 
 /**
  * Send a CAN message with the specified ID and data
  */
 int send_can_message(int can_id, uint8_t *data, int data_length) {
     (void)data; // Prevent unused parameter warning
     (void)can_id;
     (void)data_length;
     // Don't print anything to avoid cluttering the terminal
     return 1;
 }
 
 /**
  * Receive a CAN message with the specified ID
  */
 int receive_can_message(int can_id, uint8_t *data, int *data_length, int timeout_ms) {
     (void)can_id;    // Prevent unused parameter warning
     (void)timeout_ms; // Prevent unused parameter warning
     
     // This is more complex to simulate properly, for now just return success
     memset(data, 0, 8);
     *data_length = 8;
     return 1;
 }
 
 /**
  * Get the enable button status
  */
 int get_enable_button(void) {
     int result;
     pthread_mutex_lock(&can_sim.mutex);
     result = can_sim.enable_button;
     pthread_mutex_unlock(&can_sim.mutex);
     return result;
 }
 
 /**
  * Get the speed button status
  */
 int get_speed_button(void) {
     int result;
     pthread_mutex_lock(&can_sim.mutex);
     result = can_sim.speed_button;
     pthread_mutex_unlock(&can_sim.mutex);
     return result;
 }
 
 /**
  * Get the horn button status
  */
 int get_horn_button(void) {
     int result;
     pthread_mutex_lock(&can_sim.mutex);
     result = can_sim.horn_button;
     pthread_mutex_unlock(&can_sim.mutex);
     return result;
 }
 
 /**
  * Get the CAN enable button status
  */
 int get_can_enable_button(void) {
     int result;
     pthread_mutex_lock(&can_sim.mutex);
     result = can_sim.can_enable_button;
     pthread_mutex_unlock(&can_sim.mutex);
     return result;
 }
 
 /**
  * Get the emergency stop button status
  */
 int get_estop_button(void) {
     int result;
     pthread_mutex_lock(&can_sim.mutex);
     result = can_sim.estop_button;
     pthread_mutex_unlock(&can_sim.mutex);
     return result;
 }
 
 /**
  * Get the X-axis value
  */
 int get_x_axis(void) {
     int result;
     pthread_mutex_lock(&can_sim.mutex);
     result = can_sim.x_axis;
     pthread_mutex_unlock(&can_sim.mutex);
     return result;
 }
 
 /**
  * Get the Y-axis value
  */
 int get_y_axis(void) {
     int result;
     pthread_mutex_lock(&can_sim.mutex);
     result = can_sim.y_axis;
     pthread_mutex_unlock(&can_sim.mutex);
     return result;
 }
 
 /**
  * Set the yellow battery LED state
  */
 int set_yellow_bat_led(int state) {
     (void)state;
     // Don't print to avoid cluttering the terminal
     return 1;
 }
 
 /**
  * Set the red battery LED state
  */
 int set_red_bat_led(int state) {
     (void)state;
     // Don't print to avoid cluttering the terminal
     return 1;
 }
 
 /**
  * Set the overload LED state
  */
 int set_overload_led(int state) {
     (void)state;
     // Don't print to avoid cluttering the terminal
     return 1;
 }
 
 /**
  * Set the auxiliary LED state
  */
 int set_aux_led(int state) {
     (void)state;
     // Don't print to avoid cluttering the terminal
     return 1;
 }