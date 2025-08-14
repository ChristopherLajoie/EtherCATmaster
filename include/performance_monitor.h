#ifndef PERFORMANCE_MONITOR_H
#define PERFORMANCE_MONITOR_H

#include <stdint.h>
#include <time.h>

// A structure to hold all performance metrics
typedef struct
{
    // Cycle counters
    uint64_t total_cycles;
    uint64_t bad_wkc_cycles; // Working counter was less than expected
    uint64_t missed_cycles;  // Receive timed out (wkc <= 0)

    // Cycle Time (time from one cycle start to the next)
    int64_t cycle_time_target_ns;
    int64_t cycle_time_last_ns;
    int64_t cycle_time_min_ns;
    int64_t cycle_time_max_ns;
    double  cycle_time_avg_ns;

    // Jitter (deviation from target cycle time)
    int64_t jitter_last_ns;
    int64_t jitter_min_ns;
    int64_t jitter_max_ns;
    double  jitter_avg_ns;

    // Frame RTT (Send -> Receive turnaround time)
    int64_t rtt_last_ns;
    int64_t rtt_min_ns;
    int64_t rtt_max_ns;
    double  rtt_avg_ns;

    // Internal timestamps for calculations
    struct timespec last_cycle_start_ts;
    struct timespec send_ts;

} performance_metrics_t;


/**
 * @brief Initializes the performance metrics structure to default values.
 * @param metrics Pointer to the performance_metrics_t struct.
 * @param target_cycletime_us The target cycle time in microseconds.
 */
void perf_init(performance_metrics_t* metrics, int target_cycletime_us);

/**
 * @brief Marks the start of a new cycle for timing calculations.
 * @param metrics Pointer to the performance_metrics_t struct.
 */
void perf_cycle_start(performance_metrics_t* metrics);

/**
 * @brief Records the timestamp just before sending a process data frame.
 * @param metrics Pointer to the performance_metrics_t struct.
 */
void perf_send_complete(performance_metrics_t* metrics);

/**
 * @brief Processes the result of ec_receive_processdata to calculate RTT and count errors.
 * @param metrics Pointer to the performance_metrics_t struct.
 * @param wkc The working counter returned by ec_receive_processdata.
 * @param expected_wkc The expected working counter for the slave group.
 */
void perf_receive_complete(performance_metrics_t* metrics, int wkc, int expected_wkc);

/**
 * @brief Prints a formatted summary of the collected performance metrics.
 * @param metrics Pointer to the performance_metrics_t struct.
 */
void perf_log_output(const performance_metrics_t* metrics);


#endif // PERFORMANCE_MONITOR_H