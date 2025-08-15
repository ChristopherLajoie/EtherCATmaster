#ifndef PERFORMANCE_MONITOR_H
#define PERFORMANCE_MONITOR_H

#include <stdint.h>
#include <time.h>

typedef struct
{
    uint64_t total_cycles;
    uint64_t bad_wkc_cycles;
    uint64_t missed_cycles;

    int64_t cycle_time_target_ns;
    int64_t cycle_time_last_ns;
    int64_t cycle_time_min_ns;
    int64_t cycle_time_max_ns;
    double cycle_time_avg_ns;

    int64_t jitter_last_ns;
    int64_t jitter_min_ns;
    int64_t jitter_max_ns;
    double jitter_avg_ns;

    int64_t rtt_last_ns;
    int64_t rtt_min_ns;
    int64_t rtt_max_ns;
    double rtt_avg_ns;

    struct timespec last_cycle_start_ts;
    struct timespec send_ts;

} performance_metrics_t;

void perf_init(performance_metrics_t *metrics, int target_cycletime_us);
void perf_cycle_start(performance_metrics_t *metrics);
void perf_send_complete(performance_metrics_t *metrics);
void perf_receive_complete(performance_metrics_t *metrics, int wkc, int expected_wkc);
void perf_log_output(const performance_metrics_t *metrics);

#endif