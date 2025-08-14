#include "performance_monitor.h"
#include <stdio.h>
#include <string.h>
#include <limits.h>

// Helper function to calculate the difference between two timespecs in nanoseconds
static int64_t timespec_diff_ns(struct timespec start, struct timespec end)
{
    return (end.tv_sec - start.tv_sec) * 1000000000LL + (end.tv_nsec - start.tv_nsec);
}

// Helper to update min/max/avg statistics for a given metric
static void update_stats(int64_t sample, int64_t* min_val, int64_t* max_val, double* avg_val, uint64_t count)
{
    if (sample < *min_val) *min_val = sample;
    if (sample > *max_val) *max_val = sample;
    // Simple running average
    *avg_val += (sample - *avg_val) / count;
}


void perf_init(performance_metrics_t* metrics, int target_cycletime_us)
{
    memset(metrics, 0, sizeof(performance_metrics_t));

    metrics->cycle_time_target_ns = target_cycletime_us * 1000LL;
    
    // Initialize min/max with sentinel values
    metrics->cycle_time_min_ns = LLONG_MAX;
    metrics->cycle_time_max_ns = LLONG_MIN;
    metrics->jitter_min_ns = LLONG_MAX;
    metrics->jitter_max_ns = LLONG_MIN;
    metrics->rtt_min_ns = LLONG_MAX;
    metrics->rtt_max_ns = LLONG_MIN;
}

void perf_cycle_start(performance_metrics_t* metrics)
{
    struct timespec current_ts;
    clock_gettime(CLOCK_MONOTONIC, &current_ts);

    // Only calculate stats after the first cycle
    if (metrics->total_cycles > 0)
    {
        // 1. Actual Cycle Time
        metrics->cycle_time_last_ns = timespec_diff_ns(metrics->last_cycle_start_ts, current_ts);
        update_stats(metrics->cycle_time_last_ns, &metrics->cycle_time_min_ns, &metrics->cycle_time_max_ns, &metrics->cycle_time_avg_ns, metrics->total_cycles);

        // 2. Cycle Jitter vs Target
        metrics->jitter_last_ns = metrics->cycle_time_last_ns - metrics->cycle_time_target_ns;
        update_stats(metrics->jitter_last_ns, &metrics->jitter_min_ns, &metrics->jitter_max_ns, &metrics->jitter_avg_ns, metrics->total_cycles);
    }
    
    metrics->last_cycle_start_ts = current_ts;
    metrics->total_cycles++;
}

void perf_send_complete(performance_metrics_t* metrics)
{
    // Record the time the send function finished
    clock_gettime(CLOCK_MONOTONIC, &metrics->send_ts);
}

void perf_receive_complete(performance_metrics_t* metrics, int wkc, int expected_wkc)
{
    // This function is called after ec_receive_processdata(), which receives the
    // frame sent in the *previous* cycle. So we calculate RTT against the previous send time.
    if (metrics->total_cycles > 1) { // RTT is only valid after the first send/receive pair
        struct timespec receive_ts;
        clock_gettime(CLOCK_MONOTONIC, &receive_ts);
        metrics->rtt_last_ns = timespec_diff_ns(metrics->send_ts, receive_ts);
        update_stats(metrics->rtt_last_ns, &metrics->rtt_min_ns, &metrics->rtt_max_ns, &metrics->rtt_avg_ns, metrics->total_cycles - 1);
    }

    // 4. Missed/Late Cycles (Timeout / Bad WKC)
    if (wkc <= 0) {
        metrics->missed_cycles++;
    } else if (wkc < expected_wkc) {
        metrics->bad_wkc_cycles++;
    }
}

void perf_log_output(const performance_metrics_t* metrics)
{
    if (metrics->total_cycles < 2) {
        printf("Collecting performance data...\n");
        return;
    }

    printf("--- EtherCAT Master Performance ---\n");
    printf("  Cycles: Total=%-10llu Missed (Timeout)=%-5llu Bad WKC=%-5llu\n", 
           metrics->total_cycles, metrics->missed_cycles, metrics->bad_wkc_cycles);

    printf("  Cycle Time (us): Target=%-7.1f Last=%-7.1f Avg=%-7.1f Min=%-7.1f Max=%-7.1f\n",
           metrics->cycle_time_target_ns / 1000.0,
           metrics->cycle_time_last_ns / 1000.0,
           metrics->cycle_time_avg_ns / 1000.0,
           metrics->cycle_time_min_ns / 1000.0,
           metrics->cycle_time_max_ns / 1000.0);
           
    printf("  Jitter (us):     Last=%-7.1f Avg=%-7.1f Min=%-7.1f Max=%-7.1f\n",
           metrics->jitter_last_ns / 1000.0,
           metrics->jitter_avg_ns / 1000.0,
           metrics->jitter_min_ns / 1000.0,
           metrics->jitter_max_ns / 1000.0);

    printf("  Frame RTT (us):  Last=%-7.1f Avg=%-7.1f Min=%-7.1f Max=%-7.1f\n",
           metrics->rtt_last_ns / 1000.0,
           metrics->rtt_avg_ns / 1000.0,
           metrics->rtt_min_ns / 1000.0,
           metrics->rtt_max_ns / 1000.0);
    printf("-----------------------------------\n");
}