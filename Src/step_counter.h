#ifndef STEP_COUNTER_H
#define STEP_COUNTER_H

#include <stdint.h>

typedef struct {
    uint16_t min_step_threshold;  // Minimum acceleration change to consider
    uint16_t max_step_threshold;  // Maximum allowed acceleration change
    uint8_t window_size;          // Moving average window size
    uint16_t step_delay_ms;       // Minimum time between steps
    float frequency_cutoff_hz;    // Low-pass filter cutoff frequency
    uint16_t sample_rate_hz; 
} StepCounterConfig;

void step_counter_init(StepCounterConfig config);
void step_counter_update(int16_t x, int16_t y, int16_t z);
uint32_t step_counter_get_count(void);
void step_counter_reset(void);

#endif