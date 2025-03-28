#include "step_counter.h"
#include <math.h>
#include <stdlib.h>

#define PI 3.141592653589793f

typedef struct {
    StepCounterConfig config;
    
    // Filter buffers
    float *x_history;
    float *y_history;
    float *z_history;
    uint8_t filter_index;
    
    // Step detection state
    uint32_t step_count;
    uint32_t last_step_time;
    float dynamic_threshold;
    float running_variance;
    int16_t baseline[3];
} StepCounterState;

static StepCounterState state;

// Low-pass filter coefficients
static float alpha = 0.0f;

static void update_lowpass_coefficients(void) {
    float dt = 1.0f / state.config.sample_rate_hz;  // Now uses the config value
    float rc = 1.0f / (2 * PI * state.config.frequency_cutoff_hz);
    alpha = dt / (rc + dt);
}

void step_counter_init(StepCounterConfig config) {
    state.config = config;
    
    // Allocate filter buffers
    state.x_history = malloc(config.window_size * sizeof(float));
    state.y_history = malloc(config.window_size * sizeof(float));
    state.z_history = malloc(config.window_size * sizeof(float));
    memset(state.x_history, 0, config.window_size * sizeof(float));
    memset(state.y_history, 0, config.window_size * sizeof(float));
    memset(state.z_history, 0, config.window_size * sizeof(float));
    
    // Initialize state
    state.filter_index = 0;
    state.step_count = 0;
    state.last_step_time = 0;
    state.dynamic_threshold = (config.min_step_threshold + config.max_step_threshold) / 2.0f;
    state.running_variance = 0;
    state.baseline[0] = state.baseline[1] = state.baseline[2] = 0;
    
    update_lowpass_coefficients();
}

void step_counter_update(int16_t x, int16_t y, int16_t z) {
    // 1. Apply moving average filter
    state.x_history[state.filter_index] = x;
    state.y_history[state.filter_index] = y;
    state.z_history[state.filter_index] = z;
    state.filter_index = (state.filter_index + 1) % state.config.window_size;
    
    float x_filt = 0, y_filt = 0, z_filt = 0;
    for (int i = 0; i < state.config.window_size; i++) {
        x_filt += state.x_history[i];
        y_filt += state.y_history[i];
        z_filt += state.z_history[i];
    }
    x_filt /= state.config.window_size;
    y_filt /= state.config.window_size;
    z_filt /= state.config.window_size;
    
    // 2. Apply low-pass filter to baseline
    state.baseline[0] = alpha * x_filt + (1 - alpha) * state.baseline[0];
    state.baseline[1] = alpha * y_filt + (1 - alpha) * state.baseline[1];
    state.baseline[2] = alpha * z_filt + (1 - alpha) * state.baseline[2];
    
    // 3. Calculate vector magnitude of filtered signal minus baseline
    float dx = x_filt - state.baseline[0];
    float dy = y_filt - state.baseline[1];
    float dz = z_filt - state.baseline[2];
    float magnitude = sqrtf(dx*dx + dy*dy + dz*dz);
    
    // 4. Dynamic threshold adjustment
    float error = magnitude - state.dynamic_threshold;
    state.running_variance = 0.98f * state.running_variance + 0.02f * error*error;
    state.dynamic_threshold += 0.05f * error;
    
    // Constrain threshold
    if (state.dynamic_threshold < state.config.min_step_threshold) {
        state.dynamic_threshold = state.config.min_step_threshold;
    } else if (state.dynamic_threshold > state.config.max_step_threshold) {
        state.dynamic_threshold = state.config.max_step_threshold;
    }
    
    // 5. Step detection logic
    static uint8_t state_machine = 0;
    static uint32_t last_peak_time = 0;
    static float last_valley = 0;
    
    switch(state_machine) {
        case 0: // Looking for upward crossing
            if (magnitude > state.dynamic_threshold) {
                state_machine = 1;
            }
            break;
            
        case 1: // Looking for peak
            if (magnitude < state.dynamic_threshold) {
                state_machine = 0;
            } else if (magnitude > state.dynamic_threshold * 1.2f) {
                state_machine = 2;
                last_peak_time = HAL_GetTick();
            }
            break;
            
        case 2: // Looking for downward crossing
            if (magnitude < state.dynamic_threshold) {
                state_machine = 3;
                last_valley = magnitude;
            }
            break;
            
        case 3: // Verify step
            if (magnitude < last_valley) {
                last_valley = magnitude;
            } else if (HAL_GetTick() - last_peak_time > state.config.step_delay_ms) {
                if (magnitude > state.dynamic_threshold * 0.7f) {
                    state.step_count++;
                    state_machine = 0;
                }
            }
            break;
    }
}

uint32_t step_counter_get_count(void) {
    return state.step_count;
}

void step_counter_reset(void) {
    state.step_count = 0;
}