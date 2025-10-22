#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <semaphore.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <sched.h>
#include <math.h>
#include <stdbool.h>

// Function declarations
unsigned long get_current_time_ms(void);

// Kalman Filter Structure
typedef struct {
    float estimate;           // Current estimate
    float error_estimate;     // Current error estimate
    float process_variance;   // Process noise variance
    float measurement_variance; // Measurement noise variance
} kalman_filter_t;

// Asymmetric Cooldown Constants
#define STRICT_COOLDOWN_MS 200      // 200ms minimum between changes
#define UP_COOLDOWN_MS 3000         // 3s before increasing bitrate
#define MIN_CHANGE_PERCENT 5        // 5% minimum change threshold
#define EMERGENCY_RSSI_THRESHOLD 30 // Emergency drop threshold
#define EMERGENCY_BITRATE 1000      // Emergency bitrate (kbps)

// Filter Types
typedef enum {
    FILTER_TYPE_KALMAN = 0,
    FILTER_TYPE_LOWPASS = 1,
    FILTER_TYPE_MODE = 2,
    FILTER_TYPE_DERIVATIVE = 3,
    FILTER_TYPE_2POLE_LPF = 4
} filter_type_t;

// ArduPilot-style Low-Pass Filter Structure
typedef struct {
    float output;           // Current filtered output
    float alpha;           // Filter coefficient (0-1)
    bool initialised;      // Initialization flag
    float cutoff_freq;     // Cutoff frequency (Hz)
    float sample_freq;     // Sample frequency (Hz)
} lowpass_filter_t;

// Mode Filter Structure (ArduPilot style - median-like with alternating drop)
#define MODE_FILTER_SIZE 5
typedef struct {
    float samples[MODE_FILTER_SIZE];
    uint8_t sample_index;
    uint8_t return_element;
    bool drop_high_sample;
    float output;
} mode_filter_t;

// Derivative Filter Structure (for trend detection)
#define DERIVATIVE_FILTER_SIZE 5
typedef struct {
    float samples[DERIVATIVE_FILTER_SIZE];
    uint32_t timestamps[DERIVATIVE_FILTER_SIZE];
    uint8_t sample_index;
    float last_slope;
    bool new_data;
} derivative_filter_t;

// 2-Pole Low-Pass Filter Structure (biquad filter)
typedef struct {
    float delay_element_1;
    float delay_element_2;
    float cutoff_freq;
    float sample_freq;
    bool initialised;
    float output;
} lpf_2pole_t;

// Filter Chain Structure - allows multiple filters in sequence
#define MAX_FILTERS_PER_CHAIN 3
typedef struct {
    filter_type_t filters[MAX_FILTERS_PER_CHAIN];
    uint8_t filter_count;
    bool enabled;
} filter_chain_t;

// Filter Chain Instances
static filter_chain_t rssi_filter_chain = {
    .filters = {FILTER_TYPE_KALMAN, FILTER_TYPE_KALMAN, FILTER_TYPE_KALMAN},
    .filter_count = 1,
    .enabled = true
};

static filter_chain_t dbm_filter_chain = {
    .filters = {FILTER_TYPE_KALMAN, FILTER_TYPE_KALMAN, FILTER_TYPE_KALMAN},
    .filter_count = 1,
    .enabled = true
};

// Function declarations for filter operations
float kalman_filter_update(kalman_filter_t *filter, float measurement);
float lowpass_filter_apply(lowpass_filter_t *filter, float sample);
float mode_filter_apply(mode_filter_t *filter, float sample);
float derivative_filter_apply(derivative_filter_t *filter, float sample);
float lpf_2pole_apply(lpf_2pole_t *filter, float sample);
float apply_filter_chain(filter_chain_t *chain, float sample);
void parse_filter_chain(const char *config_str, filter_chain_t *chain);
void reset_filter_chain(filter_chain_t *chain);

// Global Kalman filters for different signals
static kalman_filter_t rssi_filter = {
    .estimate = 50.0f,           // Initial RSSI estimate (50%)
    .error_estimate = 1.0f,      // Initial error estimate
    .process_variance = 1e-5f,   // Process noise (small for stable signals)
    .measurement_variance = 0.1f  // Measurement noise
};

static kalman_filter_t dbm_filter = {
    .estimate = -60.0f,          // Initial dBm estimate
    .error_estimate = 1.0f,      // Initial error estimate
    .process_variance = 1e-5f,   // Process noise
    .measurement_variance = 0.5f  // Measurement noise (higher for dBm)
};

// Global Low-Pass Filter instances
static lowpass_filter_t rssi_lpf = {
    .output = 50.0f,             // Initial RSSI estimate
    .alpha = 0.1f,              // Default filter coefficient
    .initialised = false,        // Not initialized yet
    .cutoff_freq = 2.0f,         // 2Hz cutoff frequency
    .sample_freq = 10.0f         // 10Hz sample frequency
};

static lowpass_filter_t dbm_lpf = {
    .output = -60.0f,            // Initial dBm estimate
    .alpha = 0.1f,              // Default filter coefficient
    .initialised = false,        // Not initialized yet
    .cutoff_freq = 2.0f,         // 2Hz cutoff frequency
    .sample_freq = 10.0f         // 10Hz sample frequency
};

// Global Mode Filter instances
static mode_filter_t rssi_mode_filter = {
    .sample_index = 0,
    .return_element = 2,         // Return median (middle element)
    .drop_high_sample = true,
    .output = 50.0f
};

static mode_filter_t dbm_mode_filter = {
    .sample_index = 0,
    .return_element = 2,         // Return median (middle element)
    .drop_high_sample = true,
    .output = -60.0f
};

// Global Derivative Filter instances
static derivative_filter_t rssi_derivative_filter = {
    .sample_index = 0,
    .last_slope = 0.0f,
    .new_data = false
};

static derivative_filter_t dbm_derivative_filter = {
    .sample_index = 0,
    .last_slope = 0.0f,
    .new_data = false
};

// Global 2-Pole Low-Pass Filter instances
static lpf_2pole_t rssi_2pole_lpf = {
    .delay_element_1 = 0.0f,
    .delay_element_2 = 0.0f,
    .cutoff_freq = 2.0f,
    .sample_freq = 10.0f,
    .initialised = false,
    .output = 50.0f
};

static lpf_2pole_t dbm_2pole_lpf = {
    .delay_element_1 = 0.0f,
    .delay_element_2 = 0.0f,
    .cutoff_freq = 2.0f,
    .sample_freq = 10.0f,
    .initialised = false,
    .output = -60.0f
};

// Asymmetric Cooldown Timing Variables
static unsigned long last_change_time = 0;
static unsigned long last_up_time = 0;
static int last_bitrate = 0;

// PID Controller Structure
typedef struct {
    float kp;           // Proportional gain
    float ki;           // Integral gain
    float kd;           // Derivative gain
    float integral;     // Integral accumulator
    int last_error;     // Previous error for derivative calculation
    int last_output;    // Previous output for reference
} pid_controller_t;

// Global PID controller instance
static pid_controller_t bitrate_pid = {
    .kp = 1.0f,         // Proportional gain (aggressive response)
    .ki = 0.05f,        // Integral gain (eliminate steady-state error)
    .kd = 0.4f,         // Derivative gain (reduce overshoot)
    .integral = 0.0f,   // Start with no integral accumulation
    .last_error = 0,    // No previous error
    .last_output = 0    // No previous output
};

// ArduPilot-style Low-Pass Filter Functions
// Calculate alpha coefficient for low-pass filter
float calc_lowpass_alpha_dt(float dt, float cutoff_freq) {
    if (cutoff_freq <= 0.0f || dt <= 0.0f) {
        return 1.0f;  // No filtering
    }
    
    float rc = 1.0f / (2.0f * M_PI * cutoff_freq);
    return dt / (rc + dt);
}

// Initialize low-pass filter
void init_lowpass_filter(lowpass_filter_t *filter, float cutoff_freq, float sample_freq) {
    filter->cutoff_freq = cutoff_freq;
    filter->sample_freq = sample_freq;
    filter->alpha = calc_lowpass_alpha_dt(1.0f / sample_freq, cutoff_freq);
    filter->initialised = false;
    printf("Low-pass filter initialized: cutoff=%.1fHz, sample=%.1fHz, alpha=%.3f\n", 
           cutoff_freq, sample_freq, filter->alpha);
}

// Apply low-pass filter (ArduPilot style)
float lowpass_filter_apply(lowpass_filter_t *filter, float sample) {
    if (!filter->initialised) {
        filter->output = sample;
        filter->initialised = true;
    } else {
        filter->output += (sample - filter->output) * filter->alpha;
    }
    return filter->output;
}

// Reset low-pass filter
void reset_lowpass_filter(lowpass_filter_t *filter) {
    filter->initialised = false;
    printf("Low-pass filter reset\n");
}

// Mode Filter Functions (ArduPilot style)
// Insertion sort for mode filter (alternates dropping high/low samples)
void mode_filter_insert(mode_filter_t *filter, float sample) {
    uint8_t i;
    
    // If buffer isn't full, simply increase sample count
    if (filter->sample_index < MODE_FILTER_SIZE) {
        filter->sample_index++;
        filter->drop_high_sample = true;  // Default to dropping high
    }
    
    if (filter->drop_high_sample) {
        // Drop highest sample - start from top
        i = filter->sample_index - 1;
        
        // Shift samples down to make room
        while (i > 0 && filter->samples[i-1] > sample) {
            filter->samples[i] = filter->samples[i-1];
            i--;
        }
        
        // Insert new sample
        filter->samples[i] = sample;
    } else {
        // Drop lowest sample - start from bottom
        i = 0;
        
        // Shift samples up to make room
        while (i < filter->sample_index - 1 && filter->samples[i+1] < sample) {
            filter->samples[i] = filter->samples[i+1];
            i++;
        }
        
        // Insert new sample
        filter->samples[i] = sample;
    }
    
    // Alternate drop direction for next sample
    filter->drop_high_sample = !filter->drop_high_sample;
}

// Apply mode filter
float mode_filter_apply(mode_filter_t *filter, float sample) {
    mode_filter_insert(filter, sample);
    
    if (filter->sample_index < MODE_FILTER_SIZE) {
        // Buffer not full - return middle sample
        filter->output = filter->samples[filter->sample_index / 2];
    } else {
        // Buffer full - return specified element (usually median)
        filter->output = filter->samples[filter->return_element];
    }
    
    return filter->output;
}

// Reset mode filter
void reset_mode_filter(mode_filter_t *filter) {
    filter->sample_index = 0;
    filter->drop_high_sample = true;
    printf("Mode filter reset\n");
}

// Derivative Filter Functions (for trend detection)
// Update derivative filter with new sample and timestamp
void derivative_filter_update(derivative_filter_t *filter, float sample, uint32_t timestamp) {
    uint8_t i = filter->sample_index;
    uint8_t i1;
    
    if (i == 0) {
        i1 = DERIVATIVE_FILTER_SIZE - 1;
    } else {
        i1 = i - 1;
    }
    
    // Check if this is a new timestamp
    if (filter->timestamps[i1] == timestamp) {
        return; // Ignore duplicate timestamp
    }
    
    // Store timestamp and sample
    filter->timestamps[i] = timestamp;
    filter->samples[i] = sample;
    
    // Update sample index
    filter->sample_index = (filter->sample_index + 1) % DERIVATIVE_FILTER_SIZE;
    filter->new_data = true;
}

// Calculate derivative (slope) using Savitzky-Golay coefficients
float derivative_filter_slope(derivative_filter_t *filter) {
    if (!filter->new_data) {
        return filter->last_slope;
    }
    
    float result = 0.0f;
    
    // Use Savitzky-Golay coefficients for 5-point derivative
    // This provides smooth derivative calculation with noise reduction
    if (DERIVATIVE_FILTER_SIZE == 5) {
        // Calculate time differences and sample differences
        uint8_t idx_0 = (filter->sample_index - 1 + DERIVATIVE_FILTER_SIZE) % DERIVATIVE_FILTER_SIZE;
        uint8_t idx_1 = (filter->sample_index - 2 + DERIVATIVE_FILTER_SIZE) % DERIVATIVE_FILTER_SIZE;
        uint8_t idx_2 = (filter->sample_index - 3 + DERIVATIVE_FILTER_SIZE) % DERIVATIVE_FILTER_SIZE;
        uint8_t idx_3 = (filter->sample_index - 4 + DERIVATIVE_FILTER_SIZE) % DERIVATIVE_FILTER_SIZE;
        uint8_t idx_4 = (filter->sample_index - 5 + DERIVATIVE_FILTER_SIZE) % DERIVATIVE_FILTER_SIZE;
        
        // Check if we have enough data
        if (filter->timestamps[idx_0] == filter->timestamps[idx_1]) {
            return 0.0f; // Not enough data yet
        }
        
        // Calculate time differences
        float dt1 = (filter->timestamps[idx_0] - filter->timestamps[idx_2]) / 1000.0f; // Convert to seconds
        float dt2 = (filter->timestamps[idx_0] - filter->timestamps[idx_4]) / 1000.0f;
        
        if (dt1 <= 0.0f || dt2 <= 0.0f) {
            return 0.0f; // Invalid time differences
        }
        
        // Savitzky-Golay 5-point derivative coefficients
        result = 2.0f * 2.0f * (filter->samples[idx_1] - filter->samples[idx_3]) / dt1 +
                 4.0f * 1.0f * (filter->samples[idx_0] - filter->samples[idx_4]) / dt2;
        result /= 8.0f;
    }
    
    // Handle numerical errors
    if (isnan(result) || isinf(result)) {
        result = 0.0f;
    }
    
    filter->new_data = false;
    filter->last_slope = result;
    
    return result;
}

// Apply derivative filter (returns filtered value, not derivative)
float derivative_filter_apply(derivative_filter_t *filter, float sample) {
    uint32_t current_time = (uint32_t)(get_current_time_ms());
    derivative_filter_update(filter, sample, current_time);
    
    // Return the most recent sample (derivative is available via slope function)
    uint8_t idx = (filter->sample_index - 1 + DERIVATIVE_FILTER_SIZE) % DERIVATIVE_FILTER_SIZE;
    return filter->samples[idx];
}

// Reset derivative filter
void reset_derivative_filter(derivative_filter_t *filter) {
    filter->sample_index = 0;
    filter->last_slope = 0.0f;
    filter->new_data = false;
    printf("Derivative filter reset\n");
}

// 2-Pole Low-Pass Filter Functions (biquad filter)
// Initialize 2-pole low-pass filter
void init_2pole_lpf(lpf_2pole_t *filter, float cutoff_freq, float sample_freq) {
    filter->cutoff_freq = cutoff_freq;
    filter->sample_freq = sample_freq;
    filter->delay_element_1 = 0.0f;
    filter->delay_element_2 = 0.0f;
    filter->initialised = false;
    printf("2-Pole LPF initialized: cutoff=%.1fHz, sample=%.1fHz\n", cutoff_freq, sample_freq);
}

// Apply 2-pole low-pass filter (biquad implementation)
float lpf_2pole_apply(lpf_2pole_t *filter, float sample) {
    if (!filter->initialised) {
        filter->output = sample;
        filter->delay_element_1 = sample;
        filter->delay_element_2 = sample;
        filter->initialised = true;
        return sample;
    }
    
    // Calculate filter coefficients
    float omega = 2.0f * M_PI * filter->cutoff_freq / filter->sample_freq;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * 0.707f); // Q = 0.707 for Butterworth response
    
    // Biquad coefficients
    float b0 = (1.0f - cs) / 2.0f;
    float b1 = 1.0f - cs;
    float b2 = (1.0f - cs) / 2.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cs;
    float a2 = 1.0f - alpha;
    
    // Normalize coefficients
    b0 /= a0;
    b1 /= a0;
    b2 /= a0;
    a1 /= a0;
    a2 /= a0;
    
    // Apply biquad filter
    float output = b0 * sample + b1 * filter->delay_element_1 + b2 * filter->delay_element_2
                   - a1 * filter->delay_element_1 - a2 * filter->delay_element_2;
    
    // Update delay elements
    filter->delay_element_2 = filter->delay_element_1;
    filter->delay_element_1 = output;
    
    filter->output = output;
    return output;
}

// Reset 2-pole low-pass filter
void reset_2pole_lpf(lpf_2pole_t *filter) {
    filter->delay_element_1 = 0.0f;
    filter->delay_element_2 = 0.0f;
    filter->initialised = false;
    printf("2-Pole LPF reset\n");
}

// Filter Chain Functions
// Apply a complete filter chain to a sample
float apply_filter_chain(filter_chain_t *chain, float sample) {
    if (!chain->enabled || chain->filter_count == 0) {
        return sample;
    }
    
    float filtered_sample = sample;
    
    // Apply each filter in the chain sequentially
    for (uint8_t i = 0; i < chain->filter_count; i++) {
        filter_type_t filter_type = chain->filters[i];
        
        switch (filter_type) {
            case FILTER_TYPE_KALMAN:
                if (chain == &rssi_filter_chain) {
                    filtered_sample = kalman_filter_update(&rssi_filter, filtered_sample);
                } else {
                    filtered_sample = kalman_filter_update(&dbm_filter, filtered_sample);
                }
                break;
                
            case FILTER_TYPE_LOWPASS:
                if (chain == &rssi_filter_chain) {
                    filtered_sample = lowpass_filter_apply(&rssi_lpf, filtered_sample);
                } else {
                    filtered_sample = lowpass_filter_apply(&dbm_lpf, filtered_sample);
                }
                break;
                
            case FILTER_TYPE_MODE:
                if (chain == &rssi_filter_chain) {
                    filtered_sample = mode_filter_apply(&rssi_mode_filter, filtered_sample);
                } else {
                    filtered_sample = mode_filter_apply(&dbm_mode_filter, filtered_sample);
                }
                break;
                
            case FILTER_TYPE_DERIVATIVE:
                if (chain == &rssi_filter_chain) {
                    filtered_sample = derivative_filter_apply(&rssi_derivative_filter, filtered_sample);
                } else {
                    filtered_sample = derivative_filter_apply(&dbm_derivative_filter, filtered_sample);
                }
                break;
                
            case FILTER_TYPE_2POLE_LPF:
                if (chain == &rssi_filter_chain) {
                    filtered_sample = lpf_2pole_apply(&rssi_2pole_lpf, filtered_sample);
                } else {
                    filtered_sample = lpf_2pole_apply(&dbm_2pole_lpf, filtered_sample);
                }
                break;
                
            default:
                // Unknown filter type, pass through unchanged
                break;
        }
    }
    
    return filtered_sample;
}

// Parse filter chain configuration string (e.g., "2,0" for Mode->Kalman)
void parse_filter_chain(const char *config_str, filter_chain_t *chain) {
    if (!config_str || strlen(config_str) == 0) {
        // Default to single Kalman filter
        chain->filters[0] = FILTER_TYPE_KALMAN;
        chain->filter_count = 1;
        return;
    }
    
    char *config_copy = strdup(config_str);
    char *token = strtok(config_copy, ",");
    uint8_t count = 0;
    
    while (token != NULL && count < MAX_FILTERS_PER_CHAIN) {
        int filter_type = atoi(token);
        if (filter_type >= 0 && filter_type <= 4) {
            chain->filters[count] = (filter_type_t)filter_type;
            count++;
        }
        token = strtok(NULL, ",");
    }
    
    chain->filter_count = count;
    free(config_copy);
    
    // Print filter chain configuration
    printf("Filter chain configured: ");
    for (uint8_t i = 0; i < count; i++) {
        const char *filter_name = (chain->filters[i] == FILTER_TYPE_KALMAN) ? "Kalman" :
                                 (chain->filters[i] == FILTER_TYPE_LOWPASS) ? "Low-Pass" :
                                 (chain->filters[i] == FILTER_TYPE_MODE) ? "Mode" :
                                 (chain->filters[i] == FILTER_TYPE_DERIVATIVE) ? "Derivative" : "2-Pole LPF";
        printf("%s", filter_name);
        if (i < count - 1) printf(" -> ");
    }
    printf(" (%d filters)\n", count);
}

// Reset all filters in a chain
void reset_filter_chain(filter_chain_t *chain) {
    for (uint8_t i = 0; i < chain->filter_count; i++) {
        filter_type_t filter_type = chain->filters[i];
        
        switch (filter_type) {
            case FILTER_TYPE_KALMAN:
                if (chain == &rssi_filter_chain) {
                    rssi_filter.estimate = 50.0f;
                    rssi_filter.error_estimate = 1.0f;
                } else {
                    dbm_filter.estimate = -60.0f;
                    dbm_filter.error_estimate = 1.0f;
                }
                break;
                
            case FILTER_TYPE_LOWPASS:
                if (chain == &rssi_filter_chain) {
                    reset_lowpass_filter(&rssi_lpf);
                } else {
                    reset_lowpass_filter(&dbm_lpf);
                }
                break;
                
            case FILTER_TYPE_MODE:
                if (chain == &rssi_filter_chain) {
                    reset_mode_filter(&rssi_mode_filter);
                } else {
                    reset_mode_filter(&dbm_mode_filter);
                }
                break;
                
            case FILTER_TYPE_DERIVATIVE:
                if (chain == &rssi_filter_chain) {
                    reset_derivative_filter(&rssi_derivative_filter);
                } else {
                    reset_derivative_filter(&dbm_derivative_filter);
                }
                break;
                
            case FILTER_TYPE_2POLE_LPF:
                if (chain == &rssi_filter_chain) {
                    reset_2pole_lpf(&rssi_2pole_lpf);
                } else {
                    reset_2pole_lpf(&dbm_2pole_lpf);
                }
                break;
        }
    }
}

// Optimized Kalman filter update function
float kalman_filter_update(kalman_filter_t *filter, float measurement) {
    // Prediction step - predict next state
    float predicted_estimate = filter->estimate;
    float predicted_error = filter->error_estimate + filter->process_variance;
    
    // Update step - correct prediction with measurement
    float kalman_gain = predicted_error / (predicted_error + filter->measurement_variance);
    
    // Clamp kalman gain to prevent numerical issues
    if (kalman_gain > 1.0f) kalman_gain = 1.0f;
    if (kalman_gain < 0.0f) kalman_gain = 0.0f;
    
    filter->estimate = predicted_estimate + kalman_gain * (measurement - predicted_estimate);
    filter->error_estimate = (1.0f - kalman_gain) * predicted_error;
    
    // Prevent error estimate from becoming too small (numerical stability)
    if (filter->error_estimate < 1e-6f) {
        filter->error_estimate = 1e-6f;
    }
    
    return filter->estimate;
}

// Unified filter interface - applies appropriate filter based on type
float apply_filter(filter_type_t filter_type, void *filter, float measurement) {
    if (filter_type == FILTER_TYPE_KALMAN) {
        return kalman_filter_update((kalman_filter_t *)filter, measurement);
    } else if (filter_type == FILTER_TYPE_LOWPASS) {
        return lowpass_filter_apply((lowpass_filter_t *)filter, measurement);
    }
    return measurement; // No filtering
}

// Initialize filters with custom parameters
void init_filters(float rssi_process_var, float rssi_measure_var,
                 float dbm_process_var, float dbm_measure_var,
                 float lpf_cutoff_freq, float lpf_sample_freq) {
    
    // Initialize Kalman filters
    rssi_filter.process_variance = rssi_process_var;
    rssi_filter.measurement_variance = rssi_measure_var;
    rssi_filter.estimate = 50.0f;  // Reset to initial value
    rssi_filter.error_estimate = 1.0f;
    
    dbm_filter.process_variance = dbm_process_var;
    dbm_filter.measurement_variance = dbm_measure_var;
    dbm_filter.estimate = -60.0f;  // Reset to initial value
    dbm_filter.error_estimate = 1.0f;
    
    // Initialize Low-Pass filters
    init_lowpass_filter(&rssi_lpf, lpf_cutoff_freq, lpf_sample_freq);
    init_lowpass_filter(&dbm_lpf, lpf_cutoff_freq, lpf_sample_freq);
    
    // Initialize Mode filters (no parameters needed)
    printf("Mode filters initialized\n");
    
    // Initialize Derivative filters (no parameters needed)
    printf("Derivative filters initialized\n");
    
    // Initialize 2-Pole Low-Pass filters
    init_2pole_lpf(&rssi_2pole_lpf, lpf_cutoff_freq, lpf_sample_freq);
    init_2pole_lpf(&dbm_2pole_lpf, lpf_cutoff_freq, lpf_sample_freq);
    
    printf("All filter types initialized successfully\n");
}

// Reset all filters
void reset_filters() {
    // Reset filter chains
    reset_filter_chain(&rssi_filter_chain);
    reset_filter_chain(&dbm_filter_chain);
    
    printf("All filter chains reset\n");
}

// Get current time in milliseconds
unsigned long get_current_time_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (unsigned long)(tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

// PID Controller Functions
// Calculate PID output based on target and current values
int pid_calculate(pid_controller_t *pid, int target, int current) {
    // Calculate error
    int error = target - current;
    
    // Proportional term
    float p_term = pid->kp * error;
    
    // Integral term (with windup protection)
    pid->integral += error;
    
    // Integral windup protection - limit integral to prevent overshoot
    const float max_integral = 1000.0f;  // Adjust based on your bitrate range
    if (pid->integral > max_integral) pid->integral = max_integral;
    if (pid->integral < -max_integral) pid->integral = -max_integral;
    
    float i_term = pid->ki * pid->integral;
    
    // Derivative term
    int derivative = error - pid->last_error;
    float d_term = pid->kd * derivative;
    
    // Calculate PID output
    float pid_output = p_term + i_term + d_term;
    
    // Update PID state
    pid->last_error = error;
    
    // Return the adjustment (not absolute value)
    return (int)pid_output;
}

// Reset PID controller state
void pid_reset(pid_controller_t *pid) {
    pid->integral = 0.0f;
    pid->last_error = 0;
    pid->last_output = 0;
    printf("PID controller reset\n");
}

// Initialize PID controller with custom parameters
void pid_init(pid_controller_t *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid_reset(pid);
}

// Check if bitrate change should be allowed based on asymmetric cooldown
bool should_change_bitrate(int new_bitrate, int current_bitrate, 
                          unsigned long strict_cooldown_ms, unsigned long up_cooldown_ms,
                          int min_change_percent) {
    unsigned long now = get_current_time_ms();
    int delta = abs(new_bitrate - current_bitrate);
    int min_delta = current_bitrate * min_change_percent / 100;
    
    // Don't change if delta is too small
    if (delta < min_delta) {
        return false;
    }
    
    if (new_bitrate < current_bitrate) {
        // Can decrease quickly - only need strict cooldown
        return (now - last_change_time) >= strict_cooldown_ms;
    } else {
        // Must wait longer to increase - need both strict and up cooldown
        return (now - last_change_time) >= strict_cooldown_ms &&
               (now - last_up_time) >= up_cooldown_ms;
    }
}

// Emergency drop logic for sudden signal loss
void check_emergency_drop(int current_bitrate, float filtered_rssi, 
                         int emergency_rssi_threshold, int emergency_bitrate) {
    if (filtered_rssi < emergency_rssi_threshold && current_bitrate > emergency_bitrate) {
        printf("EMERGENCY DROP: RSSI=%.1f, bitrate=%d->%d\n", 
               filtered_rssi, current_bitrate, emergency_bitrate);
        
        // Set emergency bitrate
        char command[128];
        snprintf(command, sizeof(command), "wfb_tx_cmd 8000 set_bitrate %d", emergency_bitrate);
        system(command);
        
        // Update timing variables
        unsigned long now = get_current_time_ms();
        last_change_time = now;
        last_up_time = now;
        last_bitrate = emergency_bitrate;
        
        // Reset all filters and PID controller to adapt to new conditions
        reset_filters();
        pid_reset(&bitrate_pid);
    }
}

void autopower() {
    (void)system("iw wlan0 set tx power auto");
}

// Set real-time priority for ultra-high performance racing VTX
int set_realtime_priority() {
    struct sched_param param;
    
    // Set high real-time priority
    param.sched_priority = 50;  // High priority (1-99 range)
    
    // Use SCHED_FIFO for consistent timing
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("Failed to set real-time priority");
        printf("Note: Real-time priority requires root privileges\n");
        printf("Run with: sudo ./ap_alink\n");
        return -1;
    }
    
    printf("Real-time priority set successfully (SCHED_FIFO, priority 50)\n");
    return 0;
}

// Frame-sync timing for optimal video quality
static struct timespec last_frame_time;
static int target_fps = 120;  // Default racing frame rate
static long frame_interval_ns = 0;

// Initialize frame-sync timing
void init_frame_sync(int fps) {
    target_fps = fps;
    frame_interval_ns = 1000000000L / target_fps;  // Convert to nanoseconds
    
    // Get current time as starting point
    clock_gettime(CLOCK_MONOTONIC, &last_frame_time);
    
    printf("Frame-sync initialized: %d FPS (%.2f ms interval)\n", 
           target_fps, frame_interval_ns / 1000000.0);
}

// Check if it's time for next frame (returns true if frame should be processed)
int is_frame_time() {
    struct timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    
    long elapsed_ns = (current_time.tv_sec - last_frame_time.tv_sec) * 1000000000L + 
                      (current_time.tv_nsec - last_frame_time.tv_nsec);
    
    if (elapsed_ns >= frame_interval_ns) {
        last_frame_time = current_time;
        return 1;  // Time for next frame
    }
    
    return 0;  // Not yet time for next frame
}

// Raw HTTP GET implementation (much faster than wget)
int http_get(const char *path) {
    int s;
    struct sockaddr_in addr;
    char req[256];
    struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };

    if ((s = socket(AF_INET, SOCK_STREAM, 0)) < 0) return -1;

    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    addr.sin_family = AF_INET;
    addr.sin_port = htons(80);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    if (connect(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(s);
        return -1;
    }

    snprintf(req, sizeof(req), "GET %s HTTP/1.0\r\n\r\n", path);
    if (send(s, req, strlen(req), 0) < 0) {
        close(s);
        return -1;
    }

    char buf[64];
    while (recv(s, buf, sizeof(buf), 0) > 0);

    close(s);
    return 0;
}




typedef struct {
    int bitrateMcs;
    char mcspath[256];
} mcs_arg_t;

// Worker thread infrastructure
typedef enum {
    CMD_SET_BITRATE,
    CMD_SET_MCS
} cmd_type_t;

typedef struct {
    cmd_type_t type;
    union {
        int bitrate_kbps;
        mcs_arg_t mcs_data;
    } data;
} worker_cmd_t;

static pthread_t worker_thread;
static pthread_mutex_t worker_mutex = PTHREAD_MUTEX_INITIALIZER;
static sem_t worker_sem;
static worker_cmd_t pending_cmd;
static int worker_running = 0;

// Worker thread that processes API calls
void *worker_thread_func(void *arg) {
    char cmd[512];
    
    while (worker_running) {
        // Wait for work
        sem_wait(&worker_sem);
        
        pthread_mutex_lock(&worker_mutex);
        worker_cmd_t cmd_to_process = pending_cmd;
        pthread_mutex_unlock(&worker_mutex);
        
        switch (cmd_to_process.type) {
            case CMD_SET_BITRATE: {
                int bitrate_kbps = cmd_to_process.data.bitrate_kbps;
                char path[128];
                snprintf(path, sizeof(path), "/api/v1/set?video0.bitrate=%d", bitrate_kbps);
                http_get(path);
                break;
            }
            case CMD_SET_MCS: {
                mcs_arg_t *data = &cmd_to_process.data.mcs_data;
    int bitrateMcs = data->bitrateMcs;
    char *mcspath = data->mcspath;
                
    if (bitrateMcs >= 1 && bitrateMcs < 3) {
                    http_get("/api/v1/set?fpv.roiQp=30,0,0,30");
        snprintf(cmd, sizeof(cmd), "echo 0x0c > %s/rate_ctl", mcspath);
                    (void)system(cmd);
    }
                else if (bitrateMcs >= 3 && bitrateMcs < 10) {
                    http_get("/api/v1/set?fpv.roiQp=0,0,0,0");
        snprintf(cmd, sizeof(cmd), "echo 0x10 > %s/rate_ctl", mcspath);
                    (void)system(cmd);
    }
    else {
                    http_get("/api/v1/set?fpv.roiQp=0,0,0,0");
        snprintf(cmd, sizeof(cmd), "echo 0xFF > %s/rate_ctl", mcspath);
                    (void)system(cmd);
                }
                break;
            }
        }
    }
    return NULL;
}



void set_mcs_async(int bitrateMcs, const char *mcspath) {
    if (!worker_running) return;
    
    pthread_mutex_lock(&worker_mutex);
    pending_cmd.type = CMD_SET_MCS;
    pending_cmd.data.mcs_data.bitrateMcs = bitrateMcs;
    snprintf(pending_cmd.data.mcs_data.mcspath, sizeof(pending_cmd.data.mcs_data.mcspath), "%s", mcspath);
    pthread_mutex_unlock(&worker_mutex);
    
    sem_post(&worker_sem);
}






void config(const char *filename, int *BITRATE_MAX, char *WIFICARD, int *RACE, int *FPS,
             float *rssi_process_var, float *rssi_measure_var,
             float *dbm_process_var, float *dbm_measure_var,
             unsigned long *strict_cooldown, unsigned long *up_cooldown,
             int *min_change_percent, int *emergency_rssi_threshold, int *emergency_bitrate,
             float *pid_kp, float *pid_ki, float *pid_kd,
             float *lpf_cutoff_freq, float *lpf_sample_freq,
             char *rssi_filter_chain_config, char *dbm_filter_chain_config) {
    FILE* fp = fopen(filename, "r");
    if (!fp) {
        printf("No config file found\n");
        exit(1);
    }

    char line[128];

    while (fgets(line, sizeof(line), fp)) {
        if (strncmp(line, "bitrate_max=", 12) == 0) {
            sscanf(line + 12, "%d", BITRATE_MAX);
        }  
        else if (strncmp(line, "wificard=", 9) == 0) {
            sscanf(line + 9, "%63s", WIFICARD);
        }
        else if (strncmp(line, "LowLatency=", 11) == 0) {
            sscanf(line + 11, "%d", RACE);
        }
        else if (strncmp(line, "fps=", 4) == 0) {
            sscanf(line + 4, "%d", FPS);
        }
        else if (strncmp(line, "kalman_rssi_process=", 20) == 0) {
            sscanf(line + 20, "%f", rssi_process_var);
        }
        else if (strncmp(line, "kalman_rssi_measure=", 20) == 0) {
            sscanf(line + 20, "%f", rssi_measure_var);
        }
        else if (strncmp(line, "kalman_dbm_process=", 19) == 0) {
            sscanf(line + 19, "%f", dbm_process_var);
        }
        else if (strncmp(line, "kalman_dbm_measure=", 19) == 0) {
            sscanf(line + 19, "%f", dbm_measure_var);
        }
        else if (strncmp(line, "strict_cooldown_ms=", 19) == 0) {
            sscanf(line + 19, "%lu", strict_cooldown);
        }
        else if (strncmp(line, "up_cooldown_ms=", 15) == 0) {
            sscanf(line + 15, "%lu", up_cooldown);
        }
        else if (strncmp(line, "min_change_percent=", 19) == 0) {
            sscanf(line + 19, "%d", min_change_percent);
        }
        else if (strncmp(line, "emergency_rssi_threshold=", 25) == 0) {
            sscanf(line + 25, "%d", emergency_rssi_threshold);
        }
        else if (strncmp(line, "emergency_bitrate=", 18) == 0) {
            sscanf(line + 18, "%d", emergency_bitrate);
        }
        else if (strncmp(line, "pid_kp=", 7) == 0) {
            sscanf(line + 7, "%f", pid_kp);
        }
        else if (strncmp(line, "pid_ki=", 7) == 0) {
            sscanf(line + 7, "%f", pid_ki);
        }
        else if (strncmp(line, "pid_kd=", 7) == 0) {
            sscanf(line + 7, "%f", pid_kd);
        }
        else if (strncmp(line, "rssi_filter_chain=", 18) == 0) {
            sscanf(line + 18, "%63s", rssi_filter_chain_config);
        }
        else if (strncmp(line, "dbm_filter_chain=", 17) == 0) {
            sscanf(line + 17, "%63s", dbm_filter_chain_config);
        }
        else if (strncmp(line, "lpf_cutoff_freq=", 16) == 0) {
            sscanf(line + 16, "%f", lpf_cutoff_freq);
        }
        else if (strncmp(line, "lpf_sample_freq=", 16) == 0) {
            sscanf(line + 16, "%f", lpf_sample_freq);
        }
    }

    fclose(fp);
}


int get_dbm() {
    FILE *fp;
    char line[256];
    int dbm = -100;
    char *token;
    int field_count = 0;

    // Open /proc/net/wireless directly - much faster than fork/exec
    fp = fopen("/proc/net/wireless", "r");
    if (!fp) {
        perror("Failed to open /proc/net/wireless");
        return dbm;
    }

    // Skip header lines (first 2 lines)
    if (fgets(line, sizeof(line), fp) == NULL) goto cleanup;
    if (fgets(line, sizeof(line), fp) == NULL) goto cleanup;

    // Read the wlan0 line
    if (fgets(line, sizeof(line), fp) != NULL) {
        // Parse the line: "wlan0: 0000 1234 5678 90ab  cdef  1234  5678  90ab  cdef"
        // Field 2 (index 2) is the signal level in dBm
        token = strtok(line, " \t");
        field_count = 0;
        
        while (token != NULL && field_count < 3) {
            if (field_count == 2) {
                // Convert signal level to dBm (it's in centi-dBm, so divide by 100)
                dbm = atoi(token) / 100;
                    break;
                }
            token = strtok(NULL, " \t");
            field_count++;
            }
        }

cleanup:
        fclose(fp);
    return dbm;
}



int get_rssi(const char *readcmd) {
    static FILE *fp = NULL;
    static char last_path[512] = {0};
    char buffer[256];
    int rssi_percent = 0;
    char path[512];

    // Construit le chemin complet vers sta_tp_info
    // English:Builds the full path to sta_tp_info
    snprintf(path, sizeof(path), "%s/sta_tp_info", readcmd);

    // Only reopen if path changed or file not open
    if (fp == NULL || strcmp(path, last_path) != 0) {
        if (fp != NULL) {
            fclose(fp);
        }
    fp = fopen(path, "r");
    if (!fp) {
        perror("fopen");
        return rssi_percent;
    }
        strcpy(last_path, path);
    } else {
        // Rewind to beginning for fresh read
        rewind(fp);
    }

    while (fgets(buffer, sizeof(buffer), fp)) {
        char *pos = strstr(buffer, "rssi");
        if (pos) {
            // Allows optional spaces around the ':'
            if (sscanf(pos, "rssi : %d %%", &rssi_percent) != 1) {
                sscanf(pos, "rssi: %d %%", &rssi_percent);
            }
            break;
        }
    }

    return rssi_percent;
}




void mspLQ(int rssi_osd) {
    char command[128];
    snprintf(command, sizeof(command),
             "echo \"VLQ %d &B &F60 &L30\" > /tmp/MSPOSD.msg",
              rssi_osd);
              //RSSI PATTERN *** ** * 
    (void)system(command);
}


void set_bitrate_async(int bitrate_mbps) {
    if (!worker_running) return;
    
    pthread_mutex_lock(&worker_mutex);
    pending_cmd.type = CMD_SET_BITRATE;
    pending_cmd.data.bitrate_kbps = bitrate_mbps * 1024;
    pthread_mutex_unlock(&worker_mutex);
    
    sem_post(&worker_sem);
}

int main() {
    int bitrate = 4;
    int bitrate_min = 1;
    int bitrate_max = 0;
    int dbm_Max = -50;
    int dbm_Min = 0;
    int rssi = 0;
    char NIC[10] = {0};
    char driverpath[256] = {0};
    int RaceMode = 0;
    int histeris = 1;      // Reduced hysteresis for faster racing response
    int minushisteris = -1; // Reduced hysteresis for faster racing response
    int aDb = 0;
    int currentDb = 0;
    int dbm = -100;
    int loop_counter = 0;  // Counter to reduce I/O frequency
    int target_fps = 120;  // Default racing frame rate
    
    // Kalman filter parameters
    float rssi_process_var = 1e-5f;   // Default process variance for RSSI
    float rssi_measure_var = 0.1f;    // Default measurement variance for RSSI
    float dbm_process_var = 1e-5f;    // Default process variance for dBm
    float dbm_measure_var = 0.5f;     // Default measurement variance for dBm

    // Asymmetric cooldown parameters
    unsigned long strict_cooldown_ms = STRICT_COOLDOWN_MS;
    unsigned long up_cooldown_ms = UP_COOLDOWN_MS;
    int min_change_percent = MIN_CHANGE_PERCENT;
    int emergency_rssi_threshold = EMERGENCY_RSSI_THRESHOLD;
    int emergency_bitrate = EMERGENCY_BITRATE;

    // PID controller parameters
    float pid_kp = 1.0f;
    float pid_ki = 0.05f;
    float pid_kd = 0.4f;

    // Filter chain configuration parameters
    char rssi_filter_chain_config[64] = "0";  // Default to single Kalman filter
    char dbm_filter_chain_config[64] = "0";   // Default to single Kalman filter
    float lpf_cutoff_freq = 2.0f;              // Default 2Hz cutoff
    float lpf_sample_freq = 10.0f;             // Default 10Hz sample rate

    // Filtered values
    float filtered_rssi = 50.0f;
    float filtered_dbm = -60.0f;
    
    //char rssi_pattern[5] = {0};

    config("/etc/ap_alink.conf", &bitrate_max, NIC, &RaceMode, &target_fps,
           &rssi_process_var, &rssi_measure_var, &dbm_process_var, &dbm_measure_var,
           &strict_cooldown_ms, &up_cooldown_ms, &min_change_percent,
           &emergency_rssi_threshold, &emergency_bitrate,
           &pid_kp, &pid_ki, &pid_kd,
           &lpf_cutoff_freq, &lpf_sample_freq,
           rssi_filter_chain_config, dbm_filter_chain_config);
    
    // Parse and configure filter chains
    parse_filter_chain(rssi_filter_chain_config, &rssi_filter_chain);
    parse_filter_chain(dbm_filter_chain_config, &dbm_filter_chain);
    
    // Initialize filters with config values
    init_filters(rssi_process_var, rssi_measure_var, dbm_process_var, dbm_measure_var,
                 lpf_cutoff_freq, lpf_sample_freq);
    
    // Initialize PID controller with config values
    pid_init(&bitrate_pid, pid_kp, pid_ki, pid_kd);
    printf("PID Controller initialized: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", pid_kp, pid_ki, pid_kd);
    
    // Set real-time priority for ultra-high performance racing VTX
    set_realtime_priority();
    
    // Initialize frame-sync timing
    init_frame_sync(target_fps);
    
    // Initialize worker thread
    sem_init(&worker_sem, 0, 0);
    worker_running = 1;
    if (pthread_create(&worker_thread, NULL, worker_thread_func, NULL) != 0) {
        perror("Failed to create worker thread");
        exit(1);
    }
    

    if (strcmp(NIC, "8812eu2") == 0) {
        strcpy(driverpath, "/proc/net/rtl88x2eu/wlan0");

    }
    else if (strcmp(NIC, "8812au") == 0) {
        strcpy(driverpath, "/proc/net/rtl88xxau/wlan0");
    }
    
    if (RaceMode != 1 && RaceMode != 0) {
        printf("invalid value for racemode\n");
    } else if (RaceMode == 1) {
       
        printf("RACEMODE ENABLE");
        char cmd1[512];
        snprintf(cmd1, sizeof(cmd1), "echo 20 > %s/ack_timeout", driverpath);
        //SET BITRATE MAX 4MBPS
        bitrate_max=4;
        //SET BUFFER SETTING
        (void)system("sysctl -w net.core.rmem_default=16384");
        (void)system("sysctl -w net.core.rmem_max=65536");
        (void)system("sysctl -w net.core.wmem_default=16384");
        (void)system("sysctl -w net.core.wmem_max=65536");
        (void)system("ifconfig wlan0 txqueuelen 100");
        (void)system("sysctl -w net.core.netdev_max_backlog=64");
        //SET 960x720120FPS
        (void)system("wget -qO- \"http://localhost/api/v1/set?video0.size=1280x720\" > /dev/null 2>&1");
        (void)system("wget -qO- \"http://localhost/api/v1/set?video0.fps=120\" > /dev/null 2>&1");
        (void)system("wget -qO- \"http://localhost/api/v1/set?isp.exposure=11\" > /dev/null 2>&1");                
        
    } else {
        printf("racemode disable\n");

        
    }
    
    while (1) {
        // Frame-sync timing: only process when it's time for next frame
        if (!is_frame_time()) {
            usleep(100);  // Short sleep to avoid busy waiting
            continue;
        }
        
        loop_counter++;
        
        // Only read signal data every 5 frames to reduce I/O overhead
        // This gives us frame-synced control with reduced signal reading frequency
        // At 120Hz frame rate: 120Hz control loop, 24Hz signal sampling (optimal for video quality)
        if (loop_counter % 5 == 0) {
        currentDb = dbm - aDb;
            dbm = get_dbm();
        aDb = dbm; 
            rssi = get_rssi(driverpath);
            
        // Apply filter chains to smooth the signals
        filtered_rssi = apply_filter_chain(&rssi_filter_chain, (float)rssi);
        filtered_dbm = apply_filter_chain(&dbm_filter_chain, (float)dbm);
        
        // Check for emergency drop conditions
        check_emergency_drop(last_bitrate, filtered_rssi, emergency_rssi_threshold, emergency_bitrate);
        }
        
        //calculation of dbm_Max dbm_Min
        

        dbm_Max = -50;      
        dbm_Min = (filtered_rssi > 55) ? -70 : (filtered_rssi >= 40 ? -55 : -53);


        double vlq = ((double)((filtered_dbm) - (dbm_Min)) / (double)((dbm_Max) - (dbm_Min))) * 100.0;
      
#ifdef DEBUG
        // Only show debug output when we read new signal data
        if (loop_counter % 5 == 0) {
        printf("vlq = %.2f%%\n", vlq);
            printf("rssi = %d (filtered: %.1f)\n", rssi, filtered_rssi);
        printf("adb= %d\n", aDb);
            printf("dbm= %d (filtered: %.1f)\n", dbm, filtered_dbm);
        printf("current %d\n", currentDb);
        }
#endif
        mspLQ((int)filtered_rssi);

             
        
        //MAIN LOGIC

        // RSSI fallback
            // Clamp VLQ between 0 and 100
            if ( currentDb > histeris || currentDb < minushisteris ) {
                if (vlq > 100.0 || rssi > 55) {
       
                //system("wget -qO- \"http://localhost/api/v1/set?image.saturation=50\" > /dev/null 2>&1");
                bitrate = bitrate_max;
            }
            else if (vlq < 1 || rssi < 20) {
                bitrate = bitrate_min;

                //BW 
                //system("wget -qO- \"http://localhost/api/v1/set?image.saturation=0\" > /dev/null 2>&1");
                       
            }
             else {
                // Calculate target bitrate using VLQ (your existing logic)
                int target_bitrate = (int)(bitrate_max * vlq / 100.0);
                
                // Apply PID controller to smooth the transition to target
                int pid_adjustment = pid_calculate(&bitrate_pid, target_bitrate, last_bitrate);
                bitrate = last_bitrate + pid_adjustment;
                
                // Clamp bitrate to valid range
                if (bitrate < bitrate_min) bitrate = bitrate_min;
                if (bitrate > bitrate_max) bitrate = bitrate_max;
                
#ifdef DEBUG
                if (loop_counter % 5 == 0) {
                    printf("Target: %d, Current: %d, PID Adj: %d, Final: %d\n", 
                           target_bitrate, last_bitrate, pid_adjustment, bitrate);
                }
#endif
            }

            // Apply asymmetric cooldown logic before changing bitrate
            if (should_change_bitrate(bitrate, last_bitrate, strict_cooldown_ms, up_cooldown_ms, min_change_percent)) {
                set_bitrate_async(bitrate);
                set_mcs_async(bitrate, driverpath);
                
                // Update timing variables
                unsigned long now = get_current_time_ms();
                last_change_time = now;
                if (bitrate > last_bitrate) {
                    last_up_time = now;
                }
                last_bitrate = bitrate;
                
#ifdef DEBUG
                printf("Bitrate changed to %d kbps (RSSI: %.1f, dBm: %.1f)\n", 
                       bitrate, filtered_rssi, filtered_dbm);
#endif
            } else {
#ifdef DEBUG
                printf("Bitrate change blocked by cooldown (target: %d, current: %d)\n", 
                       bitrate, last_bitrate);
#endif
            }
                
#ifdef DEBUG
        fflush(stdout);
#endif
        // Frame-sync timing handles the delay automatically
        }
    }
    
    // Cleanup worker thread
    worker_running = 0;
    sem_post(&worker_sem);  // Wake up worker to exit
    pthread_join(worker_thread, NULL);
    sem_destroy(&worker_sem);

    return 0;
}
