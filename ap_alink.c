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
#include <sys/mman.h>
#include <sys/stat.h>

// Function declarations
unsigned long get_current_time_ms(void);
void cleanup_memory_maps(void);

// Kalman Filter Structure
typedef struct {
    float estimate;           // Current estimate
    float error_estimate;     // Current error estimate
    float process_variance;   // Process noise variance
    float measurement_variance; // Measurement noise variance
} kalman_filter_t;

// Control Algorithm Types
typedef enum {
    CONTROL_ALGORITHM_PID = 0,      // PID controller (complex, smooth)
    CONTROL_ALGORITHM_FIFO = 1      // Simple FIFO (fast, direct)
} control_algorithm_t;

// Asymmetric Cooldown Constants
#define STRICT_COOLDOWN_MS 200      // 200ms minimum between changes
#define UP_COOLDOWN_MS 3000         // 3s before increasing bitrate
#define EMERGENCY_COOLDOWN_MS 50    // 50ms for emergency drops (6 frames at 120fps)
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

// Global Racing Filter Chain instances (for race_mode=1)
static filter_chain_t rssi_race_filter_chain = {
    .filters = {FILTER_TYPE_LOWPASS},
    .filter_count = 1,
    .enabled = true
};

static filter_chain_t dbm_race_filter_chain = {
    .filters = {FILTER_TYPE_LOWPASS},
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

// Dynamic RSSI threshold functions
int get_dynamic_rssi_threshold(int current_mcs);
int bitrate_to_mcs(int bitrate_mbps);

// Generic popen() helper function to replace system() calls
int execute_command(const char *command) {
    // validate input parameters
    if (command == NULL) {
#ifdef DEBUG
        fprintf(stderr, "Error: execute_command() called with NULL command\n");
#endif
        return -1;
    }
    
    if (strlen(command) == 0) {
#ifdef DEBUG
        fprintf(stderr, "Error: execute_command() called with empty command\n");
#endif
        return -1;
    }
    
    if (strlen(command) > 1024) {
#ifdef DEBUG
        fprintf(stderr, "Error: execute_command() command too long (>1024 chars)\n");
#endif
        return -1;
    }
    
    FILE *pipe = popen(command, "w");
    if (pipe == NULL) {
#ifdef DEBUG
        perror("popen failed");
#endif
        return -1;
    }
    
    int status = pclose(pipe);
    if (status == -1) {
#ifdef DEBUG
        perror("pclose failed");
#endif
        return -1;
    }
    // Return the exit status of the command
    return WEXITSTATUS(status);
}

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
#ifdef DEBUG
    printf("Low-pass filter initialized: cutoff=%.1fHz, sample=%.1fHz, alpha=%.3f\n", 
           cutoff_freq, sample_freq, filter->alpha);
#endif
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
#ifdef DEBUG
    printf("Low-pass filter reset\n");
#endif
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
#ifdef DEBUG
    printf("Mode filter reset\n");
#endif
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
#ifdef DEBUG
    printf("Derivative filter reset\n");
#endif
}

// 2-Pole Low-Pass Filter Functions (biquad filter)
// Initialize 2-pole low-pass filter
void init_2pole_lpf(lpf_2pole_t *filter, float cutoff_freq, float sample_freq) {
    filter->cutoff_freq = cutoff_freq;
    filter->sample_freq = sample_freq;
    filter->delay_element_1 = 0.0f;
    filter->delay_element_2 = 0.0f;
    filter->initialised = false;
#ifdef DEBUG
    printf("2-Pole LPF initialized: cutoff=%.1fHz, sample=%.1fHz\n", cutoff_freq, sample_freq);
#endif
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
#ifdef DEBUG
    printf("2-Pole LPF reset\n");
#endif
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
#ifdef DEBUG
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
#endif
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
#ifdef DEBUG
    printf("Mode filters initialized\n");
#endif
    
    // Initialize Derivative filters (no parameters needed)
#ifdef DEBUG
    printf("Derivative filters initialized\n");
#endif
    
    // Initialize 2-Pole Low-Pass filters
    init_2pole_lpf(&rssi_2pole_lpf, lpf_cutoff_freq, lpf_sample_freq);
    init_2pole_lpf(&dbm_2pole_lpf, lpf_cutoff_freq, lpf_sample_freq);
    
#ifdef DEBUG
    printf("All filter types initialized successfully\n");
#endif
}

// Reset all filters
void reset_filters() {
    // Reset filter chains
    reset_filter_chain(&rssi_filter_chain);
    reset_filter_chain(&dbm_filter_chain);
    
#ifdef DEBUG
    printf("All filter chains reset\n");
#endif
}

// Cleanup function for memory-mapped files
void cleanup_memory_maps() {
    // Note: We can't easily clean up the static variables in get_dbm() and get_rssi()
    // because they're static and we don't have direct access to them here.
    // The OS will clean up memory maps when the process exits.
#ifdef DEBUG
    printf("Memory maps cleanup completed\n");
#endif
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
#ifdef DEBUG
    printf("PID controller reset\n");
#endif
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
                          int min_change_percent, unsigned long emergency_cooldown_ms) {
    unsigned long now = get_current_time_ms();
    int delta = abs(new_bitrate - current_bitrate);
    int min_delta = current_bitrate * min_change_percent / 100;
    
    // Don't change if delta is too small
    if (delta < min_delta) {
        return false;
    }
    
    if (new_bitrate < current_bitrate) {
        // Can decrease quickly - use emergency cooldown for faster response
        return (now - last_change_time) >= emergency_cooldown_ms;
    } else {
        // Must wait longer to increase - need both strict and up cooldown
        return (now - last_change_time) >= strict_cooldown_ms &&
               (now - last_up_time) >= up_cooldown_ms;
    }
}

// Emergency drop logic for sudden signal loss
void check_emergency_drop(int current_bitrate, float filtered_rssi, 
                         int emergency_rssi_threshold, int emergency_bitrate) {
    // Calculate dynamic RSSI threshold based on current MCS
    int current_mcs = bitrate_to_mcs(current_bitrate);
    int dynamic_threshold = get_dynamic_rssi_threshold(current_mcs);
    
    // Use the more conservative threshold (lower value = more sensitive)
    int effective_threshold = (dynamic_threshold < emergency_rssi_threshold) ? 
                             dynamic_threshold : emergency_rssi_threshold;
    
    if (filtered_rssi < effective_threshold && current_bitrate > emergency_bitrate) {
        printf("EMERGENCY DROP: RSSI=%.1f (threshold=%d, MCS=%d), bitrate=%d->%d\n", 
               filtered_rssi, effective_threshold, current_mcs, current_bitrate, emergency_bitrate);
        
        // Set emergency bitrate
        char command[128];
        snprintf(command, sizeof(command), "wfb_tx_cmd 8000 set_bitrate %d", emergency_bitrate);
        int result = execute_command(command);
        if (result != 0) {
            printf("Warning: Emergency bitrate command failed with status %d\n", result);
        }
        
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
    int result = execute_command("iw wlan0 set tx power auto");
    if (result != 0) {
        printf("Warning: WiFi power control command failed with status %d\n", result);
    }
}

// Set real-time priority for ultra-high performance racing VTX
int set_realtime_priority() {
    struct sched_param param;
    
    // Set low real-time priority (reduced to prevent SSH issues)
    param.sched_priority = 10;  // Low priority (1-99 range)
    
    // Use SCHED_FIFO for consistent timing
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("Failed to set real-time priority");
        printf("Note: Real-time priority requires root privileges\n");
        printf("Run with: sudo ./ap_alink\n");
        return -1;
    }
    
    printf("Real-time priority set successfully (SCHED_FIFO, priority 10)\n");
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

// Optimized HTTP GET implementation - fire-and-forget for FPV applications
int http_get(const char *path) {
    int s;
    struct sockaddr_in addr;
    char req[256];
    // Fast timeout since we don't wait for responses
    struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 }; // 100ms timeout

    if ((s = socket(AF_INET, SOCK_STREAM, 0)) < 0) return -1;

    // Only set send timeout since we don't receive responses
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

    // Close immediately - no need to wait for response in FPV applications
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

// QP Delta configuration removed - ROI is disabled in recent Majestic builds
// Video quality is now controlled by MCS rate control only

// MCS to RSSI threshold lookup table (based on 802.11n/ac standards)
// Values are minimum RSSI thresholds for reliable operation at each MCS
static const int mcs_rssi_thresholds[] = {
    -82,  // MCS 0 (BPSK 1/2)   - Most robust
    -79,  // MCS 1 (QPSK 1/2)
    -77,  // MCS 2 (QPSK 3/4)
    -74,  // MCS 3 (16-QAM 1/2)
    -70,  // MCS 4 (16-QAM 3/4)
    -66,  // MCS 5 (64-QAM 2/3)
    -65,  // MCS 6 (64-QAM 3/4)
    -64,  // MCS 7 (64-QAM 5/6)
    -59,  // MCS 8 (256-QAM 3/4) - VHT
    -57   // MCS 9 (256-QAM 5/6) - VHT, least robust
};

// Hardware-specific RSSI offset (configurable for different DIY builds)
static int hardware_rssi_offset = 0;

// Calculate dynamic RSSI threshold based on current MCS
int get_dynamic_rssi_threshold(int current_mcs) {
    // Clamp MCS to valid range
    if (current_mcs < 0) current_mcs = 0;
    if (current_mcs >= 10) current_mcs = 9;
    
    // Get base threshold from lookup table
    int base_threshold = mcs_rssi_thresholds[current_mcs];
    
    // Apply hardware-specific offset
    int dynamic_threshold = base_threshold + hardware_rssi_offset;
    
    // Add safety margin (3 dBm) for emergency drop
    dynamic_threshold += 3;
    
    return dynamic_threshold;
}

// Convert bitrate to approximate MCS (simplified mapping)
int bitrate_to_mcs(int bitrate_mbps) {
    // Simplified mapping based on typical bitrates
    // This should be calibrated for your specific hardware
    if (bitrate_mbps <= 1) return 0;      // MCS 0
    else if (bitrate_mbps <= 2) return 1; // MCS 1
    else if (bitrate_mbps <= 3) return 2; // MCS 2
    else if (bitrate_mbps <= 4) return 3; // MCS 3
    else if (bitrate_mbps <= 6) return 4; // MCS 4
    else if (bitrate_mbps <= 8) return 5; // MCS 5
    else if (bitrate_mbps <= 10) return 6; // MCS 6
    else if (bitrate_mbps <= 12) return 7; // MCS 7
    else if (bitrate_mbps <= 15) return 8; // MCS 8
    else return 9; // MCS 9 (highest)
}

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
                    // ROI QP Delta disabled in recent Majestic builds - using MCS rate control only
                    snprintf(cmd, sizeof(cmd), "echo 0x0c > %s/rate_ctl", mcspath);
                    int result = execute_command(cmd);
                    if (result != 0) {
                        printf("Warning: MCS rate control (low) command failed with status %d\n", result);
                    }
                }
                else if (bitrateMcs >= 3 && bitrateMcs < 10) {
                    // ROI QP Delta disabled in recent Majestic builds - using MCS rate control only
                    snprintf(cmd, sizeof(cmd), "echo 0x10 > %s/rate_ctl", mcspath);
                    int result = execute_command(cmd);
                    if (result != 0) {
                        printf("Warning: MCS rate control (medium) command failed with status %d\n", result);
                    }
                }
                else {
                    // ROI QP Delta disabled in recent Majestic builds - using MCS rate control only
                    snprintf(cmd, sizeof(cmd), "echo 0xFF > %s/rate_ctl", mcspath);
                    int result = execute_command(cmd);
                    if (result != 0) {
                        printf("Warning: MCS rate control (high) command failed with status %d\n", result);
                    }
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
             char *rssi_filter_chain_config, char *dbm_filter_chain_config,
             char *racing_rssi_filter_chain_config, char *racing_dbm_filter_chain_config,
             char *racing_video_resolution, int *racing_exposure, int *racing_fps,
             int *signal_sampling_interval, unsigned long *emergency_cooldown,
             int *control_algorithm, int *signal_sampling_freq_hz, int *hardware_rssi_offset, int *cooldown_enabled, int *frame_sync_enabled) {
    FILE* fp = fopen(filename, "r");
    if (!fp) {
        printf("No config file found\n");
        exit(1);
    }

    char line[128];

    while (fgets(line, sizeof(line), fp)) {
        if (strncmp(line, "bitrate_max=", 12) == 0) {
            sscanf(line + 12, "%d", BITRATE_MAX);
            continue;
        }
        if (strncmp(line, "wificard=", 9) == 0) {
            sscanf(line + 9, "%63s", WIFICARD);
            continue;
        }
        if (strncmp(line, "race_mode=", 10) == 0) {
            sscanf(line + 10, "%d", RACE);
            continue;
        }
        if (strncmp(line, "fps=", 4) == 0) {
            sscanf(line + 4, "%d", FPS);
            continue;
        }
        if (strncmp(line, "kalman_rssi_process=", 20) == 0) {
            sscanf(line + 20, "%f", rssi_process_var);
            continue;
        }
        if (strncmp(line, "kalman_rssi_measure=", 20) == 0) {
            sscanf(line + 20, "%f", rssi_measure_var);
            continue;
        }
        if (strncmp(line, "kalman_dbm_process=", 19) == 0) {
            sscanf(line + 19, "%f", dbm_process_var);
            continue;
        }
        if (strncmp(line, "kalman_dbm_measure=", 19) == 0) {
            sscanf(line + 19, "%f", dbm_measure_var);
            continue;
        }
        if (strncmp(line, "strict_cooldown_ms=", 19) == 0) {
            sscanf(line + 19, "%lu", strict_cooldown);
            continue;
        }
        if (strncmp(line, "up_cooldown_ms=", 15) == 0) {
            sscanf(line + 15, "%lu", up_cooldown);
            continue;
        }
        if (strncmp(line, "min_change_percent=", 19) == 0) {
            sscanf(line + 19, "%d", min_change_percent);
            continue;
        }
        if (strncmp(line, "emergency_rssi_threshold=", 25) == 0) {
            sscanf(line + 25, "%d", emergency_rssi_threshold);
            continue;
        }
        if (strncmp(line, "emergency_bitrate=", 18) == 0) {
            sscanf(line + 18, "%d", emergency_bitrate);
            continue;
        }
        if (strncmp(line, "pid_kp=", 7) == 0) {
            sscanf(line + 7, "%f", pid_kp);
            continue;
        }
        if (strncmp(line, "pid_ki=", 7) == 0) {
            sscanf(line + 7, "%f", pid_ki);
            continue;
        }
        if (strncmp(line, "pid_kd=", 7) == 0) {
            sscanf(line + 7, "%f", pid_kd);
            continue;
        }
        if (strncmp(line, "rssi_filter_chain=", 18) == 0) {
            sscanf(line + 18, "%63s", rssi_filter_chain_config);
            continue;
        }
        if (strncmp(line, "dbm_filter_chain=", 17) == 0) {
            sscanf(line + 17, "%63s", dbm_filter_chain_config);
            continue;
        }
        if (strncmp(line, "racing_rssi_filter_chain=", 24) == 0) {
            sscanf(line + 24, "%63s", racing_rssi_filter_chain_config);
            continue;
        }
        if (strncmp(line, "racing_dbm_filter_chain=", 23) == 0) {
            sscanf(line + 23, "%63s", racing_dbm_filter_chain_config);
            continue;
        }
        if (strncmp(line, "racing_video_resolution=", 24) == 0) {
            sscanf(line + 24, "%63s", racing_video_resolution);
            continue;
        }
        if (strncmp(line, "racing_exposure=", 16) == 0) {
            sscanf(line + 16, "%d", racing_exposure);
            continue;
        }
        if (strncmp(line, "racing_fps=", 11) == 0) {
            sscanf(line + 11, "%d", racing_fps);
            continue;
        }
        if (strncmp(line, "lpf_cutoff_freq=", 16) == 0) {
            sscanf(line + 16, "%f", lpf_cutoff_freq);
            continue;
        }
        if (strncmp(line, "lpf_sample_freq=", 16) == 0) {
            sscanf(line + 16, "%f", lpf_sample_freq);
            continue;
        }
        if (strncmp(line, "signal_sampling_interval=", 26) == 0) {
            sscanf(line + 26, "%d", signal_sampling_interval);
            continue;
        }
        if (strncmp(line, "emergency_cooldown_ms=", 22) == 0) {
            sscanf(line + 22, "%lu", emergency_cooldown);
            continue;
        }
        if (strncmp(line, "control_algorithm=", 18) == 0) {
            sscanf(line + 18, "%d", control_algorithm);
            continue;
        }
        // QP Delta settings removed - ROI is disabled in recent Majestic builds
        if (strncmp(line, "signal_sampling_freq_hz=", 24) == 0) {
            sscanf(line + 24, "%d", signal_sampling_freq_hz);
            continue;
        }
        if (strncmp(line, "hardware_rssi_offset=", 21) == 0) {
            sscanf(line + 21, "%d", hardware_rssi_offset);
            continue;
        }
        if (strncmp(line, "cooldown_enabled=", 17) == 0) {
            sscanf(line + 17, "%d", cooldown_enabled);
            continue;
        }
        if (strncmp(line, "frame_sync_enabled=", 19) == 0) {
            sscanf(line + 19, "%d", frame_sync_enabled);
            continue;
        }
    }

    fclose(fp);
}


int get_dbm() {
    static char *mapped_data = NULL;
    static size_t mapped_size = 0;
    static bool initialized = false;
    int dbm = -100;
    
    if (!initialized) {
        int fd = open("/proc/net/wireless", O_RDONLY);
        if (fd < 0) {
            perror("Failed to open /proc/net/wireless");
            return dbm;
        }
        
        struct stat st;
        if (fstat(fd, &st) < 0) {
            perror("fstat");
            close(fd);
            return dbm;
        }
        
        mapped_size = st.st_size;
        if (mapped_size == 0) {
            close(fd);
            return dbm;
        }
        
        mapped_data = mmap(NULL, mapped_size, PROT_READ, MAP_PRIVATE, fd, 0);
        close(fd);
        
        if (mapped_data == MAP_FAILED) {
            perror("mmap");
            mapped_data = NULL;
            return dbm;
        }
        
        initialized = true;
    }
    
    if (mapped_data == NULL) {
        return dbm;
    }
    
    // Parse the mapped data directly
    char *line_start = mapped_data;
    char *line_end;
    int line_count = 0;
    
    // Skip first two header lines
    while (line_count < 2 && (line_end = strchr(line_start, '\n')) != NULL) {
        line_start = line_end + 1;
        line_count++;
    }
    
    // Parse the wlan0 line (third line)
    if (line_end != NULL && (line_end = strchr(line_start, '\n')) != NULL) {
        *line_end = '\0';  // Null-terminate the line
        
        // Parse fields: "wlan0: 0000 1234 5678 90ab  cdef  1234  5678  90ab  cdef"
        char *token = strtok(line_start, " \t");
        int field_count = 0;
        
        while (token != NULL && field_count < 3) {
            if (field_count == 2) {
                // Field 2 is signal level in centi-dBm
                dbm = atoi(token) / 100;
                break;
            }
            token = strtok(NULL, " \t");
            field_count++;
        }
        
        *line_end = '\n';  // Restore the newline
    }
    
    return dbm;
}



int get_rssi(const char *readcmd) {
    static char *mapped_data = NULL;
    static size_t mapped_size = 0;
    static char last_path[512] = {0};
    char path[512];
    int rssi_percent = 0;
    
    // Direct string concatenation - much faster than snprintf
    strcpy(path, readcmd);
    strcat(path, "/sta_tp_info");
    
    // Only remap if path changed
    if (strcmp(path, last_path) != 0) {
        if (mapped_data != NULL) {
            munmap(mapped_data, mapped_size);
            mapped_data = NULL;
        }
        
        int fd = open(path, O_RDONLY);
        if (fd < 0) {
            perror("open");
            return rssi_percent;
        }
        
        struct stat st;
        if (fstat(fd, &st) < 0) {
            perror("fstat");
            close(fd);
            return rssi_percent;
        }
        
        mapped_size = st.st_size;
        if (mapped_size == 0) {
            close(fd);
            return rssi_percent;
        }
        
        mapped_data = mmap(NULL, mapped_size, PROT_READ, MAP_PRIVATE, fd, 0);
        close(fd);
        
        if (mapped_data == MAP_FAILED) {
            perror("mmap");
            mapped_data = NULL;
            return rssi_percent;
        }
        
        strcpy(last_path, path);
    }
    
    if (mapped_data == NULL) {
        return rssi_percent;
    }
    
    // Search for RSSI in mapped memory
    char *pos = strstr(mapped_data, "rssi");
    if (pos) {
        // Try both formats: "rssi : 85 %" and "rssi: 85 %"
        if (sscanf(pos, "rssi : %d %%", &rssi_percent) != 1) {
            sscanf(pos, "rssi: %d %%", &rssi_percent);
        }
#ifdef DEBUG
        printf("DEBUG: Found RSSI in file: %d%%\n", rssi_percent);
#endif
    } else {
#ifdef DEBUG
        printf("DEBUG: No RSSI found in file content: %s\n", mapped_data);
#endif
    }
    
    return rssi_percent;
}




void mspLQ(int rssi_osd) {
    char command[128];
    snprintf(command, sizeof(command),
             "echo \"VLQ %d &B &F60 &L30\" > /tmp/MSPOSD.msg",
              rssi_osd);
              //RSSI PATTERN *** ** * 
    int result = execute_command(command);
    if (result != 0) {
        printf("Warning: MSP OSD command failed with status %d\n", result);
    }
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
    char racing_rssi_filter_chain_config[64] = "1";  // Default to Low-Pass for racing
    char racing_dbm_filter_chain_config[64] = "1";   // Default to Low-Pass for racing
    float lpf_cutoff_freq = 2.0f;              // Default 2Hz cutoff
    float lpf_sample_freq = 10.0f;             // Default 10Hz sample rate

    // Racing mode configuration parameters
    char racing_video_resolution[64] = "1280x720";  // Default racing resolution
    int racing_exposure = 11;                       // Default racing exposure
    int racing_fps = 120;                           // Default racing frame rate

    // QP Delta configuration removed - ROI is disabled in recent Majestic builds

    // Signal sampling configuration
    int signal_sampling_interval = 5;               // Default: sample every 5 frames (legacy)
    int signal_sampling_freq_hz = 50;               // Default: 50Hz signal sampling (independent of frame rate)

    // Hardware-specific RSSI offset configuration
    int hardware_rssi_offset_config = 0;                   // Default: no offset (calibrate for your hardware)

    // Emergency cooldown configuration
    unsigned long emergency_cooldown_ms = EMERGENCY_COOLDOWN_MS;  // Default: 50ms

    // Control algorithm configuration
    int control_algorithm = CONTROL_ALGORITHM_FIFO;  // Default: Simple FIFO (more performant)

    // Cooldown system control
    int cooldown_enabled = 1;  // Default: enabled (1=enabled, 0=disabled)

    // Frame sync control
    int frame_sync_enabled = 1;  // Default: enabled (1=enabled, 0=disabled)

    // WiFi state monitoring
    int wifi_driver_available = 1;  // Track if WiFi driver path exists

    // WiFi performance mode
    int wifi_performance_mode = 1;  // Default: high performance (1=enabled, 0=disabled)
    // Note: wifi_error_recovery removed - not currently implemented

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
           rssi_filter_chain_config, dbm_filter_chain_config,
           racing_rssi_filter_chain_config, racing_dbm_filter_chain_config,
           racing_video_resolution, &racing_exposure, &racing_fps,
           &signal_sampling_interval, &emergency_cooldown_ms, &control_algorithm,
           &signal_sampling_freq_hz, &hardware_rssi_offset_config, &cooldown_enabled, &frame_sync_enabled);
    
    // Set global hardware RSSI offset
    hardware_rssi_offset = hardware_rssi_offset_config;
    
    // Parse and configure filter chains
    parse_filter_chain(rssi_filter_chain_config, &rssi_filter_chain);
    parse_filter_chain(dbm_filter_chain_config, &dbm_filter_chain);
    parse_filter_chain(racing_rssi_filter_chain_config, &rssi_race_filter_chain);
    parse_filter_chain(racing_dbm_filter_chain_config, &dbm_race_filter_chain);
    
    // Initialize filters with config values
    init_filters(rssi_process_var, rssi_measure_var, dbm_process_var, dbm_measure_var,
                 lpf_cutoff_freq, lpf_sample_freq);
    
    // Initialize PID controller with config values
    pid_init(&bitrate_pid, pid_kp, pid_ki, pid_kd);
    printf("PID Controller initialized: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", pid_kp, pid_ki, pid_kd);
    
    // Print signal sampling configuration
    printf("Signal sampling interval: %d frames (%.1fHz at %d FPS)\n", 
           signal_sampling_interval, (float)target_fps / signal_sampling_interval, target_fps);
    
    // Print emergency cooldown configuration
    printf("Emergency cooldown: %lums (%.1f frames at %d FPS)\n", 
           emergency_cooldown_ms, (float)emergency_cooldown_ms * target_fps / 1000.0, target_fps);
    
    // Print control algorithm configuration
    if (control_algorithm == CONTROL_ALGORITHM_PID) {
        printf("Control algorithm: PID Controller (smooth transitions)\n");
    } else {
        printf("Control algorithm: Simple FIFO (fast, direct)\n");
    }
    
    // Print memory mapping optimization status
    printf("Memory-mapped file optimization: ENABLED\n");
    
    // Set real-time priority for ultra-high performance racing VTX
    set_realtime_priority();
    
    // Initialize frame-sync timing
    init_frame_sync(target_fps);
    
    // Initialize signal sampling timing (independent of frame rate)
    long signal_sampling_interval_ns = 1000000000L / signal_sampling_freq_hz;  // Convert Hz to nanoseconds
    struct timespec last_signal_time = {0, 0};
    clock_gettime(CLOCK_MONOTONIC, &last_signal_time);
    
    printf("Signal sampling frequency: %d Hz (%.2f ms interval)\n", 
           signal_sampling_freq_hz, signal_sampling_interval_ns / 1000000.0);
    printf("Hardware RSSI offset: %d dBm\n", hardware_rssi_offset_config);
    printf("Dynamic RSSI thresholds enabled (MCS-based)\n");
    
    // Print cooldown system status
    if (cooldown_enabled) {
        printf("Cooldown system: ENABLED (asymmetric timing)\n");
    } else {
        printf("Cooldown system: DISABLED (immediate bitrate changes)\n");
    }
    
    // Print frame sync status
    if (frame_sync_enabled) {
        printf("Frame sync: ENABLED (synchronized to %d FPS)\n", target_fps);
    } else {
        printf("Frame sync: DISABLED (maximum responsiveness)\n");
    }
    
    // Initialize worker thread
    sem_init(&worker_sem, 0, 0);
    worker_running = 1;
    if (pthread_create(&worker_thread, NULL, worker_thread_func, NULL) != 0) {
        perror("Failed to create worker thread");
        exit(1);
    }
    
    // Startup delay to allow SSH connections and system stabilization
    printf("Starting up... allowing 5 seconds for SSH connections\n");
    sleep(5);
    printf("Startup complete - beginning adaptive link control\n");

    if (strcmp(NIC, "8812eu2") == 0) {
        strcpy(driverpath, "/proc/net/rtl88x2eu/wlan0");
#ifdef DEBUG
        printf("DEBUG: Using driver path: %s\n", driverpath);
#endif
    }
    else if (strcmp(NIC, "8812au") == 0) {
        strcpy(driverpath, "/proc/net/rtl88xxau/wlan0");
#ifdef DEBUG
        printf("DEBUG: Using driver path: %s\n", driverpath);
#endif
    }
    else {
        printf("ERROR: Unknown WiFi card: %s\n", NIC);
        printf("Available options: 8812eu2, 8812au\n");
        exit(1);
    }
    
    // Check if driver path exists
    char test_path[512];
    snprintf(test_path, sizeof(test_path), "%s/sta_tp_info", driverpath);
    if (access(test_path, R_OK) != 0) {
        printf("ERROR: Driver path does not exist: %s\n", test_path);
        printf("Please check your WiFi card configuration\n");
        printf("Common solutions:\n");
        printf("  1. Change wificard=8812au in config\n");
        printf("  2. Check if WiFi is enabled: iwconfig\n");
        printf("  3. Verify driver is loaded: lsmod | grep 88\n");
        printf("System will continue with disabled bitrate control\n");
        wifi_driver_available = 0;  // Mark WiFi driver as unavailable
    } else {
#ifdef DEBUG
        printf("DEBUG: Driver path verified: %s\n", test_path);
#endif
        wifi_driver_available = 1;  // Mark WiFi driver as available
    }
    
    if (RaceMode != 1 && RaceMode != 0) {
        printf("invalid value for racemode\n");
    } else if (RaceMode == 1) {
       
        printf("RACEMODE ENABLE\n");
        char cmd1[512];
        snprintf(cmd1, sizeof(cmd1), "echo 20 > %s/ack_timeout", driverpath);
        int ack_result = execute_command(cmd1);
        if (ack_result != 0) {
            printf("Warning: Failed to set ack_timeout\n");
        }
        //SET BITRATE MAX 4MBPS
        bitrate_max=4;
        //SET BUFFER SETTING
        int result1 = execute_command("sysctl -w net.core.rmem_default=16384");
        int result2 = execute_command("sysctl -w net.core.rmem_max=65536");
        int result3 = execute_command("sysctl -w net.core.wmem_default=16384");
        int result4 = execute_command("sysctl -w net.core.wmem_max=65536");
        int result5 = execute_command("ifconfig wlan0 txqueuelen 100");
        int result6 = execute_command("sysctl -w net.core.netdev_max_backlog=64");
        
        // Check if any network configuration commands failed
        if (result1 != 0 || result2 != 0 || result3 != 0 || result4 != 0 || result5 != 0 || result6 != 0) {
            printf("Warning: Some network buffer configuration commands failed\n");
        }
        //SET racing video configuration with optimized HTTP calls
        char video_config[256];
        snprintf(video_config, sizeof(video_config), "/api/v1/set?video0.size=%s", racing_video_resolution);
        http_get(video_config);
        
        snprintf(video_config, sizeof(video_config), "/api/v1/set?video0.fps=%d", racing_fps);
        http_get(video_config);
        
        snprintf(video_config, sizeof(video_config), "/api/v1/set?isp.exposure=%d", racing_exposure);
        http_get(video_config);                
        
    } else {
        printf("racemode disable\n");
    }
    
    while (1) {
        // Frame-sync timing: sleep for exact frame duration (if enabled)
        if (frame_sync_enabled) {
            struct timespec current_time;
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            
            long elapsed_ns = (current_time.tv_sec - last_frame_time.tv_sec) * 1000000000L + 
                              (current_time.tv_nsec - last_frame_time.tv_nsec);
            
            if (elapsed_ns < frame_interval_ns) {
                // Sleep for the remaining frame duration
                long sleep_ns = frame_interval_ns - elapsed_ns;
                struct timespec sleep_time = {
                    .tv_sec = sleep_ns / 1000000000L,
                    .tv_nsec = sleep_ns % 1000000000L
                };
                nanosleep(&sleep_time, NULL);
            }
            
            // Update frame time for next iteration
            clock_gettime(CLOCK_MONOTONIC, &last_frame_time);
        }
        
        loop_counter++;
        
        // Independent signal sampling timing (not tied to frame rate)
        // This allows for higher frequency signal sampling than frame rate
        struct timespec current_signal_time;
        clock_gettime(CLOCK_MONOTONIC, &current_signal_time);
        
        long signal_elapsed_ns = (current_signal_time.tv_sec - last_signal_time.tv_sec) * 1000000000L + 
                                 (current_signal_time.tv_nsec - last_signal_time.tv_nsec);
        
#ifdef DEBUG
        bool new_signal_data = false;  // Flag for debug output
#endif
        if (signal_elapsed_ns >= signal_sampling_interval_ns) {
            // Time to sample signal - update timing
            last_signal_time = current_signal_time;
            
            // Read signal data
            currentDb = dbm - aDb;
            dbm = get_dbm();
            aDb = dbm; 
            rssi = get_rssi(driverpath);
            
            // Apply filter chains to smooth the signals
            // Use racing filter chains when race_mode=1, normal filter chains when race_mode=0
            if (RaceMode == 1) {
                filtered_rssi = apply_filter_chain(&rssi_race_filter_chain, (float)rssi);
                filtered_dbm = apply_filter_chain(&dbm_race_filter_chain, (float)dbm);
            } else {
                filtered_rssi = apply_filter_chain(&rssi_filter_chain, (float)rssi);
                filtered_dbm = apply_filter_chain(&dbm_filter_chain, (float)dbm);
            }
            
            // Check for emergency drop conditions
            check_emergency_drop(last_bitrate, filtered_rssi, emergency_rssi_threshold, emergency_bitrate);
            
#ifdef DEBUG
            // Flag that we have new signal data for debug output
            new_signal_data = true;
#endif
        }
        
        //calculation of dbm_Max dbm_Min
        dbm_Max = -50;      
        dbm_Min = (filtered_rssi > 55) ? -70 : (filtered_rssi >= 40 ? -55 : -53);

        // Ensure dbm_Min is always less than dbm_Max to prevent divide by zero
        if (dbm_Min >= dbm_Max) {
            dbm_Min = dbm_Max - 1;  // Force at least 1dB difference
        }

        // Calculate VLQ with bounds checking
        double vlq;
        if (dbm_Max == dbm_Min) {
            // Fallback: if somehow they're still equal, use RSSI-based calculation
            vlq = (filtered_rssi > 0) ? ((double)filtered_rssi / 100.0) * 100.0 : 0.0;
        } else {
            // Additional safety check for invalid dBm values
            if (filtered_dbm < -100 || filtered_dbm > 0) {
                // Invalid dBm reading, use RSSI fallback
                vlq = (filtered_rssi > 0) ? ((double)filtered_rssi / 100.0) * 100.0 : 0.0;
            } else {
                vlq = ((double)((filtered_dbm) - (dbm_Min)) / (double)((dbm_Max) - (dbm_Min))) * 100.0;
            }
        }

        // Clamp VLQ to valid range (0-100%)
        if (vlq < 0.0) vlq = 0.0;
        if (vlq > 100.0) vlq = 100.0;
      
#ifdef DEBUG
        // Only show debug output when we read new signal data
        static int debug_counter = 0;
        if (new_signal_data && (++debug_counter % 10 == 0)) {  // Show every 10th sample
            printf("vlq = %.2f%%\n", vlq);
            printf("rssi = %d (filtered: %.1f)\n", rssi, filtered_rssi);
            printf("adb= %d\n", aDb);
            printf("dbm= %d (filtered: %.1f)\n", dbm, filtered_dbm);
            printf("dbm_Max = %d, dbm_Min = %d\n", dbm_Max, dbm_Min);
            printf("current %d\n", currentDb);
        }
#endif
        mspLQ((int)filtered_rssi);

             
        
        //MAIN LOGIC

        // Emergency stop if RSSI is zero (driver issue)
        if (rssi == 0) {
            printf("ERROR: RSSI is zero - driver path issue detected!\n");
            printf("Current driver path: %s\n", driverpath);
            printf("Please check WiFi card configuration and driver paths\n");
            printf("System will continue but bitrate changes are disabled\n");
            // Don't change bitrate if RSSI is zero to prevent constant changes
            // Sleep longer to reduce CPU usage and allow SSH access
            usleep(500000); // 500ms sleep to reduce CPU load and allow SSH
            continue;
        }

        // Skip bitrate control if WiFi driver is not available
        if (!wifi_driver_available) {
            printf("WiFi driver not available - skipping bitrate control\n");
            usleep(100000); // 100ms sleep to reduce CPU usage
            continue;
        }

        // WiFi performance mode optimization
        if (wifi_performance_mode == 0) {
            // Normal mode - standard processing
            usleep(50000); // 50ms sleep for normal mode
        } else if (wifi_performance_mode == 2) {
            // Ultra-low latency mode - minimal sleep
            usleep(10000); // 10ms sleep for ultra-low latency
        } else {
            // Mode 1 (high performance) - small sleep to prevent CPU spinning
            usleep(1000); // 1ms sleep to prevent 100% CPU
        }

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
                // Calculate target bitrate using VLQ
                int target_bitrate = (int)(bitrate_max * vlq / 100.0);
                
                if (control_algorithm == CONTROL_ALGORITHM_PID) {
                    // PID Controller: Smooth transitions with PID control
                    int pid_adjustment = pid_calculate(&bitrate_pid, target_bitrate, last_bitrate);
                    bitrate = last_bitrate + pid_adjustment;
                    
#ifdef DEBUG
                    if (new_signal_data) {
                        printf("PID: Target=%d, Current=%d, Adj=%d, Final=%d\n", 
                               target_bitrate, last_bitrate, pid_adjustment, bitrate);
                    }
#endif
                } else {
                    // Simple FIFO: Direct assignment (faster, more responsive)
                    bitrate = target_bitrate;
                    
#ifdef DEBUG
                    if (new_signal_data) {
                        printf("FIFO: Target=%d, Final=%d\n", target_bitrate, bitrate);
                    }
#endif
                }
                
                // Clamp bitrate to valid range
                if (bitrate < bitrate_min) bitrate = bitrate_min;
                if (bitrate > bitrate_max) bitrate = bitrate_max;
            }
            }

            // Apply cooldown logic before changing bitrate (if enabled)
            if (cooldown_enabled == 0 || should_change_bitrate(bitrate, last_bitrate, strict_cooldown_ms, up_cooldown_ms, min_change_percent, emergency_cooldown_ms)) {
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
    
    // Cleanup worker thread
    worker_running = 0;
    sem_post(&worker_sem);  // Wake up worker to exit
    pthread_join(worker_thread, NULL);
    sem_destroy(&worker_sem);

    // Cleanup memory maps
    cleanup_memory_maps();

    return 0;
}
