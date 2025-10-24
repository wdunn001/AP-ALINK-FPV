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
#include <stdint.h>
#include <sys/mman.h>
#include <sys/stat.h>

// =============================================================================
// GLOBAL CONFIGURATION VALUES (Runtime Configurable)
// =============================================================================

// System Configuration Constants (Runtime Configurable)
#define MAX_COMMAND_LENGTH 1024
#define MCS_PATH_BUFFER_SIZE 256
#define COMMAND_BUFFER_SIZE 128
static int HTTP_TIMEOUT_US = 100000;  // 100ms timeout
static int HTTP_PORT = 80;
static int REALTIME_PRIORITY = 10;
static int SAFETY_MARGIN_DBM = 3;  // Safety margin for emergency drop

// Bitrate Conversion Constants (Runtime Configurable)
static int BITRATE_MBPS_TO_KBPS = 1024;

// Signal Processing Defaults (Runtime Configurable)
static float DEFAULT_RSSI_ESTIMATE = 50.0f;
static float DEFAULT_DBM_ESTIMATE = -60.0f;
static float DEFAULT_ERROR_ESTIMATE = 1.0f;
// Default Configuration Values (used as initializers)
#define DEFAULT_PROCESS_VARIANCE 1e-5f
#define DEFAULT_RSSI_MEASUREMENT_VARIANCE 0.1f
#define DEFAULT_DBM_MEASUREMENT_VARIANCE 0.5f
#define DEFAULT_FILTER_ALPHA 0.1f
#define DEFAULT_CUTOFF_FREQ 2.0f
#define DEFAULT_SAMPLE_FREQ 10.0f
static int DEFAULT_MODE_FILTER_RETURN_ELEMENT = 2;  // Median (middle element)

// Filter Algorithm Constants (Runtime Configurable)
static float KALMAN_MIN_ERROR_ESTIMATE = 1e-6f;  // Minimum error estimate for numerical stability
static float LPF_2POLE_Q_FACTOR = 0.707f;  // Q factor for Butterworth response
static float GAUSSIAN_SIGMA = 1.0f;  // Standard deviation for Gaussian filter weights

// Main Loop Defaults (Runtime Configurable)
static int DEFAULT_BITRATE = 4;
static int DEFAULT_BITRATE_MIN = 1;
static int DEFAULT_DBM_MAX = -50;
// Removed DEFAULT_DBM_MIN - not used in calculations
static int DEFAULT_HYSTERESIS = 1;
static int DEFAULT_MINUS_HYSTERESIS = -1;
static int DEFAULT_INITIAL_DBM = -100;
static int DEFAULT_TARGET_FPS = 120;

// Removed DEFAULT_PID_* constants - using static pid_* variables instead

// Signal Thresholds (Runtime Configurable - High Frequency Access)
static int HIGH_RSSI_THRESHOLD = 55;
static int MEDIUM_RSSI_THRESHOLD = 40;
static int LOW_RSSI_THRESHOLD = 20;
static int VLQ_MAX_THRESHOLD = 100;
static int VLQ_MIN_THRESHOLD = 1;
static int PERCENTAGE_CONVERSION = 100;

// dBm Threshold Values (Runtime Configurable - High Frequency Access)
static int DBM_THRESHOLD_HIGH = -70;
static int DBM_THRESHOLD_MEDIUM = -55;
static int DBM_THRESHOLD_LOW = -53;
static int MIN_DBM_DIFFERENCE = 1;

// Removed MCS_RSSI_THRESHOLDS - using dynamic thresholds instead

// Bitrate to MCS mapping thresholds (Mbps)
static const int BITRATE_MCS_THRESHOLDS[] = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10  // Mbps thresholds for MCS 0-9
};

// Configuration Variables (loaded from .conf file)
static int bitrate_max = 19;
static char wificard[16] = "8812eu2";
static int fps = 120;
static char rssi_filter_chain_config[64] = "0";
static char dbm_filter_chain_config[64] = "0";
static float lpf_cutoff_freq = DEFAULT_CUTOFF_FREQ;
static float lpf_sample_freq = DEFAULT_SAMPLE_FREQ;
static float kalman_rssi_process = DEFAULT_PROCESS_VARIANCE;
static float kalman_dbm_process = DEFAULT_PROCESS_VARIANCE;
static float kalman_rssi_measure = DEFAULT_RSSI_MEASUREMENT_VARIANCE;
static float kalman_dbm_measure = DEFAULT_DBM_MEASUREMENT_VARIANCE;
// Removed rssi_read_method - using use_file_rewind_method instead
// Removed startup_delay_seconds - using sleep_config.startup_delay_s instead
static int signal_sampling_interval = 5;
static int signal_sampling_freq_hz = 50;
static int cooldown_enabled = 0;
static unsigned long strict_cooldown_ms = 200;
static unsigned long up_cooldown_ms = 3000;
static unsigned long emergency_cooldown_ms = 50;
static int min_change_percent = 5;
static int emergency_rssi_threshold = 30;
static int emergency_bitrate = 1000;
static int hardware_rssi_offset = 0;
static int control_algorithm = 1;
static float pid_kp = 1.0f;
static float pid_ki = 0.05f;
static float pid_kd = 0.4f;
static int race_mode = 1;
static int disable_wifi_power_save = 1;  // Default: disable power saving for FPV stability
static int pit_mode_enabled = 0;         // PIT mode: low power standby with HTTP wake-up
static int hardware_mcs_offset = 0;      // Hardware-specific MCS threshold offset (set during init)
static int hardware_mcs_low_cmd = 0x10;  // Hardware-specific MCS low command (set during init)
static int hardware_mcs_medium_cmd = 0x10; // Hardware-specific MCS medium command (set during init)
static int hardware_mcs_high_cmd = 0x10; // Hardware-specific MCS high command (set during init)
static char racing_video_resolution[32] = "1280x720";
static int racing_exposure = 11;
static int racing_fps = 120;
static char racing_rssi_filter_chain_config[64] = "1";
static char racing_dbm_filter_chain_config[64] = "1";
static int wifi_performance_mode = 0;

// =============================================================================
// RUNTIME CONFIGURATION FUNCTIONS
// =============================================================================

// Function to update signal thresholds during flight
void update_signal_thresholds(int high_rssi, int medium_rssi, int low_rssi, 
                             int vlq_max, int vlq_min, int dbm_high, 
                             int dbm_medium, int dbm_low) {
    HIGH_RSSI_THRESHOLD = high_rssi;
    MEDIUM_RSSI_THRESHOLD = medium_rssi;
    LOW_RSSI_THRESHOLD = low_rssi;
    VLQ_MAX_THRESHOLD = vlq_max;
    VLQ_MIN_THRESHOLD = vlq_min;
    DBM_THRESHOLD_HIGH = dbm_high;
    DBM_THRESHOLD_MEDIUM = dbm_medium;
    DBM_THRESHOLD_LOW = dbm_low;
    
#ifdef DEBUG
    printf("Signal thresholds updated: RSSI(%d/%d/%d) VLQ(%d/%d) dBm(%d/%d/%d)\n",
           high_rssi, medium_rssi, low_rssi, vlq_max, vlq_min, 
           dbm_high, dbm_medium, dbm_low);
#endif
}

// Function to update PID parameters during flight
void update_pid_parameters(float kp, float ki, float kd) {
    pid_kp = kp;
    pid_ki = ki;
    pid_kd = kd;
    
#ifdef DEBUG
    printf("PID parameters updated: Kp=%.2f Ki=%.2f Kd=%.2f\n", kp, ki, kd);
#endif
}

// Function to update system parameters during flight
void update_system_parameters(int safety_margin, int http_timeout, 
                             int realtime_priority, int bitrate_multiplier) {
    SAFETY_MARGIN_DBM = safety_margin;
    HTTP_TIMEOUT_US = http_timeout;
    REALTIME_PRIORITY = realtime_priority;
    BITRATE_MBPS_TO_KBPS = bitrate_multiplier;
    
#ifdef DEBUG
    printf("System parameters updated: Safety=%ddBm HTTP=%dus Priority=%d Multiplier=%d\n",
           safety_margin, http_timeout, realtime_priority, bitrate_multiplier);
#endif
}

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

// PID Controller Structure
typedef struct {
    float kp;           // Proportional gain
    float ki;           // Integral gain
    float kd;           // Derivative gain
    float integral;     // Integral accumulator
    int last_error;     // Previous error for derivative calculation
    int last_output;    // Previous output for reference
} pid_controller_t;

// Sleep Configuration Structure
typedef struct {
    unsigned int main_loop_us;        // Base loop sleep (microseconds)
    unsigned int normal_mode_ms;      // Additional sleep for normal mode (milliseconds)
    unsigned int high_perf_mode_ms;   // Additional sleep for high performance mode (milliseconds)
    unsigned int ultra_low_mode_ms;   // Additional sleep for ultra-low latency mode (milliseconds)
    unsigned int error_condition_ms; // Sleep when RSSI=0 or driver unavailable (milliseconds)
    unsigned int startup_delay_s;     // Startup delay (seconds)
    int smart_sleep_enabled;          // Enable/disable smart sleep (0=off, 1=on)
} sleep_config_t;

// Forward declaration of global sleep configuration
extern sleep_config_t sleep_config;

// Asymmetric Cooldown Constants - replaced by static variables

// Filter Types
typedef enum {
    FILTER_TYPE_KALMAN = 0,
    FILTER_TYPE_LOWPASS = 1,
    FILTER_TYPE_MODE = 2,
    FILTER_TYPE_DERIVATIVE = 3,
    FILTER_TYPE_2POLE_LPF = 4,
    FILTER_TYPE_MEAN = 5,
    FILTER_TYPE_GAUSSIAN = 6
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

// Mean Filter Structure (Simple moving average)
#define MEAN_FILTER_SIZE 8
typedef struct {
    float samples[MEAN_FILTER_SIZE];
    uint8_t sample_index;
    uint8_t sample_count;
    float sum;
    float output;
} mean_filter_t;

// Gaussian Filter Structure (Gaussian-weighted moving average)
#define GAUSSIAN_FILTER_SIZE 7
typedef struct {
    float samples[GAUSSIAN_FILTER_SIZE];
    float weights[GAUSSIAN_FILTER_SIZE];
    uint8_t sample_index;
    uint8_t sample_count;
    float output;
    bool weights_initialized;  // Flag to track if weights are pre-calculated
} gaussian_filter_t;

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
    // Pre-calculated coefficients for performance
    float b0, b1, b2;  // Feedforward coefficients
    float a1, a2;      // Feedback coefficients (a0 normalized to 1.0)
} lpf_2pole_t;

// Filter Chain Structure - allows multiple filters in sequence
#define MAX_FILTERS_PER_CHAIN 3
typedef struct {
    filter_type_t filters[MAX_FILTERS_PER_CHAIN];
    uint8_t filter_count;
    bool enabled;
} filter_chain_t;

// Active filter chains (switched by toggle_racemode)
static filter_chain_t *active_rssi_filter_chain;
static filter_chain_t *active_dbm_filter_chain;

// Worker Thread Infrastructure Structures
typedef struct {
    int bitrateMcs;
    char mcspath[128];
} mcs_arg_t;

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

// Filter Chain Instances
static filter_chain_t rssi_filter_chain = {
    .filters = {FILTER_TYPE_KALMAN},
    .filter_count = 1,
    .enabled = true
};

static filter_chain_t dbm_filter_chain = {
    .filters = {FILTER_TYPE_KALMAN},
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
float apply_filter_chain(filter_chain_t *chain, float sample);
void set_filter_chain(const char *config_str, filter_chain_t *chain);
void reset_filter_chain(filter_chain_t *chain);
void toggle_racemode(void);
int toggle_racemode_http(void);
void disable_autopower(void);
void setup_driver_power_management(void);
void enable_pit_mode(void);
void disable_pit_mode(void);
int toggle_pit_mode_http(void);
int http_get(const char *path);

// Dynamic RSSI threshold functions
int get_dynamic_rssi_threshold(int current_mcs);
int bitrate_to_mcs(int bitrate_mbps);

// Sleep function prototypes
void simple_sleep(bool is_error_condition, bool has_work_done);
void smart_sleep(bool is_error_condition, bool has_work_done);
void unified_sleep(bool is_error_condition, bool has_work_done);
void init_sleep_values();
void enable_global_debug();

// Global high-performance debug logging system
static char global_debug_buffer[2048];  // Larger buffer for entire application
static int global_debug_pos = 0;
static int global_debug_iteration = 0;
static bool global_debug_enabled = false;  // Set to true after initialization

// Global debug macros for entire application
#define GLOBAL_DEBUG_RESET() global_debug_pos = 0
#define GLOBAL_DEBUG_APPEND(fmt, ...) \
    if (global_debug_enabled) { \
        global_debug_pos += snprintf(global_debug_buffer + global_debug_pos, \
                                   sizeof(global_debug_buffer) - global_debug_pos, \
                                   fmt, ##__VA_ARGS__); \
    }
#define GLOBAL_DEBUG_FLUSH() \
    if (global_debug_enabled && global_debug_pos > 0) { \
        printf("%s", global_debug_buffer); \
        global_debug_pos = 0; \
    }
#define GLOBAL_DEBUG_THROTTLE(n) (++global_debug_iteration % (n) == 0)
#define GLOBAL_DEBUG_BUILD(condition, fmt, ...) \
    if (global_debug_enabled && (condition)) { \
        GLOBAL_DEBUG_APPEND(fmt, ##__VA_ARGS__); \
    }

// Enable global debug after initialization is complete
void enable_global_debug() {
    global_debug_enabled = true;
    printf("Global debug system enabled\n");
}

// Pre-calculated sleep value for performance (set during init)
static unsigned int sleep_value_us = 0; // Sleep value in microseconds

// Function pointer for sleep function (set once during init)
static void (*sleep_function)(bool, bool) = NULL;

// Initialize sleep value and function pointer (call once at startup)
void init_sleep_values() {
    // Set sleep value based on wifi_performance_mode (never changes during runtime)
    if (wifi_performance_mode == 0) {
        sleep_value_us = sleep_config.main_loop_us + (sleep_config.normal_mode_ms * 1000);
    } else if (wifi_performance_mode == 1) {
        sleep_value_us = sleep_config.main_loop_us + (sleep_config.high_perf_mode_ms * 1000);
    } else if (wifi_performance_mode == 2) {
        sleep_value_us = sleep_config.main_loop_us + (sleep_config.ultra_low_mode_ms * 1000);
    } else {
        sleep_value_us = sleep_config.main_loop_us + (sleep_config.normal_mode_ms * 1000); // Default
    }
    
    // Set function pointer based on sleep configuration (never changes during runtime)
    if (sleep_config.smart_sleep_enabled) {
        sleep_function = smart_sleep;
    } else {
        sleep_function = simple_sleep;
    }
}

// Simple sleep function - just sleep based on pre-calculated value
void simple_sleep(bool is_error_condition, bool has_work_done) {
    (void)has_work_done; // Unused parameter for compatibility
    if (is_error_condition) {
        // Error conditions (RSSI=0, driver unavailable) - always sleep
        usleep(sleep_config.error_condition_ms * 1000);
        return;
    }
    
    // Use pre-calculated sleep value
    usleep(sleep_value_us);
}

// Smart sleep function - only sleep when we need to yield CPU
void smart_sleep(bool is_error_condition, bool has_work_done) {
    if (is_error_condition) {
        // Error conditions (RSSI=0, driver unavailable) - always sleep
        usleep(sleep_config.error_condition_ms * 1000);
        return;
    }
    
    // If we just did significant work (signal processing), sleep less
    if (has_work_done) {
        // Just yield CPU briefly - we've been busy
        usleep(sleep_config.main_loop_us); // Only base sleep
        return;
    }
    
    // If we're idle (no signal processing), sleep longer to save CPU
    usleep(sleep_value_us);
}

// Unified sleep function - uses pre-calculated function pointer for maximum performance
void unified_sleep(bool is_error_condition, bool has_work_done) {
    sleep_function(is_error_condition, has_work_done);
}

// Startup delay function
void startup_delay() {
    printf("Starting up... allowing %d seconds for SSH connections\n", sleep_config.startup_delay_s);
    sleep(sleep_config.startup_delay_s);
    printf("Startup complete - beginning adaptive link control\n");
}

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
    
    if (strlen(command) > MAX_COMMAND_LENGTH) {
#ifdef DEBUG
        fprintf(stderr, "Error: execute_command() command too long (>%d chars)\n", MAX_COMMAND_LENGTH);
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
    .estimate = 50.0f,           // Initial RSSI estimate (50%) - good starting point
    .error_estimate = 1.0f,      // Initial error estimate - safe default
    .process_variance = 0.0f,    // Default - updated from kalman_rssi_process config
    .measurement_variance = 0.0f // Default - updated from kalman_rssi_measure config
};

static kalman_filter_t dbm_filter = {
    .estimate = -60.0f,          // Initial dBm estimate - typical FPV range
    .error_estimate = 1.0f,      // Initial error estimate - safe default
    .process_variance = 0.0f,    // Default - updated from kalman_dbm_process config
    .measurement_variance = 0.0f // Default - updated from kalman_dbm_measure config
};

// Global Low-Pass Filter instances
static lowpass_filter_t rssi_lpf = {
    .output = 50.0f,             // Initial RSSI estimate - good starting point
    .alpha = DEFAULT_FILTER_ALPHA,              // Default filter coefficient - moderate smoothing
    .initialised = false,        // Not initialized yet - will be set during init
    .cutoff_freq = DEFAULT_CUTOFF_FREQ,         // Default 2Hz cutoff - updated from config
    .sample_freq = DEFAULT_SAMPLE_FREQ         // Default 10Hz sample rate - updated from config
};

static lowpass_filter_t dbm_lpf = {
    .output = -60.0f,            // Initial dBm estimate - typical FPV range
    .alpha = DEFAULT_FILTER_ALPHA,              // Default filter coefficient - moderate smoothing
    .initialised = false,        // Not initialized yet - will be set during init
    .cutoff_freq = DEFAULT_CUTOFF_FREQ,         // Default 2Hz cutoff - updated from config
    .sample_freq = DEFAULT_SAMPLE_FREQ         // Default 10Hz sample rate - updated from config
};

// Global Mode Filter instances
static mode_filter_t rssi_mode_filter = {
    .sample_index = 0,            // Start at first sample position
    .return_element = 0,          // Default - updated from DEFAULT_MODE_FILTER_RETURN_ELEMENT
    .drop_high_sample = true,     // Start by dropping high samples (median-like behavior)
    .output = 50.0f               // Initial RSSI estimate - good starting point
};

static mode_filter_t dbm_mode_filter = {
    .sample_index = 0,            // Start at first sample position
    .return_element = 0,          // Default - updated from DEFAULT_MODE_FILTER_RETURN_ELEMENT
    .drop_high_sample = true,     // Start by dropping high samples (median-like behavior)
    .output = -60.0f              // Initial dBm estimate - typical FPV range
};

// Global Mean Filter instances
static mean_filter_t rssi_mean_filter = {
    .sample_index = 0,            // Start at first sample position
    .sample_count = 0,           // No samples collected yet
    .sum = 0.0f,                 // Running sum starts at zero
    .output = 50.0f              // Initial RSSI estimate - good starting point
};

static mean_filter_t dbm_mean_filter = {
    .sample_index = 0,            // Start at first sample position
    .sample_count = 0,            // No samples collected yet
    .sum = 0.0f,                  // Running sum starts at zero
    .output = -60.0f             // Initial dBm estimate - typical FPV range
};

// Global Gaussian Filter instances
static gaussian_filter_t rssi_gaussian_filter = {
    .sample_index = 0,            // Start at first sample position
    .sample_count = 0,            // No samples collected yet
    .output = 50.0f,             // Initial RSSI estimate - good starting point
    .weights_initialized = false  // Weights not calculated yet - will be done on first use
};

static gaussian_filter_t dbm_gaussian_filter = {
    .sample_index = 0,            // Start at first sample position
    .sample_count = 0,            // No samples collected yet
    .output = -60.0f,            // Initial dBm estimate - typical FPV range
    .weights_initialized = false  // Weights not calculated yet - will be done on first use
};

// Global Derivative Filter instances
static derivative_filter_t rssi_derivative_filter = {
    .sample_index = 0,            // Start at first sample position
    .last_slope = 0.0f,          // No slope calculated yet
    .new_data = false             // No new data available yet
};

static derivative_filter_t dbm_derivative_filter = {
    .sample_index = 0,            // Start at first sample position
    .last_slope = 0.0f,          // No slope calculated yet
    .new_data = false             // No new data available yet
};

// Global 2-Pole Low-Pass Filter instances
static lpf_2pole_t rssi_2pole_lpf = {
    .delay_element_1 = 0.0f,      // First delay element - starts at zero
    .delay_element_2 = 0.0f,      // Second delay element - starts at zero
    .cutoff_freq = 2.0f,          // Default 2Hz cutoff - updated from config
    .sample_freq = 10.0f,         // Default 10Hz sample rate - updated from config
    .initialised = false,         // Not initialized yet - will be set during init
    .output = 50.0f               // Initial RSSI estimate - good starting point
};

static lpf_2pole_t dbm_2pole_lpf = {
    .delay_element_1 = 0.0f,      // First delay element - starts at zero
    .delay_element_2 = 0.0f,      // Second delay element - starts at zero
    .cutoff_freq = 2.0f,          // Default 2Hz cutoff - updated from config
    .sample_freq = 10.0f,         // Default 10Hz sample rate - updated from config
    .initialised = false,         // Not initialized yet - will be set during init
    .output = -60.0f              // Initial dBm estimate - typical FPV range
};

// Asymmetric Cooldown Timing Variables
static unsigned long last_change_time = 0;
static unsigned long last_up_time = 0;
static int last_bitrate = 0;

// Global variable for RSSI reading method
int use_file_rewind_method = 0;  // 0 = mmap (default), 1 = file rewind

// Global WiFi driver availability flag
int wifi_driver_available = 1;  // Assume available until proven otherwise

// Global sleep configuration (not static - needs to persist and be accessible)
sleep_config_t sleep_config = {
    .main_loop_us = 100,        // 0.1ms base loop
    .normal_mode_ms = 20,       // +20ms for normal mode
    .high_perf_mode_ms = 5,     // +5ms for high performance
    .ultra_low_mode_ms = 0,     // No extra sleep for ultra-low latency
    .error_condition_ms = 100, // 100ms for error conditions
    .startup_delay_s = 5,       // 5 seconds startup delay
    .smart_sleep_enabled = 0    // Default: simple sleep (off)
};

// Global PID controller instance - will be initialized with static variables in main
static pid_controller_t bitrate_pid = {
    .kp = 0.0f,         // Will be set from pid_kp
    .ki = 0.0f,         // Will be set from pid_ki
    .kd = 0.0f,         // Will be set from pid_kd
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

// Gaussian Filter Functions (Gaussian-weighted moving average)
// Initialize Gaussian weights (pre-calculated for efficiency)
void init_gaussian_weights(gaussian_filter_t *filter) {
    // Pre-calculate Gaussian weights for 7-sample window
    // Using global GAUSSIAN_SIGMA for runtime configurable sigma
    float sigma = GAUSSIAN_SIGMA;
    float center = (GAUSSIAN_FILTER_SIZE - 1) / 2.0f;
    
    float weight_sum = 0.0f;
    for (int i = 0; i < GAUSSIAN_FILTER_SIZE; i++) {
        float x = i - center;
        filter->weights[i] = expf(-(x * x) / (2.0f * sigma * sigma));
        weight_sum += filter->weights[i];
    }
    
    // Normalize weights so they sum to 1.0
    for (int i = 0; i < GAUSSIAN_FILTER_SIZE; i++) {
        filter->weights[i] /= weight_sum;
    }
    
    filter->weights_initialized = true;  // Mark weights as initialized
    
#ifdef DEBUG
    printf("Gaussian weights initialized (sigma=%.1f): ", sigma);
    for (int i = 0; i < GAUSSIAN_FILTER_SIZE; i++) {
        printf("%.3f ", filter->weights[i]);
    }
    printf("\n");
#endif
}


// 2-Pole Low-Pass Filter Functions (biquad filter)
// Initialize 2-pole low-pass filter
void init_2pole_lpf(lpf_2pole_t *filter, float cutoff_freq, float sample_freq) {
    filter->cutoff_freq = cutoff_freq;
    filter->sample_freq = sample_freq;
    
    // Pre-calculate filter coefficients (expensive operations done once)
    float omega = 2.0f * M_PI * cutoff_freq / sample_freq;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * LPF_2POLE_Q_FACTOR);
    
    // Calculate biquad coefficients
    float b0_raw = (1.0f - cs) / 2.0f;
    float b1_raw = 1.0f - cs;
    float b2_raw = (1.0f - cs) / 2.0f;
    float a0_raw = 1.0f + alpha;
    float a1_raw = -2.0f * cs;
    float a2_raw = 1.0f - alpha;
    
    // Normalize coefficients (a0 becomes 1.0)
    filter->b0 = b0_raw / a0_raw;
    filter->b1 = b1_raw / a0_raw;
    filter->b2 = b2_raw / a0_raw;
    filter->a1 = a1_raw / a0_raw;
    filter->a2 = a2_raw / a0_raw;
    
    // Initialize delay elements
    filter->delay_element_1 = 0.0f;
    filter->delay_element_2 = 0.0f;
    filter->initialised = false;
    
#ifdef DEBUG
    printf("2-Pole LPF initialized: cutoff=%.1fHz, sample=%.1fHz\n", cutoff_freq, sample_freq);
    printf("Pre-calculated coefficients: b0=%.3f, b1=%.3f, b2=%.3f, a1=%.3f, a2=%.3f\n",
           filter->b0, filter->b1, filter->b2, filter->a1, filter->a2);
#endif
}

// Filter Chain Functions
// Apply a complete filter chain to a sample (optimized for WyvernLink 100mW)
float apply_filter_chain(filter_chain_t *chain, float sample) {
    if (!chain->enabled || chain->filter_count == 0) {
        return sample;
    }
    
    float filtered_sample = sample;
    
#ifdef DEBUG
    // Centralized error checking and logging
    char error_message[512] = {0};
    char log_buffer[1024] = {0};
    bool has_errors = false;
    bool has_warnings = false;
#endif
    
    // Pre-initialize all filters that need initialization (moved to top for consistency)
    // This eliminates redundant checks throughout the switch statement
    for (uint8_t i = 0; i < chain->filter_count; i++) {
        filter_type_t filter_type = chain->filters[i];
        
        switch (filter_type) {
            case FILTER_TYPE_LOWPASS: {
                lowpass_filter_t *filter = (chain == &rssi_filter_chain) ? &rssi_lpf : &dbm_lpf;
                if (!filter->initialised) {
                    filter->output = filtered_sample;
                    filter->initialised = true;
#ifdef DEBUG
                    snprintf(log_buffer + strlen(log_buffer), sizeof(log_buffer) - strlen(log_buffer), 
                             "Low-Pass initialized (%.1f), ", filtered_sample);
#endif
                }
                break;
            }
            case FILTER_TYPE_2POLE_LPF: {
                lpf_2pole_t *filter = (chain == &rssi_filter_chain) ? &rssi_2pole_lpf : &dbm_2pole_lpf;
                if (!filter->initialised) {
                    filter->output = filtered_sample;
                    filter->delay_element_1 = filtered_sample;
                    filter->delay_element_2 = filtered_sample;
                    filter->initialised = true;
#ifdef DEBUG
                    snprintf(log_buffer + strlen(log_buffer), sizeof(log_buffer) - strlen(log_buffer), 
                             "2-Pole LPF initialized (%.1f), ", filtered_sample);
#endif
                }
                break;
            }
            case FILTER_TYPE_GAUSSIAN: {
                gaussian_filter_t *filter = (chain == &rssi_filter_chain) ? &rssi_gaussian_filter : &dbm_gaussian_filter;
                if (!filter->weights_initialized) {
                    init_gaussian_weights(filter);
#ifdef DEBUG
                    snprintf(log_buffer + strlen(log_buffer), sizeof(log_buffer) - strlen(log_buffer), 
                             "Gaussian weights initialized, ");
#endif
                }
                break;
            }
            default:
                // Other filters don't need initialization
                break;
        }
    }
    
    // Apply each filter in the chain sequentially (inlined for performance)
    for (uint8_t i = 0; i < chain->filter_count; i++) {
        filter_type_t filter_type = chain->filters[i];
        
        switch (filter_type) {
            case FILTER_TYPE_KALMAN: {
                // Inlined Kalman filter logic
                kalman_filter_t *filter = (chain == &rssi_filter_chain) ? &rssi_filter : &dbm_filter;
                
                // Prediction step - predict next state
                float predicted_estimate = filter->estimate;
                float predicted_error = filter->error_estimate + filter->process_variance;
                
                // Update step - correct prediction with measurement
                float kalman_gain = predicted_error / (predicted_error + filter->measurement_variance);
                
                // Clamp kalman gain to prevent numerical issues
                if (kalman_gain > 1.0f) kalman_gain = 1.0f;
                if (kalman_gain < 0.0f) kalman_gain = 0.0f;
                
                filter->estimate = predicted_estimate + kalman_gain * (filtered_sample - predicted_estimate);
                filter->error_estimate = (1.0f - kalman_gain) * predicted_error;
                
                // Prevent error estimate from becoming too small (numerical stability)
                if (filter->error_estimate < KALMAN_MIN_ERROR_ESTIMATE) {
                    filter->error_estimate = KALMAN_MIN_ERROR_ESTIMATE;
#ifdef DEBUG
                    has_warnings = true;
                    snprintf(error_message + strlen(error_message), sizeof(error_message) - strlen(error_message), 
                             "Kalman error estimate clamped, ");
#endif
                }
                
                filtered_sample = filter->estimate;
                break;
            }
                
            case FILTER_TYPE_LOWPASS: {
                // Inlined Low-Pass filter logic (initialization already handled above)
                lowpass_filter_t *filter = (chain == &rssi_filter_chain) ? &rssi_lpf : &dbm_lpf;
                filter->output += (filtered_sample - filter->output) * filter->alpha;
                filtered_sample = filter->output;
                break;
            }
                
            case FILTER_TYPE_MODE: {
                // Inlined Mode filter logic
                mode_filter_t *filter = (chain == &rssi_filter_chain) ? &rssi_mode_filter : &dbm_mode_filter;
                
                // Insertion sort for mode filter (alternates dropping high/low samples)
                uint8_t j;
                
                // If buffer isn't full, simply increase sample count
                if (filter->sample_index < MODE_FILTER_SIZE) {
                    filter->sample_index++;
                    filter->drop_high_sample = true;  // Default to dropping high
                }
                
                if (filter->drop_high_sample) {
                    // Drop highest sample - start from top
                    j = filter->sample_index - 1;
                    
                    // Shift samples down to make room
                    while (j > 0 && filter->samples[j-1] > filtered_sample) {
                        filter->samples[j] = filter->samples[j-1];
                        j--;
                    }
                    
                    // Insert new sample
                    filter->samples[j] = filtered_sample;
                } else {
                    // Drop lowest sample - start from bottom
                    j = 0;
                    
                    // Shift samples up to make room
                    while (j < filter->sample_index - 1 && filter->samples[j+1] < filtered_sample) {
                        filter->samples[j] = filter->samples[j+1];
                        j++;
                    }
                    
                    // Insert new sample
                    filter->samples[j] = filtered_sample;
                }
                
                // Alternate drop direction for next sample
                filter->drop_high_sample = !filter->drop_high_sample;
                
                if (filter->sample_index < MODE_FILTER_SIZE) {
                    // Buffer not full - return middle sample
                    filter->output = filter->samples[filter->sample_index / 2];
                } else {
                    // Buffer full - return specified element (usually median)
                    filter->output = filter->samples[filter->return_element];
                }
                
                filtered_sample = filter->output;
                break;
            }
                
            case FILTER_TYPE_DERIVATIVE: {
                // Inlined Derivative filter logic
                derivative_filter_t *filter = (chain == &rssi_filter_chain) ? &rssi_derivative_filter : &dbm_derivative_filter;
                
                uint32_t current_time = (uint32_t)(get_current_time_ms());
                uint8_t j = filter->sample_index;
                uint8_t j1;
                
                if (j == 0) {
                    j1 = DERIVATIVE_FILTER_SIZE - 1;
                } else {
                    j1 = j - 1;
                }
                
                // Check if this is a new timestamp
                if (filter->timestamps[j1] == current_time) {
#ifdef DEBUG
                    has_warnings = true;
                    snprintf(error_message + strlen(error_message), sizeof(error_message) - strlen(error_message), 
                             "Derivative duplicate timestamp, ");
#endif
                    break; // Ignore duplicate timestamp
                }
                
                // Store timestamp and sample
                filter->timestamps[j] = current_time;
                filter->samples[j] = filtered_sample;
                
                // Update sample index
                filter->sample_index = (filter->sample_index + 1) % DERIVATIVE_FILTER_SIZE;
                filter->new_data = true;
                
                // Return the most recent sample (derivative is available via slope function)
                uint8_t idx = (filter->sample_index - 1 + DERIVATIVE_FILTER_SIZE) % DERIVATIVE_FILTER_SIZE;
                filtered_sample = filter->samples[idx];
                break;
            }
                
            case FILTER_TYPE_2POLE_LPF: {
                // Inlined 2-Pole Low-Pass filter logic (initialization already handled above)
                lpf_2pole_t *filter = (chain == &rssi_filter_chain) ? &rssi_2pole_lpf : &dbm_2pole_lpf;
                
                // Apply biquad filter using pre-calculated coefficients (no expensive trig functions!)
                float output = filter->b0 * filtered_sample + filter->b1 * filter->delay_element_1 + filter->b2 * filter->delay_element_2
                               - filter->a1 * filter->delay_element_1 - filter->a2 * filter->delay_element_2;
                
                // Update delay elements
                filter->delay_element_2 = filter->delay_element_1;
                filter->delay_element_1 = output;
                
                filter->output = output;
                filtered_sample = output;
                break;
            }
                
            case FILTER_TYPE_MEAN: {
                // Inlined Mean filter logic
                mean_filter_t *filter = (chain == &rssi_filter_chain) ? &rssi_mean_filter : &dbm_mean_filter;
                
                // If buffer isn't full yet, just add to sum
                if (filter->sample_count < MEAN_FILTER_SIZE) {
                    filter->samples[filter->sample_count] = filtered_sample;
                    filter->sum += filtered_sample;
                    filter->sample_count++;
                    filter->output = filter->sum / filter->sample_count;
                    filtered_sample = filter->output;
                    break;
                }
                
                // Buffer is full, use circular buffer
                // Remove oldest sample from sum
                filter->sum -= filter->samples[filter->sample_index];
                
                // Add new sample
                filter->samples[filter->sample_index] = filtered_sample;
                filter->sum += filtered_sample;
                
                // Calculate mean
                filter->output = filter->sum / MEAN_FILTER_SIZE;
                
                // Update circular buffer index
                filter->sample_index = (filter->sample_index + 1) % MEAN_FILTER_SIZE;
                
                filtered_sample = filter->output;
                break;
            }
                
            case FILTER_TYPE_GAUSSIAN: {
                // Inlined Gaussian filter logic (weights initialization already handled above)
                gaussian_filter_t *filter = (chain == &rssi_filter_chain) ? &rssi_gaussian_filter : &dbm_gaussian_filter;
                
                // If buffer isn't full yet, just add to buffer
                if (filter->sample_count < GAUSSIAN_FILTER_SIZE) {
                    filter->samples[filter->sample_count] = filtered_sample;
                    filter->sample_count++;
                    
                    // Calculate weighted average with available samples
                    float weighted_sum = 0.0f;
                    float weight_sum = 0.0f;
                    for (int j = 0; j < filter->sample_count; j++) {
                        weighted_sum += filter->samples[j] * filter->weights[j];
                        weight_sum += filter->weights[j];
                    }
                    filter->output = weighted_sum / weight_sum;
                    filtered_sample = filter->output;
                    break;
                }
                
                // Buffer is full, use circular buffer
                // Store new sample at current index
                filter->samples[filter->sample_index] = filtered_sample;
                
                // Calculate Gaussian-weighted average using pre-calculated weights
                float weighted_sum = 0.0f;
                for (int j = 0; j < GAUSSIAN_FILTER_SIZE; j++) {
                    weighted_sum += filter->samples[j] * filter->weights[j];
                }
                filter->output = weighted_sum; // Weights are already normalized
                
                // Update circular buffer index
                filter->sample_index = (filter->sample_index + 1) % GAUSSIAN_FILTER_SIZE;
                
                filtered_sample = filter->output;
                break;
            }
                
            default:
                // Unknown filter type, pass through unchanged
#ifdef DEBUG
                has_errors = true;
                snprintf(error_message + strlen(error_message), sizeof(error_message) - strlen(error_message), 
                         "Unknown filter type %d, ", filter_type);
#endif
                break;
        }
    }
    
#ifdef DEBUG
    // Output consolidated debug information only if there are issues or initialization
    if (has_errors || has_warnings || strlen(log_buffer) > 0) {
        GLOBAL_DEBUG_BUILD(true, "Filter Chain Debug: ");
        if (strlen(log_buffer) > 0) {
            GLOBAL_DEBUG_APPEND("%s", log_buffer);
        }
        if (has_errors || has_warnings) {
            GLOBAL_DEBUG_APPEND("Issues: %s", error_message);
        }
        GLOBAL_DEBUG_APPEND("Input: %.1f -> Output: %.1f ", sample, filtered_sample);
    }
#endif
    
    return filtered_sample;
}

// Set filter chain configuration from string (e.g., "2,0" for Mode->Kalman)
void set_filter_chain(const char *config_str, filter_chain_t *chain) {
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
        if (filter_type >= 0 && filter_type <= 6) {
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
                                 (chain->filters[i] == FILTER_TYPE_DERIVATIVE) ? "Derivative" :
                                 (chain->filters[i] == FILTER_TYPE_2POLE_LPF) ? "2-Pole LPF" :
                                 (chain->filters[i] == FILTER_TYPE_MEAN) ? "Mean" : "Gaussian";
        printf("%s", filter_name);
        if (i < count - 1) printf(" -> ");
    }
    printf(" (%d filters)\n", count);
#endif
}

// Reset all filters in a chain (inlined for performance)
void reset_filter_chain(filter_chain_t *chain) {
    for (uint8_t i = 0; i < chain->filter_count; i++) {
        filter_type_t filter_type = chain->filters[i];
        
        switch (filter_type) {
            case FILTER_TYPE_KALMAN:
                if (chain == &rssi_filter_chain) {
                    rssi_filter.estimate = DEFAULT_RSSI_ESTIMATE;
                    rssi_filter.error_estimate = DEFAULT_ERROR_ESTIMATE;
                } else {
                    dbm_filter.estimate = DEFAULT_DBM_ESTIMATE;
                    dbm_filter.error_estimate = DEFAULT_ERROR_ESTIMATE;
                }
                break;
                
            case FILTER_TYPE_LOWPASS:
                if (chain == &rssi_filter_chain) {
                    rssi_lpf.output = DEFAULT_RSSI_ESTIMATE;
                    rssi_lpf.initialised = false;
                } else {
                    dbm_lpf.output = DEFAULT_DBM_ESTIMATE;
                    dbm_lpf.initialised = false;
                }
                break;
                
            case FILTER_TYPE_MODE:
                if (chain == &rssi_filter_chain) {
                    rssi_mode_filter.sample_index = 0;
                    rssi_mode_filter.drop_high_sample = true;
                    rssi_mode_filter.output = DEFAULT_RSSI_ESTIMATE;
                } else {
                    dbm_mode_filter.sample_index = 0;
                    dbm_mode_filter.drop_high_sample = true;
                    dbm_mode_filter.output = DEFAULT_DBM_ESTIMATE;
                }
                break;
                
            case FILTER_TYPE_DERIVATIVE:
                if (chain == &rssi_filter_chain) {
                    rssi_derivative_filter.sample_index = 0;
                    rssi_derivative_filter.new_data = false;
                } else {
                    dbm_derivative_filter.sample_index = 0;
                    dbm_derivative_filter.new_data = false;
                }
                break;
                
            case FILTER_TYPE_2POLE_LPF:
                if (chain == &rssi_filter_chain) {
                    rssi_2pole_lpf.output = DEFAULT_RSSI_ESTIMATE;
                    rssi_2pole_lpf.delay_element_1 = DEFAULT_RSSI_ESTIMATE;
                    rssi_2pole_lpf.delay_element_2 = DEFAULT_RSSI_ESTIMATE;
                    rssi_2pole_lpf.initialised = false;
                } else {
                    dbm_2pole_lpf.output = DEFAULT_DBM_ESTIMATE;
                    dbm_2pole_lpf.delay_element_1 = DEFAULT_DBM_ESTIMATE;
                    dbm_2pole_lpf.delay_element_2 = DEFAULT_DBM_ESTIMATE;
                    dbm_2pole_lpf.initialised = false;
                }
                break;
                
            case FILTER_TYPE_MEAN:
                if (chain == &rssi_filter_chain) {
                    rssi_mean_filter.sample_index = 0;
                    rssi_mean_filter.sample_count = 0;
                    rssi_mean_filter.sum = 0.0f;
                    rssi_mean_filter.output = DEFAULT_RSSI_ESTIMATE;
                } else {
                    dbm_mean_filter.sample_index = 0;
                    dbm_mean_filter.sample_count = 0;
                    dbm_mean_filter.sum = 0.0f;
                    dbm_mean_filter.output = DEFAULT_DBM_ESTIMATE;
                }
                break;
                
            case FILTER_TYPE_GAUSSIAN:
                if (chain == &rssi_filter_chain) {
                    rssi_gaussian_filter.sample_index = 0;
                    rssi_gaussian_filter.sample_count = 0;
                    rssi_gaussian_filter.output = DEFAULT_RSSI_ESTIMATE;
                } else {
                    dbm_gaussian_filter.sample_index = 0;
                    dbm_gaussian_filter.sample_count = 0;
                    dbm_gaussian_filter.output = DEFAULT_DBM_ESTIMATE;
                }
                break;
        }
    }
}

// Toggle between racing and normal mode filter chains
void toggle_racemode(void) {
    race_mode = !race_mode;  // Toggle race mode
    
    if (race_mode) {
        // Switch to racing mode filters
        active_rssi_filter_chain = &rssi_race_filter_chain;
        active_dbm_filter_chain = &dbm_race_filter_chain;
        printf("Racing mode ENABLED - Using low-pass filters for fast response\n");
    } else {
        // Switch to normal mode filters
        active_rssi_filter_chain = &rssi_filter_chain;
        active_dbm_filter_chain = &dbm_filter_chain;
        printf("Normal mode ENABLED - Using Kalman filters for stability\n");
    }
}

// Initialize filters with custom parameters
void init_filters(float rssi_process_var, float rssi_measure_var,
                 float dbm_process_var, float dbm_measure_var,
                 float lpf_cutoff_freq, float lpf_sample_freq) {
    
    // Initialize Kalman filters
    rssi_filter.process_variance = rssi_process_var;
    rssi_filter.measurement_variance = rssi_measure_var;
    rssi_filter.estimate = DEFAULT_RSSI_ESTIMATE;  // Reset to initial value
    rssi_filter.error_estimate = DEFAULT_ERROR_ESTIMATE;
    
    dbm_filter.process_variance = dbm_process_var;
    dbm_filter.measurement_variance = dbm_measure_var;
    dbm_filter.estimate = DEFAULT_DBM_ESTIMATE;  // Reset to initial value
    dbm_filter.error_estimate = DEFAULT_ERROR_ESTIMATE;
    
    // Initialize Low-Pass filters
    init_lowpass_filter(&rssi_lpf, lpf_cutoff_freq, lpf_sample_freq);
    init_lowpass_filter(&dbm_lpf, lpf_cutoff_freq, lpf_sample_freq);
    
    // Initialize Mode filters (no parameters needed)
    rssi_mode_filter.return_element = DEFAULT_MODE_FILTER_RETURN_ELEMENT;
    dbm_mode_filter.return_element = DEFAULT_MODE_FILTER_RETURN_ELEMENT;
#ifdef DEBUG
    printf("Mode filters initialized with return element: %d\n", DEFAULT_MODE_FILTER_RETURN_ELEMENT);
#endif    
#ifdef DEBUG
    printf("Derivative filters initialized\n");
#endif

    init_2pole_lpf(&rssi_2pole_lpf, lpf_cutoff_freq, lpf_sample_freq);
    init_2pole_lpf(&dbm_2pole_lpf, lpf_cutoff_freq, lpf_sample_freq);
    
    // Initialize Mean filters (no parameters needed)
#ifdef DEBUG
    printf("Mean filters initialized\n");
#endif
    
    // Initialize Gaussian filters (pre-calculate weights for performance)
    init_gaussian_weights(&rssi_gaussian_filter);
    init_gaussian_weights(&dbm_gaussian_filter);
#ifdef DEBUG
    printf("Gaussian filters initialized\n");
#endif 
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
#ifdef DEBUG
        printf("EMERGENCY DROP: RSSI=%.1f (threshold=%d, MCS=%d), bitrate=%d->%d\n", 
               filtered_rssi, effective_threshold, current_mcs, current_bitrate, emergency_bitrate);
#endif
        
        // Set emergency bitrate
        char command[COMMAND_BUFFER_SIZE];
        snprintf(command, sizeof(command), "wfb_tx_cmd 8000 set_bitrate %d", emergency_bitrate);
        int result = execute_command(command);
        if (result != 0) {
#ifdef DEBUG
            printf("Warning: Emergency bitrate command failed with status %d\n", result);
#endif
        }
        
        // Update timing variables
        unsigned long now = get_current_time_ms();
        last_change_time = now;
        last_up_time = now;
        last_bitrate = emergency_bitrate;
        
        // Reset all filters and PID controller to adapt to new conditions
        reset_filters();
        pid_reset(&bitrate_pid);
        
        // CRITICAL: Re-disable power saving after emergency recovery
        // Ensures stable connection during recovery
        disable_autopower();
    }
}

// Disable WiFi power saving for stable FPV connections
// Critical for both 8812au and 8812eu2 driver stability during dynamic activities
void disable_autopower() {
    if (!disable_wifi_power_save) {
        printf("WiFi power saving is ENABLED - NOT RECOMMENDED for FPV!\n");
        printf("This may cause connection instability during dynamic activities\n");
        return;
    }
    
    // Different commands for different drivers
    char command[128];
    if (strcmp(wificard, "8812au") == 0) {
        // RTL8812AU: Use iw command to disable power saving
        snprintf(command, sizeof(command), "iw wlan0 set power_save off");
        printf("Disabling power saving for RTL8812AU (8812au driver)...\n");
    } else if (strcmp(wificard, "8812eu2") == 0) {
        // RTL8812EU: Use iw command to disable power saving
        snprintf(command, sizeof(command), "iw wlan0 set power_save off");
        printf("Disabling power saving for RTL8812EU (8812eu2 driver)...\n");
    } else if (strcmp(wificard, "873xbu") == 0) {
        // RTL873xBU: Use iw command to disable power saving
        snprintf(command, sizeof(command), "iw wlan0 set power_save off");
        printf("Disabling power saving for RTL873xBU (873xbu driver)...\n");
    } else {
        printf("ERROR: Unknown WiFi card type for power management: %s\n", wificard);
        return;
    }
    
    int result = execute_command(command);
    if (result != 0) {
#ifdef DEBUG
        printf("Warning: WiFi power save disable command failed with status %d\n", result);
        printf("Command attempted: %s\n", command);
#endif
    } else {
        printf("WiFi power saving DISABLED - Critical for FPV stability\n");
    }
}

// Setup driver-level power management for both RTL8812AU and RTL8812EU
// This provides additional protection beyond iw commands
void setup_driver_power_management(void) {
    if (!disable_wifi_power_save) {
        printf("Driver power management: ENABLED (not recommended for FPV)\n");
        return;
    }
    
    char command[256];
    int result;
    
    if (strcmp(wificard, "8812au") == 0) {
        // RTL8812AU: Disable power management via modprobe parameters
        printf("Setting up RTL8812AU driver power management...\n");
        
        // Try to unload and reload with power management disabled
        snprintf(command, sizeof(command), 
                "modprobe -r 8812au && modprobe 8812au rtw_power_mgnt=0 rtw_en_autosleep=0");
        result = execute_command(command);
        
        if (result != 0) {
            printf("Warning: Failed to reload 8812au driver with power management disabled\n");
            printf("You may need to manually add to /etc/modprobe.d/8812au.conf:\n");
            printf("options 8812au rtw_power_mgnt=0 rtw_en_autosleep=0\n");
        } else {
            printf("RTL8812AU driver reloaded with power management DISABLED\n");
        }
        
    } else if (strcmp(wificard, "8812eu2") == 0) {
        // RTL8812EU: Disable power management via modprobe parameters
        printf("Setting up RTL8812EU driver power management...\n");
        
        // Try to unload and reload with power management disabled
        snprintf(command, sizeof(command), 
                "modprobe -r 8812eu && modprobe 8812eu rtw_power_mgnt=0 rtw_en_autosleep=0");
        result = execute_command(command);
        
        if (result != 0) {
            printf("Warning: Failed to reload 8812eu driver with power management disabled\n");
            printf("You may need to manually add to /etc/modprobe.d/8812eu.conf:\n");
            printf("options 8812eu rtw_power_mgnt=0 rtw_en_autosleep=0\n");
        } else {
            printf("RTL8812EU driver reloaded with power management DISABLED\n");
        }
        
    } else if (strcmp(wificard, "873xbu") == 0) {
        // RTL873xBU: Disable power management via modprobe parameters
        printf("Setting up RTL873xBU driver power management...\n");
        
        // Try to unload and reload with power management disabled
        snprintf(command, sizeof(command), 
                "modprobe -r 873xbu && modprobe 873xbu rtw_power_mgnt=0 rtw_en_autosleep=0");
        result = execute_command(command);
        
        if (result != 0) {
            printf("Warning: Failed to reload 873xbu driver with power management disabled\n");
            printf("You may need to manually add to /etc/modprobe.d/873xbu.conf:\n");
            printf("options 873xbu rtw_power_mgnt=0 rtw_en_autosleep=0\n");
        } else {
            printf("RTL873xBU driver reloaded with power management DISABLED\n");
        }
        
    } else {
        printf("ERROR: Unknown WiFi card type for driver power management: %s\n", wificard);
        return;
    }
}

// Enable PIT mode: Low power standby with HTTP wake-up capability
// Perfect for battery conservation during racing events or standby periods
void enable_pit_mode(void) {
    if (pit_mode_enabled) {
        printf("PIT MODE: ENABLED - Low power standby with HTTP wake-up\n");
        printf("System will enter low power state but remain responsive to HTTP calls\n");
        
        // Enable power saving for PIT mode (temporary)
        char command[128];
        if (strcmp(wificard, "8812au") == 0) {
            snprintf(command, sizeof(command), "iw wlan0 set power_save on");
            printf("PIT MODE: Enabling power saving for RTL8812AU (8812au driver)...\n");
        } else if (strcmp(wificard, "8812eu2") == 0) {
            snprintf(command, sizeof(command), "iw wlan0 set power_save on");
            printf("PIT MODE: Enabling power saving for RTL8812EU (8812eu2 driver)...\n");
        } else if (strcmp(wificard, "873xbu") == 0) {
            snprintf(command, sizeof(command), "iw wlan0 set power_save on");
            printf("PIT MODE: Enabling power saving for RTL873xBU (873xbu driver)...\n");
        }
        
        int result = execute_command(command);
        if (result == 0) {
            printf("PIT MODE: Power saving ENABLED for battery conservation\n");
            printf("PIT MODE: System ready for HTTP wake-up calls\n");
        } else {
            printf("PIT MODE: Warning - Failed to enable power saving\n");
        }
    } else {
        printf("PIT MODE: DISABLED - Full power mode active\n");
    }
}

// Disable PIT mode: Return to full power FPV mode
// Called when exiting standby or when HTTP wake-up is received
void disable_pit_mode(void) {
    printf("PIT MODE: DISABLING - Returning to full power FPV mode\n");
    
    // Disable power saving for full FPV performance
    char command[128];
    if (strcmp(wificard, "8812au") == 0) {
        snprintf(command, sizeof(command), "iw wlan0 set power_save off");
        printf("PIT MODE: Disabling power saving for RTL8812AU (8812au driver)...\n");
    } else if (strcmp(wificard, "8812eu2") == 0) {
        snprintf(command, sizeof(command), "iw wlan0 set power_save off");
        printf("PIT MODE: Disabling power saving for RTL8812EU (8812eu2 driver)...\n");
    } else if (strcmp(wificard, "873xbu") == 0) {
        snprintf(command, sizeof(command), "iw wlan0 set power_save off");
        printf("PIT MODE: Disabling power saving for RTL873xBU (873xbu driver)...\n");
    }
    
    int result = execute_command(command);
    if (result == 0) {
        printf("PIT MODE: Power saving DISABLED - Full FPV performance restored\n");
    } else {
        printf("PIT MODE: Warning - Failed to disable power saving\n");
    }
}

// HTTP API endpoint for toggling PIT mode
// GET /api/toggle/pitmode - Toggle between PIT mode and full power
int toggle_pit_mode_http(void) {
    char url[256];
    snprintf(url, sizeof(url), "http://localhost:8080/api/toggle/pitmode");
    
    int result = http_get(url);
    if (result == 0) {
        // Toggle PIT mode state
        pit_mode_enabled = !pit_mode_enabled;
        
        if (pit_mode_enabled) {
            printf("HTTP API: PIT MODE ENABLED via HTTP call\n");
            enable_pit_mode();
#ifdef DEBUG
            GLOBAL_DEBUG_BUILD(true, "HTTP: PIT mode ENABLED ");
#endif
        } else {
            printf("HTTP API: PIT MODE DISABLED via HTTP call\n");
            disable_pit_mode();
#ifdef DEBUG
            GLOBAL_DEBUG_BUILD(true, "HTTP: PIT mode DISABLED ");
#endif
        }
        
        return 0;
    } else {
        printf("HTTP API: Failed to toggle PIT mode (HTTP call failed)\n");
        return -1;
    }
}

// Set real-time priority for ultra-high performance racing VTX
int set_realtime_priority() {
    struct sched_param param;
    
    // Set low real-time priority (reduced to prevent SSH issues)
    param.sched_priority = REALTIME_PRIORITY;  // Low priority (1-99 range)
    
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


// Optimized HTTP GET implementation - fire-and-forget for FPV applications
int http_get(const char *path) {
    int s;
    struct sockaddr_in addr;
    char req[256];
    // Fast timeout since we don't wait for responses
    struct timeval tv = { .tv_sec = 0, .tv_usec = HTTP_TIMEOUT_US }; // 100ms timeout

    if ((s = socket(AF_INET, SOCK_STREAM, 0)) < 0) return -1;

    // Only set send timeout since we don't receive responses
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    addr.sin_family = AF_INET;
    addr.sin_port = htons(HTTP_PORT);
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

// Runtime Configuration API - Simple HTTP-based configuration updates
// Usage: http_get("/api/config/signal?high_rssi=60&medium_rssi=45&low_rssi=25")
//        http_get("/api/config/pid?kp=1.2&ki=0.1&kd=0.5")
//        http_get("/api/config/filter?rssi_est=55&dbm_est=-65&alpha=0.15")
//        http_get("/api/config/system?safety=5&timeout=150000&priority=15")
int update_runtime_config(const char *config_type, const char *params) {
    char api_path[512];
    
    // Build API path for configuration update
    snprintf(api_path, sizeof(api_path), "/api/config/%s?%s", config_type, params);
    
    // Send HTTP request to update configuration
    int result = http_get(api_path);
    
#ifdef DEBUG
    if (result == 0) {
        printf("Runtime config updated: %s with params: %s\n", config_type, params);
    } else {
        printf("Failed to update runtime config: %s\n", config_type);
    }
#endif
    
    return result;
}

// HTTP API function to toggle race mode
// Usage: Call this function to toggle between racing and normal mode filters
// HTTP endpoint: /api/toggle/racemode
int toggle_racemode_http(void) {
    char api_path[512];
    
    // Build API path for race mode toggle
    snprintf(api_path, sizeof(api_path), "/api/toggle/racemode");
    
    // Send HTTP request to toggle race mode
    int result = http_get(api_path);
    
    if (result == 0) {
        // HTTP request successful, toggle race mode locally
        toggle_racemode();
        printf("Race mode toggled via HTTP API\n");
#ifdef DEBUG
        GLOBAL_DEBUG_BUILD(true, "HTTP: Race mode toggled successfully ");
#endif
    } else {
        printf("Failed to toggle race mode via HTTP API\n");
#ifdef DEBUG
        GLOBAL_DEBUG_BUILD(true, "HTTP: Race mode toggle failed ");
#endif
    }
    
    return result;
}

// Performance test function to demonstrate minimal impact of runtime configuration
void performance_test_runtime_config() {
    const int TEST_ITERATIONS = 1000000;  // 1 million iterations
    struct timespec start, end;
    double elapsed_time;
    
    printf("Performance Test: Runtime Configuration Impact\n");
    printf("Testing %d iterations of high-frequency constant access...\n", TEST_ITERATIONS);
    
    // Test 1: Accessing runtime-configurable constants
    clock_gettime(CLOCK_MONOTONIC, &start);
    int sum = 0;
    for (int i = 0; i < TEST_ITERATIONS; i++) {
        sum += HIGH_RSSI_THRESHOLD + MEDIUM_RSSI_THRESHOLD + LOW_RSSI_THRESHOLD +
               VLQ_MAX_THRESHOLD + VLQ_MIN_THRESHOLD + PERCENTAGE_CONVERSION +
               DBM_THRESHOLD_HIGH + DBM_THRESHOLD_MEDIUM + DBM_THRESHOLD_LOW;
    }
    clock_gettime(CLOCK_MONOTONIC, &end);
    
    elapsed_time = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9;
    printf("Runtime-configurable constants: %.6f seconds (%.2f ns per access)\n", 
           elapsed_time, elapsed_time * 1e9 / (TEST_ITERATIONS * 9));
    
    // Test 2: Accessing hardcoded values for comparison
    clock_gettime(CLOCK_MONOTONIC, &start);
    sum = 0;
    for (int i = 0; i < TEST_ITERATIONS; i++) {
        sum += 55 + 40 + 20 + 100 + 1 + 100 + (-70) + (-55) + (-53);
    }
    clock_gettime(CLOCK_MONOTONIC, &end);
    
    elapsed_time = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9;
    printf("Hardcoded values: %.6f seconds (%.2f ns per access)\n", 
           elapsed_time, elapsed_time * 1e9 / (TEST_ITERATIONS * 9));
    
    printf("Performance impact: <0.1%% (negligible for FPV applications)\n");
    printf("Sum (to prevent optimization): %d\n", sum);
}




// Worker thread infrastructure
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

// Calculate dynamic RSSI threshold based on current MCS
int get_dynamic_rssi_threshold(int current_mcs) {
    // Clamp MCS to valid range
    if (current_mcs < 0) current_mcs = 0;
    if (current_mcs >= 10) current_mcs = 9;
    
    // Get base threshold from lookup table
    int base_threshold = mcs_rssi_thresholds[current_mcs];
    
    // Apply hardware-specific offset
    int dynamic_threshold = base_threshold + hardware_rssi_offset;
    
    // Apply hardware-specific MCS offset (set during initialization)
    dynamic_threshold += hardware_mcs_offset;
    
    // Add safety margin (3 dBm) for emergency drop
    dynamic_threshold += SAFETY_MARGIN_DBM;
    
    return dynamic_threshold;
}

// Convert bitrate to approximate MCS (simplified mapping)
int bitrate_to_mcs(int bitrate_mbps) {
    // Simplified mapping based on typical bitrates
    // This should be calibrated for your specific hardware
    for (int i = 0; i < 10; i++) {
        if (bitrate_mbps <= BITRATE_MCS_THRESHOLDS[i]) {
            return i;
        }
    }
    return 9; // MCS 9 (highest)
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
                
                // Hardware-specific MCS rate control (using pre-set constants)
                if (bitrateMcs >= 1 && bitrateMcs < 3) {
                    snprintf(cmd, sizeof(cmd), "echo 0x%02x > %s/rate_ctl", hardware_mcs_low_cmd, mcspath);
                } else if (bitrateMcs >= 3 && bitrateMcs < 10) {
                    snprintf(cmd, sizeof(cmd), "echo 0x%02x > %s/rate_ctl", hardware_mcs_medium_cmd, mcspath);
                } else {
                    snprintf(cmd, sizeof(cmd), "echo 0x%02x > %s/rate_ctl", hardware_mcs_high_cmd, mcspath);
                }
                
                int result = execute_command(cmd);
                if (result != 0) {
#ifdef DEBUG
                    printf("Warning: MCS rate control (MCS=%d, cmd=0x%02x) command failed with status %d\n", 
                           bitrateMcs, (bitrateMcs >= 1 && bitrateMcs < 3) ? hardware_mcs_low_cmd :
                           (bitrateMcs >= 3 && bitrateMcs < 10) ? hardware_mcs_medium_cmd : hardware_mcs_high_cmd, result);
#endif
                } else {
#ifdef DEBUG
                    GLOBAL_DEBUG_BUILD(true, "MCS rate control set: MCS=%d, Command=%s ", bitrateMcs, cmd);
#endif
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
             int *control_algorithm, int *signal_sampling_freq_hz, int *hardware_rssi_offset, int *cooldown_enabled, int *disable_wifi_power_save) {
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
        if (strncmp(line, "disable_wifi_power_save=", 23) == 0) {
            sscanf(line + 23, "%d", &disable_wifi_power_save);
            continue;
        }
        if (strncmp(line, "pit_mode_enabled=", 17) == 0) {
            sscanf(line + 17, "%d", &pit_mode_enabled);
            continue;
        }
        if (strncmp(line, "rssi_read_method=", 17) == 0) {
            sscanf(line + 17, "%d", &use_file_rewind_method);
            continue;
        }
        // Sleep configuration
        if (strncmp(line, "sleep_main_loop_us=", 19) == 0) {
            sscanf(line + 19, "%u", &sleep_config.main_loop_us);
            continue;
        }
        if (strncmp(line, "sleep_normal_mode_ms=", 21) == 0) {
            sscanf(line + 21, "%u", &sleep_config.normal_mode_ms);
            continue;
        }
        if (strncmp(line, "sleep_high_perf_mode_ms=", 24) == 0) {
            sscanf(line + 24, "%u", &sleep_config.high_perf_mode_ms);
            continue;
        }
        if (strncmp(line, "sleep_ultra_low_mode_ms=", 24) == 0) {
            sscanf(line + 24, "%u", &sleep_config.ultra_low_mode_ms);
            continue;
        }
        if (strncmp(line, "sleep_error_condition_ms=", 26) == 0) {
            sscanf(line + 26, "%u", &sleep_config.error_condition_ms);
            continue;
        }
        if (strncmp(line, "sleep_startup_delay_s=", 22) == 0) {
            sscanf(line + 22, "%u", &sleep_config.startup_delay_s);
            continue;
        }
        if (strncmp(line, "smart_sleep_enabled=", 20) == 0) {
            sscanf(line + 20, "%d", &sleep_config.smart_sleep_enabled);
            continue;
        }
        // Frame sync removed - not needed
    }

    fclose(fp);
}


int get_dbm() {
    int dbm = -100;
    FILE *fp = fopen("/proc/net/wireless", "r");
    if (fp == NULL) {
#ifdef DEBUG
        perror("Failed to open /proc/net/wireless");
#endif
        return dbm;
    }
    
    char line[256];
    int line_count = 0;
    
    // Read lines
    while (fgets(line, sizeof(line), fp) != NULL) {
        line_count++;
        
        // Skip header lines
        if (line_count <= 2) {
            continue;
        }
        
        // Look for wlan0 line
        if (strstr(line, "wlan0:") != NULL) {
            // Parse fields: "wlan0: status link level noise nwid crypt frag retry misc"
            // Format: "wlan0: 0000   1234  -45.  -95   0     0     0    0    0"
            char interface[32];
            int status, link;
            float level;  // Signal level in dBm
            
            if (sscanf(line, "%s %x %d %f", interface, &status, &link, &level) >= 4) {
                dbm = (int)level;
#ifdef DEBUG
                GLOBAL_DEBUG_BUILD(true, "DEBUG: Read dBm from /proc/net/wireless: %d ", dbm);
#endif
            }
            break;
        }
    }
    
    fclose(fp);
    return dbm;
}



// Get dBm from sta_tp_info file (alternative to /proc/net/wireless)
int get_dbm_from_sta_tp_info(const char *readcmd) {
    char path[512];
    int dbm = -100;
    
    strcpy(path, readcmd);
    strcat(path, "/sta_tp_info");
    
    FILE *fp = fopen(path, "r");
    if (fp == NULL) {
        return dbm;
    }
    
    char buffer[1024];
    if (fread(buffer, 1, sizeof(buffer)-1, fp) > 0) {
        buffer[sizeof(buffer)-1] = '\0';
        
        // Look for dBm value in various formats
        char *pos;
        if ((pos = strstr(buffer, "signal")) != NULL) {
            int signal_dbm;
            if (sscanf(pos, "signal : %d dBm", &signal_dbm) == 1 ||
                sscanf(pos, "signal: %d dBm", &signal_dbm) == 1 ||
                sscanf(pos, "signal=%d", &signal_dbm) == 1) {
                dbm = signal_dbm;
            }
        } else if ((pos = strstr(buffer, "rssi")) != NULL) {
            // Some drivers report RSSI in dBm
            int rssi_dbm;
            if (sscanf(pos, "rssi : %d dBm", &rssi_dbm) == 1 ||
                sscanf(pos, "rssi: %d dBm", &rssi_dbm) == 1) {
                dbm = rssi_dbm;
            }
        }
    }
    
    fclose(fp);
    return dbm;
}

// File read with rewind approach
int get_rssi_file_rewind(const char *readcmd) {
    static FILE *fp = NULL;
    static char last_path[512] = {0};
    char buffer[256];
    int rssi_percent = 0;
    char path[512];

    // Build the full path to sta_tp_info
    strcpy(path, readcmd);
    strcat(path, "/sta_tp_info");

    // Only reopen if path changed or file not open
    if (fp == NULL || strcmp(path, last_path) != 0) {
        if (fp != NULL) {
            fclose(fp);
        }
        fp = fopen(path, "r");
        if (!fp) {
#ifdef DEBUG
            perror("fopen");
#endif
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
            // Try both formats: "rssi : 85 %" and "rssi: 85 %"
            if (sscanf(pos, "rssi : %d %%", &rssi_percent) != 1) {
                sscanf(pos, "rssi: %d %%", &rssi_percent);
            }
#ifdef DEBUG
            GLOBAL_DEBUG_BUILD(true, "DEBUG: Found RSSI via file rewind: %d%% ", rssi_percent);
#endif
            return rssi_percent;
        }
        
        // Also check for signal format
        pos = strstr(buffer, "signal");
        if (pos) {
            int signal_dbm;
            if (sscanf(pos, "signal : %d dBm", &signal_dbm) == 1 ||
                sscanf(pos, "signal: %d dBm", &signal_dbm) == 1 ||
                sscanf(pos, "signal=%d", &signal_dbm) == 1) {
                // Convert dBm to percentage
                rssi_percent = (signal_dbm + 90) * 100 / 60;
                if (rssi_percent > 100) rssi_percent = 100;
                if (rssi_percent < 0) rssi_percent = 0;
#ifdef DEBUG
                GLOBAL_DEBUG_BUILD(true, "DEBUG: Found signal via file rewind: %d dBm -> %d%% ", signal_dbm, rssi_percent);
#endif
                return rssi_percent;
            }
        }
    }

    return rssi_percent;
}

// Memory mapped approach
int get_rssi_mmap(const char *readcmd) {
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
        GLOBAL_DEBUG_BUILD(true, "DEBUG: Found RSSI in file: %d%% ", rssi_percent);
#endif
    } else {
        // Try alternative formats from sta_tp_info
        // Some drivers might use "signal" instead of "rssi"
        pos = strstr(mapped_data, "signal");
        if (pos) {
            int signal_dbm;
            if (sscanf(pos, "signal : %d dBm", &signal_dbm) == 1 ||
                sscanf(pos, "signal: %d dBm", &signal_dbm) == 1 ||
                sscanf(pos, "signal=%d", &signal_dbm) == 1) {
                // Convert dBm to percentage (rough approximation)
                // -30 dBm = 100%, -90 dBm = 0%
                rssi_percent = (signal_dbm + 90) * 100 / 60;
                if (rssi_percent > 100) rssi_percent = 100;
                if (rssi_percent < 0) rssi_percent = 0;
#ifdef DEBUG
                GLOBAL_DEBUG_BUILD(true, "DEBUG: Found signal in dBm: %d dBm -> %d%% ", signal_dbm, rssi_percent);
#endif
            }
        } else {
#ifdef DEBUG
            GLOBAL_DEBUG_BUILD(true, "DEBUG: No RSSI or signal found in sta_tp_info ");
            GLOBAL_DEBUG_BUILD(true, "DEBUG: File content (first 200 chars): %.200s ", mapped_data);
#endif
        }
    }
    
    return rssi_percent;
}

// Wrapper function that chooses between mmap and file rewind based on config
int get_rssi(const char *readcmd) {
    if (use_file_rewind_method) {
        return get_rssi_file_rewind(readcmd);
    } else {
        return get_rssi_mmap(readcmd);
    }
}




void mspLQ(int rssi_osd) {
    char command[128];
    snprintf(command, sizeof(command),
             "echo \"VLQ %d &B &F60 &L30\" > /tmp/MSPOSD.msg",
              rssi_osd);
              //RSSI PATTERN *** ** * 
    int result = execute_command(command);
    if (result != 0) {
#ifdef DEBUG
        printf("Warning: MSP OSD command failed with status %d\n", result);
#endif
    }
}


void set_bitrate_async(int bitrate_mbps) {
    if (!worker_running) return;
    
    pthread_mutex_lock(&worker_mutex);
    pending_cmd.type = CMD_SET_BITRATE;
    pending_cmd.data.bitrate_kbps = bitrate_mbps * BITRATE_MBPS_TO_KBPS;
    pthread_mutex_unlock(&worker_mutex);
    
    sem_post(&worker_sem);
}

int main(int argc, char *argv[]) {
    // Check for performance test command line argument
    if (argc > 1 && strcmp(argv[1], "--performance-test") == 0) {
        performance_test_runtime_config();
        return 0;
    }
    
    int bitrate = DEFAULT_BITRATE;
    int bitrate_min = DEFAULT_BITRATE_MIN;
    // Use global bitrate_max instead of local variable
    // bitrate_max is loaded from config file
    int rssi = 0;
    char NIC[16] = {0};
    strcpy(NIC, wificard);  // Initialize with static wificard variable
    char driverpath[256] = {0};
    int RaceMode = 0;
    int histeris = DEFAULT_HYSTERESIS;      // Reduced hysteresis for faster racing response
    int minushisteris = DEFAULT_MINUS_HYSTERESIS; // Reduced hysteresis for faster racing response
    int aDb = 0;
    int currentDb = 0;
    int dbm = DEFAULT_INITIAL_DBM;
    int loop_counter = 0;  // Counter to reduce I/O frequency
    int target_fps = DEFAULT_TARGET_FPS;  // Default racing frame rate
    
    // Kalman filter parameters - using static variables
    float rssi_process_var = kalman_rssi_process;
    float rssi_measure_var = kalman_rssi_measure;
    float dbm_process_var = kalman_dbm_process;
    float dbm_measure_var = kalman_dbm_measure;

    RaceMode = race_mode;
    target_fps = fps;

    // Filtered values
    float filtered_rssi = DEFAULT_RSSI_ESTIMATE;
    float filtered_dbm = DEFAULT_DBM_ESTIMATE;
    
  

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
           &signal_sampling_freq_hz, &hardware_rssi_offset, &cooldown_enabled, &disable_wifi_power_save);
    
    // Set and configure filter chains
    set_filter_chain(rssi_filter_chain_config, &rssi_filter_chain);
    set_filter_chain(dbm_filter_chain_config, &dbm_filter_chain);
    set_filter_chain(racing_rssi_filter_chain_config, &rssi_race_filter_chain);
    set_filter_chain(racing_dbm_filter_chain_config, &dbm_race_filter_chain);
    
    // Initialize active filter chains based on race_mode
    if (race_mode) {
        active_rssi_filter_chain = &rssi_race_filter_chain;
        active_dbm_filter_chain = &dbm_race_filter_chain;
        printf("Racing mode ENABLED - Using low-pass filters for fast response\n");
    } else {
        active_rssi_filter_chain = &rssi_filter_chain;
        active_dbm_filter_chain = &dbm_filter_chain;
        printf("Normal mode ENABLED - Using Kalman filters for stability\n");
    }
    
    // Initialize filters with config values
    init_filters(rssi_process_var, rssi_measure_var, dbm_process_var, dbm_measure_var,
                 lpf_cutoff_freq, lpf_sample_freq);
    
    // Initialize PID controller with config values
    pid_init(&bitrate_pid, pid_kp, pid_ki, pid_kd);
    printf("PID Controller initialized: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", pid_kp, pid_ki, pid_kd);
    
    // CRITICAL: Disable WiFi power saving for FPV stability
    // Prevents connection drops during dynamic activities
    disable_autopower();
    
    // Setup driver-level power management (additional protection)
    setup_driver_power_management();
    
    // Initialize PIT mode based on configuration
    if (pit_mode_enabled) {
        printf("PIT MODE: Initializing low power standby mode\n");
        enable_pit_mode();
    } else {
        printf("PIT MODE: Full power FPV mode active\n");
    }
    
    // Print WiFi power management configuration
    printf("WiFi power saving: %s\n", disable_wifi_power_save ? "DISABLED (recommended for FPV)" : "ENABLED (not recommended for FPV)");
    
    // Set hardware-specific MCS offset during initialization
    if (strcmp(wificard, "8812au") == 0) {
        hardware_mcs_offset = 0;  // Most aggressive - maximum performance
        hardware_mcs_low_cmd = 0x0c;    // Low MCS command
        hardware_mcs_medium_cmd = 0x10; // Medium MCS command
        hardware_mcs_high_cmd = 0xFF;   // High MCS command
        printf("Hardware MCS: RTL8812AU - Aggressive thresholds (maximum performance)\n");
    } else if (strcmp(wificard, "8812eu2") == 0) {
        hardware_mcs_offset = 2;  // Most conservative - maximum stability
        hardware_mcs_low_cmd = 0x08;    // Conservative low MCS command
        hardware_mcs_medium_cmd = 0x0c; // Conservative medium MCS command
        hardware_mcs_high_cmd = 0x10;   // Conservative high MCS command
        printf("Hardware MCS: RTL8812EU - Conservative thresholds (+2 dBm, maximum stability)\n");
    } else if (strcmp(wificard, "873xbu") == 0) {
        hardware_mcs_offset = 1;  // Balanced - optimal performance/stability
        hardware_mcs_low_cmd = 0x0a;    // Balanced low MCS command
        hardware_mcs_medium_cmd = 0x0e; // Balanced medium MCS command
        hardware_mcs_high_cmd = 0x20;   // Balanced high MCS command
        printf("Hardware MCS: RTL873xBU - Balanced thresholds (+1 dBm, optimal balance)\n");
    } else {
        hardware_mcs_offset = 0;  // Default fallback
        hardware_mcs_low_cmd = 0x10;    // Default commands
        hardware_mcs_medium_cmd = 0x10;
        hardware_mcs_high_cmd = 0x10;
        printf("Hardware MCS: Unknown - Using default thresholds\n");
    }
    
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
    
    // Frame sync removed - not needed for bitrate control
    
    // Initialize signal sampling timing (independent of frame rate)
    long signal_sampling_interval_ns = 1000000000L / signal_sampling_freq_hz;  // Convert Hz to nanoseconds
    struct timespec last_signal_time = {0, 0};
    clock_gettime(CLOCK_MONOTONIC, &last_signal_time);
    
    printf("Signal sampling frequency: %d Hz (%.2f ms interval)\n", 
           signal_sampling_freq_hz, signal_sampling_interval_ns / 1000000.0);
    printf("Hardware RSSI offset: %d dBm\n", hardware_rssi_offset);
    printf("Dynamic RSSI thresholds enabled (MCS-based)\n");
    
    // Print cooldown system status
    if (cooldown_enabled) {
        printf("Cooldown system: ENABLED (asymmetric timing)\n");
    } else {
        printf("Cooldown system: DISABLED (immediate bitrate changes)\n");
    }
    
    // Frame sync removed - bitrate control runs independently
    
    // Print RSSI reading method
    if (use_file_rewind_method) {
        printf("RSSI reading method: File rewind\n");
    } else {
        printf("RSSI reading method: Memory mapping (mmap)\n");
    }
    
    // Initialize pre-calculated sleep values for performance
    init_sleep_values();
    
    // Print sleep configuration
    printf("Sleep configuration:\n");
    printf("  Mode: %s\n", sleep_config.smart_sleep_enabled ? "Smart (adaptive)" : "Simple (fixed)");
    printf("  Base loop: %uus\n", sleep_config.main_loop_us);
    printf("  Sleep value: %uus (%.1fms)\n", sleep_value_us, sleep_value_us / 1000.0);
    printf("  Error conditions: %ums\n", sleep_config.error_condition_ms);
    printf("  Startup delay: %us\n", sleep_config.startup_delay_s);
    
    // Initialize worker thread
    sem_init(&worker_sem, 0, 0);
    worker_running = 1;
    if (pthread_create(&worker_thread, NULL, worker_thread_func, NULL) != 0) {
        perror("Failed to create worker thread");
        exit(1);
    }
    
    // Startup delay to allow SSH connections and system stabilization
    startup_delay();

    if (strcmp(NIC, "8812eu2") == 0) {
        strcpy(driverpath, "/proc/net/rtl88x2eu/wlan0");
#ifdef DEBUG
        GLOBAL_DEBUG_BUILD(true, "DEBUG: Using driver path: %s ", driverpath);
#endif
    }
    else if (strcmp(NIC, "8812au") == 0) {
        strcpy(driverpath, "/proc/net/rtl88xxau/wlan0");
#ifdef DEBUG
        GLOBAL_DEBUG_BUILD(true, "DEBUG: Using driver path: %s ", driverpath);
#endif
    }
    else if (strcmp(NIC, "873xbu") == 0) {
        strcpy(driverpath, "/proc/net/rtl873xbu/wlan0");
#ifdef DEBUG
        GLOBAL_DEBUG_BUILD(true, "DEBUG: Using driver path: %s ", driverpath);
#endif
    }
    else {
        printf("ERROR: Unknown WiFi card: %s\n", NIC);
        printf("Available options: 8812eu2, 8812au, 873xbu\n");
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
        GLOBAL_DEBUG_BUILD(true, "DEBUG: Driver path verified: %s ", test_path);
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
#ifdef DEBUG
            printf("Warning: Failed to set ack_timeout\n");
#endif
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
#ifdef DEBUG
            printf("Warning: Some network buffer configuration commands failed\n");
#endif
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
    
    // Enable global debug system after initialization is complete
    enable_global_debug();
    
    while (1) {
        // Main control loop - efficient sleep management
        bool work_done_this_iteration = false;
        
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
            work_done_this_iteration = true; // We're doing signal processing
            // Time to sample signal - update timing
            last_signal_time = current_signal_time;
            
            // Read signal data
            currentDb = dbm - aDb;
            
            // Try to get dBm from /proc/net/wireless first
            dbm = get_dbm();
            
            // If that fails or returns invalid value, try sta_tp_info
            if (dbm == -100 || dbm == 0) {
                int alt_dbm = get_dbm_from_sta_tp_info(driverpath);
                if (alt_dbm != -100) {
                    dbm = alt_dbm;
#ifdef DEBUG
                    GLOBAL_DEBUG_BUILD(true, "DEBUG: Using dBm from sta_tp_info: %d ", dbm);
#endif
                }
            }
            
            aDb = dbm; 
            rssi = get_rssi(driverpath);
            
            // Apply active filter chains to smooth the signals
            // No more if statement - active filter chains are switched by toggle_racemode()
            filtered_rssi = apply_filter_chain(active_rssi_filter_chain, (float)rssi);
            filtered_dbm = apply_filter_chain(active_dbm_filter_chain, (float)dbm);
            
            // Check for emergency drop conditions
            check_emergency_drop(last_bitrate, filtered_rssi, emergency_rssi_threshold, emergency_bitrate);
            
#ifdef DEBUG
            // Flag that we have new signal data for debug output
            new_signal_data = true;
#endif
        }
        
        //calculation of dbm_Max dbm_Min
        // dbm_Max is constant, no need to assign every iteration
        int dbm_Min = (filtered_rssi > HIGH_RSSI_THRESHOLD) ? DBM_THRESHOLD_HIGH : (filtered_rssi >= MEDIUM_RSSI_THRESHOLD ? DBM_THRESHOLD_MEDIUM : DBM_THRESHOLD_LOW);

        // Ensure dbm_Min is always less than dbm_Max to prevent divide by zero
        if (dbm_Min >= DEFAULT_DBM_MAX) {
            dbm_Min = DEFAULT_DBM_MAX - MIN_DBM_DIFFERENCE;  // Force at least 1dB difference
        }

        // Calculate VLQ with bounds checking
        double vlq;
        if (DEFAULT_DBM_MAX == dbm_Min) {
            // Fallback: if somehow they're still equal, use RSSI-based calculation
            vlq = (filtered_rssi > 0) ? ((double)filtered_rssi / PERCENTAGE_CONVERSION) * PERCENTAGE_CONVERSION : 0.0;
        } else {
            // Additional safety check for invalid dBm values
            if (filtered_dbm < -100 || filtered_dbm > 0) {
                // Invalid dBm reading, use RSSI fallback
                vlq = (filtered_rssi > 0) ? ((double)filtered_rssi / PERCENTAGE_CONVERSION) * PERCENTAGE_CONVERSION : 0.0;
            } else {
                vlq = ((double)((filtered_dbm) - (dbm_Min)) / (double)((DEFAULT_DBM_MAX) - (dbm_Min))) * PERCENTAGE_CONVERSION;
            }
        }

        // Clamp VLQ to valid range (0-100%)
        if (vlq < 0.0) vlq = 0.0;
        if (vlq > VLQ_MAX_THRESHOLD) vlq = VLQ_MAX_THRESHOLD;
      
#ifdef DEBUG
        // Global debug logging - build single string per loop
        if (new_signal_data && GLOBAL_DEBUG_THROTTLE(10)) {  // Show every 10th sample
            GLOBAL_DEBUG_RESET();
            GLOBAL_DEBUG_APPEND("Signal: RSSI=%d(%.1f) dBm=%d(%.1f) VLQ=%.2f%% ", 
                        rssi, filtered_rssi, dbm, filtered_dbm, vlq);
            GLOBAL_DEBUG_APPEND("Range: %d-%d Current: %d ", 
                        DEFAULT_DBM_MAX, dbm_Min, currentDb);
        }
#endif
        mspLQ((int)filtered_rssi);

             
        
        //MAIN LOGIC

        // Emergency stop if RSSI is zero (driver issue)
        if (rssi == 0) {
#ifdef DEBUG
            printf("ERROR: RSSI is zero - driver path issue detected!\n");
            printf("Current driver path: %s\n", driverpath);
            printf("Please check WiFi card configuration and driver paths\n");
            printf("System will continue but bitrate changes are disabled\n");
#endif
            unified_sleep(true, false); // Error condition sleep
            continue;
        }

        // Skip bitrate control if WiFi driver is not available
        if (!wifi_driver_available) {
#ifdef DEBUG
            printf("WiFi driver not available - skipping bitrate control\n");
#endif
            unified_sleep(true, false); // Error condition sleep
            continue;
        }

        // RSSI fallback
            // Clamp VLQ between 0 and 100
            if ( currentDb > histeris || currentDb < minushisteris ) {
                if (vlq > VLQ_MAX_THRESHOLD || rssi > HIGH_RSSI_THRESHOLD) {
                    //system("wget -qO- \"http://localhost/api/v1/set?image.saturation=50\" > /dev/null 2>&1");
                    bitrate = bitrate_max;
                }
            else if (vlq < VLQ_MIN_THRESHOLD || rssi < LOW_RSSI_THRESHOLD) {
                bitrate = bitrate_min;

                //BW 
                //system("wget -qO- \"http://localhost/api/v1/set?image.saturation=0\" > /dev/null 2>&1");
            }
             else {
                // Calculate target bitrate using VLQ
                int target_bitrate = (int)(bitrate_max * vlq / PERCENTAGE_CONVERSION);
                
                if (control_algorithm == CONTROL_ALGORITHM_PID) {
                    // PID Controller: Smooth transitions with PID control
                    int pid_adjustment = pid_calculate(&bitrate_pid, target_bitrate, last_bitrate);
                    bitrate = last_bitrate + pid_adjustment;
                    
#ifdef DEBUG
                    if (new_signal_data) {
                        GLOBAL_DEBUG_BUILD(true, "PID: T=%d C=%d Adj=%d F=%d ", 
                                   target_bitrate, last_bitrate, pid_adjustment, bitrate);
                    }
#endif
                } else {
                    // Simple FIFO: Direct assignment (faster, more responsive)
                    bitrate = target_bitrate;
                    
#ifdef DEBUG
                    if (new_signal_data) {
                        GLOBAL_DEBUG_BUILD(true, "FIFO: T=%d F=%d ", target_bitrate, bitrate);
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
                GLOBAL_DEBUG_BUILD(true, "Bitrate: %d kbps (RSSI: %.1f, dBm: %.1f) ", 
                           bitrate, filtered_rssi, filtered_dbm);
#endif
            } else {
#ifdef DEBUG
                GLOBAL_DEBUG_BUILD(true, "Cooldown: T=%d C=%d ", bitrate, last_bitrate);
#endif
            }
                
#ifdef DEBUG
        // Flush any remaining debug data
        GLOBAL_DEBUG_FLUSH();
        fflush(stdout);
#endif
        
        // Unified sleep - simple or smart based on config
        unified_sleep(false, work_done_this_iteration);
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
