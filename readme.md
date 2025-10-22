# AP-ALINK-FPV: Advanced Adaptive Link System

A sophisticated FPV link adaptation system for OpenIPC-based devices, featuring intelligent signal processing, adaptive bitrate control, and real-time performance optimization.

## Features

### **Core Functionality**
- **Adaptive Bitrate Control**: Dynamic bitrate adjustment based on real-time signal quality
- **Advanced Filter System**: Choose from 5 different filter types (Kalman, Low-Pass, Mode, Derivative, 2-Pole LPF)
- **PID Controller**: Smooth, stable bitrate transitions with configurable gains
- **Asymmetric Cooldown**: Prevents oscillation with faster decreases and slower increases
- **Emergency Drop Logic**: Immediate response to critical signal loss
- **Real-time Priority**: SCHED_FIFO scheduling for deterministic timing
- **Frame-sync Timing**: Synchronized control at 120Hz with 24Hz signal sampling for optimal video quality

### **Signal Processing**
- **RSSI Analysis**: Real-time signal strength monitoring
- **dBm Processing**: Power level analysis and adaptation
- **Advanced Noise Filtering**: 5 different filter algorithms for optimal signal processing
- **Independent Filter Configuration**: RSSI and dBm can use different filter types
- **Multi-antenna Support**: Handles multiple antenna configurations

### **Performance Optimizations**
- **Memory-Mapped Files**: Zero I/O overhead for signal reading (20-200x faster)
- **Configurable Sampling**: Adjustable signal reading frequency (1-120Hz)
- **Emergency Cooldown**: Ultra-fast bitrate drops (50ms default, configurable to 8ms)
- **Worker Threads**: Asynchronous API calls to prevent blocking
- **HTTP Optimization**: Raw socket implementation instead of wget
- **Memory Efficiency**: Minimal memory footprint

## Requirements

- **Hardware**: OpenIPC-compatible device (tested on Sigmastar)
- **OS**: Linux-based system with OpenIPC firmware
- **WiFi Card**: Compatible with 8812au/8812eu drivers
- **Permissions**: Root access for real-time priority

## Installation

1. **Clone/Download** the AP-ALINK-FPV repository
2. **Compile** the application:
   ```bash
   make clean && make debug
   ```
3. **Install** the binary:
   ```bash
   sudo cp ap_alink_debug /usr/local/bin/ap_alink
   sudo chmod +x /usr/local/bin/ap_alink
   ```
4. **Configure** the system (see Configuration section)

## Configuration

The system is configured via `/etc/ap_alink.conf`. Here's a complete configuration example:

```ini
# Basic Settings
bitrate_max=19
wificard=8812eu2
race_mode=0
fps=120

# Filter Chain Configuration
# Format: comma-separated filter types (0=Kalman, 1=Low-Pass, 2=Mode, 3=Derivative, 4=2-Pole LPF)
# Examples: "0" (single Kalman), "2,0" (Mode->Kalman), "1,4" (Low-Pass->2-Pole LPF)
rssi_filter_chain=0
dbm_filter_chain=0

# Low-Pass Filter Settings (used by filter types 1 and 4)
lpf_cutoff_freq=2.0
lpf_sample_freq=10.0

# Kalman Filter Parameters (used by filter type 0)
# Process variance - how much the signal is expected to change (lower = more stable)
kalman_rssi_process=0.00001
kalman_dbm_process=0.00001

# Measurement variance - how noisy the measurements are (higher = more noisy)
kalman_rssi_measure=0.1
kalman_dbm_measure=0.5

# Asymmetric Cooldown Parameters
# Minimum time between any bitrate changes (ms)
strict_cooldown_ms=200

# Additional time required before increasing bitrate (ms)
up_cooldown_ms=3000

# Minimum percentage change required to trigger bitrate change
min_change_percent=5

# Emergency drop settings
emergency_rssi_threshold=30
emergency_bitrate=1000

# PID Controller Parameters
# Proportional gain - immediate response to error (higher = more aggressive)
pid_kp=1.0

# Integral gain - eliminates steady-state error over time (higher = faster correction)
pid_ki=0.05

# Derivative gain - reduces overshoot and oscillation (higher = more damping)
pid_kd=0.4

# Racing Mode Configuration
# racing_fps is used when racing mode is enabled, fps is used for normal operation
racing_video_resolution=1280x720
racing_exposure=11
racing_fps=120

# Signal Sampling Configuration
# Number of frames between signal readings (higher = less CPU usage, lower = more responsive)
# At 120 FPS: 1=120Hz sampling, 2=60Hz, 3=40Hz, 5=24Hz, 10=12Hz
signal_sampling_interval=5

# Emergency Cooldown Configuration
# Time between bitrate decreases (lower = faster response to signal drops)
# At 120 FPS: 50ms=6 frames, 25ms=3 frames, 8ms=1 frame
# For ultra-low latency racing, use 8-25ms (1-3 frames)
emergency_cooldown_ms=50
```

### **Configuration Parameters Explained**

#### **Basic Settings**
- **`race_mode`**: Enable racing mode (1=ON, 0=OFF) - uses racing_fps and racing-specific settings
- **`fps`**: Normal mode frame rate (used when race_mode=0)
- **`racing_fps`**: Racing mode frame rate (used when race_mode=1)

#### **Filter Chain Configuration**
- **`rssi_filter_chain`**: Comma-separated filter chain for RSSI signal (e.g., "2,0" for Mode→Kalman)
- **`dbm_filter_chain`**: Comma-separated filter chain for dBm signal (e.g., "1" for single Low-Pass)

#### **Racing Mode Filter Chain Configuration**
- **`racing_rssi_filter_chain`**: Filter chain for RSSI when race_mode=1 (default: "1" for Low-Pass)
- **`racing_dbm_filter_chain`**: Filter chain for dBm when race_mode=1 (default: "1" for Low-Pass)

#### **Advanced Filter Options**
- **Kalman Filter (0)**: Sophisticated adaptive filtering with optimal noise rejection
- **Low-Pass Filter (1)**: Simple exponential moving average for fast response
- **Mode Filter (2)**: ArduPilot-style median-like filtering with alternating sample dropping
- **Derivative Filter (3)**: Savitzky-Golay trend detection with smooth derivative calculation
- **2-Pole Low-Pass Filter (4)**: Professional biquad filter with Butterworth response

#### **Low-Pass Filter Settings**
- **`lpf_cutoff_freq`**: Cutoff frequency in Hz (lower = more smoothing, used by filter types 1 and 4)
- **`lpf_sample_freq`**: Sample frequency in Hz (should match signal sampling rate, used by filter types 1 and 4)

#### **Kalman Filter Settings**
- **`kalman_rssi_process`**: Process variance for RSSI (lower = more stable)
- **`kalman_rssi_measure`**: Measurement variance for RSSI (higher = more noisy)
- **`kalman_dbm_process`**: Process variance for dBm
- **`kalman_dbm_measure`**: Measurement variance for dBm

#### **Cooldown Settings**
- **`strict_cooldown_ms`**: Minimum time between any bitrate changes
- **`up_cooldown_ms`**: Additional time required before increasing bitrate
- **`min_change_percent`**: Minimum percentage change to trigger adjustment

#### **Emergency Settings**
- **`emergency_rssi_threshold`**: RSSI threshold for emergency drop
- **`emergency_bitrate`**: Bitrate to drop to in emergency

#### **PID Controller**
- **`pid_kp`**: Proportional gain (immediate response)
- **`pid_ki`**: Integral gain (steady-state error correction)
- **`pid_kd`**: Derivative gain (overshoot prevention)

#### **Racing Mode Configuration**
- **`racing_video_resolution`**: Video resolution for racing mode (e.g., "1280x720", "1920x1080")
- **`racing_exposure`**: Camera exposure setting for racing mode (lower = faster shutter)
- **`racing_fps`**: Frame rate for racing mode (separate from normal `fps` setting)

#### **Signal Sampling Configuration**
- **`signal_sampling_interval`**: Number of frames between signal readings (higher = less CPU usage, lower = more responsive)
  - At 120 FPS: 1=120Hz sampling, 2=60Hz, 3=40Hz, 5=24Hz, 10=12Hz
  - Default: 5 (24Hz sampling at 120 FPS)

#### **Emergency Cooldown Configuration**
- **`emergency_cooldown_ms`**: Time between bitrate decreases (lower = faster response to signal drops)
  - At 120 FPS: 50ms=6 frames, 25ms=3 frames, 8ms=1 frame
  - Default: 50ms (6 frames at 120 FPS)
  - For ultra-low latency racing: use 8-25ms (1-3 frames)

## Filter Comparison Guide

### **Filter Performance Comparison**

| Filter Type | Smoothing | Noise Rejection | Responsiveness | CPU Usage | Best For |
|-------------|-----------|-----------------|----------------|-----------|----------|
| **Kalman (0)** | Excellent | Excellent | Good | Medium | General purpose, optimal performance |
| **Low-Pass (1)** | Good | Good | Excellent | Low | Racing, low latency applications |
| **Mode (2)** | Medium | Excellent | Good | Low | Noisy environments, stable median |
| **Derivative (3)** | Medium | Good | Excellent | Medium | Trend detection, rate analysis |
| **2-Pole LPF (4)** | Medium | Excellent | Good | Low | Professional filtering, Butterworth |

### **Filter Selection Guide**

#### **Choose Kalman Filter (0) when:**
- You want optimal performance for general use
- Signal quality varies significantly
- You need adaptive filtering behavior
- **Default recommendation for most users**

#### **Choose Low-Pass Filter (1) when:**
- You need maximum responsiveness
- CPU resources are limited
- You're racing and need low latency
- Signal is relatively clean

#### **Choose Mode Filter (2) when:**
- Environment is very noisy
- You need excellent noise rejection
- You want stable median-like output
- Outliers are common in your signal

#### **Choose Derivative Filter (3) when:**
- You need trend detection capabilities
- You want to analyze rate of change
- You need smooth derivative calculation
- Signal trends are important

#### **Choose 2-Pole LPF (4) when:**
- You want professional-grade filtering
- You need Butterworth response characteristics
- You want steeper rolloff than simple LPF
- You're building a professional system

### **Mixed Filter Strategies**

#### **Racing Setup**
```ini
rssi_filter_chain=1    # Low-Pass for speed
dbm_filter_chain=1     # Low-Pass for speed
```

#### **Noisy Environment**
```ini
rssi_filter_chain=2,0  # Mode→Kalman for noise rejection
dbm_filter_chain=2,0    # Mode→Kalman for stability
```

#### **Professional Setup**
```ini
rssi_filter_chain=4,0  # 2-Pole LPF→Kalman for quality
dbm_filter_chain=0      # Kalman for dBm stability
```

#### **Trend Analysis**
```ini
rssi_filter_chain=3,0  # Derivative→Kalman for trends
dbm_filter_chain=0      # Kalman for dBm stability
```

## Performance Optimizations

### **Memory-Mapped File System**
The system uses memory-mapped files for signal reading, providing:
- **20-200x faster** signal reading compared to traditional file I/O
- **Zero I/O overhead** after initial mapping
- **Perfect for real-time** applications on embedded systems
- **Automatic cleanup** when switching between different driver paths

### **Configurable Signal Sampling**
Fine-tune the balance between responsiveness and CPU usage:
- **High Performance**: `signal_sampling_interval=1` (120Hz at 120 FPS)
- **Balanced**: `signal_sampling_interval=5` (24Hz at 120 FPS) - default
- **Low CPU**: `signal_sampling_interval=10` (12Hz at 120 FPS)

### **Emergency Cooldown System**
Ultra-fast response to signal drops with configurable timing:
- **Default**: 50ms (6 frames at 120 FPS)
- **Racing**: 25ms (3 frames at 120 FPS)
- **Ultra-Low Latency**: 8ms (1 frame at 120 FPS)

### **Asymmetric Bitrate Control**
- **Bitrate Decreases**: Use emergency cooldown (fast response)
- **Bitrate Increases**: Use strict cooldown (prevents oscillation)
- **Perfect for FPV**: Fast drops, slow increases

## Usage Scenarios

### **Racing Mode (Low Latency)**
```ini
race_mode=1
fps=60                    # Normal mode frame rate
racing_fps=240            # Racing mode frame rate (higher for responsiveness)

# Normal mode filters (used when race_mode=0)
rssi_filter_chain=0        # Kalman for stability
dbm_filter_chain=0        # Kalman for stability

# Racing mode filters (used when race_mode=1)
racing_rssi_filter_chain=1 # Low-Pass for fast response
racing_dbm_filter_chain=1  # Low-Pass for fast response

# Performance optimizations
signal_sampling_interval=2  # 120Hz sampling at 240 FPS
emergency_cooldown_ms=25    # 3 frames at 240 FPS

lpf_cutoff_freq=5.0
strict_cooldown_ms=100
pid_kp=1.5
racing_video_resolution=1280x720
racing_exposure=8
```

### **Ultra-Low Latency Racing**
```ini
race_mode=1
racing_fps=120

# Maximum responsiveness filters
racing_rssi_filter_chain=1 # Low-Pass for speed
racing_dbm_filter_chain=1  # Low-Pass for speed

# Ultra-fast sampling and cooldown
signal_sampling_interval=1  # 120Hz sampling (every frame)
emergency_cooldown_ms=8     # 1 frame cooldown

# Aggressive settings
lpf_cutoff_freq=10.0        # Higher cutoff for faster response
strict_cooldown_ms=50       # Faster normal cooldown
pid_kp=2.0                  # More aggressive PID
racing_video_resolution=1280x720
racing_exposure=6           # Very fast shutter
```

### **Long Range (Stability)**
```ini
race_mode=0
rssi_filter_chain=0
dbm_filter_chain=0
kalman_rssi_process=0.000001
up_cooldown_ms=5000
pid_kp=0.8
```

### **Mixed Approach**
```ini
rssi_filter_chain=0    # Kalman for RSSI (more sophisticated)
dbm_filter_chain=1     # Low-pass for dBm (simpler, faster)
```

### **Noisy Environment (Mode Filter)**
```ini
rssi_filter_chain=2,0  # Mode→Kalman for excellent noise rejection
dbm_filter_chain=2,0   # Mode→Kalman for stable median values
```

### **Trend Detection (Derivative Filter)**
```ini
rssi_filter_chain=3,0  # Derivative→Kalman for trend analysis
dbm_filter_chain=0     # Kalman for dBm stability
```

### **Professional Setup (2-Pole LPF)**
```ini
rssi_filter_chain=4,0  # 2-Pole LPF→Kalman for professional filtering
dbm_filter_chain=4,0   # 2-Pole LPF→Kalman with Butterworth response
lpf_cutoff_freq=1.5    # Lower cutoff for more smoothing
```

### **Dual FPS System Example**
```ini
# Normal operation - conservative settings
fps=30                  # Normal mode frame rate
racing_fps=120          # Racing mode frame rate (4x faster)

# Racing mode gets higher frame rate for responsiveness
racing_video_resolution=1280x720
racing_exposure=10

# Normal mode uses higher exposure for better image quality
# (exposure setting only applies to racing mode)
```

### **Dual Filter System Example**
```ini
# Normal mode - sophisticated filtering for stability
race_mode=0
rssi_filter_chain=0        # Kalman for optimal performance
dbm_filter_chain=0        # Kalman for optimal performance

# Racing mode - fast filtering for responsiveness
race_mode=1
racing_rssi_filter_chain=1 # Low-Pass for fast response
racing_dbm_filter_chain=1  # Low-Pass for fast response

# The system automatically switches between filter sets based on race_mode
```

## Running the Application

### **Basic Usage**
```bash
sudo ./ap_alink
```

### **With Debug Output**
```bash
sudo ./ap_alink_debug
```

### **Systemd Service** (Recommended)
```bash
# Create service file
sudo nano /etc/systemd/system/ap-alink.service

[Unit]
Description=AP-ALINK-FPV Adaptive Link System
After=network.target

[Service]
Type=simple
User=root
ExecStart=/usr/local/bin/ap_alink
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target

# Enable and start
sudo systemctl enable ap-alink
sudo systemctl start ap-alink
```

## Performance Monitoring

### **Debug Output**
When running with debug enabled, you'll see:
```
Filters initialized - RSSI: Kalman, dBm: Kalman
Mode filters initialized
Derivative filters initialized
2-Pole LPF initialized: cutoff=2.0Hz, sample=10.0Hz
PID Controller initialized: Kp=1.00, Ki=0.05, Kd=0.40
Real-time priority set successfully (SCHED_FIFO, priority 50)
Frame-sync initialized: 120 FPS (8.33 ms interval)
Signal sampling: 24Hz (every 5th frame)

vlq = 85.23%
rssi = 45 (filtered: 44.8)
dbm= -55 (filtered: -55.2)
Target: 1620, Current: 1500, PID Adj: 120, Final: 1620
Bitrate changed to 1620 kbps (RSSI: 44.8, dBm: -55.2)
```

### **Key Metrics**
- **VLQ**: Video Link Quality percentage
- **Filtered Values**: Smoothed RSSI and dBm readings
- **PID Adjustment**: Controller output for bitrate changes
- **Timing**: Frame-sync and cooldown compliance

## Troubleshooting

### **Common Issues**

#### **Permission Denied**
```bash
# Ensure running as root
sudo ./ap_alink
```

#### **Real-time Priority Failed**
```bash
# Check if running as root
sudo ./ap_alink
# Or disable real-time priority in code if not needed
```

#### **No Signal Data**
```bash
# Check WiFi card compatibility
lsusb | grep -i rtl
# Verify driver is loaded
lsmod | grep 88
```

#### **High CPU Usage**
```bash
# Reduce update frequency
# Increase cooldown times
# Use low-pass filters instead of Kalman
```

### **Performance Tuning**

#### **For Racing**
- Use low-pass filters (`rssi_filter_chain=1`) or derivative filters (`rssi_filter_chain=3,0`)
- Higher cutoff frequency (`lpf_cutoff_freq=5.0`)
- Shorter cooldowns (`strict_cooldown_ms=100`)
- Higher PID gains (`pid_kp=1.5`)

#### **For Long Range**
- Use Kalman filters (`rssi_filter_chain=0`) or 2-pole LPF (`rssi_filter_chain=4,0`)
- Lower process variance (`kalman_rssi_process=0.000001`)
- Longer cooldowns (`up_cooldown_ms=5000`)
- Lower PID gains (`pid_kp=0.8`)

#### **For Noisy Environments**
- Use mode filters (`rssi_filter_chain=2,0`) for excellent noise rejection
- Standard cooldowns (`strict_cooldown_ms=200`)
- Moderate PID gains (`pid_kp=1.0`)

#### **For Professional Applications**
- Use 2-pole LPF (`rssi_filter_chain=4,0`) for Butterworth response
- Lower cutoff frequency (`lpf_cutoff_freq=1.5`)
- Longer cooldowns (`up_cooldown_ms=3000`)
- Balanced PID gains (`pid_kp=1.0`)

## Architecture

### **System Components**
```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Signal Input  │───▶│  Advanced Filter │───▶│  PID Controller │
│   (RSSI/dBm)    │    │  System (5 types)│    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                                         │
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Bitrate API   │◀───│  Cooldown Logic  │◀───│  Bitrate Calc    │
│   (wfb_tx_cmd)  │    │  (Asymmetric)     │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

### **Thread Architecture**
- **Main Thread**: Signal processing and control logic
- **Worker Thread**: Asynchronous API calls
- **Real-time Priority**: SCHED_FIFO for consistent timing

## Technical Details

### **Filtering Algorithms**

#### **Kalman Filter**
- **Prediction Step**: Estimates next state based on model
- **Update Step**: Corrects prediction with measurement
- **Dynamic Gain**: Automatically adjusts based on uncertainty
- **Numerical Stability**: Clamped gains and minimum error estimates

#### **Low-Pass Filter (ArduPilot Style)**
- **Exponential Moving Average**: `output += (sample - output) * alpha`
- **Automatic Alpha**: Calculated from cutoff frequency
- **Simple and Efficient**: Proven in flight control systems

#### **Mode Filter (ArduPilot Style)**
- **Median-like Filtering**: Alternates dropping high/low samples
- **Insertion Sort**: Efficient sample management
- **Excellent Noise Rejection**: Ideal for noisy environments
- **Stable Output**: Returns median-like values

#### **Derivative Filter (Savitzky-Golay)**
- **Trend Detection**: Calculates smooth derivatives
- **5-Point Algorithm**: Uses Savitzky-Golay coefficients
- **Timestamp Tracking**: Accounts for variable sample rates
- **Noise Reduction**: Smooth derivative calculation

#### **2-Pole Low-Pass Filter (Biquad)**
- **Butterworth Response**: Q=0.707 for optimal characteristics
- **Biquad Implementation**: Professional-grade filtering
- **Steep Rolloff**: Better than simple low-pass filters
- **Stable Coefficients**: Real-time coefficient calculation

### **PID Controller**
- **Proportional**: Immediate response to error
- **Integral**: Eliminates steady-state error
- **Derivative**: Reduces overshoot and oscillation
- **Windup Protection**: Prevents integral saturation

### **Asymmetric Cooldown**
- **Fast Decrease**: Quick response to signal degradation
- **Slow Increase**: Prevents oscillation and instability
- **Minimum Change**: Ignores small fluctuations

## Performance Metrics

### **Latency**
- **Signal Processing**: < 1ms
- **Filter Application**: < 0.1ms
- **API Calls**: Asynchronous (non-blocking)
- **Control Loop**: 120Hz (frame-synchronized)
- **Signal Sampling**: 24Hz (every 5th frame)

### **CPU Usage**
- **Low-Pass Filter**: ~0.1% CPU
- **Kalman Filter**: ~0.3% CPU
- **Mode Filter**: ~0.2% CPU
- **Derivative Filter**: ~0.3% CPU
- **2-Pole LPF**: ~0.2% CPU
- **PID Controller**: ~0.05% CPU
- **Total System**: < 1% CPU

### **Memory Usage**
- **Static Allocation**: ~2KB
- **Stack Usage**: < 1KB
- **Total Memory**: < 5KB

## Changelog

### **v2.0 - Performance Optimization Release**

#### **🚀 Major Performance Improvements**
- **Memory-Mapped Files**: 20-200x faster signal reading with zero I/O overhead
- **Configurable Signal Sampling**: Adjustable sampling frequency (1-120Hz)
- **Emergency Cooldown System**: Ultra-fast bitrate drops (50ms default, configurable to 8ms)

#### **⚙️ New Configuration Options**
- **`signal_sampling_interval`**: Control signal reading frequency (default: 5 frames)
- **`emergency_cooldown_ms`**: Configure emergency bitrate drop timing (default: 50ms)

#### **🎯 FPV-Optimized Features**
- **Asymmetric Bitrate Control**: Fast drops, slow increases (perfect for FPV)
- **Frame-Aware Timing**: All timings show exact frame counts
- **Ultra-Low Latency Mode**: Single-frame cooldown support (8ms at 120 FPS)

#### **📊 Performance Metrics**
- **Signal Reading**: 20-200x faster with memory mapping
- **Emergency Response**: 4x faster (200ms → 50ms default)
- **CPU Usage**: Reduced I/O overhead
- **Memory Efficiency**: Optimized file handling

#### **🔧 Technical Improvements**
- **Memory-Mapped I/O**: Direct memory access for `/proc/net/wireless` and driver files
- **Path-Aware Mapping**: Automatic remapping when switching driver paths
- **Error Handling**: Robust error handling for memory mapping operations
- **Cleanup System**: Proper memory map cleanup on exit

### **v1.0 - Initial Release**
- Advanced filter system with 5 filter types
- PID controller for smooth bitrate transitions
- Asymmetric cooldown system
- Real-time priority scheduling
- Frame-sync timing at 120Hz
- Racing mode support

## License

This project is completely free to use for everyone. No restrictions apply.

## 🎥 Real-World Testing

**IRL Test Video**: https://youtu.be/MFqh6s7O_1k