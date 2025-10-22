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
- **Worker Threads**: Asynchronous API calls to prevent blocking
- **Reduced I/O**: Optimized signal reading frequency (24Hz sampling)
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
LowLatency=0
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
```

### **Configuration Parameters Explained**

#### **Filter Chain Configuration**
- **`rssi_filter_chain`**: Comma-separated filter chain for RSSI signal (e.g., "2,0" for Mode→Kalman)
- **`dbm_filter_chain`**: Comma-separated filter chain for dBm signal (e.g., "1" for single Low-Pass)

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

## Usage Scenarios

### **Racing Mode (Low Latency)**
```ini
LowLatency=1
rssi_filter_chain=1
dbm_filter_chain=1
lpf_cutoff_freq=5.0
strict_cooldown_ms=100
pid_kp=1.5
```

### **Long Range (Stability)**
```ini
LowLatency=0
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

## License

This project is completely free to use for everyone. No restrictions apply.

## 🎥 Real-World Testing

**IRL Test Video**: https://youtu.be/MFqh6s7O_1k