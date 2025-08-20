# Intel RealSense D455 Installation Guide for Raspberry Pi 5 (Bookworm)

## Overview

This guide provides detailed instructions for building and installing Intel RealSense SDK 2.0 (librealsense2) on Raspberry Pi 5 running Raspbian Bookworm (64-bit). The D455 depth camera requires special configuration due to USB 3.0 bandwidth requirements and kernel module compilation.

## System Requirements

- **Hardware**: Raspberry Pi 5 (16GB recommended)
- **OS**: Raspbian OS Bookworm 64-bit (Debian 12)
- **Storage**: At least 8GB free space for compilation
- **Power**: 5V/5A USB-C power supply (27W minimum)
- **USB**: Powered USB 3.0 hub recommended for D455

## Pre-Installation Setup

### 1. Update System and Install Dependencies

```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install build essentials and dependencies
sudo apt install -y \
    git cmake build-essential \
    libssl-dev libusb-1.0-0-dev \
    pkg-config libgtk-3-dev \
    libglfw3-dev libgl1-mesa-dev \
    libglu1-mesa-dev libusb-1.0-0-dev \
    libx11-dev libxrandr-dev libxi-dev \
    python3-dev python3-pip \
    v4l-utils \
    udev \
    libudev-dev \
    libv4l-dev \
    libopencv-dev \
    libeigen3-dev

# Install kernel headers for current kernel
sudo apt install -y raspberrypi-kernel-headers

# Verify kernel version matches headers
uname -r
ls /lib/modules/$(uname -r)/build
```

### 2. Increase Swap Size (Important for Compilation)

```bash
# Temporarily increase swap for compilation
sudo dphys-swapfile swapoff
sudo sed -i 's/CONF_SWAPSIZE=.*/CONF_SWAPSIZE=2048/' /etc/dphys-swapfile
sudo dphys-swapfile setup
sudo dphys-swapfile swapon

# Verify swap size
free -h
```

### 3. Configure USB for RealSense

```bash
# Increase USB buffer memory
echo 'vm.min_free_kbytes=65536' | sudo tee -a /etc/sysctl.conf
echo 'kernel.sched_rt_runtime_us=-1' | sudo tee -a /etc/sysctl.conf
sudo sysctl -p

# Set USB autosuspend to disabled
echo 'ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b5c", ATTR{power/autosuspend}="-1"' | \
    sudo tee /etc/udev/rules.d/99-realsense-d455.rules

# Enable USB 3.0 performance mode
echo 'dwc_otg.speed=1' | sudo tee -a /boot/firmware/cmdline.txt
```

## Building librealsense2 from Source

### 1. Clone and Prepare Repository

```bash
# Create build directory
mkdir -p ~/realsense_build
cd ~/realsense_build

# Clone specific version known to work with RPi5
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
git checkout v2.54.2  # Latest stable version as of 2024

# Apply Raspberry Pi patches
./scripts/patch-realsense-ubuntu-lts-hwe.sh
```

### 2. Build with Optimized Settings for RPi5

```bash
# Create build directory
mkdir build && cd build

# Configure with RPi5 optimizations
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=true \
    -DBUILD_GRAPHICAL_EXAMPLES=true \
    -DBUILD_PYTHON_BINDINGS=true \
    -DPYTHON_EXECUTABLE=$(which python3) \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DFORCE_RSUSB_BACKEND=true \
    -DBUILD_WITH_CUDA=false \
    -DBUILD_WITH_OPENMP=true \
    -DCMAKE_CXX_FLAGS="-march=armv8.2-a+crypto+fp16+rcpc+dotprod -mtune=cortex-a76 -O3" \
    -DCMAKE_C_FLAGS="-march=armv8.2-a+crypto+fp16+rcpc+dotprod -mtune=cortex-a76 -O3"

# Build with 4 cores (RPi5 has 4 cores)
make -j4

# This will take 30-60 minutes on RPi5
```

### 3. Install librealsense2

```bash
# Install libraries and headers
sudo make install

# Update library cache
sudo ldconfig

# Install udev rules
sudo cp ../config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# Create Python bindings symlink
cd ~/realsense_build/librealsense/build/wrappers/python
sudo python3 setup.py install
```

## Post-Installation Configuration

### 1. Verify Installation

```bash
# Test if RealSense is detected
rs-enumerate-devices

# Check library installation
pkg-config --modversion realsense2

# Test with viewer (requires display)
realsense-viewer

# Test depth stream
rs-depth
```

### 2. Python Environment Setup

```bash
# Install Python dependencies
pip3 install --user \
    numpy \
    opencv-python \
    pyrealsense2

# Test Python bindings
python3 -c "import pyrealsense2 as rs; print(rs.__version__)"
```

### 3. Create Test Script

```python
#!/usr/bin/env python3
# save as test_d455.py

import pyrealsense2 as rs
import numpy as np
import cv2
import time

def test_realsense_d455():
    """Test Intel RealSense D455 on Raspberry Pi 5"""

    # Configure streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device info
    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) == 0:
        print("No RealSense devices found!")
        return

    dev = devices[0]
    print(f"Found device: {dev.get_info(rs.camera_info.name)}")
    print(f"Serial number: {dev.get_info(rs.camera_info.serial_number)}")
    print(f"Firmware: {dev.get_info(rs.camera_info.firmware_version)}")

    # Configure streams for RPi5 performance
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

    # Start streaming
    profile = pipeline.start(config)

    # Get depth sensor and set options
    depth_sensor = profile.get_device().first_depth_sensor()

    # Set Visual Preset for better quality
    if depth_sensor.supports(rs.option.visual_preset):
        depth_sensor.set_option(rs.option.visual_preset,
                               rs.l500_visual_preset.low_ambient_light)

    # Optimize for RPi5
    if depth_sensor.supports(rs.option.enable_auto_exposure):
        depth_sensor.set_option(rs.option.enable_auto_exposure, 1)

    print("\nStreaming... Press 'q' to quit")

    try:
        frame_count = 0
        start_time = time.time()

        while True:
            # Wait for frames
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # Convert to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap to depth image
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03),
                cv2.COLORMAP_JET
            )

            # Stack images horizontally
            images = np.hstack((color_image, depth_colormap))

            # Calculate FPS
            frame_count += 1
            if frame_count % 30 == 0:
                elapsed = time.time() - start_time
                fps = frame_count / elapsed
                print(f"FPS: {fps:.2f}")

            # Display (if running with display)
            cv2.imshow('RealSense D455', images)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print(f"\nTotal frames: {frame_count}")
        print(f"Average FPS: {frame_count/(time.time()-start_time):.2f}")

if __name__ == "__main__":
    test_realsense_d455()
```

## Performance Optimization for VIO

### 1. USB 3.0 Bandwidth Management

```bash
# Create bandwidth allocation script
cat << 'EOF' | sudo tee /usr/local/bin/optimize_usb_for_d455.sh
#!/bin/bash

# Disable USB autosuspend
for i in /sys/bus/usb/devices/*/power/autosuspend; do
    echo -1 > $i 2>/dev/null
done

# Set USB3 to performance mode
echo performance > /sys/devices/platform/scb/fd500000.pcie/pci0000:00/0000:00:00.0/0000:01:00.0/usb1/power/control
echo performance > /sys/devices/platform/scb/fd500000.pcie/pci0000:00/0000:00:00.0/0000:01:00.0/usb2/power/control

# Increase USB URB memory
echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb

echo "USB optimized for RealSense D455"
EOF

sudo chmod +x /usr/local/bin/optimize_usb_for_d455.sh

# Run on boot
echo '@reboot root /usr/local/bin/optimize_usb_for_d455.sh' | sudo tee -a /etc/crontab
```

### 2. CPU Governor Settings

```bash
# Set CPU governor to performance
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Make persistent
echo 'GOVERNOR="performance"' | sudo tee -a /etc/default/cpufrequtils
```

### 3. Thermal Management

```bash
# Install cooling control
sudo apt install -y raspi-config

# Set fan to aggressive cooling (if using active cooling)
echo 'dtoverlay=gpio-fan,gpiopin=14,temp=50000' | sudo tee -a /boot/firmware/config.txt

# Monitor temperature during operation
watch -n 1 vcgencmd measure_temp
```

## VIO Integration Configuration

### 1. Create RealSense VIO Configuration

```yaml
# Save as config/realsense_vio.yaml
realsense_d455:
  serial_number: ""  # Leave empty for auto-detect

  # Stream configuration optimized for RPi5
  depth_stream:
    width: 640
    height: 480
    fps: 15
    format: "Z16"

  color_stream:
    width: 640
    height: 480
    fps: 15
    format: "BGR8"

  imu_stream:
    enabled: true
    fps: 200

  # Processing options
  options:
    enable_auto_exposure: true
    visual_preset: "low_ambient_light"
    depth_units: 0.001  # meters
    disparity_shift: 0
    laser_power: 150  # 0-360
    confidence_threshold: 2  # 0-3

  # Post-processing filters
  filters:
    decimation:
      enabled: true
      magnitude: 2  # Reduce resolution by factor of 2

    spatial:
      enabled: true
      magnitude: 2
      smooth_alpha: 0.5
      smooth_delta: 20

    temporal:
      enabled: true
      smooth_alpha: 0.4
      smooth_delta: 20
      persistence_control: 2

    hole_filling:
      enabled: true
      mode: "farest_from_around"
```

### 2. Integration with OpenVINS

```cpp
// realsense_driver.cpp - RealSense D455 driver for VIO

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>

class RealSenseD455Driver {
private:
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::align align_to_color{RS2_STREAM_COLOR};

    // Filters for depth processing
    rs2::decimation_filter dec_filter;
    rs2::spatial_filter spat_filter;
    rs2::temporal_filter temp_filter;
    rs2::hole_filling_filter hole_filter;

    std::mutex data_mutex;

public:
    RealSenseD455Driver() {
        // Configure streams for RPi5 performance
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 15);
        cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 200);
        cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200);

        // Configure filters for RPi5
        dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
        spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
        spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5f);
        temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4f);
    }

    bool start() {
        try {
            // Start pipeline with config
            auto profile = pipe.start(cfg);

            // Get depth sensor and configure
            auto depth_sensor = profile.get_device()
                .first<rs2::depth_sensor>();

            // Set visual preset for better quality
            if (depth_sensor.supports(RS2_OPTION_VISUAL_PRESET)) {
                depth_sensor.set_option(RS2_OPTION_VISUAL_PRESET,
                    RS2_L500_VISUAL_PRESET_LOW_AMBIENT_LIGHT);
            }

            // Enable auto exposure
            if (depth_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
                depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
            }

            // Set laser power for outdoor use
            if (depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
                depth_sensor.set_option(RS2_OPTION_LASER_POWER, 150);
            }

            // Warm up camera
            for (int i = 0; i < 30; i++) {
                pipe.wait_for_frames();
            }

            return true;

        } catch (const rs2::error& e) {
            std::cerr << "RealSense error: " << e.what() << std::endl;
            return false;
        }
    }

    void get_synchronized_frames(cv::Mat& color, cv::Mat& depth,
                                 float accel[3], float gyro[3]) {
        std::lock_guard<std::mutex> lock(data_mutex);

        rs2::frameset frames = pipe.wait_for_frames();

        // Align depth to color
        frames = align_to_color.process(frames);

        // Get color frame
        auto color_frame = frames.get_color_frame();
        color = cv::Mat(cv::Size(640, 480), CV_8UC3,
                       (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // Process depth with filters
        auto depth_frame = frames.get_depth_frame();
        depth_frame = dec_filter.process(depth_frame);
        depth_frame = spat_filter.process(depth_frame);
        depth_frame = temp_filter.process(depth_frame);
        depth_frame = hole_filter.process(depth_frame);

        // Convert to OpenCV Mat
        depth = cv::Mat(cv::Size(320, 240), CV_16UC1,
                       (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        // Get IMU data
        if (auto accel_frame = frames.first_or_default(RS2_STREAM_ACCEL)) {
            auto accel_data = reinterpret_cast<const float*>(
                accel_frame.get_data());
            memcpy(accel, accel_data, 3 * sizeof(float));
        }

        if (auto gyro_frame = frames.first_or_default(RS2_STREAM_GYRO)) {
            auto gyro_data = reinterpret_cast<const float*>(
                gyro_frame.get_data());
            memcpy(gyro, gyro_data, 3 * sizeof(float));
        }
    }
};
```

## Troubleshooting

### Common Issues and Solutions

#### 1. Device Not Found
```bash
# Check if device is detected
lsusb | grep 8086:0b5c  # D455 device ID

# Reset USB subsystem
sudo modprobe -r uvcvideo
sudo modprobe uvcvideo

# Check permissions
sudo usermod -a -G video $USER
logout and login again
```

#### 2. Low FPS or Stuttering
```bash
# Check USB speed
lsusb -t | grep "Intel"
# Should show "5000M" for USB 3.0

# Monitor CPU usage
htop

# Reduce stream resolution in config
# Use 640x480 @ 15fps instead of higher
```

#### 3. Compilation Errors
```bash
# Missing dependencies
sudo apt install -y libprotobuf-dev protobuf-compiler

# Out of memory during compilation
# Increase swap size to 4GB
sudo sed -i 's/CONF_SWAPSIZE=.*/CONF_SWAPSIZE=4096/' /etc/dphys-swapfile
sudo dphys-swapfile swapoff && sudo dphys-swapfile setup && sudo dphys-swapfile swapon

# Use fewer cores for compilation
make -j2  # Instead of -j4
```

#### 4. Python Import Errors
```bash
# Rebuild Python bindings
cd ~/realsense_build/librealsense/build
cmake .. -DBUILD_PYTHON_BINDINGS=true -DPYTHON_EXECUTABLE=$(which python3)
make -j4
sudo make install

# Add to Python path
echo 'export PYTHONPATH=$PYTHONPATH:/usr/local/lib' >> ~/.bashrc
source ~/.bashrc
```

## Performance Benchmarks

### Expected Performance on Raspberry Pi 5

| Configuration | Color FPS | Depth FPS | CPU Usage | Temperature |
|--------------|-----------|-----------|-----------|-------------|
| 640x480 @ 15fps | 15 | 15 | 35-40% | 65-70°C |
| 640x480 @ 30fps | 25-30 | 25-30 | 60-65% | 75-80°C |
| 848x480 @ 15fps | 15 | 15 | 40-45% | 70-75°C |
| 1280x720 @ 15fps | 12-15 | N/A | 50-55% | 75-80°C |

### Recommended Settings for VIO
- **Resolution**: 640x480 for both color and depth
- **FPS**: 15 Hz (synchronized)
- **IMU Rate**: 200 Hz
- **Filters**: Decimation + Spatial + Temporal
- **CPU Affinity**: Bind to cores 2-3

## Integration with VIO Pipeline

### Launch Script
```bash
#!/bin/bash
# Save as launch_vio_with_d455.sh

# Optimize USB
sudo /usr/local/bin/optimize_usb_for_d455.sh

# Set CPU performance
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Launch VIO with RealSense
cd ~/vio_system
python3 vio_bridge.py --camera realsense_d455 --config config/realsense_vio.yaml &

# Monitor performance
gnome-system-monitor &

echo "VIO system with RealSense D455 started"
```

## References

- [Intel RealSense SDK Documentation](https://github.com/IntelRealSense/librealsense)
- [Raspberry Pi 5 Technical Specifications](https://www.raspberrypi.com/products/raspberry-pi-5/)
- [RealSense D455 Datasheet](https://www.intelrealsense.com/depth-camera-d455/)
- [librealsense ARM Build Guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_raspbian.md)
