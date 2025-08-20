# VIO Development Notes

## Project Structure

```
vio/
|-- src/
|   |-- core/               # C++ VIO pipeline
|   |   |-- cameras/        # Camera drivers (RealSense, V4L2)
|   |   |-- features/       # Feature extraction (OpenCV, HAILO)
|   |   |-- vio/            # OpenVINS wrapper, IMU handling
|   |   |-- comms/          # ZeroMQ publisher
|   |-- bridge/             # Python MAVLink bridge
|   |-- tools/              # Utilities (calibration, monitoring)
|-- config/                 # YAML configurations
|-- cmake/                  # CMake modules
|-- tests/                  # Unit and integration tests
|-- launch/                 # System launch scripts
```

## Architecture Decisions

### Language Split
- **C++**: All performance-critical vision/VIO processing
  - Camera capture and synchronization
  - Feature extraction and tracking
  - OpenVINS VIO estimation
  - IMU data processing
  
- **Python**: Communication and monitoring only
  - MAVLink bridge to ArduPilot
  - System health monitoring
  - Configuration management
  - Data recording/playback

### Communication
- **ZeroMQ**: IPC between C++ VIO and Python bridge
- **MAVLink**: Direct connection to ArduPilot (no ROS2)
- **Shared memory**: Optional for high-bandwidth camera data

## Development Phases

### Phase 1: Core Infrastructure (Current)
- [ ] Set up CMake build system
- [ ] Create RealSense capture module
- [ ] Implement V4L2 driver for USB cameras
- [ ] Basic ZeroMQ publisher
- [ ] Python MAVLink bridge skeleton

### Phase 2: Feature Pipeline
- [ ] OpenCV feature extraction (ORB/FAST)
- [ ] Multi-camera synchronization
- [ ] Feature tracking across frames
- [ ] HAILO SuperPoint integration

### Phase 3: VIO Integration
- [ ] OpenVINS configuration
- [ ] IMU data from Pixhawk
- [ ] Sensor fusion pipeline
- [ ] Coordinate frame transformations

### Phase 4: Testing & Optimization
- [ ] ArduPilot SITL testing
- [ ] Performance profiling
- [ ] USB bandwidth optimization
- [ ] Field testing

## Build Instructions

```bash
# Dependencies
sudo apt install -y cmake build-essential libopencv-dev libeigen3-dev libzmq3-dev

# Build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

# Run
./vio_core --config ../config/vio.yaml
```

## Testing

```bash
# Unit tests
cd build
ctest

# SITL test
./launch/sitl_test.sh

# Hardware test
./launch/vio_system.sh
```

## Performance Targets

- **Camera capture**: 4 cameras @ 15-20 FPS
- **Feature extraction**: <10ms per frame
- **VIO update rate**: 20Hz minimum
- **End-to-end latency**: <50ms
- **CPU usage**: <60% on RPi5

## Current Issues

- [ ] None yet

## Next Steps

1. Create CMakeLists.txt with RealSense/OpenCV dependencies
2. Implement basic RealSense capture test
3. Set up ZeroMQ pub/sub test
4. Verify MAVLink connection to Pixhawk

## Useful Commands

```bash
# Monitor USB bandwidth
usb-devices | grep -A 5 "Intel"

# Check camera devices
v4l2-ctl --list-devices

# Monitor CPU/thermal
htop
vcgencmd measure_temp

# ArduPilot SITL
sim_vehicle.py -v ArduCopter --console --map
```

## Dependencies

### C++ Libraries
- OpenVINS (git submodule)
- librealsense2 (2.54.2)
- OpenCV (4.x)
- Eigen3
- ZeroMQ (cppzmq)

### Python Libraries
- pymavlink
- pyzmq
- numpy
- pyyaml

## Notes

- Keep camera resolution at 640x480 for RPi5 performance
- USB bandwidth limit: ~35-40 MB/s sustained
- Use MJPEG compression for USB cameras
- Bind processes to specific CPU cores for consistency
- D455 IMU runs at 200Hz, cameras at 15Hz