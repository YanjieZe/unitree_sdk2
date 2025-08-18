# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is the Unitree SDK2, a C++ library for controlling and communicating with Unitree robots including G1, H1, H1-2, Go2, and B2 series. The SDK provides both high-level and low-level control interfaces using DDS (Data Distribution Service) for real-time communication.

## Common Development Commands

### Building the Project

```bash
# Basic build
mkdir build
cd build
cmake ..
make

# Build with Python bindings
cmake -DBUILD_PYTHON_BINDING=ON -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

# Install to system directory
sudo make install

# Install to custom directory
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
sudo make install
```

### Python Binding

```bash
# Build from main project (recommended)
cmake -DBUILD_PYTHON_BINDING=ON -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

# Or build separately
cd python_binding
./build.sh --sdk-path /opt/unitree_sdk2
```

## Architecture Overview

### Core Components

1. **Common Layer** (`include/unitree/common/`):
   - DDS communication framework (`common/dds/`)
   - Logging system (`common/log/`)
   - Threading utilities (`common/thread/`)
   - JSON serialization (`common/json/`)
   - File system utilities (`common/filesystem/`)

2. **IDL Definitions** (`include/unitree/idl/`):
   - `go2/`: Go2 robot message definitions (LowCmd_, LowState_, SportModeCmd_, etc.)
   - `hg/`: Humanoid/G1 robot message definitions
   - `ros2/`: ROS2 compatible message types

3. **Robot-Specific APIs** (`include/unitree/robot/`):
   - `g1/`: G1 humanoid robot (arm control, locomotion, audio)
   - `h1/`: H1 humanoid robot (locomotion)
   - `go2/`: Go2 quadruped robot (sport mode, video, voice UI)
   - `b2/`: B2 series robots

4. **Communication Layer**:
   - `robot/channel/`: DDS channel publishers/subscribers
   - `robot/client/`: High-level client interfaces with lease management
   - `robot/server/`: Server-side components

### Message Types

- **HG Format**: Used by humanoid robots (G1, H1-2) - supports 29 motors
- **GO2 Format**: Used by Go2 and H1 robots - supports 19 motors for H1, 12 for Go2

### Control Modes

- **High-Level Control**: Uses client APIs (e.g., `g1::LocoClient`, `go2::SportClient`)
- **Low-Level Control**: Direct motor control via LowCmd/LowState messages
- **Real-time Control**: 500Hz control loops for precise motor control

## Robot-Specific Details

### G1 Humanoid (29 DOF)
- High-level: Locomotion, arm actions, audio
- Low-level: Direct motor control with dual-arm support
- Message format: HG
- Examples: `example/g1/`

### H1 Humanoid (19 DOF)
- High-level: Locomotion control
- Low-level: 27 DOF control examples available
- Message format: GO2 (despite being humanoid)
- Examples: `example/h1/`

### Go2 Quadruped (12 DOF)
- High-level: Sport mode, video streaming, voice UI
- Low-level: Motor control, trajectory following
- Message format: GO2
- Examples: `example/go2/`

## Key Conventions

### Network Interface
- Most examples use command-line argument `--network_interface=<interface>` (default: "lo" or "eth0")
- Initialize with: `ChannelFactory::Instance()->Init(0, interface_name)`

### Control Loop Pattern
```cpp
// Typical low-level control structure:
1. Initialize channels and subscribers
2. Set up control parameters (Kp, Kd gains)
3. Run control loop at fixed rate (500Hz typical)
4. Read robot state → Process → Send commands
```

### Safety Considerations
- Always implement emergency stop mechanisms
- Monitor joint temperatures and limits
- Use wireless controller B button as emergency stop
- Gradual motion changes for safety

### Python Integration
- Supports G1, H1, H1-2 robots through unified interface
- Factory methods: `create_g1()`, `create_h1()`, `create_h1_2()`
- Type-safe with `.pyi` stub files
- Thread-safe data exchange via buffers

## Development Notes

- Built on Cyclone DDS for real-time communication
- C++17 required
- Architecture support: x86_64 and aarch64
- Precompiled libraries provided in `lib/` directory
- Header-only interface with static linking to `libunitree_sdk2.a`
- Examples demonstrate both high-level API usage and low-level motor control