# KinectPong

A gesture-controlled Pong game using Xbox Kinect for hand tracking. The system consists of two modules: a C++ server that captures depth and RGB data from the Kinect sensor, and a Unity game that receives hand coordinates via UDP to control the paddles.

## Overview

This project demonstrates real-time hand tracking using the Xbox Kinect sensor to control a Pong game. The architecture separates sensor processing from game logic through UDP communication.

### Architecture

```
Xbox Kinect → C++ Server → UDP Protocol → Unity Game
   (Sensor)   (Processing)   (Network)     (Rendering)
```

## Modules

### Module 1: Kinect Server (C++)

Located in `src/udp_server/`, this module handles:
- Depth and RGB data capture from Xbox Kinect using libfreenect
- Hand detection using OpenCV
- Coordinate normalization to [0, 1] range
- UDP server broadcasting hand positions in JSON format

**Key Files:**
- `src/udp_server/src/simple_server.cpp` - Main UDP server with Kinect integration
- `src/udp_server/src/simple_detector.hpp` - Hand detection logic
- `src/udp_server/src/capture-cv.hpp` - Kinect capture interface
- `src/udp_server/src/calibrate-qt.cpp` - Calibration tool with Qt GUI

### Module 2: Unity Pong Game (C#)

Located in `Unity/`, this module provides:
- UDP client receiving normalized hand coordinates
- Pong game mechanics with paddle control
- Real-time visualization of hand positions
- Keyboard fallback controls (W/S keys)

**Key Files:**
- `Unity/Assets/Scripts/KinectHandTracker.cs` - UDP client and coordinate mapping
- `Unity/Assets/Scripts/Engine.cs` - Game engine and calibration triggers
- `Unity/Assets/Scripts/GameBehaviour.cs` - Ball physics and scoring
- `Unity/Assets/Scripts/movePaddle.cs` - Paddle movement logic

## Getting Started

### Prerequisites

**For Kinect Server:**
- Xbox Kinect sensor
- libfreenect (0.7.0)
- OpenCV (with imgproc, calib3d, core, imgcodecs)
- CMake (3.14+)
- Conan package manager
- Qt6 (for calibration tool)
- GLUT and OpenGL

**For Unity Game:**
- Unity 2022.3 or later
- .NET 4.x runtime

### Installation

#### 1. Set Up Kinect Server

```bash
cd src/udp_server

# Install dependencies with Conan
conan install . --build=missing

# Build with CMake
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake
cmake --build build

# Configure network settings
cp .env.example .env
# Edit .env to set UDP_SERVER_PORT and IP addresses
```

#### 2. Set Up Unity Game

```bash
# Extract Unity project
./run.sh untar

# Open in Unity
# File → Open Project → Select 'build' directory
```

Alternatively, open the `Unity/` directory directly in Unity Editor.

### Running the System

#### Step 1: Start the Kinect Server

```bash
cd src/udp_server
./build/simple_server
```

The server will:
- Initialize Kinect sensor (or fall back to dummy mode)
- Start UDP server on configured port (default: 8888)
- Begin broadcasting hand coordinates

#### Step 2: Launch Unity Game

1. Open the Unity project
2. Load the main scene
3. Press Play in Unity Editor, or build and run the executable

#### Step 3: Calibrate (Optional)

In Unity, press `C` or click "Calibrate" button to:
- Trigger depth calibration
- Adjust detection thresholds
- Optimize hand tracking

## UDP Protocol

The server broadcasts JSON messages with hand coordinates:

```json
{
  "timestamp": 1234567890,
  "mode": "kinect",
  "hands": [
    {"x": 0.45, "y": 0.67, "z": 1.2, "id": 0},
    {"x": 0.55, "y": 0.33, "z": 1.3, "id": 1}
  ],
  "objects": []
}
```

**Coordinate System:**
- `x`, `y`: Normalized to [0, 1] range
- `z`: Depth in meters
- `id`: 0 for left hand, 1 for right hand

## Project Structure

```
KinectPong/
├── src/udp_server/           # C++ Kinect server
│   ├── src/
│   │   ├── simple_server.cpp    # Main server
│   │   ├── simple_detector.hpp  # Hand detection
│   │   ├── capture-cv.cpp       # Kinect capture
│   │   └── calibrate-qt.cpp     # Calibration tool
│   ├── CMakeLists.txt
│   └── conanfile.txt
├── Unity/                    # Unity game project
│   └── Assets/
│       └── Scripts/
│           ├── KinectHandTracker.cs  # UDP client
│           ├── Engine.cs             # Game engine
│           ├── GameBehaviour.cs      # Ball physics
│           └── movePaddle.cs         # Paddle control
├── .env                      # Network configuration
└── run.sh                    # Build helper script
```

## Configuration

Edit `.env` file to configure network settings:

```bash
UDP_IP_WSL=172.22.181.115      # Server IP (WSL)
UDP_IP_UBUNTU=127.0.0.1        # Server IP (Ubuntu)
UDP_SERVER_PORT=8888           # Server broadcast port
UDP_CLIENT_PORT=8000           # Client listening port
```

## Features

- Real-time hand tracking using Xbox Kinect depth sensor
- Dual control modes: gesture-based and keyboard fallback
- Automatic depth calibration
- Smooth coordinate interpolation with configurable smoothing
- Visual debugging with coordinate overlay
- Score tracking and timer system
- Dual-player support (left and right paddles)

## Troubleshooting

**Kinect not detected:**
- Ensure Xbox Kinect is properly connected via USB
- Check libfreenect installation and permissions
- Server will fall back to dummy mode if Kinect unavailable

**Unity not receiving data:**
- Verify IP address and port in `.env` match Unity settings
- Check firewall settings allow UDP traffic on specified port
- Ensure server is running before starting Unity game

**Hand detection issues:**
- Run calibration routine (press `C` in Unity)
- Adjust lighting conditions
- Ensure hands are within Kinect depth range (0.5m - 4.5m)

## License

See individual source files for license information. Kinect integration code based on OpenKinect Project examples.