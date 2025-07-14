# Alexa_Robotic_Mmanipulator_Arm

A collection of ROS 2 packages for an Arduino-based robotic system with voice control integration through Amazon Alexa.

## Overview

This repository contains ROS 2 packages for a robotic system built around Arduino hardware with advanced features including motion planning, voice control, and remote operation capabilities. The system is designed to be controlled through voice commands via Amazon Alexa integration.

**Work in Progress:** This project is currently under active development. The Arduino board integration is not yet complete. Currently, the system supports simulation-based operation with plans for full hardware integration.

## Repository Structure

```
├── alexabot_bringup/          # Main launch package for Alexa integration
├── arduinobot_controller/     # Robot control and driver nodes
├── arduinobot_cpp/           # C++ implementation and utilities
├── arduinobot_description/    # Robot URDF, meshes, and configuration
├── arduinobot_moveit/        # MoveIt! motion planning configuration
├── arduinobot_msgs/          # Custom message definitions
├── arduinobot_python_examples/ # Python examples and demos
├── arduinobot_remote/        # Remote control functionality
└── arduinobot_utils/         # Utility functions and helpers
```

## Package Descriptions

### **alexabot_bringup** (Main Package)
The primary launch package that brings up the entire robot system with Alexa voice control integration. This package orchestrates all other components and provides the main entry point for the system.

### **arduinobot_controller**
Contains the low-level controller nodes that interface with Arduino hardware. Handles motor control, sensor reading, and hardware communication protocols.

### **arduinobot_cpp**
C++ implementations for performance-critical components including real-time control loops, hardware interfaces, and computational algorithms.

### **arduinobot_description**
Robot description files including:
- URDF models
- 3D meshes and visual representations
- Joint configurations
- Physical parameters and constraints

### **arduinobot_moveit**
MoveIt! motion planning configuration providing:
- Path planning capabilities
- Collision detection
- Inverse kinematics solving
- Motion execution

### **arduinobot_msgs**
Custom ROS message definitions for inter-node communication including specialized messages for voice commands, robot states, and sensor data.

### **arduinobot_python_examples**
Python-based examples and demonstration scripts showcasing various robot capabilities and usage patterns.

### **arduinobot_remote**
Remote control functionality enabling:
- Web-based control interfaces
- Mobile app integration
- Network communication protocols

### **arduinobot_utils**
Utility functions and helper modules used across multiple packages including common algorithms, data processing, and system utilities.

## Prerequisites

- ROS 2 (Humble/Iron/Rolling)
- Arduino IDE or Platform IO
- Python 3.8+
- MoveIt! 2
- Amazon Alexa Skills Kit (for voice control)

## Installation

1. **Create a ROS 2 workspace:**
   ```bash
   mkdir -p ~/arduinobot_ws/src
   cd ~/arduinobot_ws/src
   ```

2. **Clone this repository:**
   ```bash
   git clone <repository-url> .
   ```

3. **Install dependencies:**
   ```bash
   cd ~/arduinobot_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Build the packages:**
   ```bash
   colcon build
   ```

5. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## Quick Start

### Simulation Mode (Current)

1. **Launch the simulated robot:**
   ```bash
   ros2 launch alexabot_bringup simulated_robot.launch.py
   ```

2. **Launch with specific options:**
   ```bash
   # Without voice control
   ros2 launch alexabot_bringup simulated_robot.launch.py use_alexa:=false
   
   # With custom world
   ros2 launch alexabot_bringup simulated_robot.launch.py world:=my_world.world
   ```

3. **Test voice commands (if Alexa is enabled):**
   - Say "Alexa, tell Arduino bot to move forward"
   - Say "Alexa, ask Arduino bot to stop"

4. **Monitor system status:**
   ```bash
   ros2 topic echo /robot_status
   ```
## Demo Video

**[Watch Arduino Bot Demo](https://drive.google.com/file/d/1qvHi43QQj6JAHF6cioCBSAB-RQ5DIhqP/view?usp=sharing)**

This video demonstrates:
- Robot simulation in Gazebo
- Voice control integration
- Motion planning capabilities
- RViz visualization

### Hardware Mode (Coming Soon)

Hardware integration with Arduino boards is planned for future releases. This will include:
- Real-time motor control
- Sensor data acquisition
- Physical robot operation

## Usage Examples

### Voice Control
The system responds to natural language commands through Alexa:
- "Move to the kitchen"
- "Pick up the object"
- "Return to charging station"

### Remote Interface
Access the web interface at `http://localhost:8080` for manual control and monitoring.

## Configuration

Key configuration files:
- `config/robot_params.yaml` - Robot parameters
- `config/alexa_skills.json` - Voice command mappings
- `config/controllers.yaml` - Controller configurations

## Development Status

### Current Features
- ✅ Simulation environment with Gazebo
- ✅ Robot visualization with RViz
- ✅ Motion planning with MoveIt!
- ✅ Voice control integration framework
- ✅ Remote control interfaces
- ✅ Message definitions and utilities

### Planned Features
- 🔄 Arduino board integration
- 🔄 Real-time hardware control
- 🔄 Physical sensor integration
- 🔄 Production-ready voice commands
- 🔄 Mobile app integration
- 🔄 Advanced manipulation capabilities

## Development

### Adding New Voice Commands
1. Update Alexa skill configuration
2. Add command handlers in `alexabot_bringup`
3. Test with voice recognition

### Extending Functionality
1. Create new nodes in appropriate packages
2. Update launch files
3. Add relevant message definitions if needed

## Troubleshooting

### Common Issues

**Robot not responding to voice commands:**
- Check Alexa skill is enabled
- Verify network connectivity
- Ensure ROS nodes are running

**Hardware communication errors:**
- Note: Hardware integration is not yet implemented
- Arduino connection features are planned for future releases
- Current focus is on simulation-based development

**Motion planning failures:**
- Check joint limits
- Verify collision models
- Review MoveIt! configuration

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request
