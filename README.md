# SCARA Publisher - ROS2 Package

A comprehensive ROS2 package for SCARA (Selective Compliance Assembly Robot Arm) robot control and simulation. This package provides publisher and controller nodes for SCARA robot joint control, with full Visual Studio Code integration for easy development and deployment.

## Features

- **SCARA Publisher Node**: Publishes joint states and commands for a 4-DOF SCARA robot
- **SCARA Controller Node**: Subscribes to commands and provides position/velocity control
- **Forward and Inverse Kinematics**: Complete kinematic calculations for SCARA robot
- **Configurable Parameters**: YAML-based configuration for robot parameters
- **Launch Files**: Easy deployment with ROS2 launch system
- **VS Code Integration**: Complete development environment setup
- **Testing Framework**: Unit tests for all major components

## Package Structure

```
scara_publisher/
├── scara_publisher/           # Python package source code
│   ├── __init__.py
│   ├── scara_publisher_node.py    # Main publisher node
│   └── scara_controller_node.py   # Controller node
├── launch/                    # Launch files
│   └── scara_publisher_launch.py
├── config/                    # Configuration files
│   └── scara_config.yaml
├── test/                      # Unit tests
│   └── test_scara_publisher.py
├── .vscode/                   # VS Code configuration
│   ├── settings.json
│   ├── launch.json
│   ├── tasks.json
│   └── extensions.json
├── package.xml                # ROS2 package manifest
├── setup.py                   # Python package setup
├── setup.cfg                  # Setup configuration
├── requirements.txt           # Python dependencies
└── README.md                  # This file
```

## Prerequisites

- ROS2 Humble (or compatible distribution)
- Python 3.8+
- Visual Studio Code (recommended)

### Required ROS2 Packages

```bash
sudo apt install ros-humble-rclpy ros-humble-std-msgs ros-humble-geometry-msgs ros-humble-sensor-msgs
```

## Installation

### 1. Clone the Repository

```bash
# Create a ROS2 workspace if you don't have one
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/Cami-source360/Scara_publisher.git
```

### 2. Install Dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select scara_publisher
source install/setup.bash
```

## Usage

### Running the Nodes

#### Option 1: Using Launch File (Recommended)

```bash
# Source your ROS2 installation
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch all nodes
ros2 launch scara_publisher scara_publisher_launch.py

# Launch with custom parameters
ros2 launch scara_publisher scara_publisher_launch.py publish_rate:=20.0 control_rate:=50.0
```

#### Option 2: Running Individual Nodes

```bash
# Terminal 1: Publisher Node
ros2 run scara_publisher scara_publisher_node

# Terminal 2: Controller Node
ros2 run scara_publisher scara_controller_node
```

### Monitoring Topics

```bash
# List all topics
ros2 topic list

# Monitor joint states
ros2 topic echo /joint_states

# Monitor end effector position
ros2 topic echo /end_effector_position

# Monitor motor commands
ros2 topic echo /motor_commands
```

### Sending Commands

```bash
# Send joint position commands
ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray "data: [0.5, 0.3, 0.1, 0.8]"

# Send cartesian position commands
ros2 topic pub /cartesian_commands geometry_msgs/msg/Point "x: 0.4, y: 0.2, z: 0.1"
```

## Visual Studio Code Development

### 1. Open in VS Code

```bash
cd ~/ros2_ws/src/Scara_publisher
code .
```

### 2. Install Recommended Extensions

VS Code will automatically suggest the recommended extensions when you open the project. Install them for the best development experience:

- Python
- ROS
- XML Tools
- YAML Support
- And more...

### 3. Development Features

- **IntelliSense**: Full Python and ROS2 API completion
- **Debugging**: Launch configurations for debugging nodes
- **Tasks**: Build, test, and lint tasks
- **Formatting**: Automatic code formatting with Black
- **Linting**: Code quality checking with Flake8 and Pylint

### 4. Running from VS Code

1. Press `Ctrl+Shift+P` and type "Python: Select Interpreter"
2. Choose your Python interpreter (usually `/usr/bin/python3`)
3. Press `F5` to start debugging, or use the Run and Debug panel
4. Select the appropriate launch configuration:
   - "ROS2: Launch SCARA Publisher"
   - "ROS2: Launch SCARA Controller"
   - "ROS2: Launch All Nodes"

## Configuration

Edit `config/scara_config.yaml` to customize robot parameters:

```yaml
scara_publisher:
  ros__parameters:
    publish_rate: 10.0
    joint_names: ['joint1', 'joint2', 'joint3', 'joint4']
    workspace_radius: 0.5

scara_controller:
  ros__parameters:
    control_rate: 20.0
    max_joint_velocity: 1.0
    position_tolerance: 0.01
```

## Testing

### Running Tests

```bash
# Run all tests
cd ~/ros2_ws
colcon test --packages-select scara_publisher

# Run specific test
python3 -m pytest src/scara_publisher/test/test_scara_publisher.py

# Run tests with coverage
python3 -m pytest src/scara_publisher/test/ --cov=scara_publisher
```

### Adding New Tests

Create test files in the `test/` directory following the naming convention `test_*.py`.

## Robot Parameters

### SCARA Robot Configuration

The package is configured for a typical 4-DOF SCARA robot:

- **Joint 1**: Base rotation (revolute) - ±180°
- **Joint 2**: Shoulder (revolute) - ±90°
- **Joint 3**: Vertical movement (prismatic) - ±20cm
- **Joint 4**: Wrist rotation (revolute) - ±180°

### Physical Parameters

- **Link 1 Length**: 0.3m
- **Link 2 Length**: 0.2m
- **Workspace Radius**: 0.5m (configurable)

## Troubleshooting

### Common Issues

1. **Module not found error**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   export PYTHONPATH=$PYTHONPATH:~/ros2_ws/src/scara_publisher
   ```

2. **Permission denied**:
   ```bash
   chmod +x scara_publisher/scara_publisher_node.py
   chmod +x scara_publisher/scara_controller_node.py
   ```

3. **Build errors**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select scara_publisher --cmake-clean-cache
   ```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests for new functionality
5. Ensure all tests pass
6. Submit a pull request

### Code Style

This project uses:
- **Black** for code formatting
- **Flake8** for linting
- **Pylint** for additional code quality checks

Run these tools before submitting:

```bash
black scara_publisher/
flake8 scara_publisher/
pylint scara_publisher/
```

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- ROS2 community for the excellent robotics framework
- SCARA robot kinematic references and implementations
- Open source robotics community

## Contact

For questions or support, please open an issue on GitHub or contact the maintainer at maintainer@example.com.