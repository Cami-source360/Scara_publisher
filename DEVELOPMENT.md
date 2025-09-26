# Development Guide - SCARA Publisher

This guide explains how to develop and upload this ROS2 package using Visual Studio Code.

## Quick Start

### 1. Clone and Open in VS Code

```bash
git clone https://github.com/Cami-source360/Scara_publisher.git
cd Scara_publisher
code .
```

### 2. Install Recommended Extensions

When you open the project, VS Code will suggest installing recommended extensions. Click "Install All" or install them manually:

- **Python** - Python language support
- **ROS** - ROS/ROS2 support and tools
- **XML Tools** - XML formatting and validation
- **YAML** - YAML language support
- **Black Formatter** - Python code formatting
- **Flake8** - Python linting

### 3. Configure Python Interpreter

1. Press `Ctrl+Shift+P`
2. Type "Python: Select Interpreter"
3. Choose `/usr/bin/python3` or your ROS2 Python environment

## Development Workflow

### Building the Package

#### Option 1: Using VS Code Tasks
1. Press `Ctrl+Shift+P`
2. Type "Tasks: Run Task"
3. Select "colcon: build"

#### Option 2: Terminal
```bash
# From workspace root (parent of this package)
colcon build --packages-select scara_publisher
```

### Running and Debugging

#### Method 1: Using VS Code Debug Configuration
1. Open the Run and Debug panel (`Ctrl+Shift+D`)
2. Select a configuration:
   - "ROS2: Launch SCARA Publisher" - Run publisher node
   - "ROS2: Launch SCARA Controller" - Run controller node
   - "ROS2: Launch All Nodes" - Run complete system
3. Press `F5` or click the green play button

#### Method 2: Using Terminal
```bash
# Method 2a: Launch all nodes
ros2 launch scara_publisher scara_publisher_launch.py

# Method 2b: Run individual nodes
ros2 run scara_publisher scara_publisher_node
ros2 run scara_publisher scara_controller_node
```

### Testing

#### Run Tests via VS Code
1. Press `Ctrl+Shift+P`
2. Type "Tasks: Run Task"
3. Select "colcon: test"

#### Run Tests via Terminal
```bash
# All tests
colcon test --packages-select scara_publisher

# Specific test file
python3 -m pytest test/test_scara_publisher.py

# With coverage
python3 -m pytest test/ --cov=scara_publisher
```

### Code Quality

#### Automatic Formatting
- **Black**: Automatically formats Python code on save
- Configure in `.vscode/settings.json`

#### Linting
- **Flake8**: Checks code style and common errors
- **Pylint**: Additional code quality checks
- Errors and warnings appear in VS Code Problems panel

#### Manual Formatting/Linting
```bash
# Format code
black scara_publisher/

# Lint code
flake8 scara_publisher/
pylint scara_publisher/
```

## Project Structure for Development

```
scara_publisher/
├── scara_publisher/           # Main package code
│   ├── scara_publisher_node.py    # Publisher node
│   └── scara_controller_node.py   # Controller node
├── launch/                    # Launch files
├── config/                    # Configuration files
├── test/                      # Unit tests
├── examples/                  # Usage examples
├── .vscode/                   # VS Code configuration
└── docs/                      # Documentation (if added)
```

## VS Code Features

### IntelliSense and Code Completion
- ROS2 API completion
- Python standard library completion
- Custom package imports

### Debugging Features
- Breakpoints in Python code
- Variable inspection
- Step-through debugging
- Console output

### Integrated Terminal
- Multiple terminals
- ROS2 environment automatically sourced
- Easy command execution

### File Navigation
- Quick file search (`Ctrl+P`)
- Symbol search (`Ctrl+Shift+O`)
- Go to definition (`F12`)

## Upload/Deployment Workflow

### 1. Version Management
Update version in these files when releasing:
- `package.xml` - `<version>` tag
- `setup.py` - `version` parameter

### 2. Git Workflow
```bash
# Add changes
git add .

# Commit with descriptive message
git commit -m "Add new feature: description"

# Push to repository
git push origin main
```

### 3. Testing Before Upload
Always run tests before pushing:
```bash
# Syntax check
python3 -m py_compile scara_publisher/*.py

# Unit tests
python3 -m pytest test/

# Integration test
python3 examples/basic_usage.py
```

### 4. Package Validation
```bash
# Check package structure
python3 setup.py check

# Validate XML
python3 -c "import xml.etree.ElementTree as ET; ET.parse('package.xml')"

# Validate YAML
python3 -c "import yaml; yaml.safe_load(open('config/scara_config.yaml'))"
```

## Troubleshooting

### Common Issues

#### "Module not found" errors
```bash
# Ensure ROS2 is sourced
source /opt/ros/humble/setup.bash

# Build and source package
colcon build --packages-select scara_publisher
source install/setup.bash

# Add to Python path if needed
export PYTHONPATH=$PYTHONPATH:$(pwd)
```

#### VS Code Python interpreter issues
1. Press `Ctrl+Shift+P`
2. Type "Python: Select Interpreter"
3. Choose the correct Python path
4. Restart VS Code if needed

#### Build errors
```bash
# Clean build
rm -rf build/ install/ log/
colcon build --packages-select scara_publisher
```

#### Launch file not found
```bash
# Ensure package is built and sourced
source install/setup.bash

# Check launch file path
ros2 pkg prefix scara_publisher
```

### Debug Tips

1. **Use print statements** for quick debugging
2. **Set breakpoints** in VS Code for detailed inspection
3. **Check ROS2 topics** with `ros2 topic list` and `ros2 topic echo`
4. **Verify node status** with `ros2 node list`
5. **Check parameters** with `ros2 param list`

## Advanced Features

### Custom Tasks
Add custom tasks to `.vscode/tasks.json` for frequent operations.

### Launch Configurations
Modify `.vscode/launch.json` to add custom debug configurations.

### Settings Sync
Use VS Code Settings Sync to share configurations across machines.

### Extensions
Recommended additional extensions for ROS2 development:
- GitLens
- Python Docstring Generator
- Todo Tree
- Bracket Pair Colorizer

## Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/your-feature`
3. Make changes following the coding standards
4. Add tests for new functionality
5. Ensure all tests pass
6. Submit a pull request

## Resources

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [VS Code Python Extension](https://marketplace.visualstudio.com/items?itemName=ms-python.python)
- [ROS VS Code Extension](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros)
- [Colcon Documentation](https://colcon.readthedocs.io/)