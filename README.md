# 🤖 TurtleBot3 Wall Follower
> **Autonomous wall-following robot using ROS2 and laser-based navigation**

## 📋 Project Overview
A robust wall-following system that enables TurtleBot3 to autonomously navigate by maintaining a safe distance from walls while avoiding obstacles. Built with ROS2 and designed for real-world applications.

## ⚡ Key Features
- **🎯 Precise Distance Control** - Maintains 0.6m from walls using PID control
- **🛡️ Safety-First Design** - Emergency obstacle avoidance with front laser scanning
- **🔄 Adaptive Navigation** - Switches between right/left wall following automatically
- **⚖️ Stable Performance** - No spinning or oscillation behaviors
- **📡 Real-time Processing** - 10Hz control loop for responsive navigation

## 🏗️ Project Structure
```
turtlebot3_wall_follower_ws/
├── src/
│   └── wall_following_project/
│       ├── package.xml
│       ├── setup.py
│       ├── config/
│       │   ├── nav2_params.yaml
│       │   └── wall_following_params.yaml
│       ├── launch/
│       │   ├── wall_following.launch.py
│       │   └── wall_follower_gazebo.launch.py
│       ├── rviz/
│       │   └── wall_follower_config.rviz
│       └── wall_following_project/
│           ├── __init__.py
│           ├── wall_follower_controller.py  # Main control logic
│           ├── wall_detector.py             # Wall detection algorithms  
│           └── pid_controller.py            # PID control system
└── README.md
```

### 📁 Key Files Description
- **`wall_follower_controller.py`** - Main wall-following logic and ROS2 node
- **`wall_detector.py`** - Detects walls using laser scan data analysis
- **`pid_controller.py`** - PID controller for smooth wall following behavior
- **`wall_following_params.yaml`** - Configurable parameters for wall-following behavior
- **`wall_following.launch.py`** - Main launch file for the wall follower
- **`wall_follower_gazebo.launch.py`** - Launch file with Gazebo integration

## 🔧 Prerequisites
Ensure you have the following installed:
- **ROS2 Humble** 🟢
- **TurtleBot3 Packages** 🟢  
- **Gazebo Simulation** 🟢
- **Python 3.8+** 🟢

```bash
# Install TurtleBot3 packages (if not already installed)
sudo apt update
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-gazebo-*

# Set TurtleBot3 model environment variable
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

## 🚀 Build Instructions

### 1. 🔨 Clone and Setup Workspace
```bash
# Create workspace directory
mkdir -p ~/turtlebot3_wall_follower_ws/src
cd ~/turtlebot3_wall_follower_ws/src

# Clone your project (replace with your repository URL)
# git clone <your-repository-url> wall_following_project
# Or if you have the project locally, copy it to src/wall_following_project/

cd ~/turtlebot3_wall_follower_ws
```

### 2. 🔨 Build the Project
```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Build the specific package
colcon build --packages-select wall_following_project

# Source the workspace
source install/setup.bash
```

### 3. ✅ Verify Build
```bash
# Check if the package is properly built
ros2 pkg list | grep wall_following_project

# Verify launch files are accessible
ros2 launch wall_following_project --help
```

## 🎮 Running the System

### Method 1: Complete Simulation (Recommended for Testing)

**Terminal 1** - Launch Gazebo Simulation with Wall Follower
```bash
# Source workspace
cd ~/turtlebot3_wall_follower_ws
source install/setup.bash

# Export TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Launch complete system (Gazebo + Wall Follower)
ros2 launch wall_following_project wall_follower_gazebo.launch.py
```

### Method 2: Separate Launch (For Development/Debugging)

**Terminal 1** - Launch Gazebo Environment
```bash
# Export TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Launch Gazebo with TurtleBot3
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2** - Start Wall Follower Controller
```bash
# Source workspace
cd ~/turtlebot3_wall_follower_ws
source install/setup.bash

# Launch wall following controller
ros2 launch wall_following_project wall_following.launch.py
```

### Method 3: Real Robot Deployment
```bash
# On the robot or connected computer
cd ~/turtlebot3_wall_follower_ws
source install/setup.bash

# Ensure robot's lidar is active
ros2 launch turtlebot3_bringup robot.launch.py

# In another terminal, start wall follower
ros2 launch wall_following_project wall_following.launch.py
```

## 📊 Monitoring and Debugging

**Terminal 3** (Optional) - Monitor System Performance
```bash
# View robot velocity commands
ros2 topic echo /cmd_vel

# Monitor laser scan data
ros2 topic echo /scan --once

# Check node status
ros2 node list
ros2 node info /wall_follower_controller

# Monitor custom topics (if any)
ros2 topic list | grep wall
```

**Launch RViz for Visualization**
```bash
# Launch RViz with custom configuration
rviz2 -d ~/turtlebot3_wall_follower_ws/src/wall_following_project/rviz/wall_follower_config.rviz

# Or launch basic RViz
rviz2
```

## ⚙️ Configuration

### Modify Wall Following Parameters
Edit the configuration file to customize behavior:
```bash
nano ~/turtlebot3_wall_follower_ws/src/wall_following_project/config/wall_following_params.yaml
```

Common parameters to adjust:
- `target_distance`: Desired distance from wall (default: 0.6m)
- `max_linear_speed`: Maximum forward speed
- `max_angular_speed`: Maximum turning speed
- `pid_gains`: PID controller tuning parameters

### After Parameter Changes
```bash
# Rebuild and source
cd ~/turtlebot3_wall_follower_ws
colcon build --packages-select wall_following_project
source install/setup.bash

# Restart the system
ros2 launch wall_following_project wall_following.launch.py
```

## 🧠 Algorithm Logic
1. **🔍 Wall Detection** - Multi-point laser scan analysis for consistent wall identification
2. **📏 Distance Calculation** - Real-time measurement using 270° laser range
3. **🎮 PID Control** - Smooth angular velocity control for stable following
4. **⚠️ Obstacle Avoidance** - Front-facing safety override system

## 🛠️ Technical Specifications
| Parameter | Value | Description |
|-----------|--------|-------------|
| Target Distance | 0.6m | Safe following distance |
| Max Speed | 0.25 m/s | Forward velocity limit |
| Control Frequency | 10Hz | Update rate |
| Safety Range | 0.5m | Emergency stop distance |

## 🎯 Use Cases
- **🏭 Industrial Inspection** - Automated facility perimeter monitoring
- **🏠 Domestic Robotics** - Room boundary navigation
- **📚 Educational Demos** - Robotics and control system learning
- **🔬 Research Platform** - Navigation algorithm development

## 🐛 Troubleshooting

### Common Issues:
1. **"Package not found" error**
   ```bash
   # Ensure package is built and sourced
   colcon build --packages-select wall_following_project
   source install/setup.bash
   ```

2. **Robot not moving**
   ```bash
   # Check if cmd_vel topic is being published
   ros2 topic hz /cmd_vel
   # Verify laser data is available
   ros2 topic hz /scan
   ```

3. **Build failures**
   ```bash
   # Clean build and retry
   rm -rf build/ install/ log/
   colcon build --packages-select wall_following_project
   ```
