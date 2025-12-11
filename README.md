# 🤖 TurtleBot3 Wall Following Robot
A production-ready ROS2 wall following implementation using PID control and reactive navigation for TurtleBot3.
---
## 📋 Prerequisites
- [![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/humble/Installation.html) **ROS2 Humble**
- [![Python](https://img.shields.io/badge/Python-3.8+-yellow?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/downloads/) **Python 3.8+**
- [![Gazebo](https://img.shields.io/badge/Gazebo-Classic-orange?style=for-the-badge&logo=gazebo&logoColor=white)](https://classic.gazebosim.org/tutorials?tut=install_ubuntu) **Gazebo Classic**
- [![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange?style=for-the-badge&logo=ubuntu&logoColor=white)](https://ubuntu.com/) **Ubuntu 22.04**
---
## 🏗️ Project Structure
```
turtlebot3_wall_follower_ws/
├── src/
│   └── wall_following_project/
│       ├── package.xml                       # ROS2 package metadata
│       ├── setup.py                          # Python package configuration
│       ├── config/
│       │   └── wall_following_params.yaml    # Tunable parameters (PID, speeds, safety)
│       ├── launch/
│       │   ├── wall_following.launch.py      # Controller launch file
│       │   └── wall_follower_gazebo.launch.py # Full simulation launch
│       ├── rviz/
│       │   └── wall_follower_config.rviz     # Visualization config
│       └── wall_following_project/
│           ├── __init__.py
│           └── wall_follower_controller.py   # Main control logic (PID + state machine)
└── README.md
```
---
## 🧠 Algorithm Overview
### Core Concept: Reactive Wall Following
The robot maintains a **constant distance from walls** using sensor feedback without requiring maps or localization (SLAM-free navigation).
### Control Flow
```
┌─────────────┐
│  SEARCHING  │  ← Robot rotates to find nearest wall
└──────┬──────┘
       │ Wall detected
       ▼
┌─────────────┐
│  FOLLOWING  │  ← PID control maintains target distance
└──────┬──────┘
       │ Obstacle detected
       ▼
┌─────────────┐
│  AVOIDING   │  ← Emergency stop + escape maneuver
└──────┬──────┘
       │ Clear path
       └──────► Return to FOLLOWING
```
### PID Control System
```
Error = Desired_Distance - Current_Wall_Distance
Angular_Velocity = Kp × Error + Kd × (Error - Previous_Error) / dt
Where:
  Kp = 0.7  (Proportional gain - responsiveness)
  Kd = 1.0  (Derivative gain - smoothness)
  dt = 0.05 (50ms control loop)
```
**How it works:**
- **Error > 0** (too far from wall) → Turn toward wall
- **Error < 0** (too close to wall) → Turn away from wall
- **Derivative term** prevents oscillation by damping aggressive corrections
### Architecture Components
| Component | Purpose |
|-----------|---------|
| **Laser Scan Processing** | Filters LiDAR data at multiple angles (-90° to +90°) |
| **State Machine** | Manages behavior transitions (SEARCHING/FOLLOWING/AVOIDING) |
| **PID Controller** | Calculates angular velocity to maintain wall distance |
| **Collision Detector** | Multi-zone safety checks (front, sides, wide angles) |
| **Velocity Publisher** | Sends movement commands to `/cmd_vel` |
---
## ⚙️ Configuration Parameters
**File:** `config/wall_following_params.yaml`
| Parameter | Default | Description |
|-----------|---------|-------------|
| `desired_distance` | 1.8m | Target distance from wall |
| `forward_speed` | 0.30 m/s | Normal forward speed |
| `search_speed` | 0.16 m/s | Speed while searching for wall |
| `max_angular_speed` | 0.6 rad/s | Maximum turn rate |
| `kp` | 0.7 | PID proportional gain |
| `kd` | 1.0 | PID derivative gain |
| `emergency_stop_distance` | 0.55m | Immediate stop threshold |
| `slow_down_distance` | 0.8m | Begin speed reduction |
| `wall_min_distance` | 0.45m | Minimum wall clearance |
| `wall_lost_distance` | 2.5m | Distance to consider wall lost |
| `side_clearance` | 0.4m | Minimum side obstacle margin |
---
## 🚀 Installation
```bash
# 1. Install TurtleBot3 packages
sudo apt update
sudo apt install ros-humble-turtlebot3*
# 2. Set TurtleBot3 model
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
# 3. Clone and build workspace
cd ~/
git clone <your-repo-url> turtlebot3_wall_follower_ws
cd turtlebot3_wall_follower_ws
colcon build
source install/setup.bash
```
---
## 🎮 Usage
### Quick Start (Single Command)
```bash
ros2 launch wall_following_project wall_follower_gazebo.launch.py
```
### Manual Launch (Two Terminals)
**Terminal 1 - Gazebo:**
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
**Terminal 2 - Wall Follower:**
```bash
cd ~/turtlebot3_wall_follower_ws
source install/setup.bash
ros2 launch wall_following_project wall_following.launch.py
```
### RViz Visualization (Optional)
```bash
rviz2 -d ~/turtlebot3_wall_follower_ws/src/wall_following_project/rviz/wall_follower_config.rviz
```
---
## 🔧 Runtime Parameter Tuning
### Change Parameters Without Stopping Simulation
**Change wall distance (while running):**
```bash
ros2 param set /wall_follower_controller desired_distance 1.0
```
**Change forward speed:**
```bash
ros2 param set /wall_follower_controller forward_speed 0.25
```
**Change turn speed:**
```bash
ros2 param set /wall_follower_controller max_angular_speed 0.7
```
**Change PID gains:**
```bash
ros2 param set /wall_follower_controller kp 1.5
ros2 param set /wall_follower_controller kd 0.8
```
**View all current parameters:**
```bash
ros2 param list /wall_follower_controller
ros2 param get /wall_follower_controller desired_distance
```
> **Note:** Changes take effect immediately but are **not saved**. To make permanent changes, edit `config/wall_following_params.yaml` and restart.
---
## 🎯 Tuning Guide
### For Faster Following
```yaml
forward_speed: 0.35        # Increase from 0.30
kp: 1.0                    # More aggressive correction
kd: 1.2                    # Dampen oscillation
```
### For Tighter Walls (Narrow Corridors)
```yaml
desired_distance: 1.0      # Closer following
emergency_stop_distance: 0.50
side_clearance: 0.35
```
### For Better Stability (Smooth Following)
```yaml
kp: 0.5                    # Less aggressive
kd: 1.5                    # More damping
forward_speed: 0.20        # Slower = smoother
```
### For Obstacle-Dense Environments
```yaml
emergency_stop_distance: 0.65  # Earlier braking
slow_down_distance: 1.0        # More gradual slowdown
side_clearance: 0.5            # Wider safety margin
```
---
## 🐛 Troubleshooting
### Robot doesn't move
```bash
# Check laser data
ros2 topic echo /scan
# Check velocity commands
ros2 topic echo /cmd_vel
# Verify simulation time
ros2 param get /wall_follower_controller use_sim_time  # Should be true
```
### Robot bumps into walls
- Increase `emergency_stop_distance` (e.g., 0.65)
- Increase `desired_distance` (e.g., 2.0)
- Decrease `forward_speed` (e.g., 0.20)
### Robot loses wall frequently
- Increase `wall_lost_distance` (e.g., 3.0)
- Decrease `kp` for less aggressive turning (e.g., 0.5)
### Oscillating/unstable following
- Increase `kd` for more damping (e.g., 1.5)
- Decrease `kp` (e.g., 0.5)
- Decrease `forward_speed` (e.g., 0.20)
---
## 📡 ROS2 Topics
### Subscribed
- `/scan` (sensor_msgs/LaserScan) - LiDAR data for wall detection
### Published
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands
- `/wall_follower/state` (std_msgs/String) - Current state (SEARCHING/FOLLOWING/AVOIDING)
---
## 🎓 Technical Details
### Key Features
✅ **YAML-configurable parameters** - No recompilation needed for tuning  
✅ **Parameter validation** - Runtime checks prevent invalid configurations  
✅ **Explicit state machine** - Clear behavior transitions for debugging  
✅ **Graceful shutdown** - Robot stops safely on Ctrl+C  
✅ **Multi-zone collision detection** - Front, sides, and wide-angle safety checks  
✅ **Adaptive speed control** - Slows down near obstacles  
✅ **State diagnostics** - Published state for monitoring  
### Control Loop
- **Frequency:** 20Hz (50ms cycle time)
- **Laser scan range:** 360° coverage, ~3.5m max range
- **Angular sampling:** Front (±30°), Sides (±90°), Wide (±70°)
### Dependencies
- `rclpy` - ROS2 Python client library
- `sensor_msgs` - LaserScan message type
- `geometry_msgs` - Twist (velocity) message type
- `std_msgs` - String message for state publishing
---
## 🔗 Resources
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [TurtleBot3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [PID Control Tutorial](https://en.wikipedia.org/wiki/PID_controller)
