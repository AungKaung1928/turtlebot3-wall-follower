#  TurtleBot3 Wall Following Robot
A ROS2 reactive wall-following controller for TurtleBot3: filtered PD control on the side-arc range, with a three-state behaviour machine and multi-zone collision avoidance.
---
##  Prerequisites
- [![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/humble/Installation.html) **ROS2 Humble**
- [![Python](https://img.shields.io/badge/Python-3.8+-yellow?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/downloads/) **Python 3.8+**
- [![Gazebo](https://img.shields.io/badge/Gazebo-Classic-orange?style=for-the-badge&logo=gazebo&logoColor=white)](https://classic.gazebosim.org/tutorials?tut=install_ubuntu) **Gazebo Classic**
- [![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange?style=for-the-badge&logo=ubuntu&logoColor=white)](https://ubuntu.com/) **Ubuntu 22.04**
---
##  Project Structure
```
turtlebot3-wall-follower/                     # clone into <ws>/src/
├── package.xml                               # ROS2 package metadata
├── setup.py                                  # Python package configuration
├── config/
│   └── wall_following_params.yaml            # Tunable parameters (PD, speeds, safety)
├── launch/
│   ├── wall_following.launch.py              # Controller launch file
│   └── wall_follower_gazebo.launch.py        # Full simulation launch
├── rviz/
│   └── wall_follower_config.rviz             # Visualization config
├── wall_following_project/
│   ├── __init__.py
│   └── wall_follower_controller.py           # Main control logic (PD + state machine)
└── README.md
```
---
##  Algorithm Overview
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
│  FOLLOWING  │  ← PD control maintains target distance
└──────┬──────┘
       │ Obstacle detected
       ▼
┌─────────────┐
│  AVOIDING   │  ← Emergency stop + escape maneuver
└──────┬──────┘
       │ Clear path
       └──────► Return to FOLLOWING
```
### PD Control System
```
Error = Desired_Distance - Current_Wall_Distance
Angular_Velocity = Kp × Error + Kd × (Error - Previous_Error) / dt
Where:
  Kp = 2.0  (Proportional gain - responsiveness)
  Kd = 0.6  (Derivative gain - smoothness)
  dt = 0.05 (50ms control loop)
```
There is no integral term, so this is a PD controller, not a PID.
The raw derivative is `(error - prev_error) / 0.05`, which amplifies scan noise
20x, so it is low-pass filtered (EMA, alpha = 0.3) before the gain is applied.
**How it works:**
- **Error > 0** (too far from wall) → Turn toward wall
- **Error < 0** (too close to wall) → Turn away from wall
- **Derivative term** prevents oscillation by damping aggressive corrections
### Architecture Components
| Component | Purpose |
|-----------|---------|
| **Laser Scan Processing** | Filters LiDAR data at multiple angles (-90° to +90°) |
| **State Machine** | Manages behavior transitions (SEARCHING/FOLLOWING/AVOIDING) |
| **PD Controller** | Calculates angular velocity to maintain wall distance |
| **Collision Detector** | Multi-zone safety checks (front, sides, wide angles) |
| **Velocity Publisher** | Sends movement commands to `/cmd_vel` |
---
##  Configuration Parameters
**File:** `config/wall_following_params.yaml`
| Parameter | Default | Description |
|-----------|---------|-------------|
| `desired_distance` | 0.5 m | Target distance from wall |
| `forward_speed` | 0.18 m/s | Normal forward speed |
| `search_speed` | 0.12 m/s | Speed while searching for wall |
| `max_angular_speed` | 1.0 rad/s | Maximum turn rate |
| `kp` | 2.0 | Proportional gain |
| `kd` | 0.6 | Derivative gain (applied to the filtered derivative) |
| `emergency_stop_distance` | 0.35 m | Immediate stop threshold |
| `slow_down_distance` | 0.80 m | Begin speed reduction |
| `wall_min_distance` | 0.28 m | Minimum wall clearance |
| `wall_lost_distance` | 1.2 m | Distance to consider wall lost |
| `side_clearance` | 0.25 m | Minimum side obstacle margin |

These must satisfy `side_clearance < wall_min < desired < wall_lost` and
`emergency_stop < slow_down`. Values outside the ranges validated in
`_load_parameters()` are rejected at startup and replaced by the range midpoint,
so an out-of-range value silently changes the behaviour rather than failing.
`turtlebot3_world` is a 5x5 m arena whose free corridor is about 1.2 m wide, which
is what sets the 0.5 m standoff.
---
##  Measured Performance

Recorded by an independent node (not the controller) at 10 Hz, headless
`gzserver`, `turtlebot3_world`, burger spawned at (-2.0, -0.5), 120 s per run,
target 0.5 m. "Followed-wall distance" is `min(right, left)`, because the
controller locks onto whichever side is nearer.

| Metric | Before fix | After fix |
|---|---|---|
| Path length in 120 s | 4.19 m | **10.29 m** |
| Longest stall (net move < 5 cm) | 36.8 s | **6.8 s** |
| Time FOLLOWING | 59.2 % | **76.6 %** |
| Time AVOIDING | 40.8 % | **13.1 %** |
| State changes / min | 58.0 | **13.5** |
| Tracking RMSE | 0.599 m | **0.179 m** |
| Std. dev. | 0.292 m | **0.172 m** |
| Within +/-0.15 m of target | 13.6 % | **75.0 %** |

The "before" run is scored against its own configured target (1.4 m), the
"after" run against 0.5 m; both use the identical scenario and recorder.

### What was actually wrong

`get_distance_at_angle()` computed `idx = (radians(angle) - angle_min) / angle_increment`
and then **clamped** the result into `[0, n-1]`. The TurtleBot3 LDS publishes
`angle_min = 0.0, angle_max = 2*pi`, so every negative bearing produced a negative
index that clamped to **0 - straight ahead**. With `wall_side = 'right'` (-90 deg),
the controller was regulating the *front* beam, not the wall. Indices are now
wrapped modulo a full turn, which is correct for both the `0..2*pi` and the
`-pi..+pi` conventions.

Three further defects were found by measurement, not by reading:
1. `avoid_collision()` re-picked its escape direction every 50 ms, so it dithered
   around the decision boundary instead of completing a turn. The direction is now
   latched on entry.
2. Avoidance could only rotate, never translate, so a robot wedged into a corner
   span in place indefinitely (93.6 s in one run). It now backs off if the turn
   has not cleared the obstacle, and only when the rear arc is clear.
3. Trip and clear used the same thresholds, so the state machine chattered on the
   boundary. Clearing now requires a 1.3x margin.

---
##  Installation
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
##  Usage
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
rviz2 -d $(ros2 pkg prefix wall_following_project)/share/wall_following_project/rviz/wall_follower_config.rviz
```
---
##  Parameter Tuning

> **Parameters are read once, in the node constructor.** There is no
> `add_on_set_parameters_callback`, so `ros2 param set` updates the parameter
> server but does **not** change the running controller. Edit
> `config/wall_following_params.yaml`, rebuild, and relaunch.
> The commands below are useful for *inspecting* the loaded values.

**Change wall distance (has no effect until relaunch):**
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
**Monitor robot state (SEARCHING/FOLLOWING/AVOIDING):**
```bash
ros2 topic echo /wall_follower/state
```

> **Note:** `ros2 param set` does not affect the running control loop - see above.
---
##  Tuning Guide
### For Faster Following
```yaml
forward_speed: 0.25        # Increase from 0.18
kp: 2.4                    # More aggressive correction
kd: 0.8                    # Dampen oscillation
```
Measured: kp 2.6 made tracking *worse* (RMSE 0.229 -> 0.270 m, angular
saturation 3.4% -> 8.1%). Raise kp only with a measurement to back it.
### For Tighter Walls (Narrow Corridors)
```yaml
desired_distance: 0.4      # Closer following (valid range 0.3 - 1.5)
emergency_stop_distance: 0.30
wall_min_distance: 0.25
side_clearance: 0.22
```
### For Better Stability (Smooth Following)
```yaml
kp: 1.5                    # Less aggressive
kd: 0.9                    # More damping
forward_speed: 0.12        # Slower = smoother
```
### For Obstacle-Dense Environments
```yaml
emergency_stop_distance: 0.45  # Earlier braking
slow_down_distance: 1.0        # More gradual slowdown
side_clearance: 0.30           # Wider safety margin
```
---
##  Troubleshooting
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
- Increase `emergency_stop_distance` (e.g., 0.45)
- Increase `desired_distance` (e.g., 0.7 - the validated range is 0.3 to 1.5)
- Decrease `forward_speed` (e.g., 0.12)
### Robot loses wall frequently
- Increase `wall_lost_distance` (e.g., 1.5 - the validated range is 0.8 to 3.0)
- Decrease `kp` for less aggressive turning (e.g., 1.5)
### Oscillating/unstable following
- Increase `kd` for more damping (e.g., 1.5)
- Decrease `kp` (e.g., 0.5)
- Decrease `forward_speed` (e.g., 0.20)
---
##  ROS2 Topics
### Subscribed
- `/scan` (sensor_msgs/LaserScan) - LiDAR data for wall detection
### Published
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands
- `/wall_follower/state` (std_msgs/String) - Current state (SEARCHING/FOLLOWING/AVOIDING)
---
##  Technical Details
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
##  Resources
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [TurtleBot3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [PID Control Tutorial](https://en.wikipedia.org/wiki/PID_controller)
