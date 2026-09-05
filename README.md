#  TurtleBot3 Wall Following Robot
A ROS2 reactive wall-following controller for TurtleBot3: filtered PD control on the side-arc range, with a three-state behaviour machine and multi-zone collision avoidance.
---
##  Prerequisites
- [![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/humble/Installation.html) **ROS2 Humble**
- [![Python](https://img.shields.io/badge/Python-3.8+-yellow?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/downloads/) **Python 3.8+**
- [![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange?style=for-the-badge&logo=gazebo&logoColor=white)](https://gazebosim.org/docs/harmonic/install_ubuntu/) **Gazebo Harmonic** (gz-sim 8) via `ros-humble-ros-gzharmonic`
- [![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange?style=for-the-badge&logo=ubuntu&logoColor=white)](https://ubuntu.com/) **Ubuntu 22.04**
---
##  Project Structure
```
turtlebot3-wall-follower/                     # clone into <ws>/src/
├── package.xml                               # ROS2 package metadata
├── setup.py                                  # Python package configuration
├── config/
│   ├── wall_following_params.yaml            # Tunable parameters (PD, speeds, safety)
│   └── gz_bridge.yaml                        # ros_gz_bridge topics (/scan /cmd_vel /odom /tf /clock)
├── description/
│   └── turtlebot3_burger.urdf.xacro          # Burger + gz-sim DiffDrive / gpu_lidar plugins
├── worlds/
│   ├── wall_follow_world.sdf                 # 6x6 m room with partition + block (default)
│   └── turtlebot3_world.sdf                  # Original hexagon + 9 pillars benchmark arena
├── models/turtlebot3_world/                  # Mesh model used by turtlebot3_world.sdf
├── launch/
│   ├── wall_following.launch.py              # Controller only
│   └── wall_follower_gazebo.launch.py        # Gazebo Harmonic + bridge + robot + controller
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
alpha      = atan2(b - a·cos θ, a·sin θ)       # heading angle to the wall (two beams: side b, swung a)
d_perp     = b · cos alpha                      # true perpendicular distance
d_pred     = d_perp - lookahead · sin alpha     # distance after lookahead metres on this heading
Error      = d_pred - Desired_Distance          # > 0: too far
Angular_Velocity = ±(Kp × Error + Kd × dError/dt)   # sign steers toward the followed wall
Where:
  Kp = 2.0, Kd = 0.6, dt = 0.05, lookahead = 0.3 m, θ = 40°
```
Steering on the predicted distance instead of the raw side range is what keeps
the lock: a single side beam inflates as soon as the robot yaws toward the wall,
so the old check declared the wall lost every time the P term steered inward.
There is no integral term, so this is a PD controller, not a PID.
The raw derivative is `(error - prev_error) / 0.05`, which amplifies scan noise
20x, so it is low-pass filtered (EMA, alpha = 0.3) before the gain is applied.
**How it works:**
- **Error > 0** (too far from wall) → Turn toward wall
- **Error < 0** (too close, or converging too fast) → Turn away from wall
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
| `lookahead_distance` | 0.3 m | Horizon for the predicted wall distance |
| `beam_spread_deg` | 40 | Angle between side beam and forward wall beam |
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

Recorded by an independent node (not the controller) at 10 Hz, headless Gazebo
Harmonic (`headless:=true`), target 0.5 m. "Followed-wall distance" is
`min(right, left)` over **all** samples, including search and avoidance, because
the controller locks onto whichever side is nearer. Contact = any scan return
below 0.16 m (the LDS is 0.12 m from the shell).

| Metric | Room, before fixes (120 s) | Room, after (170 s) | turtlebot3_world, after (120 s) |
|---|---|---|---|
| Path length | 10.40 m | **23.92 m** (full lap) | **18.00 m** |
| Time FOLLOWING | 69.9 % | **92.7 %** | **100 %** |
| Time AVOIDING | 15.7 % | 7.3 % (corners) | 0 % |
| Wall-lost events | 24 | **0** | **0** |
| Collision-avoid episodes | 8 | 6 (= the 6 corners) | **0** |
| State changes / min | 25.5 | 4.2 | **0.0** |
| Tracking RMSE | 0.455 m | **0.139 m** | **0.066 m** |
| Within +/-0.15 m of target | 32.5 % | **95.4 %** | **98.3 %** |
| Contact samples | 0 | 0 | 0 |

Room = `worlds/wall_follow_world.sdf` (default, 6 x 6 m, partition + block), spawn
(-2, -2), right-hand lap: south, east, north, west wall, partition north face,
around its tip, partition south face, west wall, back to the south wall.
turtlebot3_world spawn (-2.0, -0.5). Before the Harmonic port, the best Classic
result in turtlebot3_world was RMSE 0.179 m / 75 % within 0.15 m.

### What was actually wrong

**Single-beam wall distance (fixed 2026-09-05).** `follow_wall()` measured the
wall as the minimum over a +/-15 deg side arc and compared that to
`wall_lost_distance`. As soon as the P term steered toward the wall, the arc
rotated off the wall normal and the reading inflated by 1/cos(yaw), tripping
"wall lost" every 1-2 s and dropping the robot back into SEARCHING; it then
drove at the wall until collision avoidance turned it away. The controller now
takes two beams (side and 40 deg forward), solves the wall angle, and steers on
the distance predicted `lookahead_distance` ahead. The lost check uses a
+/-45 deg arc as well, so yawing no longer counts as losing the wall. When both
beams look past the end of a wall while the rear arc still has it (an outside
corner), the robot arcs around the corner at the standoff radius instead of
falling into the open-space search sweep, which used to wander into the tip.

**Parameter file never loaded (fixed 2026-09-05).** The Classic launch included
`gazebo_ros/gzserver.launch.py`, which also declares a `params_file` argument;
launch arguments are global, so ours resolved to `''` and the node ran on the
defaults compiled into the code. The Harmonic launch passes the file explicitly.

**Scan indexing (fixed earlier).** `get_distance_at_angle()` clamped negative
bearings to index 0, so with `angle_min = 0` the right-wall controller was
regulating the *front* beam. Indices are now wrapped modulo a full turn, which
is correct for both the `0..2*pi` (Classic LDS) and `-pi..+pi` (gpu_lidar)
conventions.

Three further defects were found by measurement, not by reading:
1. `avoid_collision()` re-picked its escape direction every 50 ms, so it dithered
   around the decision boundary instead of completing a turn. The direction is now
   latched on entry.
2. Avoidance could only rotate, never translate, so a robot wedged into a corner
   span in place indefinitely (93.6 s in one run). It now backs off if the turn
   has not cleared the obstacle, and only when the rear arc is clear.
3. Trip and clear used the same thresholds, so the state machine chattered on the
   boundary. Clearing now requires a 1.3x margin.

### Known limitation
Inside corners are handled by the AVOIDING state (front < 0.35 m, stop, turn
away, resume), a hard stop-and-turn rather than a planned arc; it costs about
2 s per corner. A front-arc steering term in `follow_wall()` is the planned
improvement. The controller is a single 480-line Python node; a C++ port is the
other open item.

---
##  Installation
```bash
# 1. Gazebo Harmonic + ros_gz for Humble (packages.osrfoundation.org, see gazebosim.org/docs/harmonic/ros_installation)
sudo apt install gz-harmonic ros-humble-ros-gzharmonic
# 2. Robot description meshes + launch helpers
sudo apt install ros-humble-turtlebot3-description ros-humble-xacro ros-humble-robot-state-publisher
# 3. Clone into a workspace and build
mkdir -p ~/wall_follower_ws/src && cd ~/wall_follower_ws/src
git clone https://github.com/AungKaung1928/turtlebot3-wall-follower.git
cd .. && colcon build --symlink-install && source install/setup.bash
```
No `turtlebot3_gazebo` and no Gazebo Classic: the robot's Gazebo plugins live in
`description/turtlebot3_burger.urdf.xacro` and the worlds are plain SDF 1.8.
---
##  Usage
### Quick Start (Single Command)
```bash
ros2 launch wall_following_project wall_follower_gazebo.launch.py
```
Launch arguments: `world` (SDF path), `headless:=true` (server only, for WSL/CI),
`x_pose y_pose yaw` (spawn), `params_file`. Always clear stale servers first:
`pkill -9 -f "gz sim"`; two gz servers on one bus corrupt `/clock` and `/odom`.
### Controller only (sim already running)
```bash
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
- `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs` - controller
- `ros_gz_sim`, `ros_gz_bridge` (`ros-humble-ros-gzharmonic`) - Gazebo Harmonic launch + bridge
- `xacro`, `robot_state_publisher`, `turtlebot3_description` - robot model and meshes
---
##  Resources
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [TurtleBot3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [PID Control Tutorial](https://en.wikipedia.org/wiki/PID_controller)
