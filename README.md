# TurtleBot3 Wall Following Robot

A ROS2-based wall following robot implementation for TurtleBot3 with enhanced safety features and collision avoidance.

## Prerequisites

Before running this project, ensure you have the following installed:

- **ROS2 Humble**: [https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)
- **Python 3.8+**: [https://www.python.org/downloads/](https://www.python.org/downloads/)
- **Gazebo Classic**: [https://classic.gazebosim.org/tutorials?tut=install_ubuntu](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)
- **RViz2**: [https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html)

## Project Structure

```
turtlebot3_wall_follower_ws/
├── src/
│   └── wall_following_project/
│       ├── package.xml                    # Package dependencies and metadata
│       ├── setup.py                       # Package setup and entry points
│       ├── config/
│       │   ├── nav2_params.yaml          # Basic navigation parameters
│       │   └── wall_following_params.yaml # Wall following parameters
│       ├── launch/
│       │   ├── wall_following.launch.py   # Main wall follower launch file
│       │   └── wall_follower_gazebo.launch.py # Gazebo simulation launch
│       ├── rviz/
│       │   └── wall_follower_config.rviz  # RViz visualization configuration
│       └── wall_following_project/
│           ├── __init__.py
│           ├── wall_follower_controller.py # Main control logic
│           ├── wall_detector.py           # Wall detection algorithms  
│           └── pid_controller.py          # PID control system
└── README.md
```

## Features

- **Enhanced Safety System**: Multiple collision detection zones with configurable safety margins
- **Adaptive Wall Following**: PID-controlled wall following with dynamic speed adjustment
- **Intelligent Search**: Wall detection with stuck prevention and recovery behaviors
- **Collision Avoidance**: Emergency stop and escape maneuvers when obstacles detected
- **Real-time Visualization**: Pre-configured RViz setup for monitoring robot behavior

## Installation

1. **Install TurtleBot3 packages:**
   ```bash
   sudo apt update
   sudo apt install ros-humble-turtlebot3*
   ```

2. **Set TurtleBot3 model:**
   ```bash
   echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
   source ~/.bashrc
   ```

3. **Clone and build the workspace:**
   ```bash
   cd ~/
   mkdir -p turtlebot3_wall_follower_ws/src
   cd turtlebot3_wall_follower_ws/src
   # Copy the wall_following_project folder here
   cd ..
   colcon build
   source install/setup.bash
   ```

## Usage

### Quick Start (Gazebo Simulation)

Launch the complete simulation environment with wall follower:

```bash
source ~/turtlebot3_wall_follower_ws/install/setup.bash
ros2 launch wall_following_project wall_follower_gazebo.launch.py
```

This command will:
- Start Gazebo with TurtleBot3 world
- Spawn the TurtleBot3 robot
- Launch the wall following controller
- Begin autonomous wall following behavior

### Manual Launch (Step by Step)

1. **Start Gazebo simulation:**
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. **Launch wall follower (in new terminal):**
   ```bash
   source ~/turtlebot3_wall_follower_ws/install/setup.bash
   ros2 launch wall_following_project wall_following.launch.py
   ```

3. **Start RViz for visualization (optional):**
   ```bash
   rviz2 -d ~/turtlebot3_wall_follower_ws/src/wall_following_project/rviz/wall_follower_config.rviz
   ```

## Configuration

### Safety Parameters (wall_follower_controller.py)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `desired_distance` | 0.6m | Target distance from wall |
| `emergency_stop` | 0.55m | Emergency brake distance |
| `slow_down_dist` | 0.8m | Distance to start slowing down |
| `side_clearance` | 0.4m | Minimum side obstacle clearance |
| `forward_speed` | 0.20 m/s | Normal forward speed |
| `max_angular_speed` | 0.6 rad/s | Maximum turning speed |

### Control Parameters

- **PID Gains**: Kp=1.8, Kd=0.7 (optimized for responsive control)
- **Wall Detection Range**: 1.5m maximum wall detection distance
- **Search Behavior**: Alternating search pattern with stuck detection

## Behavior Modes

1. **Wall Following**: Maintains constant distance from detected wall using PID control
2. **Wall Search**: Rotates to locate nearby walls when none detected
3. **Collision Avoidance**: Emergency stop and escape maneuvers
4. **Recovery**: Stuck detection with aggressive recovery maneuvers

## Troubleshooting

### Robot doesn't move
- Check if simulation time is properly set: `use_sim_time: true`
- Verify laser scan topic: `ros2 topic echo /scan`
- Check velocity commands: `ros2 topic echo /cmd_vel`

### Robot crashes into walls
- Increase safety distances in controller parameters
- Check laser scan data quality and range
- Verify emergency stop distance is appropriate for robot speed

### Wall following not smooth
- Adjust PID parameters (Kp, Kd values)
- Modify desired distance from wall
- Check laser scan filtering and averaging

### Gazebo performance issues
- Reduce Gazebo physics update rate
- Close unnecessary applications
- Use headless mode: `HEADLESS=1 ros2 launch ...`

## ROS2 Topics

### Subscribed Topics
- `/scan` (sensor_msgs/LaserScan): Laser scan data for wall detection

### Published Topics
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands for robot movement

## Development

### Adding New Behaviors

To extend the wall follower with new behaviors:

1. **Modify wall_follower_controller.py**: Add new state variables and logic
2. **Update wall_detector.py**: Enhance wall detection algorithms
3. **Tune pid_controller.py**: Adjust control parameters for new behaviors
4. **Test in simulation**: Use Gazebo for safe testing before real robot deployment

### Parameter Tuning

Key parameters for tuning robot behavior:
- Safety distances for different environments
- PID gains for smoother or more responsive control
- Speed parameters for different robot capabilities
- Detection thresholds for various wall materials

## Hardware Deployment

To run on real TurtleBot3:

1. **Set up TurtleBot3**: Follow official TurtleBot3 setup guide
2. **Update parameters**: Adjust safety distances for real-world conditions  
3. **Test gradually**: Start with very conservative parameters
4. **Monitor closely**: Always be ready to emergency stop

## License

This project is licensed under the terms specified in the package.xml file.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Test thoroughly in simulation
4. Submit a pull request with detailed description

## Support

For issues and questions:
- Check ROS2 Humble documentation
- Review TurtleBot3 official tutorials
- Examine Gazebo Classic documentation
- Test in simulation before hardware deployment
