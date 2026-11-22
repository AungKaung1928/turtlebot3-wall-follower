# TurtleBot3 Wall Follower

A ROS 2 Humble wall-following controller for TurtleBot3 in Gazebo Harmonic: filtered PD control on the predicted wall distance, a three-state behaviour machine (searching, following, avoiding) and multi-zone collision checks. Holds 0.5 m to the wall with 0.066 m RMSE in turtlebot3_world.

## Measured results

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

## Quick start

Requires Ubuntu 22.04, ROS 2 Humble and Gazebo Harmonic (`gz-harmonic`, `ros-humble-ros-gzharmonic` from packages.osrfoundation.org).

```bash
sudo apt install ros-humble-turtlebot3-description ros-humble-xacro ros-humble-robot-state-publisher
mkdir -p ~/wall_follower_ws/src && cd ~/wall_follower_ws/src
git clone <this repo URL>
cd .. && colcon build --symlink-install && source install/setup.bash
ros2 launch wall_following_project wall_follower_gazebo.launch.py   # Gazebo opens, the burger finds the nearest wall and laps the room
```

Launch arguments: `world` (SDF path), `headless:=true` (server only, for WSL/CI), `x_pose y_pose yaw` (spawn), `params_file`. Clear stale servers first (`pkill -9 -f "gz sim"`): two gz servers on one bus corrupt `/clock` and `/odom`. Controller only, sim already running: `ros2 launch wall_following_project wall_following.launch.py`. RViz: `rviz2 -d $(ros2 pkg prefix wall_following_project)/share/wall_following_project/rviz/wall_follower_config.rviz`.

Without Gazebo: `python3 -m pytest test/test_node_topics.py` (after `source install/setup.bash`) runs the node against synthetic scans over real topics: open space keeps searching, a wall on the right starts following, an obstacle ahead triggers avoidance with no forward motion. `launch_test test/test_launch_scenes.py` starts the installed node as its own process with the shipped parameter file and checks it resumes following after an obstacle clears, then exits cleanly. CI runs both in `ros:humble` on every push.

## Limits

Inside corners are handled by the AVOIDING state (front < 0.35 m, stop, turn
away, resume), a hard stop-and-turn rather than a planned arc; it costs about
2 s per corner. A front-arc steering term in `follow_wall()` is the planned
improvement. The controller is a single 480-line Python node; the C++ port is
[turtlebot3-wall-follower-cpp](https://github.com/AungKaung1928/turtlebot3-wall-follower-cpp).

More: algorithm and equations, all parameters, the bugs found by measurement, tuning and troubleshooting in [docs/DETAILS.md](docs/DETAILS.md).

## License

Apache-2.0, see LICENSE.
