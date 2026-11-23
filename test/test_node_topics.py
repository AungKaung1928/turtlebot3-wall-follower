"""
Node-level integration test: the real controller node, real ROS 2 topics.

A probe node publishes a synthetic TurtleBot3 LDS scan on /scan (best-effort, as the
real driver does) and records /cmd_vel and /wall_follower/state.
"""
import math
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from wall_following_project.wall_follower_controller import WallFollowerController


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_scan(range_at_bearing):
    """360 beams, angle_min 0 (LDS convention); bearing in degrees, left positive."""
    scan = LaserScan()
    scan.angle_min = 0.0
    scan.angle_increment = 2.0 * math.pi / 360
    scan.angle_max = scan.angle_increment * 359
    scan.range_min = 0.12
    scan.range_max = 3.5
    scan.ranges = [float(range_at_bearing(i if i <= 180 else i - 360)) for i in range(360)]
    return scan


def run_scene(range_at_bearing, seconds=1.5):
    node = WallFollowerController()
    probe = rclpy.create_node('scene_probe')
    states, cmds = [], []
    probe.create_subscription(String, '/wall_follower/state', lambda m: states.append(m.data), 10)
    probe.create_subscription(Twist, '/cmd_vel', cmds.append, 10)
    pub = probe.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(probe)
    scan = make_scan(range_at_bearing)
    end = time.monotonic() + seconds
    try:
        while time.monotonic() < end:
            pub.publish(scan)
            executor.spin_once(timeout_sec=0.02)
    finally:
        executor.shutdown()
        node.destroy_node()
        probe.destroy_node()
    return states, cmds


def test_open_space_keeps_searching():
    states, cmds = run_scene(lambda b: 3.0)
    assert cmds, 'no /cmd_vel received: /scan never reached the node'
    assert set(states) == {'SEARCHING'}


def test_wall_on_right_starts_following():
    states, cmds = run_scene(lambda b: 0.6 if -120 <= b <= -60 else 3.0)
    assert 'FOLLOWING' in states
    assert 'AVOIDING' not in states


def test_obstacle_ahead_avoids_without_driving_forward():
    states, cmds = run_scene(lambda b: 0.3 if -30 <= b <= 30 else 3.0)
    assert states and states[-1] == 'AVOIDING'
    assert all(c.linear.x <= 0.0 for c in cmds[-5:])
