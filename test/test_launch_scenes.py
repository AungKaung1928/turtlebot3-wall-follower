"""Launch test: the installed node runs as its own process with the shipped parameter file.

The test process publishes synthetic TurtleBot3 LDS scans on /scan and follows
/wall_follower/state through one scene sequence:
open space -> wall on the right -> obstacle ahead -> obstacle gone.
After shutdown the node process must exit cleanly.
Run: launch_test test/test_launch_scenes.py (after sourcing the workspace).
"""
import math
import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

PARAMS = os.path.join(
    get_package_share_directory('wall_following_project'), 'config', 'wall_following_params.yaml')


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='wall_following_project', executable='wall_follower',
        name='wall_follower_controller', output='screen',
        # The shipped file sets use_sim_time for Gazebo; there is no /clock here.
        parameters=[PARAMS, {'use_sim_time': False}])
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {'node': node}


def make_scan(range_at_bearing):
    """360 beams, angle_min 0 (LDS convention); bearing in degrees, left positive."""
    scan = LaserScan()
    scan.header.frame_id = 'base_scan'
    scan.angle_min = 0.0
    scan.angle_increment = 2.0 * math.pi / 360
    scan.angle_max = scan.angle_increment * 359
    scan.range_min = 0.12
    scan.range_max = 3.5
    scan.ranges = [float(range_at_bearing(i if i <= 180 else i - 360)) for i in range(360)]
    return scan


OPEN = make_scan(lambda b: 3.0)
WALL_RIGHT = make_scan(lambda b: 0.5 if -120 <= b <= -60 else 3.0)
WALL_RIGHT_OBSTACLE_AHEAD = make_scan(
    lambda b: 0.2 if -30 <= b <= 30 else (0.5 if -120 <= b <= -60 else 3.0))


class TestScenes(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.probe = rclpy.create_node('launch_scene_probe')
        self.states = []
        self.probe.create_subscription(
            String, '/wall_follower/state', lambda m: self.states.append(m.data), 10)
        self.pub = self.probe.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)

    def tearDown(self):
        self.probe.destroy_node()

    def drive(self, scan, until_state, timeout=10.0):
        """Publish scan at ~20 Hz until the node reports until_state; return the time taken."""
        self.states.clear()
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            self.pub.publish(scan)
            rclpy.spin_once(self.probe, timeout_sec=0.05)
            if self.states and self.states[-1] == until_state:
                return time.monotonic() - start
        self.fail(f'no {until_state} within {timeout} s; last states {self.states[-5:]}')

    def test_scene_sequence(self):
        self.drive(OPEN, 'SEARCHING', timeout=20.0)  # first wait also covers node start-up
        self.drive(WALL_RIGHT, 'FOLLOWING')
        self.drive(WALL_RIGHT_OBSTACLE_AHEAD, 'AVOIDING')
        # Once the obstacle is gone the node resumes the wall it had, no new search.
        self.drive(WALL_RIGHT, 'FOLLOWING')
        self.assertNotIn('SEARCHING', self.states)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info, node):
        launch_testing.asserts.assertExitCodes(proc_info, process=node)
