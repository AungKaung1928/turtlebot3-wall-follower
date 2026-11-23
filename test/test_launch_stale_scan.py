"""Launch test: when /scan stops (LiDAR unplugged, driver crash) the node must stop the robot,
not keep driving on the last scan. The installed node with the shipped params follows a wall on
synthetic scans, then the scans stop; within about 1 s every /cmd_vel must be zero.
Run: launch_test test/test_launch_stale_scan.py (after sourcing the workspace).
"""
import math
import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

PKG = 'wall_following_project'
EXE = 'wall_follower'
PARAMS = os.path.join(get_package_share_directory(PKG), 'config', 'wall_following_params.yaml')


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(package=PKG, executable=EXE, name='wall_follower_controller',
                                   output='screen', parameters=[PARAMS, {'use_sim_time': False}])
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {'node': node}


def wall_right_scan():
    scan = LaserScan()
    scan.angle_min, scan.angle_increment = 0.0, 2.0 * math.pi / 360
    scan.angle_max = scan.angle_increment * 359
    scan.range_min, scan.range_max = 0.12, 3.5
    scan.ranges = [0.5 if -120 <= (i if i <= 180 else i - 360) <= -60 else 3.0 for i in range(360)]
    return scan


class TestStaleScan(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_robot_stops_when_scans_stop(self):
        probe = rclpy.create_node('stale_probe')
        states, cmds = [], []
        probe.create_subscription(
            String, '/wall_follower/state', lambda m: states.append(m.data), 10)
        probe.create_subscription(Twist, '/cmd_vel', cmds.append, 10)
        pub = probe.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        scan = wall_right_scan()
        try:
            start = time.monotonic()  # follow until it drives forward
            while time.monotonic() - start < 20.0 and not (
                    states and states[-1] == 'FOLLOWING' and cmds and cmds[-1].linear.x > 0.05):
                pub.publish(scan)
                rclpy.spin_once(probe, timeout_sec=0.05)
            self.assertTrue(cmds and cmds[-1].linear.x > 0.05,
                            'never drove forward while following')
            cmds.clear()
            end = time.monotonic() + 2.0  # scans stop here
            while time.monotonic() < end:
                rclpy.spin_once(probe, timeout_sec=0.05)
        finally:
            probe.destroy_node()
        late = [c for c in cmds[len(cmds) // 2:]]  # the second second after the last scan
        self.assertTrue(late, 'no /cmd_vel after the scans stopped')
        tail = [(round(c.linear.x, 2), round(c.angular.z, 2)) for c in late[-3:]]
        self.assertTrue(all(c.linear.x == 0.0 and c.angular.z == 0.0 for c in late),
                        f'still moving 1-2 s after the last scan: {tail}')
