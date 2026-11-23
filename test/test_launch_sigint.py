"""
Launch test: Ctrl+C stops the robot.

The installed node follows a wall on synthetic scans; the test sends SIGINT to the node
process while scans keep coming. The last /cmd_vel it sends must be zero (main() keeps the
context alive through SIGINT so shutdown()'s stop is delivered).
Run: launch_test test/test_launch_sigint.py (after sourcing the workspace).
"""
import math
import os
import signal
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
    # Keeps the launch (and the shared rclpy context) alive after the node exits on SIGINT.
    keepalive = launch.actions.ExecuteProcess(cmd=['sleep', '60'])
    return (launch.LaunchDescription([node, keepalive, launch_testing.actions.ReadyToTest()]),
            {'node': node})


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

    def test_sigint_sends_a_final_stop(self, proc_info, node):
        probe = rclpy.create_node('sigint_probe')
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
            os.kill(proc_info[node].pid, signal.SIGINT)
            end = time.monotonic() + 3.0  # scans keep coming, so only shutdown() can send the stop
            while time.monotonic() < end:
                pub.publish(scan)
                rclpy.spin_once(probe, timeout_sec=0.05)
        finally:
            probe.destroy_node()
        last = cmds[-1]
        self.assertEqual((last.linear.x, last.angular.z), (0.0, 0.0),
                         'last command after SIGINT is not a stop: '
                         f'{(last.linear.x, last.angular.z)}')
