#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class WallFollowerController(Node):
    def __init__(self):
        super().__init__('wall_follower_controller')
        
        # Parameters
        self.target_dist = 0.6
        self.max_speed = 0.25
        self.max_turn = 0.8
        self.safe_dist = 0.5
        self.wall_range = 1.5
        
        # PID
        self.kp, self.kd = 0.5, 0.1
        self.prev_error = 0.0
        
        # State
        self.wall_right = True
        self.laser = None
        
        # ROS
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.timer = self.create_timer(0.1, self.control)

    def scan_cb(self, msg):
        self.laser = msg

    def get_range(self, angle, tol=10):
        if not self.laser: return float('inf')
        
        idx = int((math.radians(angle) - self.laser.angle_min) / self.laser.angle_increment)
        tol_idx = int(math.radians(tol) / self.laser.angle_increment)
        
        ranges = self.laser.ranges[max(0, idx-tol_idx):min(len(self.laser.ranges), idx+tol_idx+1)]
        valid = [r for r in ranges if not (math.isnan(r) or math.isinf(r)) and r > 0.1]
        return min(valid) if valid else float('inf')

    def detect_wall(self):
        # Check right wall
        right_dists = [self.get_range(a) for a in [-90, -45, -135]]
        right_valid = [d for d in right_dists if d < self.wall_range]
        if len(right_valid) >= 2 and max(right_valid) - min(right_valid) < 0.3:
            self.wall_right = True
            return True, sum(right_valid) / len(right_valid)
        
        # Check left wall
        left_dists = [self.get_range(a) for a in [90, 45, 135]]
        left_valid = [d for d in left_dists if d < self.wall_range]
        if len(left_valid) >= 2 and max(left_valid) - min(left_valid) < 0.3:
            self.wall_right = False
            return True, sum(left_valid) / len(left_valid)
        
        return False, float('inf')

    def control(self):
        if not self.laser: return
        
        cmd = Twist()
        front = self.get_range(0, 20)
        
        # Emergency stop
        if front < self.safe_dist:
            cmd.angular.z = -0.5 if self.get_range(-90) > self.get_range(90) else 0.5
            self.prev_error = 0.0
        else:
            wall_found, wall_dist = self.detect_wall()
            
            if wall_found:
                # PID control
                error = self.target_dist - wall_dist
                angular = self.kp * error + self.kd * (error - self.prev_error) / 0.1
                if not self.wall_right: angular = -angular
                
                cmd.linear.x = self.max_speed * (1.0 - 0.5 * abs(angular) / self.max_turn)
                cmd.linear.x = max(cmd.linear.x, 0.1)
                cmd.angular.z = max(min(angular, self.max_turn), -self.max_turn)
                
                self.prev_error = error
            else:
                # Search for wall
                cmd.linear.x = 0.15
                cmd.angular.z = 0.3
                self.prev_error = 0.0
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = WallFollowerController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.cmd_pub.publish(Twist())
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()