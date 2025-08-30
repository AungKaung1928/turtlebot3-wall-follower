#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class WallFollowerController(Node):
    def __init__(self):
        super().__init__('wall_follower_controller')
        
        # Parameters - EXACTLY SAME VALUES
        self.desired_distance = 0.5
        self.forward_speed = 0.22
        self.search_speed = 0.18
        self.max_angular_speed = 0.6
        self.kp, self.kd = 1.5, 0.6
        
        # Safety distances - SAME
        self.emergency_stop = 0.35
        self.slow_down_dist = 0.5
        self.wall_min = 0.35
        self.wall_lost = 1.2
        
        # State
        self.following_wall = False
        self.wall_side = 'right'
        self.laser_data = None
        self.prev_error = 0.0
        self.search_dir = 1
        self.counter = 0
        
        # ROS
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('Wall Follower - Safe Speed Mode')
    
    def laser_callback(self, msg):
        self.laser_data = msg
    
    def get_dist(self, angle, avg=False):
        """Get distance at angle (or average if avg=True)"""
        if not self.laser_data:
            return float('inf')
        
        angle_rad = math.radians(angle)
        idx = int((angle_rad - self.laser_data.angle_min) / self.laser_data.angle_increment)
        
        # Get multiple readings for averaging
        indices = range(max(0, idx-2), min(len(self.laser_data.ranges), idx+3))
        dists = []
        
        for i in indices:
            d = self.laser_data.ranges[i]
            if not (math.isnan(d) or math.isinf(d)) and 0.05 < d < 3.5:
                dists.append(d)
        
        if not dists:
            return float('inf')
        
        return sum(dists)/len(dists) if avg else min(dists)
    
    def get_min_in_range(self, start, end, step=3):
        """Get minimum distance in angle range"""
        dists = [self.get_dist(a) for a in range(start, end+1, step)]
        return min(dists) if dists else float('inf')
    
    def check_collision(self):
        """Check for collision risk"""
        # Check multiple front zones
        front = min(
            self.get_min_in_range(-30, 30, 2),
            self.get_min_in_range(-45, -25, 3),
            self.get_min_in_range(25, 45, 3)
        )
        
        # Check sides
        right = self.get_min_in_range(-90, -70, 3)
        left = self.get_min_in_range(70, 90, 3)
        
        return (front < self.emergency_stop or right < 0.25 or left < 0.25), front
    
    def find_wall(self):
        """Find nearest wall"""
        right = self.get_dist(-90, avg=True)
        left = self.get_dist(90, avg=True)
        
        if right < self.wall_lost or left < self.wall_lost:
            self.wall_side = 'right' if right < left else 'left'
            self.following_wall = True
            self.counter = 0
            self.get_logger().info(f'Following {self.wall_side.upper()} wall')
            return True
        return False
    
    def wall_follow(self):
        """Wall following control"""
        cmd = Twist()
        
        # Get wall distance
        angle = -90 if self.wall_side == 'right' else 90
        wall_dist = self.get_dist(angle, avg=True)
        
        # Lost wall?
        if wall_dist > self.wall_lost:
            self.following_wall = False
            return self.search()
        
        # Too close to wall?
        if wall_dist < self.wall_min:
            cmd.linear.x = 0.1
            cmd.angular.z = 0.4 if self.wall_side == 'right' else -0.4
            return cmd
        
        # PID control
        error = self.desired_distance - wall_dist
        angular = self.kp * error + self.kd * (error - self.prev_error) / 0.05
        
        if self.wall_side == 'left':
            angular = -angular
        
        cmd.angular.z = max(min(angular, self.max_angular_speed), -self.max_angular_speed)
        
        # Speed based on front clearance
        front = self.get_min_in_range(-35, 35, 3)
        if front < self.slow_down_dist:
            factor = max(0.3, (front - self.emergency_stop) / (self.slow_down_dist - self.emergency_stop))
            cmd.linear.x = self.forward_speed * factor
        else:
            cmd.linear.x = self.forward_speed
        
        # Slow down in turns
        if abs(cmd.angular.z) > 0.3:
            cmd.linear.x *= (1.0 - abs(cmd.angular.z) / self.max_angular_speed * 0.5)
        
        cmd.linear.x = max(cmd.linear.x, 0.05)
        self.prev_error = error
        
        return cmd
    
    def search(self):
        """Search for walls"""
        cmd = Twist()
        
        front = self.get_min_in_range(-30, 30, 3)
        cmd.linear.x = self.search_speed if front > self.slow_down_dist else self.search_speed * 0.5
        
        # Search pattern
        self.counter += 1
        if self.counter > 40:
            self.search_dir *= -1
            self.counter = 0
        
        cmd.angular.z = 0.3 * self.search_dir
        return cmd
    
    def control_loop(self):
        """Main control"""
        if not self.laser_data:
            return
        
        # Emergency check
        collision, min_dist = self.check_collision()
        if collision:
            cmd = Twist()
            cmd.linear.x = 0.0
            left = self.get_min_in_range(30, 90, 5)
            right = self.get_min_in_range(-90, -30, 5)
            cmd.angular.z = 0.5 if left > right else -0.5
            self.cmd_pub.publish(cmd)
            self.get_logger().warn(f'Collision avoidance! {min_dist:.2f}m')
            return
        
        # Main logic
        if self.following_wall:
            cmd = self.wall_follow()
        elif self.find_wall():
            cmd = self.wall_follow()
        else:
            cmd = self.search()
        
        # Safety limits
        cmd.linear.x = max(0.0, min(cmd.linear.x, self.forward_speed))
        cmd.angular.z = max(-self.max_angular_speed, min(cmd.angular.z, self.max_angular_speed))
        
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