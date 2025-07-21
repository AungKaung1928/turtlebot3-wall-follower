#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import numpy as np

class WallFollowerController(Node):
    def __init__(self):
        super().__init__('wall_follower_controller')
        
        # Core parameters - tuned for stable wall following
        self.target_distance = 0.6       # Target distance from wall (meters)
        self.max_speed = 0.25            # Maximum forward speed
        self.min_speed = 0.1             # Minimum forward speed
        self.max_turn_speed = 0.8        # Maximum angular speed
        self.safe_front_distance = 0.5   # Stop distance for front obstacles
        self.wall_detection_range = 1.5  # Maximum range to detect wall
        
        # PID parameters - simplified and stable
        self.kp = 0.5   # Proportional gain
        self.ki = 0.0   # Integral gain (disabled for stability)
        self.kd = 0.1   # Derivative gain
        
        # Control variables
        self.previous_error = 0.0
        self.integral = 0.0
        
        # Robot state
        self.following_wall = False
        self.wall_on_right = True  # Assume right wall following
        
        # ROS2 setup
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.laser_data = None
        self.get_logger().info('Wall Follower Started - Looking for wall to follow...')

    def scan_callback(self, msg):
        """Store the latest laser scan data"""
        self.laser_data = msg

    def get_range_at_angle(self, angle_degrees, tolerance=10):
        """Get the minimum range reading at a specific angle with tolerance"""
        if self.laser_data is None:
            return float('inf')
        
        # Convert angle to index
        angle_rad = math.radians(angle_degrees)
        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment
        
        center_index = int((angle_rad - angle_min) / angle_increment)
        tolerance_indices = int(math.radians(tolerance) / angle_increment)
        
        # Get range of indices to check
        start_idx = max(0, center_index - tolerance_indices)
        end_idx = min(len(self.laser_data.ranges), center_index + tolerance_indices + 1)
        
        # Find minimum valid range in the sector
        min_range = float('inf')
        valid_readings = 0
        
        for i in range(start_idx, end_idx):
            if i < len(self.laser_data.ranges):
                range_val = self.laser_data.ranges[i]
                if not math.isnan(range_val) and not math.isinf(range_val) and range_val > 0.1:
                    min_range = min(min_range, range_val)
                    valid_readings += 1
        
        return min_range if valid_readings > 0 else float('inf')

    def detect_wall(self):
        """Detect if there's a wall suitable for following"""
        # Check right side for wall
        right_distance = self.get_range_at_angle(-90)  # Right side
        front_right = self.get_range_at_angle(-45)     # Front-right
        back_right = self.get_range_at_angle(-135)     # Back-right
        
        # Check left side for wall
        left_distance = self.get_range_at_angle(90)    # Left side
        front_left = self.get_range_at_angle(45)       # Front-left
        back_left = self.get_range_at_angle(135)       # Back-left
        
        # Determine if there's a consistent wall on right side
        right_wall_detected = (
            right_distance < self.wall_detection_range and
            front_right < self.wall_detection_range * 1.2 and
            back_right < self.wall_detection_range * 1.2 and
            abs(right_distance - front_right) < 0.3  # Wall should be relatively straight
        )
        
        # Determine if there's a consistent wall on left side
        left_wall_detected = (
            left_distance < self.wall_detection_range and
            front_left < self.wall_detection_range * 1.2 and
            back_left < self.wall_detection_range * 1.2 and
            abs(left_distance - front_left) < 0.3
        )
        
        # Prefer right wall, but use left if right is not available
        if right_wall_detected:
            self.wall_on_right = True
            return True, right_distance
        elif left_wall_detected:
            self.wall_on_right = False
            return True, left_distance
        else:
            return False, float('inf')

    def calculate_wall_following_command(self, wall_distance):
        """Calculate command for wall following using PID control"""
        cmd = Twist()
        
        # Calculate error (positive = too far from wall, negative = too close)
        error = self.target_distance - wall_distance
        
        # PID calculations
        self.integral += error * 0.1  # dt = 0.1 seconds
        derivative = (error - self.previous_error) / 0.1
        
        # Limit integral to prevent windup
        self.integral = max(min(self.integral, 1.0), -1.0)
        
        # Calculate angular velocity
        angular_velocity = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        # Adjust sign based on which side the wall is on
        if not self.wall_on_right:
            angular_velocity = -angular_velocity
        
        # Limit angular velocity
        angular_velocity = max(min(angular_velocity, self.max_turn_speed), -self.max_turn_speed)
        
        # Set forward speed - reduce when turning sharply
        speed_reduction = abs(angular_velocity) / self.max_turn_speed
        forward_speed = self.max_speed * (1.0 - 0.5 * speed_reduction)
        forward_speed = max(forward_speed, self.min_speed)
        
        cmd.linear.x = forward_speed
        cmd.angular.z = angular_velocity
        
        # Store error for next iteration
        self.previous_error = error
        
        return cmd

    def control_loop(self):
        """Main control loop"""
        if self.laser_data is None:
            return
        
        cmd = Twist()
        
        # Check front for obstacles - safety first!
        front_distance = self.get_range_at_angle(0, 20)  # Front with wide tolerance
        
        # Emergency stop if obstacle too close in front
        if front_distance < self.safe_front_distance:
            # Stop and turn away from the closest side
            cmd.linear.x = 0.0
            
            # Check which side is clearer and turn that way
            right_distance = self.get_range_at_angle(-90)
            left_distance = self.get_range_at_angle(90)
            
            if right_distance > left_distance:
                cmd.angular.z = -0.5  # Turn right
                self.get_logger().info(f'Obstacle ahead! Turning right. Front: {front_distance:.2f}m')
            else:
                cmd.angular.z = 0.5   # Turn left  
                self.get_logger().info(f'Obstacle ahead! Turning left. Front: {front_distance:.2f}m')
            
            # Reset PID to avoid accumulated errors
            self.integral = 0.0
            self.previous_error = 0.0
            
        else:
            # Try to detect wall
            wall_detected, wall_distance = self.detect_wall()
            
            if wall_detected:
                self.following_wall = True
                cmd = self.calculate_wall_following_command(wall_distance)
                
                side = "right" if self.wall_on_right else "left"
                self.get_logger().info(
                    f'Following {side} wall: {wall_distance:.2f}m, '
                    f'Speed: {cmd.linear.x:.2f}, Turn: {cmd.angular.z:.2f}'
                )
            else:
                # No wall detected - search for wall
                self.following_wall = False
                cmd.linear.x = 0.15  # Move forward slowly
                cmd.angular.z = 0.3  # Turn left gently to search for wall
                
                self.get_logger().info('Searching for wall...')
                
                # Reset PID when not following wall
                self.integral = 0.0
                self.previous_error = 0.0
        
        # Publish the command
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        wall_follower = WallFollowerController()
        rclpy.spin(wall_follower)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        if 'wall_follower' in locals():
            stop_cmd = Twist()
            wall_follower.cmd_pub.publish(stop_cmd)
            wall_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()