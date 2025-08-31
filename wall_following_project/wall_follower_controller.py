#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class WallFollowerController(Node):
   def __init__(self):
       super().__init__('wall_follower_controller')
      
       # Parameters - ADJUSTED FOR SAFETY
       self.desired_distance = 0.6  # Increased from 0.5 for more clearance
       self.forward_speed = 0.20    # Slightly reduced for better control
       self.search_speed = 0.16
       self.max_angular_speed = 0.6
       self.kp, self.kd = 1.8, 0.7  # More responsive control
      
       # Safety distances - INCREASED FOR NO CONTACT
       self.emergency_stop = 0.55   # Increased from 0.35 - much earlier detection
       self.slow_down_dist = 0.8    # Increased from 0.5
       self.wall_min = 0.45         # Increased from 0.35 - minimum wall distance
       self.wall_lost = 1.5         # Increased from 1.2
       self.side_clearance = 0.4    # Increased from 0.25 - side obstacle clearance
      
       # State
       self.following_wall = False
       self.wall_side = 'right'
       self.laser_data = None
       self.prev_error = 0.0
       self.search_dir = 1
       self.counter = 0
       self.stuck_counter = 0       # Add stuck detection
      
       # ROS
       self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
       self.scan_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
       self.timer = self.create_timer(0.05, self.control_loop)
      
       self.get_logger().info('Wall Follower - Enhanced Safety Mode')

   def laser_callback(self, msg):
       self.laser_data = msg

   def get_dist(self, angle, avg=False):
       """Get distance at angle (or average if avg=True)"""
       if not self.laser_data:
           return float('inf')
      
       angle_rad = math.radians(angle)
       idx = int((angle_rad - self.laser_data.angle_min) / self.laser_data.angle_increment)
      
       # Get multiple readings for averaging
       indices = range(max(0, idx-3), min(len(self.laser_data.ranges), idx+4))  # Wider range
       dists = []
      
       for i in indices:
           d = self.laser_data.ranges[i]
           if not (math.isnan(d) or math.isinf(d)) and 0.05 < d < 3.5:
               dists.append(d)
      
       if not dists:
           return float('inf')
      
       return sum(dists)/len(dists) if avg else min(dists)

   def get_min_in_range(self, start, end, step=2):  # Reduced step for more thorough scanning
       """Get minimum distance in angle range"""
       dists = [self.get_dist(a) for a in range(start, end+1, step)]
       return min(dists) if dists else float('inf')

   def check_collision(self):
       """Enhanced collision detection with more zones"""
       # Check front zones more thoroughly
       front_center = self.get_min_in_range(-20, 20, 2)
       front_left = self.get_min_in_range(20, 50, 3)
       front_right = self.get_min_in_range(-50, -20, 3)
       front_wide_left = self.get_min_in_range(50, 70, 5)
       front_wide_right = self.get_min_in_range(-70, -50, 5)
      
       front = min(front_center, front_left, front_right)
      
       # Check sides with increased clearance
       right = self.get_min_in_range(-90, -60, 3)
       left = self.get_min_in_range(60, 90, 3)
      
       # Collision if any zone is too close
       collision = (front < self.emergency_stop or
                   right < self.side_clearance or
                   left < self.side_clearance or
                   front_wide_left < 0.35 or
                   front_wide_right < 0.35)
      
       return collision, front

   def find_wall(self):
       """Find nearest wall"""
       right = self.get_dist(-90, avg=True)
       left = self.get_dist(90, avg=True)
      
       if right < self.wall_lost or left < self.wall_lost:
           self.wall_side = 'right' if right < left else 'left'
           self.following_wall = True
           self.counter = 0
           self.stuck_counter = 0
           self.get_logger().info(f'Following {self.wall_side.upper()} wall')
           return True
      
       return False

   def wall_follow(self):
       """Enhanced wall following with better safety margins"""
       cmd = Twist()
      
       # Get wall distance
       angle = -90 if self.wall_side == 'right' else 90
       wall_dist = self.get_dist(angle, avg=True)
      
       # Lost wall?
       if wall_dist > self.wall_lost:
           self.following_wall = False
           return self.search()
      
       # Too close to wall? More aggressive avoidance
       if wall_dist < self.wall_min:
           cmd.linear.x = 0.08  # Slower when too close
           cmd.angular.z = 0.6 if self.wall_side == 'right' else -0.6  # More aggressive turn
           return cmd
      
       # PID control with enhanced responsiveness
       error = self.desired_distance - wall_dist
       angular = self.kp * error + self.kd * (error - self.prev_error) / 0.05
      
       if self.wall_side == 'left':
           angular = -angular
      
       cmd.angular.z = max(min(angular, self.max_angular_speed), -self.max_angular_speed)
      
       # Speed control based on front clearance with more conservative scaling
       front = self.get_min_in_range(-30, 30, 2)
       if front < self.slow_down_dist:
           # More conservative speed reduction
           factor = max(0.2, (front - self.emergency_stop) / (self.slow_down_dist - self.emergency_stop))
           cmd.linear.x = self.forward_speed * factor
       else:
           cmd.linear.x = self.forward_speed
      
       # Further slow down for sharp turns
       if abs(cmd.angular.z) > 0.3:
           cmd.linear.x *= (1.0 - abs(cmd.angular.z) / self.max_angular_speed * 0.7)  # More aggressive slowdown
      
       cmd.linear.x = max(cmd.linear.x, 0.05)
       self.prev_error = error
      
       return cmd

   def search(self):
       """Enhanced search with stuck detection"""
       cmd = Twist()
       front = self.get_min_in_range(-25, 25, 2)
      
       # Speed based on front clearance
       if front > self.slow_down_dist:
           cmd.linear.x = self.search_speed
       else:
           cmd.linear.x = self.search_speed * 0.4
      
       # Search pattern with stuck detection
       self.counter += 1
       if self.counter > 60:  # Longer search periods
           self.search_dir *= -1
           self.counter = 0
           self.stuck_counter += 1
      
       # If stuck for too long, try more aggressive maneuver
       if self.stuck_counter > 3:
           cmd.angular.z = 0.6 * self.search_dir
           cmd.linear.x = 0.1
           if self.counter > 30:  # Reset stuck counter after trying
               self.stuck_counter = 0
       else:
           cmd.angular.z = 0.25 * self.search_dir
      
       return cmd

   def escape_collision(self):
       """Enhanced collision escape behavior"""
       cmd = Twist()
       cmd.linear.x = 0.0  # Stop immediately
      
       # Analyze escape directions
       left_clearance = self.get_min_in_range(45, 135, 5)
       right_clearance = self.get_min_in_range(-135, -45, 5)
       back_left = self.get_min_in_range(135, 180, 5)
       back_right = self.get_min_in_range(-180, -135, 5)
      
       # Choose best escape direction
       if left_clearance > right_clearance and left_clearance > 0.8:
           cmd.angular.z = 0.8  # Turn left aggressively
       elif right_clearance > 0.8:
           cmd.angular.z = -0.8  # Turn right aggressively
       elif back_left > back_right and back_left > 0.6:
           cmd.angular.z = 0.8  # Turn toward back-left
       elif back_right > 0.6:
           cmd.angular.z = -0.8  # Turn toward back-right
       else:
           # If no good escape, rotate to find opening
           cmd.angular.z = 0.6 if self.counter % 40 < 20 else -0.6
      
       return cmd

   def control_loop(self):
       """Main control with enhanced collision handling"""
       if not self.laser_data:
           return
      
       # Enhanced collision check
       collision, min_dist = self.check_collision()
       if collision:
           cmd = self.escape_collision()
           self.cmd_pub.publish(cmd)
           if self.counter % 10 == 0:  # Reduce log frequency
               self.get_logger().warn(f'Collision avoidance! {min_dist:.2f}m - Escaping')
           self.counter += 1
           return
      
       # Reset counter when not in collision
       self.counter = 0
      
       # Main navigation logic
       if self.following_wall:
           cmd = self.wall_follow()
       elif self.find_wall():
           cmd = self.wall_follow()
       else:
           cmd = self.search()
      
       # Enhanced safety limits
       cmd.linear.x = max(0.0, min(cmd.linear.x, self.forward_speed))
       cmd.angular.z = max(-self.max_angular_speed, min(cmd.angular.z, self.max_angular_speed))
      
       # Final safety check before publishing
       immediate_front = self.get_min_in_range(-15, 15, 1)
       if immediate_front < 0.4:  # Emergency brake
           cmd.linear.x = 0.0
      
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
