#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
from enum import Enum

class RobotState(Enum):
    """Explicit state machine for behavior clarity"""
    SEARCHING = 1
    FOLLOWING = 2
    AVOIDING = 3

class WallFollowerController(Node):
    def __init__(self):
        super().__init__('wall_follower_controller')
        
        # Declare all parameters with defaults and validation
        self._declare_and_validate_parameters()
        
        # Load validated parameters
        self._load_parameters()
        
        # State machine
        self.state = RobotState.SEARCHING
        self.wall_side = 'right'
        self.laser_data = None
        self.prev_error = 0.0
        self.search_direction = 1
        self.state_counter = 0
        
        # ROS communication
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/wall_follower/state', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz control
        
        self.get_logger().info('='*50)
        self.get_logger().info('Wall Follower Controller Initialized')
        self.get_logger().info(f'Target wall distance: {self.desired_distance}m')
        self.get_logger().info(f'PID gains: Kp={self.kp}, Kd={self.kd}')
        self.get_logger().info('='*50)

    def _declare_and_validate_parameters(self):
        """Declare parameters with type checking and validation"""
        # Control parameters
        self.declare_parameter('desired_distance', 0.6)
        self.declare_parameter('forward_speed', 0.20)
        self.declare_parameter('search_speed', 0.16)
        self.declare_parameter('max_angular_speed', 0.6)
        self.declare_parameter('kp', 1.8)
        self.declare_parameter('kd', 0.7)
        
        # Safety parameters
        self.declare_parameter('emergency_stop_distance', 0.55)
        self.declare_parameter('slow_down_distance', 0.8)
        self.declare_parameter('wall_min_distance', 0.45)
        self.declare_parameter('wall_lost_distance', 1.5)
        self.declare_parameter('side_clearance', 0.4)
        
        # Behavior parameters
        self.declare_parameter('search_period_cycles', 60)
        self.declare_parameter('stuck_threshold_cycles', 3)

    def _load_parameters(self):
        """Load and validate parameters with safety checks"""
        # Control parameters
        self.desired_distance = self._get_validated_param('desired_distance', 0.3, 1.5)
        self.forward_speed = self._get_validated_param('forward_speed', 0.05, 0.5)
        self.search_speed = self._get_validated_param('search_speed', 0.05, 0.3)
        self.max_angular_speed = self._get_validated_param('max_angular_speed', 0.1, 1.5)
        self.kp = self._get_validated_param('kp', 0.1, 5.0)
        self.kd = self._get_validated_param('kd', 0.0, 2.0)
        
        # Safety parameters
        self.emergency_stop = self._get_validated_param('emergency_stop_distance', 0.2, 1.0)
        self.slow_down_dist = self._get_validated_param('slow_down_distance', 0.3, 1.5)
        self.wall_min = self._get_validated_param('wall_min_distance', 0.2, 0.8)
        self.wall_lost = self._get_validated_param('wall_lost_distance', 0.8, 3.0)
        self.side_clearance = self._get_validated_param('side_clearance', 0.2, 0.8)
        
        # Behavior parameters
        self.search_period = self._get_validated_param('search_period_cycles', 20, 200)
        self.stuck_threshold = self._get_validated_param('stuck_threshold_cycles', 2, 10)
        
        # Cross-validation
        if self.emergency_stop >= self.slow_down_dist:
            self.get_logger().warn('emergency_stop >= slow_down_distance! Auto-correcting.')
            self.emergency_stop = self.slow_down_dist * 0.7

    def _get_validated_param(self, name, min_val, max_val):
        """Get parameter with range validation"""
        value = self.get_parameter(name).value
        if not (min_val <= value <= max_val):
            self.get_logger().error(
                f'Parameter {name}={value} out of range [{min_val}, {max_val}]! Using default.'
            )
            # Use midpoint as safe fallback
            return (min_val + max_val) / 2
        return value

    def laser_callback(self, msg):
        """Store latest laser scan data"""
        self.laser_data = msg

    def get_distance_at_angle(self, angle_deg, average=False):
        """
        Get distance at specified angle with optional averaging.
        Args:
            angle_deg: Angle in degrees (0=front, -90=right, 90=left)
            average: If True, average over ±3 indices
        Returns:
            Distance in meters, or inf if no valid reading
        """
        if self.laser_data is None:
            return float('inf')
        
        angle_rad = math.radians(angle_deg)
        idx = int((angle_rad - self.laser_data.angle_min) / self.laser_data.angle_increment)
        
        # Clamp index to valid range
        idx = max(0, min(len(self.laser_data.ranges) - 1, idx))
        
        if average:
            # Average over ±3 indices for noise reduction
            indices = range(max(0, idx-3), min(len(self.laser_data.ranges), idx+4))
            valid_distances = []
            for i in indices:
                d = self.laser_data.ranges[i]
                if not (math.isnan(d) or math.isinf(d)) and 0.05 < d < 3.5:
                    valid_distances.append(d)
            return sum(valid_distances) / len(valid_distances) if valid_distances else float('inf')
        else:
            d = self.laser_data.ranges[idx]
            if not (math.isnan(d) or math.isinf(d)) and 0.05 < d < 3.5:
                return d
            return float('inf')

    def get_min_distance_in_arc(self, start_angle, end_angle, step=3):
        """Get minimum distance in angular range"""
        distances = [self.get_distance_at_angle(a) for a in range(start_angle, end_angle+1, step)]
        return min(distances) if distances else float('inf')

    def detect_collision_threat(self):
        """
        Multi-zone collision detection.
        Returns: (is_collision, front_distance)
        """
        # Critical front zone
        front_center = self.get_min_distance_in_arc(-20, 20, 2)
        front_left = self.get_min_distance_in_arc(20, 50, 3)
        front_right = self.get_min_distance_in_arc(-50, -20, 3)
        
        # Wide front zone
        front_wide_left = self.get_min_distance_in_arc(50, 70, 5)
        front_wide_right = self.get_min_distance_in_arc(-70, -50, 5)
        
        # Side zones
        right_side = self.get_min_distance_in_arc(-90, -60, 3)
        left_side = self.get_min_distance_in_arc(60, 90, 3)
        
        min_front = min(front_center, front_left, front_right)
        
        # Check collision conditions
        collision = (
            front_center < self.emergency_stop or
            front_left < self.emergency_stop or
            front_right < self.emergency_stop or
            right_side < self.side_clearance or
            left_side < self.side_clearance or
            front_wide_left < 0.35 or
            front_wide_right < 0.35
        )
        
        return collision, min_front

    def search_for_wall(self):
        """
        Search behavior: rotate to find wall.
        Returns: Twist command
        """
        # Check for walls on both sides
        right_dist = self.get_distance_at_angle(-90, average=True)
        left_dist = self.get_distance_at_angle(90, average=True)
        
        # Found wall? Switch to following
        if right_dist < self.wall_lost or left_dist < self.wall_lost:
            self.wall_side = 'right' if right_dist < left_dist else 'left'
            self.state = RobotState.FOLLOWING
            self.state_counter = 0
            self.prev_error = 0.0  # Reset PID
            self.get_logger().info(f'Wall detected on {self.wall_side.upper()} at {min(right_dist, left_dist):.2f}m')
            return Twist()  # Stop for one cycle before following
        
        # No wall found - continue searching
        cmd = Twist()
        front_dist = self.get_min_distance_in_arc(-25, 25, 2)
        
        # Adjust speed based on front clearance
        cmd.linear.x = self.search_speed if front_dist > self.slow_down_dist else self.search_speed * 0.4
        
        # Oscillating search pattern
        self.state_counter += 1
        if self.state_counter > self.search_period:
            self.search_direction *= -1
            self.state_counter = 0
        
        cmd.angular.z = 0.25 * self.search_direction
        
        return cmd

    def follow_wall(self):
        """
        Wall following with PID control.
        Returns: Twist command
        """
        cmd = Twist()
        
        # Get wall distance
        angle = -90 if self.wall_side == 'right' else 90
        wall_dist = self.get_distance_at_angle(angle, average=True)
        
        # Lost wall? Return to search
        if wall_dist > self.wall_lost:
            self.state = RobotState.SEARCHING
            self.search_direction = 1 if self.wall_side == 'right' else -1
            self.get_logger().info(f'Wall lost (distance: {wall_dist:.2f}m)')
            return Twist()
        
        # Too close to wall - emergency turn away
        if wall_dist < self.wall_min:
            cmd.linear.x = 0.08
            cmd.angular.z = 0.6 if self.wall_side == 'right' else -0.6
            return cmd
        
        # PID control for wall following
        error = self.desired_distance - wall_dist
        derivative = (error - self.prev_error) / 0.05  # 20Hz = 0.05s
        angular_velocity = self.kp * error + self.kd * derivative
        
        # Invert angular velocity for left wall
        if self.wall_side == 'left':
            angular_velocity = -angular_velocity
        
        # Clamp angular velocity
        cmd.angular.z = max(-self.max_angular_speed, min(angular_velocity, self.max_angular_speed))
        
        # Speed control based on front clearance
        front_dist = self.get_min_distance_in_arc(-30, 30, 2)
        if front_dist < self.slow_down_dist:
            # Linear interpolation between emergency_stop and slow_down_dist
            speed_factor = (front_dist - self.emergency_stop) / (self.slow_down_dist - self.emergency_stop)
            speed_factor = max(0.2, min(1.0, speed_factor))
            cmd.linear.x = self.forward_speed * speed_factor
        else:
            cmd.linear.x = self.forward_speed
        
        # Reduce speed during sharp turns
        if abs(cmd.angular.z) > 0.3:
            turn_factor = 1.0 - (abs(cmd.angular.z) / self.max_angular_speed) * 0.7
            cmd.linear.x *= turn_factor
        
        # Ensure minimum forward speed
        cmd.linear.x = max(0.05, cmd.linear.x)
        
        # Update PID state
        self.prev_error = error
        
        return cmd

    def avoid_collision(self):
        """
        Emergency collision avoidance.
        Returns: Twist command
        """
        cmd = Twist()
        cmd.linear.x = 0.0  # Full stop
        
        # Analyze escape directions
        left_clearance = self.get_min_distance_in_arc(45, 135, 5)
        right_clearance = self.get_min_distance_in_arc(-135, -45, 5)
        back_left = self.get_min_distance_in_arc(135, 180, 5)
        back_right = self.get_min_distance_in_arc(-180, -135, 5)
        
        # Choose best escape direction
        if left_clearance > right_clearance and left_clearance > 0.8:
            cmd.angular.z = 0.8  # Turn left
        elif right_clearance > 0.8:
            cmd.angular.z = -0.8  # Turn right
        elif back_left > back_right and back_left > 0.6:
            cmd.angular.z = 0.8
        elif back_right > 0.6:
            cmd.angular.z = -0.8
        else:
            # No clear escape - rotate in place
            self.state_counter += 1
            cmd.angular.z = 0.6 if (self.state_counter // 20) % 2 == 0 else -0.6
        
        return cmd

    def control_loop(self):
        """Main control loop - state machine execution"""
        if self.laser_data is None:
            return
        
        # Collision detection takes priority
        collision_detected, front_dist = self.detect_collision_threat()
        
        if collision_detected:
            if self.state != RobotState.AVOIDING:
                self.get_logger().warn(f'COLLISION THREAT! Front: {front_dist:.2f}m - Avoiding')
                self.state = RobotState.AVOIDING
                self.state_counter = 0
            
            cmd = self.avoid_collision()
        else:
            # Exit avoidance state if collision cleared
            if self.state == RobotState.AVOIDING:
                self.get_logger().info('Collision cleared - resuming normal operation')
                self.state = RobotState.SEARCHING
                self.state_counter = 0
            
            # Execute state-specific behavior
            if self.state == RobotState.SEARCHING:
                cmd = self.search_for_wall()
            elif self.state == RobotState.FOLLOWING:
                cmd = self.follow_wall()
            else:
                cmd = Twist()
        
        # Final safety clamp
        cmd.linear.x = max(0.0, min(cmd.linear.x, self.forward_speed))
        cmd.angular.z = max(-self.max_angular_speed, min(cmd.angular.z, self.max_angular_speed))
        
        # Emergency brake for immediate front obstacles
        immediate_front = self.get_min_distance_in_arc(-15, 15, 1)
        if immediate_front < 0.4:
            cmd.linear.x = 0.0
        
        # Publish commands and state
        self.cmd_pub.publish(cmd)
        self.publish_state()

    def publish_state(self):
        """Publish current state for debugging"""
        state_msg = String()
        state_msg.data = self.state.name
        self.state_pub.publish(state_msg)

    def shutdown(self):
        """Graceful shutdown - stop robot"""
        self.get_logger().info('Shutting down - stopping robot')
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = WallFollowerController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f'Fatal error: {e}')
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()