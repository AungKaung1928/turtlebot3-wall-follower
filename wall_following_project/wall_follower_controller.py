#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
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
        self.filtered_derivative = 0.0
        self.search_direction = 1
        self.state_counter = 0
        self.resume_state = RobotState.SEARCHING
        self.escape_dir = 1
        
        # ROS communication
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/wall_follower/state', 10)
        # The TurtleBot3 LDS driver publishes /scan BEST_EFFORT; a default
        # RELIABLE subscription matches in Gazebo but receives nothing on
        # the real robot.
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, sensor_qos)
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
        self.declare_parameter('lookahead_distance', 0.3)
        self.declare_parameter('beam_spread_deg', 40.0)
        
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
        self.lookahead = self._get_validated_param('lookahead_distance', 0.1, 1.0)
        self.beam_spread_deg = self._get_validated_param('beam_spread_deg', 20.0, 70.0)
        
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

    def _angle_to_index(self, angle_deg):
        """
        Convert a bearing in the robot frame to a scan index.

        The TurtleBot3 LDS publishes angle_min=0.0, angle_max=2*pi, so a
        negative bearing has no direct index. Wrapping the offset modulo a
        full turn handles that convention and the -pi..+pi convention alike.
        """
        scan = self.laser_data
        n = len(scan.ranges)
        offset = (math.radians(angle_deg) - scan.angle_min) % (2.0 * math.pi)
        return int(round(offset / scan.angle_increment)) % n

    def _range_at_index(self, idx):
        """Return ranges[idx] if it is a usable measurement, else inf."""
        d = self.laser_data.ranges[idx]
        if math.isnan(d) or math.isinf(d):
            return float('inf')
        if self.laser_data.range_min < d < self.laser_data.range_max:
            return d
        return float('inf')

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

        n = len(self.laser_data.ranges)
        idx = self._angle_to_index(angle_deg)

        if average:
            # Average over ±3 indices for noise reduction (wrapping at the seam)
            valid_distances = []
            for k in range(-3, 4):
                d = self._range_at_index((idx + k) % n)
                if math.isfinite(d):
                    valid_distances.append(d)
            return sum(valid_distances) / len(valid_distances) if valid_distances else float('inf')

        return self._range_at_index(idx)

    def get_min_distance_in_arc(self, start_angle, end_angle, step=3):
        """Get minimum distance in angular range"""
        distances = [self.get_distance_at_angle(a) for a in range(start_angle, end_angle+1, step)]
        return min(distances) if distances else float('inf')

    def detect_collision_threat(self, margin=1.0):
        """
        Multi-zone collision detection.
        margin scales every threshold. Clearing with a wider margin than
        the one that tripped gives the state machine hysteresis; without
        it the robot re-trips on the same obstacle every other cycle.
        Returns: (is_collision, front_distance)
        """
        stop = self.emergency_stop * margin
        side = self.side_clearance * margin
        # Critical front zone
        front_center = self.get_min_distance_in_arc(-20, 20, 2)
        front_left = self.get_min_distance_in_arc(20, 50, 3)
        front_right = self.get_min_distance_in_arc(-50, -20, 3)
        
        # Wide front zone
        front_wide_left = self.get_min_distance_in_arc(50, 70, 5)
        front_wide_right = self.get_min_distance_in_arc(-70, -50, 5)
        
        # Side zones. The wall currently being followed is expected to be
        # close, so exclude it here - follow_wall() guards it with wall_min.
        right_side = self.get_min_distance_in_arc(-90, -60, 3)
        left_side = self.get_min_distance_in_arc(60, 90, 3)
        if self.state == RobotState.FOLLOWING:
            if self.wall_side == 'right':
                right_side = float('inf')
            else:
                left_side = float('inf')
        
        min_front = min(front_center, front_left, front_right)
        
        # Check collision conditions
        collision = (
            front_center < stop or
            front_left < stop or
            front_right < stop or
            right_side < side or
            left_side < side or
            front_wide_left < side or
            front_wide_right < side
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
            self.filtered_derivative = 0.0
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

    def _wall_geometry(self):
        """
        Two-beam wall estimate: (perpendicular distance, heading angle to wall).

        b is the side beam (+-90 deg), a the beam swung beam_spread_deg toward
        the front. alpha > 0 means the heading converges on the wall. A single
        side beam inflates as soon as the robot yaws, which made the lost-wall
        check fire every time the P term steered inward.
        Returns None when either beam has no return.
        """
        sign = -1 if self.wall_side == 'right' else 1
        theta = math.radians(self.beam_spread_deg)
        b = self.get_distance_at_angle(sign * 90, average=True)
        a = self.get_distance_at_angle(sign * (90 - self.beam_spread_deg), average=True)
        if not (math.isfinite(a) and math.isfinite(b)):
            return None
        alpha = math.atan2(b - a * math.cos(theta), a * math.sin(theta))
        alpha = max(-1.0, min(1.0, alpha))
        return b * math.cos(alpha), alpha

    def follow_wall(self):
        """
        Wall following with PD control on the look-ahead wall distance.
        Returns: Twist command
        """
        cmd = Twist()
        sign = -1 if self.wall_side == 'right' else 1
        lo, hi = (-135, -45) if self.wall_side == 'right' else (45, 135)
        arc_min = self.get_min_distance_in_arc(lo, hi, 3)
        geometry = self._wall_geometry()
        if geometry is None:
            wall_dist, alpha = arc_min, 0.0
        else:
            wall_dist, alpha = geometry

        # Wall ended beside us (outside corner): both beams look past it while
        # the rear-side arc still has it. Arc around the corner at the standoff
        # radius instead of dropping into the open-space search sweep.
        if geometry is None and arc_min <= self.wall_lost:
            cmd.linear.x = self.search_speed
            cmd.angular.z = sign * self.search_speed / self.desired_distance
            return cmd

        # Lost wall? Only when neither the estimate nor the wide arc sees it.
        if min(wall_dist, arc_min) > self.wall_lost:
            self.state = RobotState.SEARCHING
            # An outside corner: curve toward the side the wall was on to wrap it.
            self.search_direction = -1 if self.wall_side == 'right' else 1
            self.get_logger().info(f'Wall lost (distance: {wall_dist:.2f}m)')
            return Twist()

        # Too close to wall - emergency turn away
        if wall_dist < self.wall_min:
            cmd.linear.x = 0.08
            cmd.angular.z = -sign * 0.6
            return cmd

        # Distance the robot will have after lookahead metres at the current
        # heading. Steering on this instead of the raw side range damps the
        # approach: converging fast shrinks the error before the wall arrives.
        predicted = wall_dist - self.lookahead * math.sin(alpha)
        error = predicted - self.desired_distance  # > 0: too far from wall
        raw_derivative = (error - self.prev_error) / 0.05  # 20Hz = 0.05s
        # Dividing by dt amplifies scan noise 20x, so filter before using it
        self.filtered_derivative = (0.7 * self.filtered_derivative
                                    + 0.3 * raw_derivative)
        angular_velocity = self.kp * error + self.kd * self.filtered_derivative
        # sign steers toward the followed wall when too far, away when too close
        cmd.angular.z = sign * max(-self.max_angular_speed,
                                   min(angular_velocity, self.max_angular_speed))

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

    def pick_escape_direction(self):
        """
        Choose which way to turn out of a collision, once, on entry.

        Re-deciding this every 50ms makes the robot dither around the
        decision boundary and never complete a turn, so the result is
        latched in self.escape_dir for the whole avoidance episode.
        Returns: +1 for left, -1 for right
        """
        if self.state == RobotState.FOLLOWING:
            # Turn away from the wall being followed - turning into it
            # is never the escape.
            return 1 if self.wall_side == 'right' else -1

        left_clearance = self.get_min_distance_in_arc(45, 135, 5)
        right_clearance = self.get_min_distance_in_arc(-135, -45, 5)
        return 1 if left_clearance >= right_clearance else -1

    def avoid_collision(self):
        """
        Emergency collision avoidance.
        Returns: Twist command
        """
        cmd = Twist()
        self.state_counter += 1

        # Rotating in place cannot help when every heading is blocked, which
        # is how the robot wedges itself into a corner. Back off once the
        # turn has clearly failed, but only if there is room behind.
        cmd.linear.x = 0.0
        if self.state_counter > self.stuck_threshold * 20:
            rear = self.get_min_distance_in_arc(150, 210, 5)
            if rear > 0.30:
                cmd.linear.x = -0.06

        cmd.angular.z = 0.8 * self.escape_dir
        return cmd

    def control_loop(self):
        """Main control loop - state machine execution"""
        if self.laser_data is None:
            return
        
        # Collision detection takes priority. Clearing needs a wider
        # margin than tripping, otherwise the robot oscillates on the
        # threshold instead of escaping.
        margin = 1.3 if self.state == RobotState.AVOIDING else 1.0
        collision_detected, front_dist = self.detect_collision_threat(margin)
        
        if collision_detected:
            if self.state != RobotState.AVOIDING:
                self.get_logger().warn(f'COLLISION THREAT! Front: {front_dist:.2f}m - Avoiding')
                self.resume_state = self.state
                self.escape_dir = self.pick_escape_direction()
                self.state = RobotState.AVOIDING
                self.state_counter = 0
            
            cmd = self.avoid_collision()
        else:
            # Exit avoidance state if collision cleared
            if self.state == RobotState.AVOIDING:
                # Resume following if we still have the wall; a corner should
                # not throw away the lock and restart the search.
                self.state = self.resume_state
                self.get_logger().info(f'Collision cleared - resuming {self.state.name}')
                self.state_counter = 0
                self.prev_error = 0.0
                self.filtered_derivative = 0.0
            
            # Execute state-specific behavior
            if self.state == RobotState.SEARCHING:
                cmd = self.search_for_wall()
            elif self.state == RobotState.FOLLOWING:
                cmd = self.follow_wall()
            else:
                cmd = Twist()
        
        # Final safety clamp
        # Lower bound is negative so avoid_collision can back out of a
        # corner; every other state only ever commands forward motion.
        cmd.linear.x = max(-0.08, min(cmd.linear.x, self.forward_speed))
        cmd.angular.z = max(-self.max_angular_speed, min(cmd.angular.z, self.max_angular_speed))
        
        # Emergency brake for immediate front obstacles. Must sit below
        # emergency_stop_distance or it pre-empts the avoid behaviour.
        immediate_front = self.get_min_distance_in_arc(-15, 15, 1)
        if immediate_front < self.emergency_stop * 0.7 and cmd.linear.x > 0.0:
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
        if rclpy.ok():
            self.cmd_pub.publish(Twist())

def main(args=None):
    # Keep the context alive through SIGINT so the zero Twist in shutdown() is
    # actually delivered; rclpy's own handler would invalidate it first.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    
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
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()