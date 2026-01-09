#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
import math
import time

# ----------------- Utility Functions -----------------
def quat_to_yaw(x, y, z, w):
    """Convert quaternion orientation to yaw angle."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

def clamp_angle(theta):
    """Normalize angle into [-pi, pi]."""
    while theta > math.pi:
        theta -= 2.0 * math.pi
    while theta < -math.pi:
        theta += 2.0 * math.pi
    return theta

# ----------------- Main Class -----------------
class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('ebot_nav_task2a_logic')

        # Publishers / Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.detection_status_pub = self.create_publisher(String, '/detection_status', 10) 
        self.advance_complete_pub = self.create_publisher(Bool, '/advance_complete_signal', 10)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(String, '/nav_command', self.command_callback, 10) 
        self.create_subscription(Odometry, '/fake_odom', self.fake_odom_callback, 10)
        
        # ------------------- WAYPOINTS (x, y, final_yaw) -------------------
        self.goals = [
            (0.02, -1.43, 0.02),   # p1
            (2.16, -1.70, -0.07),  # (Dock Station) - Stop point
            (4.45, -1.76, 0.00),   # p2
            (4.49, -0.27, 2.19),   # p3
            (0.95, 0.05, 3.00),   # p4 
            (0.57, 1.08, 1.49),   # p5
            (1.13, 1.61, 0.06),   # p6
            (4.36, 1.70,-0.34),
            (4.87,0.42,-2.256),   # 
            (3.76, 0.01,-3.10),   # p8
            (0.00, 0.00, -3.14), # eBOT point
        ]
        
        self.current_goal_idx = 0
        self.P1_INDEX = 1 
        
        # Robot Pose and Sensor Data
        self.robot_x, self.robot_y, self.robot_yaw = 0.0, 0.0, 0.0
        self.lidar_ready = False
        self.laser_ranges = []
        self.angle_min, self.angle_increment = 0.0, 0.0
        self.use_fake_odom = False

        # ------------------- STATE MANAGEMENT -------------------
        self.state = 'NAVIGATING' 
        # Flag to track if the dock station (P1 or initial shape detection) has been handled
        self.dock_station_reported = False         
        self.detection_cooldown_timer = None 
        self.COOLDOWN_DURATION = 2.0     
        
        # --- Variables for the ADVANCE logic ---
        self.advance_distance = 0.0
        self.initial_x = 0.0
        self.initial_y = 0.0
        
        # ------------------- CONTROL PARAMETERS -------------------
        self.Kp_lin, self.Kp_ang = 0.8, 1.5
        self.max_v, self.max_w = 0.5, 1.0  # Velocity limits: max 0.5 m/s linear, 1.0 rad/s angular
        
        self.GENERAL_DIST_THRESHOLD = 0.5      
        self.DOCK_DIST_THRESHOLD = 0.02
        self.angle_threshold = math.radians(10) 
        
        # Obstacle avoidance params
        self.obs_front_limit = 0.55
        self.obs_clear_limit = 0.9
        self.avoid_mode = False
        self.avoid_side = 0

        # Main loop (30 Hz)
        self.create_timer(1.0 / 30.0, self.update_control)
        self.get_logger().info("WaypointNavigator initialized. Starting navigation...")

    # ---------- Callbacks ---------------------------------------

    def odom_callback(self, msg: Odometry):
        """Update robot's current pose."""
        if self.use_fake_odom:
            return
            
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

    def fake_odom_callback(self, msg: Odometry):
        """Handle fake odometry input from terminal."""
        self.use_fake_odom = True
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

    def lidar_callback(self, msg: LaserScan):
        """Store LiDAR data."""
        self.laser_ranges = list(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.lidar_ready = True
        
    def command_callback(self, msg: String):
        """
        Handle STOP/RESUME commands.
        DOCK_FOUND is ignored/removed as odometry handles dock reporting now.
        """
        parts = msg.data.upper().split(',')
        command = parts[0]
        
        # Extract advance distance for STOP command
        try:
            self.advance_distance = float(parts[1]) if len(parts) > 1 else 0.0
        except ValueError:
            self.advance_distance = 0.0
            self.get_logger().error(f"Invalid advance distance in command: {msg.data}")

        if command == 'STOP' and self.state == 'NAVIGATING':
            # Stop immediately and switch to ADVANCE state (for plant detection)
            self.state = 'ADVANCE_TO_SHAPE' 
            self.initial_x = self.robot_x
            self.initial_y = self.robot_y
            self.publish_twist(0.0, 0.0)
            self.get_logger().warn(f"⚠️ Received STOP command. Preparing to advance {self.advance_distance:.2f}m.")

    def _resume_external_stop(self):
        """
        Resumes motion after an external (LiDAR) stop and PUBLISHES THE COMPLETION SIGNAL.
        """
        if self.detection_cooldown_timer:
            self.detection_cooldown_timer.cancel()
            self.detection_cooldown_timer = None
            
        # 1. Signal the Detector that the advance and hold is complete
        self.advance_complete_pub.publish(Bool(data=True))
        self.get_logger().info("📢 Advance/Hold Complete. Publishing /advance_complete_signal.")

        # 2. Resume Navigation
        self.state = 'NAVIGATING'
        self.get_logger().info("✅ Resuming navigation.")

    def _resume_after_dock(self):
        """Resume navigation after dock wait."""
        if self.detection_cooldown_timer:
            self.detection_cooldown_timer.cancel()
            self.detection_cooldown_timer = None

        self.state = 'NAVIGATING'
        self.current_goal_idx += 1  # Move to next waypoint
        self.get_logger().info("✅ Dock wait complete. Moving to next waypoint.")

    def _handle_dock_report(self):
        """
        Handles the reporting of the dock station at P1.
        Does NOT stop or wait for fertilizer.
        """
        if self.dock_station_reported:
            return

        # 1. Publish the DOCK_STATION status: STATUS,X,Y,PLANT_ID=0
        report_x, report_y = self.robot_x, self.robot_y
        plant_id = 0 
        
        msg = String()
        # Format: STATUS,X,Y,PLANT_ID
        msg.data = f"DOCK_STATION,{report_x:.2f},{report_y:.2f},{plant_id}" 
        self.detection_status_pub.publish(msg)
        self.get_logger().info(f'📢 PUBLISHED DOCK_STATION REPORT: {msg.data}')
        
        self.dock_station_reported = True # Flag dock station as complete

    # ---------- LiDAR Processing (for Obstacle Avoidance) ----------
    def sector_distance(self, angle_center, angle_width):
        """Get minimum distance in a LiDAR sector."""
        if not self.lidar_ready: return float('inf')
        N = len(self.laser_ranges)
        
        idx_center = int((angle_center - self.angle_min) / self.angle_increment)
        half_range = int(angle_width / abs(self.angle_increment) / 2) 
        start = max(0, idx_center - half_range)
        end = min(N - 1, idx_center + half_range)

        valid = [r for r in self.laser_ranges[start:end + 1] 
                 if r > 0.1 and not math.isinf(r) and not math.isnan(r)]
                 
        return min(valid) if valid else float('inf')

    def front_distance(self):
        """Get minimum distance in the front sector (40 degrees wide)."""
        return self.sector_distance(0.0, math.radians(40)) 

    def choose_side(self):
        """Pick left or right based on which side is more open."""
        left_clear = self.sector_distance(math.radians(45), math.radians(45))
        right_clear = self.sector_distance(-math.radians(45), math.radians(45))
        return 1 if left_clear > right_clear else -1 

    # ---------- Control algorithm ---------------------------------
    def update_control(self):
        """Main control loop for navigation, state checking, and obstacle avoidance."""
        
        # --- REMOVED LIDAR ACTIVATION LOGIC HERE ---
        
        # --- HALT CONDITION ---
        if self.robot_x is None or self.state in ['STOP_WAYPOINT', 'STOP_EXTERNAL', 'DOCK_WAIT']:
            self.publish_twist(0.0, 0.0)
            return

        # ------------------- ADVANCE LOGIC -------------------
        if self.state == 'ADVANCE_TO_SHAPE':
            current_distance_moved = math.hypot(self.robot_x - self.initial_x, self.robot_y - self.initial_y)

            if current_distance_moved < self.advance_distance:
                self.publish_twist(0.5, 0.0)  # Advance at max linear velocity
                return
            else:
                self.get_logger().info("✅ Advance complete. Initiating 2-second status hold.")
                
                self.state = 'STOP_EXTERNAL' 
                self.publish_twist(0.0, 0.0)

                if self.detection_cooldown_timer:
                    self.detection_cooldown_timer.cancel()
                self.detection_cooldown_timer = self.create_timer(self.COOLDOWN_DURATION, self._resume_external_stop)
                return
        
        # End of mission check
        if self.current_goal_idx >= len(self.goals):
            self.publish_twist(0.0, 0.0)
            self.state = 'STOP_WAYPOINT' 
            self.get_logger().info("🏁 All waypoints reached. Final stop.")
            return

        # --- Determine current distance threshold ---
        current_dist_threshold = self.GENERAL_DIST_THRESHOLD
        if self.current_goal_idx == self.P1_INDEX or self.current_goal_idx == len(self.goals) - 1:
            current_dist_threshold = self.DOCK_DIST_THRESHOLD
        
        gx, gy, gyaw = self.goals[self.current_goal_idx]
        
        dx, dy = gx - self.robot_x, gy - self.robot_y
        dist_error = math.hypot(dx, dy)
        final_yaw_err = clamp_angle(gyaw - self.robot_yaw)

        # 1. Waypoint Reached Logic
        if dist_error <= current_dist_threshold:
            if abs(final_yaw_err) > self.angle_threshold:
                w_align = max(-self.max_w, min(self.max_w, self.Kp_ang * final_yaw_err))  # Clamp to max angular velocity
                self.publish_twist(0.0, w_align)
                return
            else:
                self.publish_twist(0.0, 0.0)
                
                # Report Dock Station at P1 (Index 1)
                # We stop, report, wait 2s, then resume.
                if self.current_goal_idx == self.P1_INDEX and not self.dock_station_reported: 
                    self.publish_twist(0.0, 0.0)
                    self.state = 'DOCK_WAIT'
                    self._handle_dock_report()
                    
                    self.get_logger().info("⏳ Waiting 2 seconds at Dock Station...")
                    if self.detection_cooldown_timer:
                        self.detection_cooldown_timer.cancel()
                    self.detection_cooldown_timer = self.create_timer(2.0, self._resume_after_dock)
                    return
                
                # Handle Final stop (last index)
                if self.current_goal_idx == len(self.goals) - 1:
                    self.state = 'STOP_WAYPOINT'
                    self.get_logger().info("🏁 Final waypoint reached. Stopping.")
                    return 
                
                # Move to next waypoint
                self.current_goal_idx += 1
                self.get_logger().info(f"Reached waypoint {self.current_goal_idx}. Moving to next.")
                return 

        # 2. Obstacle Avoidance State Transition and Logic
        d_front = self.front_distance()
        if d_front < self.obs_front_limit and not self.avoid_mode:
            self.avoid_mode = True
            self.avoid_side = self.choose_side()
            self.get_logger().warn(f"🚧 Obstacle detected at {d_front:.2f} m. Initiating avoidance.")
        elif self.avoid_mode and d_front > self.obs_clear_limit:
            self.avoid_mode = False
            self.get_logger().info("✨ Obstacle cleared, resuming waypoint path.")

        # 3. PID (P-Control) Calculations
        v_cmd = self.Kp_lin * dist_error
        v_cmd = max(0.0, min(self.max_v, v_cmd))

        heading_error = clamp_angle(math.atan2(dy, dx) - self.robot_yaw)
        w_cmd = self.Kp_ang * heading_error
        w_cmd = max(-self.max_w, min(self.max_w, w_cmd))

        # 4. Obstacle Override Logic
        if self.avoid_mode:
            turn_power = (self.obs_clear_limit - d_front) / (self.obs_clear_limit - self.obs_front_limit)
            w_cmd = self.avoid_side * self.max_w * turn_power
            v_cmd = min(v_cmd, 0.5)  # Cap linear velocity during obstacle avoidance

        self.publish_twist(v_cmd, w_cmd)

    # ---------- Command Helpers ----------
    def publish_twist(self, v, w):
        """Publishes the linear and angular velocity commands."""
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_vel_pub.publish(msg)

# ----------------- Main -----------------
def main(args=None):
    rclpy.init(args=args)
    nav_node = WaypointNavigator()
    try:
        rclpy.spin(nav_node)
    except KeyboardInterrupt:
        pass
    finally:
        nav_node.publish_twist(0.0, 0.0)
        nav_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()