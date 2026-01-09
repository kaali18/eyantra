#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Quaternion
from std_msgs.msg import String, Float32
from std_srvs.srv import SetBool
from tf2_ros import TransformException, TransformStamped, Buffer, TransformListener
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import math
import time
from typing import List, Dict

# --- Constants ---
LINEAR_P_GAIN = 1.0   
ANGULAR_P_GAIN = 1.2 

# Reduced velocities for hardware safety
MAX_LINEAR_VEL = 0.2  
MAX_ANGULAR_VEL = 0.5 
POS_TOLERANCE = 0.05  
ORI_TOLERANCE = 0.15  
WAYPOINT_HOLD_TIME = 1.0 

EE_LINK = "wrist_3_link" 
BASE_LINK = "base_link"
TEAM_ID = 2203 

# Target TF Frame Names 
FERTILISER_FRAME = f'{TEAM_ID}_fertilizer_1'
EBOT_DROP_FRAME = f'{TEAM_ID}_ebot_base'
BAD_FRUIT_1_FRAME = f'{TEAM_ID}_bad_fruit_1' 
BAD_FRUIT_2_FRAME = f'{TEAM_ID}_bad_fruit_2'
BAD_FRUIT_3_FRAME = f'{TEAM_ID}_bad_fruit_3'
TRASHBIN_DROP_FRAME = 'trash_bin_drop_point' 

# --- Waypoint Definitions ---
WAYPOINTS = [
    # 1. Home/Safe Pose (P1) - ATTACH FERTILIZER HERE
    { 'name': 'P1', 'pos': [-0.218, -0.547, 0.632], 'ori': [0.707, 0.028, 0.034, 0.707], 'action': 'ACTIVATE_MAGNET' },
    
    # 2. Retreat from P1 after picking fertilizer
    { 'name': 'P1_RETREAT', 'pos': [0.2, -0.200, 0.632], 'ori': [0.707, 0.028, 0.034, 0.707], 'action': 'NONE' },
    
    # 3. Approach eBot (higher approach)
    { 
        'name': 'EBOT_APPROACH', 
        'target_frame': EBOT_DROP_FRAME, 
        'x_offset': -0.20, 
        'y_offset': 0.00, 
        'z_offset': 0.30, 
        'action': 'NONE'
    },
    
    # 4. Drop fertilizer can on eBot
    { 
        'name': 'EBOT_DROP', 
        'target_frame': EBOT_DROP_FRAME, 
        'x_offset': 0.10, 
        'y_offset': 0.00, 
        'z_offset': 0.15, 
        'action': 'DEACTIVATE_MAGNET' 
    },

    # ------------------- BAD FRUIT 1 SEQUENCE -------------------
    # 5. Approach above Bad Fruit 1
    { 'name': 'FRUIT_1_GRASP', 'target_frame': BAD_FRUIT_1_FRAME, 'z_offset': 0.02, 'action': 'ACTIVATE_MAGNET'},
    # 6. Retreat with Fruit 1
    { 'name': 'FRUIT_1_RETREAT', 'target_frame': BAD_FRUIT_1_FRAME, 'z_offset': 0.25, 'action': 'NONE'},

    # 7. Place Fruit 1 in Trashbin
    { 'name': 'TRASHBIN_DROP_1', 'pos': [-0.806, 0.010, 0.182], 'ori': [-0.684, 0.726, 0.05, 0.008], 'action': 'DEACTIVATE_MAGNET'},

    # 8. Move to intermediate safe position (P2)
    { 'name': 'P2_INTERMEDIATE_1', 'pos': [-0.159, 0.501, 0.415], 'ori': [0.029, 0.997, 0.045, 0.033], 'action': 'NONE'},
    
    # ------------------- BAD FRUIT 2 SEQUENCE -------------------
    # 9. Approach above Bad Fruit 2
    { 'name': 'FRUIT_2_GRASP', 'target_frame': BAD_FRUIT_2_FRAME, 'z_offset': 0.02, 'action': 'ACTIVATE_MAGNET'},
    # 10. Retreat with Fruit 2
    { 'name': 'FRUIT_2_RETREAT', 'target_frame': BAD_FRUIT_2_FRAME, 'z_offset': 0.25, 'action': 'NONE'},

    # 11. Place Fruit 2 in Trashbin
    { 'name': 'TRASHBIN_DROP_2', 'pos': [-0.806, 0.010, 0.182], 'ori': [-0.684, 0.726, 0.05, 0.008], 'action': 'DEACTIVATE_MAGNET'},
    
    # 12. Move to intermediate safe position (P2)
    { 'name': 'P2_INTERMEDIATE_2', 'pos': [-0.159, 0.501, 0.415], 'ori': [0.029, 0.997, 0.045, 0.033], 'action': 'NONE'},

    # ------------------- BAD FRUIT 3 SEQUENCE -------------------
    # 13. Approach above Bad Fruit 3
    { 'name': 'FRUIT_3_GRASP', 'target_frame': BAD_FRUIT_3_FRAME, 'z_offset': 0.02, 'action': 'ACTIVATE_MAGNET'},
    # 14. Retreat with Fruit 3
    { 'name': 'FRUIT_3_RETREAT', 'target_frame': BAD_FRUIT_3_FRAME, 'z_offset': 0.20, 'action': 'NONE'},

    # 15. Place Fruit 3 in Trashbin
    { 'name': 'TRASHBIN_DROP_3', 'pos': [-0.806, 0.010, 0.182], 'ori': [-0.684, 0.726, 0.05, 0.008], 'action': 'DEACTIVATE_MAGNET'},
]


class WaypointServoingNode(Node):
    
    def __init__(self):
        super().__init__('ur5_waypoint_servo_node')
        self.cb_group = ReentrantCallbackGroup() 
        self.get_logger().info("UR5 Waypoint Servoing Node started (HARDWARE MODE)")

        self.is_started = True # CHANGED: Started immediately to bypass wait

        # Publisher for TwistStamped (Hardware requirement)
        self.pub = self.create_publisher(TwistStamped, '/delta_twist_cmds', 10)
        
        # Publisher for fertilizer signal
        self.fertilizer_pub = self.create_publisher(String, '/fertilizer_can_received', 10)

        # Force monitoring
        self.create_subscription(Float32, '/net_wrench', self.force_callback, 10, callback_group=self.cb_group)
        self.latest_force_z = 0.0

        # TF Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Magnet Service Client
        self.magnet_client = self.create_client(SetBool, '/magnet', callback_group=self.cb_group)
        while not self.magnet_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Magnet service not available, waiting again...')
        
        # Trigger subscription
        self.create_subscription(
            String, '/detection_status', self.detection_status_cb, 10, callback_group=self.cb_group
        )

        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.control_loop, callback_group=self.cb_group)

        self.waypoints = WAYPOINTS
        self.current_waypoint_index = 0
        self.is_holding = False
        self.hold_start_time = 0.0
        
        self.get_logger().info("Starting immediately (Wait condition removed)...")

    def force_callback(self, msg):
        self.latest_force_z = msg.data

    def detection_status_cb(self, msg):
        """Enable manipulation only when 'DOCK_STATION' is received."""
        if "DOCK_STATION" in msg.data:
            if not self.is_started:
                self.is_started = True
                self.get_logger().info(f"Received Trigger: {msg.data}. Manipulation System STARTED.")

    def control_magnet(self, activate: bool):
        """Calls the /magnet service."""
        request = SetBool.Request()
        request.data = activate
        
        future = self.magnet_client.call_async(request)
        # Non-blocking service call handling would be better, but for sequential action we wait briefly
        # or just fire and forget if we trust it. Here we wait with a small timeout in a loop (not ideal but simple)
        # Ideally, we should just let the callback group handle it, but we are inside the control loop logic flow.
        # Since we use ReentrantCallbackGroup, we can wait on the future? 
        # Actually, spinning inside a callback is risky. We'll use a short timeout loop.
        
        timeout = 1.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout:
             time.sleep(0.01)

        if future.done():
            try:
                res = future.result()
                if res.success:
                    self.get_logger().info(f"🧲 Magnet {'ACTIVATED' if activate else 'DEACTIVATED'} successfully.")
                else:
                    self.get_logger().error(f"❌ Magnet service returned failure.")
            except Exception as e:
                self.get_logger().error(f"❌ Magnet service call failed: {e}")
        else:
            self.get_logger().warn("⚠️ Magnet service call timed out (proceeding anyway).")

    def get_target_pose(self, waypoint: Dict) -> tuple[List[float], List[float]]:
        if 'target_frame' in waypoint:
            target_frame = waypoint['target_frame']
            x_offset = waypoint.get('x_offset', 0.0)
            y_offset = waypoint.get('y_offset', 0.0)
            z_offset = waypoint.get('z_offset', 0.0)
            
            try:
                target_tf: TransformStamped = self.tf_buffer.lookup_transform(
                    BASE_LINK, target_frame, rclpy.time.Time()
                )
                
                pos = [
                    target_tf.transform.translation.x + x_offset,
                    target_tf.transform.translation.y + y_offset,
                    target_tf.transform.translation.z + z_offset
                ]
                q = target_tf.transform.rotation
                ori = [q.x, q.y, q.z, q.w]
                
                return pos, ori

            except TransformException as ex:
                self.get_logger().warn(f'Could not lookup transform {BASE_LINK} to {target_frame}: {ex}', throttle_duration_sec=1.0)
                return None, None 
        else:
            return waypoint['pos'], waypoint['ori']

    def control_loop(self):
        if not self.is_started:
            return

        # --- VISION SYNC CHECK (Wait for 3 Bad Fruits) ---
        if self.current_waypoint_index == 0 and not self.is_holding:
            missing_fruits = []
            for i in range(1, 4): # bad_fruit_1, bad_fruit_2, bad_fruit_3
                frame_name = f'{TEAM_ID}_bad_fruit_{i}'
                try:
                    if not self.tf_buffer.can_transform(BASE_LINK, frame_name, rclpy.time.Time()):
                        missing_fruits.append(frame_name)
                except Exception:
                    missing_fruits.append(frame_name)
            
            if missing_fruits:
                self.get_logger().warn(f"⏳ Waiting for Vision: Missing TFs for {missing_fruits}...", throttle_duration_sec=2.0)
                self.publish_zero_twist()
                return

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached. Node shutting down.")
            self.timer.cancel()
            return

        target_waypoint = self.waypoints[self.current_waypoint_index]
        
        # 1. Stop and wait state management
        if self.is_holding:
            time_elapsed = self.get_clock().now().nanoseconds / 1e9 - self.hold_start_time
            if time_elapsed < WAYPOINT_HOLD_TIME:
                self.publish_zero_twist()
                return
            else:
                self.is_holding = False
                
                if target_waypoint['name'] == 'EBOT_DROP':
                    self.get_logger().info("📣 FERTILIZER_LOADED signal published.")
                    msg = String()
                    msg.data = 'FERTILIZER_LOADED'
                    self.fertilizer_pub.publish(msg)
                
                self.current_waypoint_index += 1
                if self.current_waypoint_index < len(self.waypoints):
                    self.get_logger().info(f"✅ Moving to: {self.waypoints[self.current_waypoint_index]['name']}")
                else:
                    self.get_logger().info("✅ All done!")
                return

        # 2. Get Target Pose
        target_p, target_q_list = self.get_target_pose(target_waypoint)
        if target_p is None:
            self.publish_zero_twist()
            return 

        # 3. Get current Pose
        try:
            current_pose_tf = self.tf_buffer.lookup_transform(
                BASE_LINK, EE_LINK, rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f'TF Error: {ex}', throttle_duration_sec=1.0)
            self.publish_zero_twist()
            return

        # 4. Error Calculation
        current_p = current_pose_tf.transform.translation
        pos_error = [target_p[0] - current_p.x, target_p[1] - current_p.y, target_p[2] - current_p.z]
        dist_error = math.sqrt(sum(e**2 for e in pos_error))

        current_q = current_pose_tf.transform.rotation
        target_q = Quaternion(x=target_q_list[0], y=target_q_list[1], z=target_q_list[2], w=target_q_list[3])
        ori_vec, ori_angle = self.quat_error_to_rotation_vector(current_q, target_q)
        
        # 5. Check Goal Reached
        pos_tol = POS_TOLERANCE
        ori_tol = ORI_TOLERANCE
        if 'DROP' in target_waypoint['name']:
            pos_tol = 0.086
            ori_tol = 0.30
        
        if dist_error <= pos_tol and ori_angle <= ori_tol:
            self.get_logger().info(f"✅ Reached {target_waypoint['name']} (Err: {dist_error:.3f}m)")
            self.publish_zero_twist()
            
            # Action (Magnet)
            action = target_waypoint.get('action', 'NONE')
            if action == 'ACTIVATE_MAGNET':
                self.control_magnet(True)
            elif action == 'DEACTIVATE_MAGNET':
                self.control_magnet(False)
            
            self.is_holding = True
            self.hold_start_time = self.get_clock().now().nanoseconds / 1e9
            return

        # 6. Command Velocity (TwistStamped)
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = BASE_LINK

        twist.twist.linear.x = pos_error[0] * LINEAR_P_GAIN
        twist.twist.linear.y = pos_error[1] * LINEAR_P_GAIN
        twist.twist.linear.z = pos_error[2] * LINEAR_P_GAIN

        twist.twist.angular.x = ori_vec[0] * ANGULAR_P_GAIN
        twist.twist.angular.y = ori_vec[1] * ANGULAR_P_GAIN
        twist.twist.angular.z = ori_vec[2] * ANGULAR_P_GAIN

        # Clamping
        lin_mag = math.sqrt(twist.twist.linear.x**2 + twist.twist.linear.y**2 + twist.twist.linear.z**2)
        if lin_mag > MAX_LINEAR_VEL:
            scale = MAX_LINEAR_VEL / lin_mag
            twist.twist.linear.x *= scale
            twist.twist.linear.y *= scale
            twist.twist.linear.z *= scale

        ang_mag = math.sqrt(twist.twist.angular.x**2 + twist.twist.angular.y**2 + twist.twist.angular.z**2)
        if ang_mag > MAX_ANGULAR_VEL:
            scale = MAX_ANGULAR_VEL / ang_mag
            twist.twist.angular.x *= scale
            twist.twist.angular.y *= scale
            twist.twist.angular.z *= scale
        
        self.pub.publish(twist)
        
    def quaternion_normalize(self, q: Quaternion) -> Quaternion:
        norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        if norm == 0.0: return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        return Quaternion(x=q.x/norm, y=q.y/norm, z=q.z/norm, w=q.w/norm)

    def quaternion_inverse(self, q: Quaternion) -> Quaternion:
        return Quaternion(x=-q.x, y=-q.y, z=-q.z, w=q.w)

    def quaternion_multiply(self, q1: Quaternion, q2: Quaternion) -> Quaternion:
        return Quaternion(
            x=q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
            y=q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x,
            z=q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w,
            w=q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z
        )

    def quat_error_to_rotation_vector(self, current_q: Quaternion, target_q: Quaternion):
        q_curr_inv = self.quaternion_inverse(current_q)
        q_err = self.quaternion_multiply(target_q, q_curr_inv)
        q_err = self.quaternion_normalize(q_err)

        w = q_err.w
        if abs(w) > 1.0: w = math.copysign(1.0, w)
        angle = 2.0 * math.acos(w)
        
        if angle > math.pi:
            angle = 2.0 * math.pi - angle
            q_err.x = -q_err.x
            q_err.y = -q_err.y
            q_err.z = -q_err.z
            
        sin_half = math.sqrt(1.0 - w*w)
        if sin_half < 1e-6: return [0.0, 0.0, 0.0], 0.0
        
        scale = angle / sin_half
        return [q_err.x*scale, q_err.y*scale, q_err.z*scale], angle
        
    def publish_zero_twist(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = BASE_LINK
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor() 
    node = WaypointServoingNode()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_zero_twist()
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()