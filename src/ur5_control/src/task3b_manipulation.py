#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import String, Bool
from tf2_ros import TransformException, TransformStamped, Buffer, TransformListener
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from linkattacher_msgs.srv import AttachLink, DetachLink
import math
import time

# --- CONSTANTS ---
LINEAR_P_GAIN = 1.0   
ANGULAR_P_GAIN = 1.2 
MAX_LINEAR_VEL = 0.7  
MAX_ANGULAR_VEL = 1.0 
POS_TOLERANCE = 0.05
ORI_TOLERANCE = 0.15  
WAYPOINT_HOLD_TIME = 1.0 

EE_LINK = "wrist_3_link"
BASE_LINK = "base_link"
TEAM_ID = 2203 
FERTILISER_FRAME = f'{TEAM_ID}_fertilizer_1' 
EBOT_DROP_FRAME = f'{TEAM_ID}_ebot_base'

# --- WAYPOINTS SEQUENCE ---
WAYPOINTS = [
    # 1. Home Pose
    { 'name': 'P1_HOME', 'pos': [-0.218, -0.547, 0.632], 'ori': [0.707, 0.028, 0.034, 0.707], 'action': 'NONE' },
    
    # 2. Pick Up (Uses TF from Vision)
    { 'name': 'GRASP', 'target_frame': FERTILISER_FRAME, 'z_offset': 0.02, 'action': 'ATTACH'},
    
    # 3. Retreat
    { 'name': 'RETREAT', 'pos': [0.2, -0.200, 0.632], 'ori': [0.707, 0.028, 0.034, 0.707], 'action': 'NONE' },
    
    # 4. Approach eBot
    { 'name': 'APPROACH_EBOT', 'target_frame': EBOT_DROP_FRAME, 'z_offset': 0.30, 'action': 'NONE'},
    
    # 5. Place on eBot
    { 'name': 'DROP_ON_EBOT', 'target_frame': EBOT_DROP_FRAME, 'z_offset': 0.10, 'action': 'DETACH'},
]

class WaypointServoingNode(Node):
    
    def __init__(self):
        super().__init__('ur5_waypoint_servo_node')
        self.cb_group = ReentrantCallbackGroup() 
        
        # --- LOGIC FLAGS ---
        self.is_started = False       # Triggered by Vision Node
        self.task_complete = False    # Ensures we publish completion only once

        # --- COMMUNICATIONS ---
        self.pub = self.create_publisher(Twist, '/delta_twist_cmds', 10)
        
        # PUBLISHER for Requirement: "after placing publish a topic fertilizer can recieved"
        self.completion_pub = self.create_publisher(String, '/fertilizer_can_received', 10)

        # TRIGGER SUBSCRIBER
        self.create_subscription(Bool, '/task3b/start_loading', self.start_callback, 10, callback_group=self.cb_group)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Service Clients
        self.attach_client = self.create_client(AttachLink, '/attach_link', callback_group=self.cb_group)
        self.detach_client = self.create_client(DetachLink, '/detach_link', callback_group=self.cb_group)
        
        self.timer = self.create_timer(0.02, self.control_loop, callback_group=self.cb_group)

        self.waypoints = WAYPOINTS
        self.current_waypoint_index = 0
        self.is_holding = False
        self.hold_start_time = 0.0
        
        self.get_logger().info("Manipulator Standby. Waiting for trigger...")

    def start_callback(self, msg):
        """Activates the manipulation sequence."""
        if msg.data and not self.is_started:
            self.get_logger().info("🚀 Trigger Received! Starting Loading Sequence.")
            self.is_started = True

    def control_loop(self):
        # 1. Wait for Start
        if not self.is_started: return
        
        # 2. Check Completion
        if self.current_waypoint_index >= len(self.waypoints):
            if not self.task_complete:
                self.get_logger().info("✅ Sequence Complete. Publishing to /fertilizer_can_received")
                
                # FINAL REQUIREMENT: Publish completion message
                msg = String()
                msg.data = "FERTILIZER_LOADED"
                self.completion_pub.publish(msg)
                
                self.task_complete = True
                self.pub.publish(Twist()) # Stop robot
            return

        # 3. Handle Holding (Wait State)
        if self.is_holding:
            if (self.get_clock().now().nanoseconds / 1e9 - self.hold_start_time) < WAYPOINT_HOLD_TIME:
                self.pub.publish(Twist())
                return
            else:
                self.is_holding = False
                self.current_waypoint_index += 1
                return

        # 4. Servoing Logic
        target_wp = self.waypoints[self.current_waypoint_index]
        target_p, target_q = self.get_target_pose(target_wp)
        
        if target_p is None:
            self.pub.publish(Twist()) # Stop if TF not found
            return

        try:
            current_tf = self.tf_buffer.lookup_transform(BASE_LINK, EE_LINK, rclpy.time.Time())
            curr_p = current_tf.transform.translation
            curr_q = current_tf.transform.rotation
            
            # Error Calc
            err_pos = [target_p[0]-curr_p.x, target_p[1]-curr_p.y, target_p[2]-curr_p.z]
            err_pos_mag = math.sqrt(sum(e**2 for e in err_pos))
            
            t_quat = Quaternion(x=target_q[0], y=target_q[1], z=target_q[2], w=target_q[3])
            err_ori_vec, err_ori_ang = self.quat_error(curr_q, t_quat)
            
            # Check Tolerance
            pos_tol = 0.08 if target_wp['name'] == 'DROP_ON_EBOT' else POS_TOLERANCE
            
            if err_pos_mag <= pos_tol and err_ori_ang <= ORI_TOLERANCE:
                self.get_logger().info(f"Reached {target_wp['name']}")
                self.pub.publish(Twist())
                
                # Perform Action
                if target_wp['action'] == 'ATTACH': self.handle_gripper('attach')
                elif target_wp['action'] == 'DETACH': self.handle_gripper('detach')
                
                self.is_holding = True
                self.hold_start_time = self.get_clock().now().nanoseconds / 1e9
                return
            
            # Move Robot
            cmd = Twist()
            cmd.linear.x = err_pos[0] * LINEAR_P_GAIN
            cmd.linear.y = err_pos[1] * LINEAR_P_GAIN
            cmd.linear.z = err_pos[2] * LINEAR_P_GAIN
            cmd.angular.x = err_ori_vec[0] * ANGULAR_P_GAIN
            cmd.angular.y = err_ori_vec[1] * ANGULAR_P_GAIN
            cmd.angular.z = err_ori_vec[2] * ANGULAR_P_GAIN
            
            # Velocity Clamping
            l_mag = math.sqrt(cmd.linear.x**2 + cmd.linear.y**2 + cmd.linear.z**2)
            if l_mag > MAX_LINEAR_VEL:
                r = MAX_LINEAR_VEL / l_mag
                cmd.linear.x *= r; cmd.linear.y *= r; cmd.linear.z *= r
            
            self.pub.publish(cmd)
            
        except TransformException: pass

    # --- HELPERS ---
    def get_target_pose(self, wp):
        if 'target_frame' in wp:
            try:
                tf = self.tf_buffer.lookup_transform(BASE_LINK, wp['target_frame'], rclpy.time.Time())
                return ([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z + wp.get('z_offset',0)], 
                        [tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w])
            except TransformException: return None, None
        return wp['pos'], wp['ori']

    def handle_gripper(self, mode):
        req = AttachLink.Request() if mode == 'attach' else DetachLink.Request()
        req.model1_name = 'fertiliser_can'
        req.link1_name = 'body'
        req.model2_name = 'ur5'
        req.link2_name = EE_LINK
        
        client = self.attach_client if mode == 'attach' else self.detach_client
        client.call_async(req)
        self.get_logger().info(f"Gripper Action: {mode.upper()}")

    def quat_error(self, q_curr, q_tgt):
        # Inverse Current
        q_inv = Quaternion(x=-q_curr.x, y=-q_curr.y, z=-q_curr.z, w=q_curr.w)
        # Multiply Target * Inverse
        q_err = Quaternion(
            x=q_tgt.w*q_inv.x + q_tgt.x*q_inv.w + q_tgt.y*q_inv.z - q_tgt.z*q_inv.y,
            y=q_tgt.w*q_inv.y - q_tgt.x*q_inv.z + q_tgt.y*q_inv.w + q_tgt.z*q_inv.x,
            z=q_tgt.w*q_inv.z + q_tgt.x*q_inv.y - q_tgt.y*q_inv.x + q_tgt.z*q_inv.w,
            w=q_tgt.w*q_inv.w - q_tgt.x*q_inv.x - q_tgt.y*q_inv.y - q_tgt.z*q_inv.z
        )
        # Normalize
        norm = math.sqrt(q_err.x**2 + q_err.y**2 + q_err.z**2 + q_err.w**2)
        q_err.w /= norm; q_err.x /= norm; q_err.y /= norm; q_err.z /= norm
        
        w = q_err.w
        if abs(w) > 1.0: w = 1.0 if w > 0 else -1.0
        angle = 2.0 * math.acos(w)
        if angle > math.pi:
            angle = 2.0 * math.pi - angle
            q_err.x = -q_err.x; q_err.y = -q_err.y; q_err.z = -q_err.z
            
        sin_half = math.sqrt(1.0 - w*w)
        if sin_half < 1e-6: return [0,0,0], 0
        s = angle/sin_half
        return [q_err.x*s, q_err.y*s, q_err.z*s], angle

def main(args=None):
    rclpy.init(args=args)
    node = WaypointServoingNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()