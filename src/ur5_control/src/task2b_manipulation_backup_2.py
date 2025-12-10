#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
*****************************************************************************************
*
* ===============================================
* Krishi coBot (KC) Theme (eYRC 2025-26)
* ===============================================
*
* This script implements Task 2B manipulation of Krishi coBot (KC) Theme (eYRC 2025-26).
* * This script:
* 1. Reads TF transforms of detected objects (bad fruits and fertilizer)
* 2. Uses servo control (Task 1C method) to move the UR5 arm
* 3. Picks objects using magnetic gripper (link attacher service)
* 4. Places them at designated locations:
* - Bad fruits -> Back Tray (behind manipulator)
* - Fertilizer -> Fertilizer Plate (shelf with pots)
*
* MODIFICATION: Introduced explicit intermediate waypoints for smoother transitions
* and improved robustness based on the Task 1C approach to prevent the manipulator
* from getting stuck.
*
*****************************************************************************************
'''

# Team ID: 2203

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from tf2_ros import TransformException, TransformStamped, Buffer, TransformListener
from linkattacher_msgs.srv import AttachLink, DetachLink
import math
import time
from typing import List, Dict, Optional


# ========== CONTROL PARAMETERS ==========
LINEAR_P_GAIN = 0.5
ANGULAR_P_GAIN = 0.7

MAX_LINEAR_VEL = 0.4
MAX_ANGULAR_VEL = 0.6

POS_TOLERANCE = 0.08  # 8 cm tolerance for position
ORI_TOLERANCE = 0.15  # Orientation tolerance

APPROACH_OFFSET = 0.15  # Approach from 15 cm above
PICKUP_DISTANCE = 0.08  # Final distance for magnetic attachment (< 0.1m required)

WAYPOINT_HOLD_TIME = 1.0  # Wait time at each waypoint
HOLD_TIME = 0.5  # Hold time for stability after reaching target

# TF Frames
EE_LINK = "tool0"
BASE_LINK = "base_link"
TEAM_ID = 2203

# ========== PREDEFINED LOCATIONS ==========
# Home position
HOME_POSITION = {
    'name': 'HOME',
    'pos': [-0.214, -0.532, 0.557],
    'ori': [0.707, 0.028, 0.034, 0.707]
}

# Intermediate waypoint (safe travel position - high and centered)
INTERMEDIATE_POSITION = {
    'name': 'INTERMEDIATE',
    'pos': [0.200, 0.000, 0.500],  # Centered, high position to avoid limits
    'ori': [0.029, 0.997, 0.045, 0.033]
}

# Safe high position before approaching objects
SAFE_HIGH_POSITION = {
    'name': 'SAFE_HIGH',
    'pos': [0.000, 0.000, 0.600],  # Very high, centered position
    'ori': [0.0, 1.0, 0.0, 0.0]
}

# Intermediate high position for transition after picking and before dropping
# This is a general safe point *between* the pick-up area and the drop-off areas.
TRANSIT_HIGH_POSITION = {
    'name': 'TRANSIT_HIGH',
    'pos': [0.000, 0.000, 0.400],  # Slightly lower than SAFE_HIGH for less joint movement
    'ori': [0.029, 0.997, 0.045, 0.033] # Keep the orientation from the INTERMEDIATE
}

# Back tray location for bad fruits (behind the manipulator)
BACK_TRAY_POSITION = {
    'name': 'BACK_TRAY',
    'pos': [-0.806, 0.010, 0.182],
    'ori': [-0.684, 0.726, 0.05, 0.008]
}

# Fertilizer plate location for fertilizer cans (shelf with pots)
FERTILIZER_PLATE_POSITION = {
    'name': 'FERTILIZER_PLATE',
    'pos': [-0.159, 0.501, 0.415],
    'ori': [0.029, 0.997, 0.045, 0.033]
}


class Task2BManipulation(Node):
    """
    ROS2 Node for UR5 arm manipulation to pick and place objects.
    Uses a simplified waypoint-based control inspired by Task 1C.
    """
    
    def __init__(self):
        super().__init__('task2b_manipulation')
        self.get_logger().info("Task 2B Manipulation Node Started - Team ID: 2203")
        
        # Publisher for arm control
        self.twist_pub = self.create_publisher(Twist, '/delta_twist_cmds', 10)
        
        # TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Service clients for magnetic gripper
        self.attach_client = self.create_client(AttachLink, '/attach_link')
        self.detach_client = self.create_client(DetachLink, '/detach_link')
        
        # Wait for services
        self.get_logger().info("Waiting for gripper services...")
        self.attach_client.wait_for_service(timeout_sec=5.0)
        self.detach_client.wait_for_service(timeout_sec=5.0)
        self.get_logger().info("Gripper services ready!")
        
        # Waypoint navigation state
        self.waypoints = []
        self.current_waypoint_index = 0
        self.target_reached = False
        self.hold_start_time = 0.0
        
        # Scanning control
        self.scan_start_time = self.get_clock().now().nanoseconds / 1e9
        self.has_scanned = False
        self.scan_delay = 5.0  # Wait 5 seconds for detection node to publish transforms
        
        # Control timer
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.control_loop)
    
    # ========== QUATERNION MATH (From Task 1C) ==========
    
    def quaternion_normalize(self, q: Quaternion) -> Quaternion:
        """Normalizes a quaternion."""
        norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        if norm == 0.0:
            return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        return Quaternion(x=q.x / norm, y=q.y / norm, z=q.z / norm, w=q.w / norm)
    
    def quaternion_inverse(self, q: Quaternion) -> Quaternion:
        """Computes quaternion inverse."""
        return Quaternion(x=-q.x, y=-q.y, z=-q.z, w=q.w)
    
    def quaternion_multiply(self, q1: Quaternion, q2: Quaternion) -> Quaternion:
        """Multiplies two quaternions."""
        x1, y1, z1, w1 = q1.x, q1.y, q1.z, q1.w
        x2, y2, z2, w2 = q2.x, q2.y, q2.z, q2.w
        
        return Quaternion(
            x=w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            y=w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            z=w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w=w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        )
    
    def quat_error_to_rotation_vector(self, current_q: Quaternion, target_q: Quaternion):
        """Converts quaternion error to rotation vector for control."""
        q_curr_inv = self.quaternion_inverse(current_q)
        q_error = self.quaternion_multiply(target_q, q_curr_inv)
        q_error = self.quaternion_normalize(q_error)
        
        w = q_error.w
        if abs(w) > 1.0:
            w = math.copysign(1.0, w)
        
        angle = 2.0 * math.acos(w)
        
        if angle < 1e-6:
            return [0.0, 0.0, 0.0], 0.0
        
        if angle > math.pi:
            angle = 2.0 * math.pi - angle
            w = math.cos(angle / 2.0)
            q_error.x = -q_error.x
            q_error.y = -q_error.y
            q_error.z = -q_error.z
        
        sin_half_angle = math.sqrt(1.0 - w * w)
        
        if sin_half_angle < 1e-6:
            return [0.0, 0.0, 0.0], 0.0
        
        scale_factor = angle / sin_half_angle
        rotation_vector = [
            q_error.x * scale_factor,
            q_error.y * scale_factor,
            q_error.z * scale_factor
        ]
        
        return rotation_vector, angle
    
    # ========== GRIPPER CONTROL ==========
    
    def attach_object(self, object_name: str):
        """Attaches object to gripper using magnetic attachment."""
        req = AttachLink.Request()
        req.model1_name = object_name
        req.link1_name = 'body'
        req.model2_name = 'ur5'
        req.link2_name = 'wrist_3_link' 
        
        self.get_logger().info(f'Attaching {object_name}...')
        future = self.attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0) 
        
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'Successfully attached {object_name}')
        else:
            self.get_logger().error(f'Failed to attach {object_name} (Result: {future.result()})')
    
    def detach_object(self, object_name: str):
        """Detaches object from gripper."""
        req = DetachLink.Request()
        req.model1_name = object_name
        req.link1_name = 'body'
        req.model2_name = 'ur5'
        req.link2_name = 'wrist_3_link'
        
        self.get_logger().info(f'Detaching {object_name}...')
        future = self.detach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'Successfully detached {object_name}')
        else:
            self.get_logger().error(f'Failed to detach {object_name} (Result: {future.result()})')
    
    # ========== TF LOOKUP ==========
    
    def get_object_position(self, frame_name: str) -> Optional[Dict]:
        """Gets position and orientation of an object from TF."""
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                BASE_LINK, frame_name, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            pos = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]
            
            ori = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            
            return {'pos': pos, 'ori': ori, 'name': frame_name}
        
        except TransformException:
            return None
    
    # ========== WAYPOINT & TASK GENERATION ==========

    def build_waypoint_sequence(self):
        """Scans for objects and builds a complete waypoint sequence."""
        self.get_logger().info("Building waypoint sequence...")
        
        # Start at home
        self.waypoints.append(HOME_POSITION)

        # Scan for objects
        fertilizer_tasks = []
        fruit_tasks = []
        
        for i in range(10):
            fert_frame = f'{TEAM_ID}_fertilizer_{i}_base'
            fert_pos = self.get_object_position(fert_frame)
            if fert_pos:
                self.get_logger().info(f'Found fertilizer with ArUco ID {i}')
                fertilizer_tasks.append({
                    'object_frame': fert_frame,
                    'object_name': f'fertiliser_can_{i}',
                    'drop_location': FERTILIZER_PLATE_POSITION
                })
        
        for i in range(1, 5):
            fruit_frame = f'{TEAM_ID}_bad_fruit_{i}'
            fruit_pos = self.get_object_position(fruit_frame)
            if fruit_pos:
                self.get_logger().info(f'Found bad fruit {i}')
                fruit_tasks.append({
                    'object_frame': fruit_frame,
                    'object_name': f'bad_fruit_{i}',
                    'drop_location': BACK_TRAY_POSITION
                })
        
        # Combine tasks: Fertilizer FIRST
        all_tasks = fertilizer_tasks + fruit_tasks
        
        if not all_tasks:
            self.get_logger().warn("No objects detected! Moving to HOME.")
            self.waypoints.append(HOME_POSITION)
            return

        # Generate waypoints for each task
        for task in all_tasks:
            obj_pos = self.get_object_position(task['object_frame'])
            if not obj_pos:
                self.get_logger().warn(f"Could not get position for {task['object_name']}, skipping.")
                continue

            # 1. Move to a safe height above the object
            self.waypoints.append({
                'name': f"SAFE_ABOVE_{task['object_name']}",
                'pos': [obj_pos['pos'][0], obj_pos['pos'][1], obj_pos['pos'][2] + APPROACH_OFFSET + 0.1],
                'ori': [0.0, 1.0, 0.0, 0.0] # Gripper down
            })

            # 2. Approach the object
            self.waypoints.append({
                'name': f"APPROACH_{task['object_name']}",
                'pos': [obj_pos['pos'][0], obj_pos['pos'][1], obj_pos['pos'][2] + APPROACH_OFFSET],
                'ori': [0.0, 1.0, 0.0, 0.0]
            })

            # 3. Move to final pickup position
            self.waypoints.append({
                'name': f"PICK_{task['object_name']}",
                'pos': [obj_pos['pos'][0], obj_pos['pos'][1], obj_pos['pos'][2] + PICKUP_DISTANCE],
                'ori': [0.0, 1.0, 0.0, 0.0],
                'action': 'attach',
                'object_name': task['object_name']
            })

            # 4. Lift the object to a safe height
            self.waypoints.append(SAFE_HIGH_POSITION)

            # 5. Move to the drop location
            drop_location = task['drop_location']
            self.waypoints.append({
                'name': f"DROP_{task['object_name']}",
                'pos': drop_location['pos'],
                'ori': drop_location['ori'],
                'action': 'detach',
                'object_name': task['object_name']
            })

            # 6. Retreat to
            return False
        
        # Calculate position error
        current_p = current_pose_tf.transform.translation
        target_p = target_waypoint['pos']
        
        pos_error_vector = [
            target_p[0] - current_p.x,
            target_p[1] - current_p.y,
            target_p[2] - current_p.z
        ]
        pos_error_magnitude = math.sqrt(sum(e**2 for e in pos_error_vector))
        
        # Calculate orientation error
        current_q = current_pose_tf.transform.rotation
        target_q_list = target_waypoint['ori']
        target_q = Quaternion(
            x=target_q_list[0], y=target_q_list[1],
            z=target_q_list[2], w=target_q_list[3]
        )
        
        ori_error_vector, ori_error_angle = self.quat_error_to_rotation_vector(current_q, target_q)
        
        # Check if reached
        if pos_error_magnitude <= POS_TOLERANCE and ori_error_angle <= ORI_TOLERANCE:
            self.get_logger().info(
                f"Reached {target_waypoint['name']} | "
                f"P_Err: {pos_error_magnitude:.3f}m, R_Err: {ori_error_angle:.3f}rad"
            )
            self.publish_zero_twist()
            return True
        
        # Generate control command
        twist_cmd = Twist()
        
        # Linear velocity (P-Control)
        twist_cmd.linear.x = pos_error_vector[0] * LINEAR_P_GAIN
        twist_cmd.linear.y = pos_error_vector[1] * LINEAR_P_GAIN
        twist_cmd.linear.z = pos_error_vector[2] * LINEAR_P_GAIN
        
        # Angular velocity (P-Control)
        twist_cmd.angular.x = ori_error_vector[0] * ANGULAR_P_GAIN
        twist_cmd.angular.y = ori_error_vector[1] * ANGULAR_P_GAIN
        twist_cmd.angular.z = ori_error_vector[2] * ANGULAR_P_GAIN
        
        # Clamp linear velocity
        linear_vel_mag = math.sqrt(
            twist_cmd.linear.x**2 + twist_cmd.linear.y**2 + twist_cmd.linear.z**2
        )
        if linear_vel_mag > MAX_LINEAR_VEL:
            ratio = MAX_LINEAR_VEL / linear_vel_mag
            twist_cmd.linear.x *= ratio
            twist_cmd.linear.y *= ratio
            twist_cmd.linear.z *= ratio
        
        # Clamp angular velocity
        angular_vel_mag = math.sqrt(
            twist_cmd.angular.x**2 + twist_cmd.angular.y**2 + twist_cmd.angular.z**2
        )
        if angular_vel_mag > MAX_ANGULAR_VEL:
            ratio = MAX_ANGULAR_VEL / angular_vel_mag
            twist_cmd.angular.x *= ratio
            twist_cmd.angular.y *= ratio
            twist_cmd.angular.z *= ratio
        
        self.twist_pub.publish(twist_cmd)
        return False
    
    def publish_zero_twist(self):
        """Stops the arm."""
        self.twist_pub.publish(Twist())
    
    # ========== STATE MACHINE ==========
    
    def control_loop(self):
        """Main control loop implementing state machine."""
        
        # INIT state - wait for object scan
        if self.current_state == 'INIT':
            if not self.has_scanned:
                current_time = self.get_clock().now().nanoseconds / 1e9
                time_elapsed = current_time - self.scan_start_time
                
                if time_elapsed >= self.scan_delay:
                    # Move to HOME first before scanning to ensure good detection angle
                    self.get_logger().info("Initializing: Moving to HOME position for scanning...")
                    self.current_target = HOME_POSITION
                    self.current_state = 'MOVE_TO_HOME_FOR_SCAN'
                else:
                    # Log waiting status every second
                    time_remaining = self.scan_delay - time_elapsed
                    self.get_logger().info(f"Waiting {time_remaining:.1f}s before initialization...", throttle_duration_sec=1.0)
            self.publish_zero_twist()
            return

        # MOVE_TO_HOME_FOR_SCAN
        if self.current_state == 'MOVE_TO_HOME_FOR_SCAN':
            if self.move_to_target(self.current_target):
                self.get_logger().info("Reached home, starting scan.")
                self.scan_for_objects()
                self.has_scanned = True
                # If no objects found, scan_for_objects will transition to RETURN_HOME, 
                # otherwise it transitions to EXECUTE_TASK.
            return
        
        # HOLDING state - wait at waypoint
        if self.is_holding:
            time_elapsed = self.get_clock().now().nanoseconds / 1e9 - self.hold_start_time
            if time_elapsed < WAYPOINT_HOLD_TIME:
                self.publish_zero_twist()
                return
            else:
                self.is_holding = False
                self.get_logger().info(f"Finished holding, resuming state machine.")
                return
        
        # EXECUTE_TASK - start new pick and place task
        if self.current_state == 'EXECUTE_TASK':
            if self.current_task_index >= len(self.task_queue):
                self.current_state = 'RETURN_HOME'
                return
            
            task = self.task_queue[self.current_task_index]
            self.get_logger().info(f"Starting task {self.current_task_index + 1}/{len(self.task_queue)}")
            self.get_logger().info(f"Object: {task['object_name']} at {task['object_frame']}")
            
            # Get object position from TF one last time before starting the move sequence
            obj_pos = self.get_object_position(task['object_frame'])
            if obj_pos is None:
                self.get_logger().error(f"Cannot find object {task['object_frame']} - skipping task.")
                self.current_task_index += 1
                return
            
            # First, move to a high, safe intermediate position (INTERMEDIATE_POSITION)
            self.current_target = INTERMEDIATE_POSITION
            self.current_state = 'MOVE_TO_INTERMEDIATE'
            return

        # MOVE_TO_INTERMEDIATE - safe travel to avoid obstacles/limits
        if self.current_state == 'MOVE_TO_INTERMEDIATE':
            if self.move_to_target(self.current_target):
                self.get_logger().info("Reached INTERMEDIATE, moving to SAFE_HIGH next.")
                # Now set the target to SAFE_HIGH, above the object
                self.current_target = SAFE_HIGH_POSITION
                self.current_state = 'MOVE_TO_SAFE_HIGH_FOR_PICK'
                self.is_holding = True
                self.hold_start_time = self.get_clock().now().nanoseconds / 1e9
            return
        
        # MOVE_TO_SAFE_HIGH_FOR_PICK - move to high position to prepare for vertical approach
        if self.current_state == 'MOVE_TO_SAFE_HIGH_FOR_PICK':
            if self.move_to_target(self.current_target):
                self.get_logger().info("Reached SAFE_HIGH, preparing to approach object.")
                # After reaching safe high, set up approach waypoint and transition
                task = self.task_queue[self.current_task_index]
                obj_pos = self.get_object_position(task['object_frame'])
                if obj_pos:
                    # Create approach waypoint (above object)
                    self.current_target = {
                        'name': f"APPROACH_{task['object_name']}",
                        'pos': [
                            obj_pos['pos'][0],
                            obj_pos['pos'][1],
                            obj_pos['pos'][2] + APPROACH_OFFSET
                        ],
                        'ori': [0.0, 1.0, 0.0, 0.0]  # Gripper pointing down
                    }
                    self.get_logger().info(f"Approaching object at: [{obj_pos['pos'][0]:.3f}, {obj_pos['pos'][1]:.3f}, {obj_pos['pos'][2] + APPROACH_OFFSET:.3f}]")
                    self.current_state = 'APPROACH_OBJECT'
                    self.is_holding = True
                    self.hold_start_time = self.get_clock().now().nanoseconds / 1e9
                else:
                    self.get_logger().error("Lost object tracking for approach! Skipping task.")
                    self.current_task_index += 1
                    self.current_state = 'EXECUTE_TASK'
            return
        
        # APPROACH_OBJECT - move above object
        if self.current_state == 'APPROACH_OBJECT':
            if self.move_to_target(self.current_target):
                self.get_logger().info("Reached APPROACH position, moving to PICK.")
                self.current_state = 'PICK_OBJECT'
                self.is_holding = True
                self.hold_start_time = self.get_clock().now().nanoseconds / 1e9
            return
        
        # PICK_OBJECT - descend and attach
        if self.current_state == 'PICK_OBJECT':
            task = self.task_queue[self.current_task_index]
            obj_pos = self.get_object_position(task['object_frame'])
            
            if obj_pos is None:
                self.get_logger().error("Lost object tracking for pickup!")
                self.current_task_index += 1
                self.current_state = 'EXECUTE_TASK'
                return
            
            # Move to final pickup position (PICKUP_DISTANCE above object)
            self.current_target = {
                'name': f"PICKUP_{task['object_name']}",
                'pos': [
                    obj_pos['pos'][0],
                    obj_pos['pos'][1],
                    obj_pos['pos'][2] + PICKUP_DISTANCE
                ],
                'ori': [0.0, 1.0, 0.0, 0.0]
            }
            
            if self.move_to_target(self.current_target):
                self.get_logger().info("Reached pickup point. Attaching...")
                # Add a brief pause before attachment for stability
                self.publish_zero_twist()
                time.sleep(HOLD_TIME) 
                
                # Attach object
                success = self.attach_object(task['object_name'])
                if success:
                    self.current_state = 'LIFT_OBJECT'
                    self.is_holding = True
                    self.hold_start_time = self.get_clock().now().nanoseconds / 1e9
                else:
                    self.get_logger().error("Failed to attach object! Skipping task.")
                    self.current_task_index += 1
                    self.current_state = 'EXECUTE_TASK'
            return
        
        # LIFT_OBJECT - lift up to SAFE_HIGH after picking
        if self.current_state == 'LIFT_OBJECT':
            self.current_target = SAFE_HIGH_POSITION # Lift straight up to safe high
            
            if self.move_to_target(self.current_target):
                self.get_logger().info("Lifted to SAFE_HIGH, moving to TRANSIT_HIGH.")
                # Now transition to the intermediate position for smooth travel to drop zone
                self.current_target = TRANSIT_HIGH_POSITION
                self.current_state = 'MOVE_TO_TRANSIT_HIGH'
                self.is_holding = True
                self.hold_start_time = self.get_clock().now().nanoseconds / 1e9
            return

        # MOVE_TO_TRANSIT_HIGH - safe travel to drop zone
        if self.current_state == 'MOVE_TO_TRANSIT_HIGH':
            if self.move_to_target(self.current_target):
                self.get_logger().info("Reached TRANSIT_HIGH, moving to drop location.")
                # Set target to the drop location
                task = self.task_queue[self.current_task_index]
                self.current_target = task['drop_location']
                self.get_logger().info(f"Drop location: {task['drop_location']['name']}")
                self.current_state = 'MOVE_TO_DROP'
                self.is_holding = True
                self.hold_start_time = self.get_clock().now().nanoseconds / 1e9
            return
        
        # MOVE_TO_DROP - move to drop location
        if self.current_state == 'MOVE_TO_DROP':
            if self.move_to_target(self.current_target):
                self.get_logger().info("Reached drop location, preparing to release object.")
                self.current_state = 'DROP_OBJECT'
                self.is_holding = True
                self.hold_start_time = self.get_clock().now().nanoseconds / 1e9
            return
        
        # DROP_OBJECT - release object
        if self.current_state == 'DROP_OBJECT':
            task = self.task_queue[self.current_task_index]
            
            self.get_logger().info(f"Reached drop point. Detaching {task['object_name']}...")
            # Add a brief pause before detachment for stability
            self.publish_zero_twist()
            time.sleep(HOLD_TIME) 

            if self.attached_object:
                self.detach_object(self.attached_object)
            
            self.get_logger().info(f"Task {self.current_task_index + 1}/{len(self.task_queue)} completed!")
            self.current_task_index += 1
            
            # Transition to intermediate position after dropping
            self.current_target = TRANSIT_HIGH_POSITION
            self.current_state = 'MOVE_TO_TRANSIT_HIGH_AFTER_DROP'
            
            self.is_holding = True
            self.hold_start_time = self.get_clock().now().nanoseconds / 1e9
            return
        
        # MOVE_TO_TRANSIT_HIGH_AFTER_DROP - move away from drop zone
        if self.current_state == 'MOVE_TO_TRANSIT_HIGH_AFTER_DROP':
            if self.move_to_target(self.current_target):
                self.get_logger().info("Cleared drop zone.")
                # Move to next task or return home
                if self.current_task_index < len(self.task_queue):
                    self.get_logger().info(f"Moving to next task ({self.current_task_index + 1}/{len(self.task_queue)})")
                    self.current_state = 'EXECUTE_TASK'
                else:
                    self.get_logger().info("All tasks complete! Returning home.")
                    self.current_state = 'RETURN_HOME'
                
                self.is_holding = True
                self.hold_start_time = self.get_clock().now().nanoseconds / 1e9
            return
        
        # RETURN_HOME - go back to home
        if self.current_state == 'RETURN_HOME':
            self.current_target = HOME_POSITION
            
            if self.move_to_target(self.current_target):
                self.get_logger().info("Returned to home position!")
                self.current_state = 'DONE'
                self.is_holding = True
                self.hold_start_time = self.get_clock().now().nanoseconds / 1e9
            return
        
        # DONE - all tasks completed
        if self.current_state == 'DONE':
            self.publish_zero_twist()
            self.get_logger().info("Task 2B completed successfully!")
            self.timer.cancel()
            return


def main(args=None):
    rclpy.init(args=args)
    node = Task2BManipulation()
    
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
    
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received.')
    finally:
        if rclpy.ok():
            node.publish_zero_twist()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()