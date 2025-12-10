#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String  # <--- Added Import
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped, PointStamped, Quaternion
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math

try:
    from tf2_geometry_msgs import do_transform_point
except ImportError:
    do_transform_point = None

SHOW_IMAGE = True
TEAM_ID = 2203 

# Marker IDs and Frame Names for ArUco
FERTILIZER_ARUCO_ID = 3 
FERTILIZER_BASE_ARUCO_ID = 6 
FRAME_NAMES = {
    FERTILIZER_ARUCO_ID: f'{TEAM_ID}_fertiliser_can',
    FERTILIZER_BASE_ARUCO_ID: f'{TEAM_ID}_ebot_base'
}

# Configuration for Robust Locking
CONFIDENCE_THRESHOLD = 5 

class VisionAndTFPublisher(Node):
    def __init__(self):
        super().__init__('vision_and_tf_publisher')
        self.bridge = CvBridge()
        self.cv_image = None
        self.depth_image = None

        # --- NEW: State Management ---
        self.vision_active = False # Logic starts Disabled
        # -----------------------------

        # Camera Intrinsics 
        self.centerCamX = 642.724365234375
        self.centerCamY = 361.9780578613281
        self.focalX = 915.3003540039062
        self.focalY = 914.0320434570312
        self.camera_matrix = np.array([[self.focalX, 0, self.centerCamX], [0, self.focalY, self.centerCamY], [0, 0, 1]])
        self.dist_coeffs = np.zeros((4, 1))
        self.marker_size = 0.13
        
        # ArUco Setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.minMarkerPerimeterRate = 0.03
        self.aruco_params.maxMarkerPerimeterRate = 4.0
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cb_group = ReentrantCallbackGroup() 

        # TF Locking Mechanism
        self.locked_tfs = {id: None for id in FRAME_NAMES.keys()}
        self.detection_confidence_counter = {id: 0 for id in FRAME_NAMES.keys()}
        
        self.locked_fruit_tfs = {} 
        self.fruit_detection_confidence = {} 
        self.fruit_detection_positions = {} 
        
        self.robot_is_moving = False
        self.robot_initial_joint_positions = None
        self.max_locked_fruit_id = 0
        self.movement_threshold = 0.01
        
        # Subscriptions 
        self.create_subscription(
            Image, '/camera/image_raw', self.colorimagecb, 10, callback_group=self.cb_group
        )
        self.create_subscription(
            Image, '/camera/depth/image_raw', self.depthimagecb, 10, callback_group=self.cb_group
        )
        self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10, callback_group=self.cb_group
        )

        # --- NEW: Detection Status Subscriber ---
        self.create_subscription(
            String, '/detection_status', self.detection_status_cb, 10, callback_group=self.cb_group
        )
        # ----------------------------------------
        
        # Main loop timer 
        self.create_timer(0.1, self.process_vision_and_tf, callback_group=self.cb_group)

        if SHOW_IMAGE:
            cv2.namedWindow('Vision and TF Publisher', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Vision and TF Publisher', 1280, 720)
        
        self.get_logger().info(f"VisionAndTFPublisher node started. Waiting for 'DOCK_STATION' trigger...")

    ## --- CALLBACKS ---
    def colorimagecb(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting color image: {e}')

    def depthimagecb(self, data):
        try:
            depth_img = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            if depth_img.dtype == np.uint16:
                self.depth_image = depth_img.astype(np.float32) / 1000.0 
            else:
                self.depth_image = depth_img
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')

    # --- NEW: Trigger Callback ---
    def detection_status_cb(self, msg):
        """Activates vision logic only when correct message is received."""
        if not self.vision_active:
            # Check if message starts with expected string
            if msg.data.startswith("DOCK_STATION"):
                self.vision_active = True
                self.get_logger().info(f"✅ TRIGGER RECEIVED: '{msg.data}'. Vision System ACTIVATED.")
    # -----------------------------
    
    def joint_state_callback(self, msg):
        try:
            if self.robot_initial_joint_positions is None:
                self.robot_initial_joint_positions = list(msg.position)
                return
            
            max_movement = 0.0
            for i, (current_pos, initial_pos) in enumerate(zip(msg.position, self.robot_initial_joint_positions)):
                movement = abs(current_pos - initial_pos)
                max_movement = max(max_movement, movement)
            
            if max_movement > self.movement_threshold:
                if not self.robot_is_moving:
                    self.robot_is_moving = True
                    self.get_logger().info('🤖 Robot movement detected! Stopping new fruit detections.')
                    if self.locked_fruit_tfs:
                        self.max_locked_fruit_id = max(self.locked_fruit_tfs.keys())
        except Exception as e:
            self.get_logger().error(f'Error in joint state callback: {e}')
    
    # --- HELPER METHODS ---
    def rvec_tvec_to_transform(self, rvec, tvec):
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        q = Quaternion()
        trace = rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]
        if trace > 0:
            s = math.sqrt(trace + 1.0) * 2
            q.w = 0.25 * s
            q.x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
            q.y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
            q.z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
        else:
            s = math.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1]) * 2
            q.w = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
            q.x = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
            q.y = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
            q.z = 0.25 * s
        
        transform = TransformStamped()
        transform.transform.translation.x = float(tvec[0][0])
        transform.transform.translation.y = float(tvec[0][1])
        transform.transform.translation.z = float(tvec[0][2])
        transform.transform.rotation = q
        return transform

    def detect_aruco_markers(self, image):
        detected_markers = []
        if image is None: return detected_markers
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            if ids is None or len(ids) == 0: return detected_markers
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )
            for i in range(len(ids)):
                detected_markers.append({
                    'id': int(ids[i][0]),
                    'corners': corners[i],
                    'rvec': rvecs[i],
                    'tvec': tvecs[i]
                })
        except Exception as e:
            self.get_logger().error(f'Error in ArUco detection: {e}')
        return detected_markers
        
    def draw_axes(self, image, rvec, tvec, length=0.07):
        try:
            cv2.drawFrameAxes(
                image, self.camera_matrix, self.dist_coeffs, rvec, tvec, length, thickness=6
            )
        except Exception as e:
            self.get_logger().error(f'Error drawing axes: {e}') 
            
    def get_depth_at_point(self, x, y):
        if self.depth_image is None:
            return None
        try:
            height, width = self.depth_image.shape
            x = max(0, min(x, width - 1))
            y = max(0, min(y, height - 1))
            depth_value = self.depth_image[y, x]
            if depth_value <= 0.0 or np.isnan(depth_value):
                return None
            return float(depth_value)
        except Exception as e:
            self.get_logger().error(f'Error getting depth: {e}')
            return None
            
    def compute_3d_position(self, pixel_x, pixel_y, depth):
        z_depth = float(depth)
        x_optical = z_depth * (pixel_x - self.centerCamX) / self.focalX
        y_optical = z_depth * (pixel_y - self.centerCamY) / self.focalY
        pos_x_cam = z_depth    
        pos_y_cam = -x_optical 
        pos_z_cam = -y_optical 
        return pos_x_cam, pos_y_cam, pos_z_cam

    def create_fruit_mask(self, image):
        height, width = image.shape[:2]
        mask = np.zeros((height, width), dtype=np.uint8)
        tray_x_start = 50
        tray_x_end = width // 2
        tray_y_start = height // 4
        tray_y_end = 3 * height // 4
        mask[tray_y_start:tray_y_end, tray_x_start:tray_x_end] = 255
        return mask

    def bad_fruit_detection(self, rgb_image):
        bad_fruits = []
        if rgb_image is None:
            return bad_fruits

        try:
            tray_mask = self.create_fruit_mask(rgb_image)
            hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
            
            brown_mask = cv2.inRange(hsv_image, np.array([5, 50, 20]), np.array([25, 255, 100]))
            grey_mask = cv2.inRange(hsv_image, np.array([0, 0, 100]), np.array([180, 50, 255]))
            dark_mask = cv2.inRange(hsv_image, np.array([0, 0, 0]), np.array([180, 255, 60]))
            
            combined_mask = cv2.bitwise_or(brown_mask, grey_mask)
            combined_mask = cv2.bitwise_or(combined_mask, dark_mask)
            final_mask = cv2.bitwise_and(combined_mask, tray_mask)
            
            kernel = np.ones((5, 5), np.uint8)
            final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_CLOSE, kernel)
            final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_OPEN, kernel)
            
            contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            fruit_id = 1
            for contour in contours:
                area = cv2.contourArea(contour)
                if 800 < area < 25000:
                    x, y, w, h = cv2.boundingRect(contour)
                    aspect_ratio = w / h if h > 0 else 0
                    if 0.5 < aspect_ratio < 2.0:
                        center_x = x + w // 2
                        center_y = y + h // 2
                        depth = self.get_depth_at_point(center_x, center_y)
                        if depth is None or depth == 0:
                            depth = 0.5 
                        
                        fruit_info = {
                            'id': fruit_id,
                            'center': (center_x, center_y),
                            'depth': depth,
                            'bbox': (x, y, w, h),
                            'contour': contour,
                        }
                        bad_fruits.append(fruit_info)
                        fruit_id += 1
        except Exception as e:
            self.get_logger().error(f'Error in bad fruit detection: {e}')
        return bad_fruits

    def match_fruit_to_tracked(self, pixel_x, pixel_y, depth):
        if not self.locked_fruit_tfs and not self.fruit_detection_positions:
            return None
        
        pos_x, pos_y, pos_z = self.compute_3d_position(pixel_x, pixel_y, depth)
        POSITION_THRESHOLD = 0.10
        
        for fruit_id, positions in self.fruit_detection_positions.items():
            if positions:
                last_pos = positions[-1]
                distance = math.sqrt(
                    (pos_x - last_pos[0])**2 + 
                    (pos_y - last_pos[1])**2 + 
                    (pos_z - last_pos[2])**2
                )
                if distance < POSITION_THRESHOLD:
                    return fruit_id
        return None
    
    def lock_fruit_tf(self, fruit_id, pos_x_cam, pos_y_cam, pos_z_cam):
        try:
            current_stamp = self.get_clock().now().to_msg()
            frame_name = f'{TEAM_ID}_bad_fruit_{fruit_id}'
            
            cam_pt = PointStamped()
            cam_pt.header.frame_id = 'camera_link'
            cam_pt.header.stamp = current_stamp
            cam_pt.point.x = pos_x_cam
            cam_pt.point.y = pos_y_cam
            cam_pt.point.z = pos_z_cam
            
            camera_to_base = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
            base_pt = do_transform_point(cam_pt, camera_to_base)
            
            tf_msg = TransformStamped()
            tf_msg.header.frame_id = 'base_link'
            tf_msg.child_frame_id = frame_name
            tf_msg.transform.translation.x = base_pt.point.x
            tf_msg.transform.translation.y = base_pt.point.y
            tf_msg.transform.translation.z = base_pt.point.z
            tf_msg.transform.rotation.x = 0.0
            tf_msg.transform.rotation.y = 1.0
            tf_msg.transform.rotation.z = 0.0
            tf_msg.transform.rotation.w = 0.0
            
            self.locked_fruit_tfs[fruit_id] = tf_msg
            self.get_logger().info(f'✅ LOCKED {frame_name.upper()}')
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF Error: {e}', throttle_duration_sec=1.0)
            self.fruit_detection_confidence[fruit_id] = 0
        except Exception as e:
            self.get_logger().error(f'Error locking fruit TF: {e}')
            self.fruit_detection_confidence[fruit_id] = 0

    def lock_marker_tf(self, marker):
        marker_id = marker['id']
        rvec = marker['rvec']
        tvec = marker['tvec']
        frame_name = FRAME_NAMES[marker_id]
        
        try:
            current_stamp = self.get_clock().now().to_msg()
            aruco_tf_optical = self.rvec_tvec_to_transform(rvec, tvec)
            x_opt = aruco_tf_optical.transform.translation.x
            y_opt = aruco_tf_optical.transform.translation.y
            z_opt = aruco_tf_optical.transform.translation.z
            
            x_link = z_opt  
            y_link = -x_opt 
            z_link = -y_opt 
            
            cam_pt = PointStamped()
            cam_pt.header.frame_id = 'camera_link'
            cam_pt.header.stamp = current_stamp
            cam_pt.point.x, cam_pt.point.y, cam_pt.point.z = x_link, y_link, z_link

            camera_to_base = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
            base_pt = do_transform_point(cam_pt, camera_to_base)
            
            q_fixed = Quaternion()
            if marker_id == FERTILIZER_ARUCO_ID:
                q_fixed = Quaternion(x=0.5, y=0.5, z=-0.5, w=0.5) 
            elif marker_id == FERTILIZER_BASE_ARUCO_ID:
                q_fixed = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0) 
            
            tf_msg_base = TransformStamped()
            tf_msg_base.header.frame_id = 'base_link'
            tf_msg_base.child_frame_id = frame_name
            tf_msg_base.transform.translation.x = base_pt.point.x
            tf_msg_base.transform.translation.y = base_pt.point.y
            tf_msg_base.transform.translation.z = base_pt.point.z 
            tf_msg_base.transform.rotation = q_fixed
            
            self.locked_tfs[marker_id] = tf_msg_base
            self.get_logger().info(f'✅ LOCKED {frame_name.upper()}')
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF Error: {e}', throttle_duration_sec=1.0)
            self.detection_confidence_counter[marker_id] = 0 
        except Exception as e:
            self.get_logger().error(f'Error locking ArUco TF: {e}')
            self.detection_confidence_counter[marker_id] = 0

    def process_vision_and_tf(self):
        if self.cv_image is None:
            return

        # --- NEW: Trigger Logic Check ---
        if not self.vision_active:
            if SHOW_IMAGE:
                # Show waiting screen if triggered hasn't happened yet
                display_image = self.cv_image.copy()
                cv2.putText(display_image, "WAITING FOR TRIGGER...", (50, 50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.imshow('Vision and TF Publisher', display_image)
                cv2.waitKey(1)
            return
        # --------------------------------

        current_stamp = self.get_clock().now().to_msg()
        display_image = self.cv_image.copy()
        
        # --- 1. ArUco Detection ---
        detected_ids_in_frame = set()
        aruco_markers = self.detect_aruco_markers(self.cv_image)
        
        for marker in aruco_markers:
            marker_id = marker['id']
            if marker_id not in FRAME_NAMES:
                continue
            detected_ids_in_frame.add(marker_id)
            if display_image is not None:
                cv2.aruco.drawDetectedMarkers(display_image, [marker['corners']], borderColor=(0, 255, 255))
                self.draw_axes(display_image, marker['rvec'], marker['tvec'], length=0.07)
            if self.locked_tfs[marker_id] is None:
                self.detection_confidence_counter[marker_id] = min(
                    CONFIDENCE_THRESHOLD, self.detection_confidence_counter[marker_id] + 1
                )
                if self.detection_confidence_counter[marker_id] >= CONFIDENCE_THRESHOLD:
                    self.lock_marker_tf(marker)
        
        for marker_id in self.locked_tfs:
            if self.locked_tfs[marker_id] is None and marker_id not in detected_ids_in_frame:
                self.detection_confidence_counter[marker_id] = 0
                
        for marker_id, tf_msg in self.locked_tfs.items():
            if tf_msg is not None:
                tf_msg.header.stamp = current_stamp
                self.tf_broadcaster.sendTransform(tf_msg)
                
        # --- 2. Bad Fruit Detection ---
        bad_fruits = self.bad_fruit_detection(self.cv_image)
        detected_fruit_ids_in_frame = set()
        
        for fruit in bad_fruits:
            center_x, center_y = fruit['center']
            depth = fruit['depth']
            x, y, w, h = fruit['bbox']
            contour = fruit['contour']
            pos_x_cam, pos_y_cam, pos_z_cam = self.compute_3d_position(center_x, center_y, depth)
            
            matched_fruit_id = self.match_fruit_to_tracked(center_x, center_y, depth)
            
            if matched_fruit_id is None:
                if self.robot_is_moving:
                    continue
                existing_ids = list(self.fruit_detection_confidence.keys())
                new_fruit_id = max(existing_ids) + 1 if existing_ids else 1
                matched_fruit_id = new_fruit_id
                self.fruit_detection_confidence[matched_fruit_id] = 0
                self.fruit_detection_positions[matched_fruit_id] = []
            
            detected_fruit_ids_in_frame.add(matched_fruit_id)
            self.fruit_detection_positions[matched_fruit_id].append((pos_x_cam, pos_y_cam, pos_z_cam))
            if len(self.fruit_detection_positions[matched_fruit_id]) > 10:
                self.fruit_detection_positions[matched_fruit_id].pop(0)
            
            frame_name = f'{TEAM_ID}_bad_fruit_{matched_fruit_id}'
            cv2.drawContours(display_image, [contour], -1, (0, 255, 255), 2)
            cv2.rectangle(display_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(display_image, (center_x, center_y), 5, (0, 0, 255), -1)
            
            if matched_fruit_id in self.locked_fruit_tfs:
                label = f'bad fruit {matched_fruit_id} (LOCKED)'
                color = (0, 255, 255) 
            else:
                conf_count = self.fruit_detection_confidence.get(matched_fruit_id, 0)
                label = f'bad fruit {matched_fruit_id} ({conf_count}/{CONFIDENCE_THRESHOLD})'
                color = (0, 255, 0) 
            
            cv2.putText(display_image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            
            if matched_fruit_id not in self.locked_fruit_tfs:
                self.fruit_detection_confidence[matched_fruit_id] = min(
                    CONFIDENCE_THRESHOLD, 
                    self.fruit_detection_confidence[matched_fruit_id] + 1
                )
                if self.fruit_detection_confidence[matched_fruit_id] >= CONFIDENCE_THRESHOLD:
                    self.lock_fruit_tf(matched_fruit_id, pos_x_cam, pos_y_cam, pos_z_cam)
        
        for fruit_id in list(self.fruit_detection_confidence.keys()):
            if fruit_id not in self.locked_fruit_tfs and fruit_id not in detected_fruit_ids_in_frame:
                self.fruit_detection_confidence[fruit_id] = max(0, self.fruit_detection_confidence[fruit_id] - 1)
                if self.fruit_detection_confidence[fruit_id] == 0:
                    del self.fruit_detection_confidence[fruit_id]
                    if fruit_id in self.fruit_detection_positions:
                        del self.fruit_detection_positions[fruit_id]
        
        for fruit_id, tf_msg in self.locked_fruit_tfs.items():
            if tf_msg is not None:
                tf_msg.header.stamp = current_stamp
                self.tf_broadcaster.sendTransform(tf_msg)
            
        if SHOW_IMAGE and display_image is not None:
            cv2.imshow('Vision and TF Publisher', display_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VisionAndTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        if SHOW_IMAGE:
            cv2.destroyAllWindows()

if __name__ == '__main__':
    main()