#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image, JointState, CameraInfo
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
# NOTE: Ensure the Fertilizer ID matches the physical can on the remote setup (usually ID 3)
FERTILIZER_ARUCO_ID = 3 

FRAME_NAMES = {
    # Requirement: <team_id>_fertilizer_1
    FERTILIZER_ARUCO_ID: f'{TEAM_ID}_fertilizer_1'
}

# Configuration for Robust Locking
CONFIDENCE_THRESHOLD = 5 # Number of consecutive detections required to lock the ArUco TF

class VisionAndTFPublisher(Node):
    def __init__(self):
        super().__init__('vision_and_tf_publisher')
        self.bridge = CvBridge()
        self.cv_image = None
        self.depth_image = None
        self.camera_info_received = False

        # Placeholder Camera Intrinsics (Will be overwritten by CameraInfo topic)
        self.centerCamX = 640.0
        self.centerCamY = 360.0
        self.focalX = 640.0
        self.focalY = 640.0
        self.camera_matrix = np.eye(3)
        self.dist_coeffs = np.zeros((5, 1))
        self.marker_size = 0.13 # Size of the ArUco marker in meters
        
        # ArUco Setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        # Tune detection parameters
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
        
        self.locked_fruit_tfs = {}  # {fruit_id: TransformStamped}
        self.fruit_detection_confidence = {}  # {fruit_id: counter}
        self.fruit_detection_positions = {}  # {fruit_id: [(x, y, z), ...]}
        
        self.robot_is_moving = False
        self.robot_initial_joint_positions = None
        self.movement_threshold = 0.01
        
        # --- SUBSCRIPTIONS (UPDATED FOR REMOTE SETUP) ---
        
        # 1. Camera Info (Crucial for accurate 3D projection)
        self.create_subscription(
            CameraInfo, 
            '/camera/camera/color/camera_info', 
            self.camera_info_callback, 
            10, 
            callback_group=self.cb_group
        )

        # 2. Color Image
        self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.colorimagecb, 
            10, 
            callback_group=self.cb_group
        )
        
        # 3. Aligned Depth Image
        self.create_subscription(
            Image, 
            '/camera/depth/image_raw', 
            self.depthimagecb, 
            10, 
            callback_group=self.cb_group
        )
        
        # 4. Joint States
        self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10, 
            callback_group=self.cb_group
        )
        
        # Main loop timer 
        self.create_timer(0.1, self.process_vision_and_tf, callback_group=self.cb_group)

        if SHOW_IMAGE:
            cv2.namedWindow('Vision and TF Publisher', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Vision and TF Publisher', 1280, 720)
        
        self.get_logger().info(f"Vision Node Started. Waiting for Camera Info...")

    ## --- CALLBACKS ---
    
    def camera_info_callback(self, msg):
        """Updates camera intrinsics from the actual hardware."""
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.focalX = self.camera_matrix[0, 0]
            self.focalY = self.camera_matrix[1, 1]
            self.centerCamX = self.camera_matrix[0, 2]
            self.centerCamY = self.camera_matrix[1, 2]
            self.camera_info_received = True
            self.get_logger().info(f"Camera Info Received. FX: {self.focalX}, CX: {self.centerCamX}")

    def colorimagecb(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting color image: {e}')

    def depthimagecb(self, data):
        try:
            # Use 'passthrough' for 16-bit encoding
            depth_img = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            if depth_img.dtype == np.uint16:
                # Convert mm (uint16) to meters (float32)
                self.depth_image = depth_img.astype(np.float32) / 1000.0 
            else:
                self.depth_image = depth_img
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')
    
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
                    self.get_logger().info('Robot moving. Detection paused.')
        except Exception as e:
            pass
    
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
            if (rotation_matrix[0, 0] > rotation_matrix[1, 1]) and (rotation_matrix[0, 0] > rotation_matrix[2, 2]):
                s = math.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2]) * 2
                q.w = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
                q.x = 0.25 * s
                q.y = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                q.z = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
            elif (rotation_matrix[1, 1] > rotation_matrix[2, 2]):
                s = math.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2]) * 2
                q.w = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
                q.x = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                q.y = 0.25 * s
                q.z = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
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
        if image is None or not self.camera_info_received: return detected_markers
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            if ids is None or len(ids) == 0: return detected_markers
            
            # Use dynamic camera matrix
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
        
    def get_depth_at_point(self, x, y):
        if self.depth_image is None: return None
        try:
            height, width = self.depth_image.shape
            x = max(0, min(x, width - 1))
            y = max(0, min(y, height - 1))
            depth_value = self.depth_image[y, x]
            if depth_value <= 0.0 or np.isnan(depth_value): return None
            return float(depth_value)
        except Exception:
            return None
            
    def compute_3d_position(self, pixel_x, pixel_y, depth):
        z_depth = float(depth)
        # Use dynamic intrinsics
        x_optical = z_depth * (pixel_x - self.centerCamX) / self.focalX
        y_optical = z_depth * (pixel_y - self.centerCamY) / self.focalY
        
        pos_x_cam = z_depth    
        pos_y_cam = -x_optical 
        pos_z_cam = -y_optical 
        return pos_x_cam, pos_y_cam, pos_z_cam

    def bad_fruit_detection(self, rgb_image):
        bad_fruits = []
        if rgb_image is None or not self.camera_info_received: return bad_fruits

        try:
            # ROI Mask (Center of image)
            height, width = rgb_image.shape[:2]
            tray_mask = np.zeros((height, width), dtype=np.uint8)
            tray_mask[height//4:3*height//4, 50:width//2] = 255
            
            hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
            
            # Combined mask for bad fruits (browns, greys, darks)
            brown_mask = cv2.inRange(hsv_image, np.array([5, 50, 20]), np.array([25, 255, 100]))
            grey_mask = cv2.inRange(hsv_image, np.array([0, 0, 100]), np.array([180, 50, 255]))
            dark_mask = cv2.inRange(hsv_image, np.array([0, 0, 0]), np.array([180, 255, 60]))
            
            final_mask = cv2.bitwise_or(brown_mask, grey_mask)
            final_mask = cv2.bitwise_or(final_mask, dark_mask)
            final_mask = cv2.bitwise_and(final_mask, tray_mask)
            
            # Cleanup
            kernel = np.ones((5, 5), np.uint8)
            final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_CLOSE, kernel)
            final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_OPEN, kernel)
            
            contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Use detected_fruit_ids_in_frame to manage IDs dynamically in main loop
            for contour in contours:
                area = cv2.contourArea(contour)
                if 800 < area < 25000:
                    x, y, w, h = cv2.boundingRect(contour)
                    aspect_ratio = w / h if h > 0 else 0
                    if 0.5 < aspect_ratio < 2.0:
                        center_x = x + w // 2
                        center_y = y + h // 2
                        depth = self.get_depth_at_point(center_x, center_y)
                        
                        if depth is None or depth == 0: depth = 0.5 
                        
                        bad_fruits.append({
                            'center': (center_x, center_y),
                            'depth': depth,
                            'bbox': (x, y, w, h),
                            'contour': contour,
                        })
        except Exception as e:
            self.get_logger().error(f'Error detection: {e}')
        
        return bad_fruits

    def lock_tf(self, child_frame, pos_x, pos_y, pos_z, q=None):
        """Generic TF Locker for both Fruits and ArUco"""
        try:
            current_stamp = self.get_clock().now().to_msg()
            
            cam_pt = PointStamped()
            cam_pt.header.frame_id = 'camera_link'
            cam_pt.header.stamp = current_stamp
            cam_pt.point.x = pos_x
            cam_pt.point.y = pos_y
            cam_pt.point.z = pos_z
            
            camera_to_base = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
            base_pt = do_transform_point(cam_pt, camera_to_base)
            
            tf_msg = TransformStamped()
            tf_msg.header.frame_id = 'base_link'
            tf_msg.child_frame_id = child_frame
            tf_msg.transform.translation.x = base_pt.point.x
            tf_msg.transform.translation.y = base_pt.point.y
            tf_msg.transform.translation.z = base_pt.point.z
            
            if q:
                tf_msg.transform.rotation = q
            else:
                # Default rotation for fruits (pointing down)
                tf_msg.transform.rotation.x = 0.0
                tf_msg.transform.rotation.y = 1.0
                tf_msg.transform.rotation.z = 0.0
                tf_msg.transform.rotation.w = 0.0
                
            return tf_msg
        except Exception as e:
            self.get_logger().warn(f"TF Lookup Failed: {e}")
            return None

    def process_vision_and_tf(self):
        if self.cv_image is None or not self.camera_info_received:
            return

        current_stamp = self.get_clock().now().to_msg()
        display_image = self.cv_image.copy()
        
        # --- 1. Fertilizer (ArUco) Detection ---
        detected_ids_in_frame = set()
        aruco_markers = self.detect_aruco_markers(self.cv_image)
        
        for marker in aruco_markers:
            marker_id = marker['id']
            if marker_id not in FRAME_NAMES: continue

            detected_ids_in_frame.add(marker_id)
            
            # Draw
            cv2.aruco.drawDetectedMarkers(display_image, [marker['corners']], borderColor=(0, 255, 255))
            
            # Lock Logic
            if self.locked_tfs[marker_id] is None:
                self.detection_confidence_counter[marker_id] += 1
                if self.detection_confidence_counter[marker_id] >= CONFIDENCE_THRESHOLD:
                    # Calculate position
                    aruco_tf_optical = self.rvec_tvec_to_transform(marker['rvec'], marker['tvec'])
                    x_opt = aruco_tf_optical.transform.translation.x
                    y_opt = aruco_tf_optical.transform.translation.y
                    z_opt = aruco_tf_optical.transform.translation.z
                    
                    # Optical to Link Frame
                    x_link, y_link, z_link = z_opt, -x_opt, -y_opt
                    
                    # Fertilizer specific rotation
                    q_fixed = Quaternion(x=0.5, y=0.5, z=-0.5, w=0.5) 
                    
                    tf_msg = self.lock_tf(FRAME_NAMES[marker_id], x_link, y_link, z_link, q_fixed)
                    if tf_msg:
                        self.locked_tfs[marker_id] = tf_msg
                        self.get_logger().info(f"LOCKED {FRAME_NAMES[marker_id]}")

        # Broadcast Locked ArUco
        for marker_id, tf_msg in self.locked_tfs.items():
            if tf_msg:
                tf_msg.header.stamp = current_stamp
                self.tf_broadcaster.sendTransform(tf_msg)

        # --- 2. Bad Fruit Detection ---
        bad_fruits = self.bad_fruit_detection(self.cv_image)
        
        # Simple tracking matching based on distance
        detected_fruit_ids_in_frame = set()
        
        for fruit in bad_fruits:
            cx, cy = fruit['center']
            depth = fruit['depth']
            px, py, pz = self.compute_3d_position(cx, cy, depth)
            
            matched_id = None
            
            # Match against existing trackers
            for fid, positions in self.fruit_detection_positions.items():
                if positions:
                    last_pos = positions[-1]
                    dist = math.sqrt((px-last_pos[0])**2 + (py-last_pos[1])**2 + (pz-last_pos[2])**2)
                    if dist < 0.15: # 15cm threshold
                        matched_id = fid
                        break
            
            if matched_id is None and not self.robot_is_moving:
                # Assign new ID
                existing = list(self.fruit_detection_positions.keys())
                matched_id = max(existing) + 1 if existing else 1
                self.fruit_detection_positions[matched_id] = []
                self.fruit_detection_confidence[matched_id] = 0
            
            if matched_id is not None:
                detected_fruit_ids_in_frame.add(matched_id)
                self.fruit_detection_positions[matched_id].append((px, py, pz))
                if len(self.fruit_detection_positions[matched_id]) > 10: self.fruit_detection_positions[matched_id].pop(0)

                # Visualization
                x, y, w, h = fruit['bbox']
                label = f"Bad Fruit {matched_id}"
                cv2.rectangle(display_image, (x, y), (x+w, y+h), (0,0,255), 2)
                cv2.putText(display_image, label, (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)

                # Lock Logic
                if matched_id not in self.locked_fruit_tfs:
                    self.fruit_detection_confidence[matched_id] += 1
                    if self.fruit_detection_confidence[matched_id] >= CONFIDENCE_THRESHOLD:
                        frame_name = f"{TEAM_ID}_bad_fruit_{matched_id}"
                        tf_msg = self.lock_tf(frame_name, px, py, pz)
                        if tf_msg:
                            self.locked_fruit_tfs[matched_id] = tf_msg
                            self.get_logger().info(f"LOCKED {frame_name}")

        # Broadcast Locked Fruits
        for fid, tf_msg in self.locked_fruit_tfs.items():
            if tf_msg:
                tf_msg.header.stamp = current_stamp
                self.tf_broadcaster.sendTransform(tf_msg)

        if SHOW_IMAGE:
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