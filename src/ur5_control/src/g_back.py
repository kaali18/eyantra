#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
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
    def do_transform_point(point, transform):
        raise NotImplementedError("tf2_geometry_msgs is required for do_transform_point")

SHOW_IMAGE = True
TEAM_ID = 2203 

# Marker IDs
FERTILIZER_ARUCO_ID = 3 
FERTILIZER_BASE_ARUCO_ID = 6 
FRAME_NAMES = {
    FERTILIZER_ARUCO_ID: f'{TEAM_ID}_fertiliser_can',
    FERTILIZER_BASE_ARUCO_ID: f'{TEAM_ID}_ebot_base'
}

# --- CONFIGURATION ---
CONFIDENCE_THRESHOLD = 5 
POSITION_THRESHOLD = 0.08 # 8cm separation threshold

class VisionAndTFPublisher(Node):
    def __init__(self):
        super().__init__('vision_and_tf_publisher')
        self.bridge = CvBridge()
        self.cv_image = None
        self.depth_image = None 

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
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cb_group = ReentrantCallbackGroup() 

        # Locking Mechanism
        self.locked_tfs = {id: None for id in FRAME_NAMES.keys()}
        self.detection_confidence_counter = {id: 0 for id in FRAME_NAMES.keys()}
        self.locked_fruit_tfs = {} 
        self.fruit_detection_confidence = {}
        self.fruit_detection_positions = {} 
        
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.colorimagecb, 10, callback_group=self.cb_group)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10, callback_group=self.cb_group)
        self.create_timer(0.1, self.process_vision_and_tf, callback_group=self.cb_group)

        if SHOW_IMAGE:
            cv2.namedWindow('Vision and TF Publisher', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Vision and TF Publisher', 1280, 720)
        
        self.get_logger().info(f"VisionAndTFPublisher node started. Team ID: {TEAM_ID}")

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
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
            for i in range(len(ids)):
                detected_markers.append({'id': int(ids[i][0]), 'corners': corners[i], 'rvec': rvecs[i], 'tvec': tvecs[i]})
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

    # --------------------------------------------------------------------------------
    # 🌟 BAD FRUIT DETECTION (ROTATION INVARIANT) 🌟
    # --------------------------------------------------------------------------------
    def bad_fruit_detection(self, rgb_image):
        bad_fruits = []
        if rgb_image is None:
            return bad_fruits

        try:
            height, width = rgb_image.shape[:2]

            # 1. Blur to smooth out the "bumpy" texture of the fruit
            blurred = cv2.GaussianBlur(rgb_image, (11, 11), 0)
            hsv_image = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            
            # 2. COLOR SELECTION: PURPLE BODY
            # The bad fruit is distinct because of its purple/grey bumpy body.
            # Real world purple is often desaturated.
            lower_purple = np.array([115, 40, 40])
            upper_purple = np.array([170, 255, 255])
            
            purple_mask = cv2.inRange(hsv_image, lower_purple, upper_purple)
            
            # 3. CRITICAL STEP: EROSION FOR SEPARATION
            # If two fruits are touching, they form one blob. 
            # Erosion shrinks the white blobs. If we shrink them enough, they will disconnect.
            kernel = np.ones((5, 5), np.uint8)
            eroded_mask = cv2.erode(purple_mask, kernel, iterations=3) 
            
            # 4. Cleanup noise
            final_mask = cv2.dilate(eroded_mask, kernel, iterations=1)
            
            if SHOW_IMAGE:
                cv2.imshow('Debug Mask', final_mask)
            
            contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            fruit_id = 1
            for contour in contours:
                area = cv2.contourArea(contour)
                # Area filter: Ignore small noise and massive glare
                if 1500 < area < 30000:
                    x, y, w, h = cv2.boundingRect(contour)
                    aspect_ratio = w / h if h > 0 else 0
                    
                    # Shape filter: Fruits are roughly square/circular
                    if 0.6 < aspect_ratio < 1.6:
                        center_x = x + w // 2
                        center_y = y + h // 2
                        depth = self.get_depth_at_point(center_x, center_y)
                        
                        if depth is None or depth == 0: depth = 0.3 
                        
                        # Valid depth check
                        if 0.1 < depth < 1.5:
                            bad_fruits.append({
                                'id': fruit_id, 
                                'center': (center_x, center_y),
                                'depth': depth,
                                'bbox': (x, y, w, h),
                                'contour': contour,
                            })
                            fruit_id += 1
        except Exception as e:
            self.get_logger().error(f'Error in bad fruit detection: {e}')
        return bad_fruits

    # --- MATCHING LOGIC ---
    def match_fruit_to_tracked(self, pixel_x, pixel_y, depth):
        pos_x, pos_y, pos_z = self.compute_3d_position(pixel_x, pixel_y, depth)
        best_match_id = None
        min_dist = float('inf')

        # Check pre-locked
        for fruit_id, positions in self.fruit_detection_positions.items():
            if positions:
                last_pos = positions[-1] 
                dist = math.sqrt((pos_x - last_pos[0])**2 + (pos_y - last_pos[1])**2 + (pos_z - last_pos[2])**2)
                if dist < POSITION_THRESHOLD and dist < min_dist:
                    min_dist = dist
                    best_match_id = fruit_id

        # Check locked
        for fruit_id, tf_msg in self.locked_fruit_tfs.items():
            try:
                base_pt = PointStamped()
                base_pt.header.frame_id = tf_msg.header.frame_id 
                base_pt.header.stamp = rclpy.time.Time().to_msg() 
                base_pt.point.x = tf_msg.transform.translation.x
                base_pt.point.y = tf_msg.transform.translation.y
                base_pt.point.z = tf_msg.transform.translation.z
                base_to_camera = self.tf_buffer.lookup_transform('camera_link', 'base_link', rclpy.time.Time())
                cam_pt_locked = do_transform_point(base_pt, base_to_camera)
                dist_x = pos_x - cam_pt_locked.point.x
                dist_y = pos_y - cam_pt_locked.point.y
                dist_z = pos_z - cam_pt_locked.point.z
                dist = math.sqrt(dist_x**2 + dist_y**2 + dist_z**2)
                if dist < POSITION_THRESHOLD and dist < min_dist:
                    min_dist = dist
                    best_match_id = fruit_id
            except: pass
        return best_match_id
    
    def lock_fruit_tf(self, fruit_id, pos_x_cam, pos_y_cam, pos_z_cam):
        try:
            current_stamp = self.get_clock().now().to_msg()
            frame_name = f'{TEAM_ID}_bad_fruit_{fruit_id}'
            cam_pt = PointStamped()
            cam_pt.header.frame_id = 'camera_link'
            cam_pt.header.stamp = current_stamp
            cam_pt.point.x, cam_pt.point.y, cam_pt.point.z = pos_x_cam, pos_y_cam, pos_z_cam
            camera_to_base = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
            base_pt = do_transform_point(cam_pt, camera_to_base)
            tf_msg = TransformStamped()
            tf_msg.header.frame_id = 'base_link'
            tf_msg.child_frame_id = frame_name
            tf_msg.transform.translation.x = base_pt.point.x
            tf_msg.transform.translation.y = base_pt.point.y
            tf_msg.transform.translation.z = base_pt.point.z
            tf_msg.transform.rotation = Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)
            self.locked_fruit_tfs[fruit_id] = tf_msg
            self.get_logger().info(f'✅ LOCKED {frame_name.upper()}')
        except Exception as e:
            self.get_logger().error(f'Error locking fruit TF: {e}')

    def lock_marker_tf(self, marker):
        marker_id = marker['id']
        frame_name = FRAME_NAMES[marker_id]
        try:
            current_stamp = self.get_clock().now().to_msg()
            aruco_tf_optical = self.rvec_tvec_to_transform(marker['rvec'], marker['tvec'])
            x_opt, y_opt, z_opt = aruco_tf_optical.transform.translation.x, aruco_tf_optical.transform.translation.y, aruco_tf_optical.transform.translation.z
            x_link, y_link, z_link = z_opt, -x_opt, -y_opt 
            cam_pt = PointStamped()
            cam_pt.header.frame_id = 'camera_link'
            cam_pt.header.stamp = current_stamp
            cam_pt.point.x, cam_pt.point.y, cam_pt.point.z = x_link, y_link, z_link
            camera_to_base = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
            base_pt = do_transform_point(cam_pt, camera_to_base)
            q_fixed = Quaternion(x=0.5, y=0.5, z=-0.5, w=0.5) if marker_id == FERTILIZER_ARUCO_ID else Quaternion(x=1.0, y=0.0, z=0.0, w=0.0) 
            tf_msg_base = TransformStamped()
            tf_msg_base.header.frame_id = 'base_link'
            tf_msg_base.child_frame_id = frame_name
            tf_msg_base.transform.translation.x = base_pt.point.x
            tf_msg_base.transform.translation.y = base_pt.point.y
            tf_msg_base.transform.translation.z = base_pt.point.z 
            tf_msg_base.transform.rotation = q_fixed
            self.locked_tfs[marker_id] = tf_msg_base
            self.get_logger().info(f'✅ LOCKED {frame_name.upper()}')
        except Exception: pass

    def process_vision_and_tf(self):
        if self.cv_image is None: return
        current_stamp = self.get_clock().now().to_msg()
        display_image = self.cv_image.copy()
        
        # 1. ArUco
        aruco_markers = self.detect_aruco_markers(self.cv_image)
        for marker in aruco_markers:
            if marker['id'] in FRAME_NAMES:
                if display_image is not None: cv2.aruco.drawDetectedMarkers(display_image, [marker['corners']], borderColor=(0, 255, 255))
                if self.locked_tfs[marker['id']] is None:
                    self.detection_confidence_counter[marker['id']] = min(CONFIDENCE_THRESHOLD, self.detection_confidence_counter[marker['id']] + 1)
                    if self.detection_confidence_counter[marker['id']] >= CONFIDENCE_THRESHOLD: self.lock_marker_tf(marker)
        
        for marker_id, tf_msg in self.locked_tfs.items():
            if tf_msg: 
                tf_msg.header.stamp = current_stamp
                self.tf_broadcaster.sendTransform(tf_msg)
                
        # 2. Bad Fruit
        bad_fruits = self.bad_fruit_detection(self.cv_image)
        detected_fruit_ids_in_frame = set()
        
        for fruit in bad_fruits:
            center_x, center_y = fruit['center']
            depth = fruit['depth']
            x, y, w, h = fruit['bbox']
            
            pos_x_cam, pos_y_cam, pos_z_cam = self.compute_3d_position(center_x, center_y, depth)
            matched_fruit_id = self.match_fruit_to_tracked(center_x, center_y, depth)
            
            # Prevent ID collision in same frame
            if matched_fruit_id is not None and matched_fruit_id in detected_fruit_ids_in_frame:
                matched_fruit_id = None 

            if matched_fruit_id is None:
                all_ids = list(self.fruit_detection_confidence.keys()) + list(self.locked_fruit_tfs.keys())
                unique_ids = [int(i) for i in set(all_ids)]
                matched_fruit_id = max(unique_ids) + 1 if unique_ids else 1
                self.fruit_detection_confidence[matched_fruit_id] = 0
                self.fruit_detection_positions[matched_fruit_id] = []
            
            detected_fruit_ids_in_frame.add(matched_fruit_id)
            if matched_fruit_id not in self.locked_fruit_tfs:
                self.fruit_detection_positions[matched_fruit_id].append((pos_x_cam, pos_y_cam, pos_z_cam))
                if len(self.fruit_detection_positions[matched_fruit_id]) > 10: self.fruit_detection_positions[matched_fruit_id].pop(0)
            
            cv2.rectangle(display_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(display_image, f'BAD {matched_fruit_id}', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            if matched_fruit_id not in self.locked_fruit_tfs:
                self.fruit_detection_confidence[matched_fruit_id] = min(CONFIDENCE_THRESHOLD, self.fruit_detection_confidence[matched_fruit_id] + 1)
                if self.fruit_detection_confidence[matched_fruit_id] >= CONFIDENCE_THRESHOLD:
                    self.lock_fruit_tf(matched_fruit_id, pos_x_cam, pos_y_cam, pos_z_cam)
        
        # Cleanup
        for fruit_id in list(self.fruit_detection_confidence.keys()):
            if fruit_id not in self.locked_fruit_tfs and fruit_id not in detected_fruit_ids_in_frame:
                self.fruit_detection_confidence[fruit_id] = max(0, self.fruit_detection_confidence[fruit_id] - 2)
                if self.fruit_detection_confidence[fruit_id] == 0:
                    del self.fruit_detection_confidence[fruit_id]
                    if fruit_id in self.fruit_detection_positions: del self.fruit_detection_positions[fruit_id]
        
        for fruit_id, tf_msg in self.locked_fruit_tfs.items():
            if tf_msg:
                tf_msg.header.stamp = current_stamp
                self.tf_broadcaster.sendTransform(tf_msg)
            
        if SHOW_IMAGE and display_image is not None:
            cv2.imshow('Vision and TF Publisher', display_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VisionAndTFPublisher()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok(): node.destroy_node(); rclpy.shutdown()
        if SHOW_IMAGE: cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
