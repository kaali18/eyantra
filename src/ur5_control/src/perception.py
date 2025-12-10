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
import math

# Check for tf2_geometry_msgs
try:
    from tf2_geometry_msgs import do_transform_point
except ImportError:
    def do_transform_point(point, transform):
        raise NotImplementedError("tf2_geometry_msgs is required for do_transform_point")

# --- CONFIGURATION ---
SHOW_IMAGE = True
TEAM_ID = 2203 

# Marker IDs
FERTILIZER_ARUCO_ID = 3 
FERTILIZER_BASE_ARUCO_ID = 6 
FRAME_NAMES = {
    FERTILIZER_ARUCO_ID: f'{TEAM_ID}_fertiliser_can',
    FERTILIZER_BASE_ARUCO_ID: f'{TEAM_ID}_ebot_base'
}

# Tracking Config
CONFIDENCE_THRESHOLD = 4 
POSITION_THRESHOLD = 0.15 

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
        self.marker_size = 0.12
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.cb_group = ReentrantCallbackGroup() 

        self.locked_tfs = {id: None for id in FRAME_NAMES.keys()}
        self.detection_confidence_counter = {id: 0 for id in FRAME_NAMES.keys()}
        
        self.locked_fruit_tfs = {}  
        self.fruit_detection_confidence = {} 
        self.fruit_detection_positions = {}
        
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.colorimagecb, 10, callback_group=self.cb_group)
        self.create_subscription(Image, 'camera/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10, callback_group=self.cb_group)
        
        self.create_timer(0.1, self.process_vision_and_tf, callback_group=self.cb_group)

        if SHOW_IMAGE:
            cv2.namedWindow('Vision and TF Publisher', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Vision and TF Publisher', 1280, 720)
            cv2.namedWindow('Process Debug', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Process Debug', 640, 480)
        
        self.get_logger().info(f"VisionAndTFPublisher node started. Team ID: {TEAM_ID}")

    def colorimagecb(self, data):
        try: self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except: pass

    def depthimagecb(self, data):
        try:
            depth_img = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            if depth_img.dtype == np.uint16: self.depth_image = depth_img.astype(np.float32) / 1000.0 
            else: self.depth_image = depth_img
        except: pass
    
    def rvec_tvec_to_transform(self, rvec, tvec):
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        q = Quaternion()
        trace = rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]
        if trace > 0:
            s = math.sqrt(trace + 1.0) * 2
            q.w = 0.25 * s; q.x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
            q.y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s; q.z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
        else:
            s = math.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1]) * 2
            q.w = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s; q.x = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
            q.y = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s; q.z = 0.25 * s
        
        t = TransformStamped()
        t.transform.translation.x = float(tvec[0][0])
        t.transform.translation.y = float(tvec[0][1])
        t.transform.translation.z = float(tvec[0][2])
        t.transform.rotation = q
        return t

    def detect_aruco_markers(self, image):
        if image is None: return []
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            if ids is None: return []
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
            return [{'id': int(ids[i][0]), 'corners': corners[i], 'rvec': rvecs[i], 'tvec': tvecs[i]} for i in range(len(ids))]
        except: return []
            
    def get_depth_at_point(self, x, y):
        if self.depth_image is None: return None
        try:
            h, w = self.depth_image.shape
            return float(self.depth_image[max(0, min(y, h - 1)), max(0, min(x, w - 1))])
        except: return None
            
    def compute_3d_position(self, pixel_x, pixel_y, depth):
        z = float(depth)
        x_opt = z * (pixel_x - self.centerCamX) / self.focalX
        y_opt = z * (pixel_y - self.centerCamY) / self.focalY
        return z, -x_opt, -y_opt

    # -------------------------------------------------------------------------
    # 🔍 SOLIDITY-BASED DETECTION (DENT/HOLE LOGIC) 🔍
    # -------------------------------------------------------------------------
    def bad_fruit_detection(self, rgb_image):
        bad_fruits = []
        if rgb_image is None: return bad_fruits

        try:
            height, width = rgb_image.shape[:2]
            hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
            
            # --- 1. ROI: QUADRILATERAL (CUSTOM POLYGON) ---
            # Define the 4 coordinates provided:
            # (111, 403), (408, 409), (367, 603), (19, 607)
            roi_points = np.array([[[111, 403], [408, 409], [367, 603], [19, 607]]], dtype=np.int32)
            
            # Create a black mask of the same size as the image
            roi_mask_area = np.zeros((height, width), dtype=np.uint8)
            
            # Fill the polygon with white (255)
            cv2.fillPoly(roi_mask_area, roi_points, 255)

            if SHOW_IMAGE:
                # Draw the Polygon boundary in Cyan (255, 255, 0) for visual debugging
                cv2.polylines(rgb_image, roi_points, True, (255, 255, 0), 2)

            # --- 2. GREYISH WHITE FILTER (To find the object body) ---
            lower_grey = np.array([0, 0, 60])
            upper_grey = np.array([180, 55, 255])
            
            grey_mask = cv2.inRange(hsv_image, lower_grey, upper_grey)
            
            # Apply the Polygon Mask to the Grey Detection Mask
            final_mask = cv2.bitwise_and(grey_mask, roi_mask_area)
            
            # Morphology to solidify the object before checking solidity
            kernel = np.ones((5, 5), np.uint8)
            final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_CLOSE, kernel, iterations=3)
            final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_OPEN, kernel, iterations=1)
            
            if SHOW_IMAGE:
                cv2.imshow('Process Debug', final_mask)
            
            # --- 3. CONTOUR & SOLIDITY ANALYSIS ---
            contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            fruit_id = 1
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Size Filter
                if 1200 < area < 40000:
                    
                    # Calculate Convex Hull
                    hull = cv2.convexHull(contour)
                    hull_area = cv2.contourArea(hull)
                    
                    if hull_area == 0: continue
                    
                    # --- THE CORE LOGIC: SOLIDITY ---
                    # Solidity = Contour Area / Hull Area
                    solidity = area / hull_area
                    
                    if 0.4 < solidity < 0.9:
                        
                        # --- UPDATED: USE ENCLOSING CIRCLE FOR CENTER ---
                        # Instead of bounding box, we use the enclosing circle 
                        # to find the center of the 'sphere' regardless of the cut.
                        ((cx_float, cy_float), radius) = cv2.minEnclosingCircle(hull)
                        center_x = int(cx_float)
                        center_y = int(cy_float)

                        # We still get bbox strictly for drawing the debug rectangle
                        x, y, w, h = cv2.boundingRect(hull)
                        
                        depth = self.get_depth_at_point(center_x, center_y)
                        if depth is None or depth == 0: continue 
                        
                        if 0.2 < depth < 1.5:
                            bad_fruits.append({
                                'id': fruit_id, 
                                'center': (center_x, center_y), 
                                'depth': depth,
                                'bbox': (x, y, w, h), 
                                'contour': contour,
                                'hull': hull,
                                'solidity': solidity # Stored for debug
                            })
                            fruit_id += 1

        except Exception as e: 
            self.get_logger().error(f'Error in bad_fruit_detection: {e}')
        
        return bad_fruits

    # -------------------------------------------------------------------------
    # TRACKING LOGIC
    # -------------------------------------------------------------------------
    def match_fruit_to_tracked(self, pixel_x, pixel_y, depth):
        pos_x, pos_y, pos_z = self.compute_3d_position(pixel_x, pixel_y, depth)
        
        for fruit_id, positions in self.fruit_detection_positions.items():
            if positions:
                last_pos = positions[-1] 
                dist = math.sqrt((pos_x - last_pos[0])**2 + (pos_y - last_pos[1])**2 + (pos_z - last_pos[2])**2)
                if dist < POSITION_THRESHOLD: return fruit_id

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
                dist = math.sqrt((pos_x - cam_pt_locked.point.x)**2 + (pos_y - cam_pt_locked.point.y)**2 + (pos_z - cam_pt_locked.point.z)**2)
                if dist < POSITION_THRESHOLD: return fruit_id
            except: pass
        return None 
    
    def lock_fruit_tf(self, fruit_id, pos_x_cam, pos_y_cam, pos_z_cam):
        try:
            current_stamp = self.get_clock().now().to_msg()
            frame_name = f'{TEAM_ID}_bad_fruit_{fruit_id}'
            
            cam_pt = PointStamped()
            cam_pt.header.frame_id = 'camera_link'; cam_pt.header.stamp = current_stamp
            cam_pt.point.x = pos_x_cam; cam_pt.point.y = pos_y_cam; cam_pt.point.z = pos_z_cam
            
            camera_to_base = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
            base_pt = do_transform_point(cam_pt, camera_to_base)
            
            tf_msg = TransformStamped()
            tf_msg.header.frame_id = 'base_link'; tf_msg.child_frame_id = frame_name
            tf_msg.transform.translation.x = base_pt.point.x
            tf_msg.transform.translation.y = base_pt.point.y
            tf_msg.transform.translation.z = base_pt.point.z
            tf_msg.transform.rotation = Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)
            
            self.locked_fruit_tfs[fruit_id] = tf_msg
            self.get_logger().info(f'✅ LOCKED {frame_name.upper()} at depth {pos_z_cam:.3f}m')
        except Exception as e: 
            self.get_logger().error(f'Failed to lock fruit TF: {e}')
            self.fruit_detection_confidence[fruit_id] = 0

    def lock_marker_tf(self, marker):
        mid = marker['id']; rvec = marker['rvec']; tvec = marker['tvec']; frame_name = FRAME_NAMES[mid]
        try:
            current_stamp = self.get_clock().now().to_msg()
            aruco_tf = self.rvec_tvec_to_transform(rvec, tvec)
            x_opt = aruco_tf.transform.translation.x; y_opt = aruco_tf.transform.translation.y; z_opt = aruco_tf.transform.translation.z
            cam_pt = PointStamped(); cam_pt.header.frame_id = 'camera_link'; cam_pt.header.stamp = current_stamp
            cam_pt.point.x = z_opt; cam_pt.point.y = -x_opt; cam_pt.point.z = -y_opt

            camera_to_base = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
            base_pt = do_transform_point(cam_pt, camera_to_base)
            
            q_fixed = Quaternion()
            if mid == FERTILIZER_ARUCO_ID: q_fixed = Quaternion(x=0.5, y=0.5, z=-0.5, w=0.5) 
            elif mid == FERTILIZER_BASE_ARUCO_ID: q_fixed = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0) 
            
            tf_msg = TransformStamped()
            tf_msg.header.frame_id = 'base_link'; tf_msg.child_frame_id = frame_name
            tf_msg.transform.translation.x = base_pt.point.x; tf_msg.transform.translation.y = base_pt.point.y; tf_msg.transform.translation.z = base_pt.point.z; tf_msg.transform.rotation = q_fixed
            
            self.locked_tfs[mid] = tf_msg
            self.get_logger().info(f'✅ LOCKED {frame_name.upper()}')
        except: self.detection_confidence_counter[mid] = 0

    def process_vision_and_tf(self):
        if self.cv_image is None: return
        current_stamp = self.get_clock().now().to_msg()
        disp = self.cv_image.copy()
        
        # 1. ArUco Logic
        det_ids = set()
        for m in self.detect_aruco_markers(self.cv_image):
            mid = m['id']
            if mid not in FRAME_NAMES: continue
            det_ids.add(mid)
            if SHOW_IMAGE: cv2.aruco.drawDetectedMarkers(disp, [m['corners']], borderColor=(0, 255, 255))
            if self.locked_tfs[mid] is None:
                self.detection_confidence_counter[mid] = min(CONFIDENCE_THRESHOLD, self.detection_confidence_counter[mid] + 1)
                if self.detection_confidence_counter[mid] >= CONFIDENCE_THRESHOLD: self.lock_marker_tf(m)
        
        for m, tf in self.locked_tfs.items():
            if tf: tf.header.stamp = current_stamp; self.tf_broadcaster.sendTransform(tf)

        # 2. Bad Fruit Logic
        det_fids = set()
        detected_fruits = self.bad_fruit_detection(self.cv_image)
        
        for f in detected_fruits:
            cx, cy = f['center']; depth = f['depth']
            px, py, pz = self.compute_3d_position(cx, cy, depth)
            
            mid = self.match_fruit_to_tracked(cx, cy, depth)
            
            if mid is None:
                all_ids = list(self.fruit_detection_confidence.keys()) + list(self.locked_fruit_tfs.keys())
                mid = max([int(i) for i in set(all_ids)]) + 1 if all_ids else 1
                self.fruit_detection_confidence[mid] = 0; self.fruit_detection_positions[mid] = []
            
            det_fids.add(mid)
            
            if mid not in self.locked_fruit_tfs:
                self.fruit_detection_positions[mid].append((px, py, pz))
                if len(self.fruit_detection_positions[mid]) > 10: self.fruit_detection_positions[mid].pop(0)
            
            x, y, w, h = f['bbox']
            if SHOW_IMAGE:
                # DRAW RED HULL AND GREEN CONTOUR TO SHOW THE DENT
                cv2.drawContours(disp, [f['hull']], -1, (0, 0, 255), 2)     # Hull in Red
                cv2.drawContours(disp, [f['contour']], -1, (0, 255, 0), 2)  # Actual shape in Green
                cv2.rectangle(disp, (x, y), (x + w, y + h), (0, 0, 255), 2)
                
                # DRAW THE DETECTED CENTER POINT (Cyan Circle)
                cv2.circle(disp, (cx, cy), 5, (255, 255, 0), -1)

                # Show Solidity value on screen for debugging
                lbl = f'F{mid} S:{f["solidity"]:.2f}' 
                cv2.putText(disp, lbl, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            if mid not in self.locked_fruit_tfs:
                self.fruit_detection_confidence[mid] = min(CONFIDENCE_THRESHOLD, self.fruit_detection_confidence[mid] + 1)
                if self.fruit_detection_confidence[mid] >= CONFIDENCE_THRESHOLD: self.lock_fruit_tf(mid, px, py, pz)

        for fid in list(self.fruit_detection_confidence.keys()):
            if fid not in self.locked_fruit_tfs and fid not in det_fids:
                self.fruit_detection_confidence[fid] = max(0, self.fruit_detection_confidence[fid] - 2)
                if self.fruit_detection_confidence[fid] == 0:
                    del self.fruit_detection_confidence[fid]
                    if fid in self.fruit_detection_positions: del self.fruit_detection_positions[fid]

        for _, tf in self.locked_fruit_tfs.items():
            if tf: tf.header.stamp = current_stamp; self.tf_broadcaster.sendTransform(tf)
            
        if SHOW_IMAGE:
            cv2.imshow('Vision and TF Publisher', disp)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args); node = VisionAndTFPublisher()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok(): node.destroy_node(); rclpy.shutdown()
        if SHOW_IMAGE: cv2.destroyAllWindows()

if __name__ == '__main__': main()
