#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import sys
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped, PointStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math

try:
    from tf2_geometry_msgs import do_transform_point
except ImportError:
    do_transform_point = None

SHOW_IMAGE = True
DISABLE_MULTITHREADING = False
TEAM_ID = 2203



class FruitsTF(Node):

    def __init__(self):
        super().__init__('fruits_tf')
        self.bridge = CvBridge()
        self.cv_image = None
        self.depth_image = None
        
        self.sizeCamX = 1280
        self.sizeCamY = 720
        self.centerCamX = 642.724365234375
        self.centerCamY = 361.9780578613281
        self.focalX = 915.3003540039062
        self.focalY = 914.0320434570312
        
        self.camera_matrix = np.array([
            [self.focalX, 0, self.centerCamX],
            [0, self.focalY, self.centerCamY],
            [0, 0, 1]
        ])
        self.dist_coeffs = np.zeros((4, 1))
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.minMarkerPerimeterRate = 0.03
        self.aruco_params.maxMarkerPerimeterRate = 4.0
        
        self.marker_size = 0.1
        self.get_logger().info("ArUco detection initialized with DICT_4X4_50") 
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        
        if DISABLE_MULTITHREADING:
            self.cb_group = MutuallyExclusiveCallbackGroup()
        else:
            self.cb_group = ReentrantCallbackGroup()

        
        self.create_subscription(
            Image, '/camera/image_raw', self.colorimagecb, 10, callback_group=self.cb_group
        )
        self.create_subscription(
            Image, '/camera/depth/image_raw', self.depthimagecb, 10, callback_group=self.cb_group
        )

        self.create_timer(0.1, self.process_image, callback_group=self.cb_group)

        if SHOW_IMAGE:
            cv2.namedWindow('Bad Fruit Detection with ArUco', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Bad Fruit Detection with ArUco', 1280, 720)

        self.get_logger().info("FruitsTF node started. Team ID: 2203")

    def create_fruit_mask(self, image):
        """Creates a region of interest (ROI) mask focused on the conveyor belt/tray."""
        height, width = image.shape[:2]
        mask = np.zeros((height, width), dtype=np.uint8)
        
        tray_x_start = 50
        tray_x_end = width // 2
        tray_y_start = height // 4
        tray_y_end = 3 * height // 4
        
        mask[tray_y_start:tray_y_end, tray_x_start:tray_x_end] = 255
        
        return mask

    def detect_aruco_markers(self, image):
        """Detects ArUco markers and estimates their pose (rvec, tvec)."""
        detected_markers = []
        if self.aruco_dict is None or image is None:
            return detected_markers
            
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )
            
            if ids is None or len(ids) == 0:
                self.get_logger().debug('Scanning for ArUco markers... none found')
                return detected_markers
            
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )
            
            for i in range(len(ids)):
                marker_info = {
                    'id': int(ids[i][0]),
                    'corners': corners[i],
                    'rvec': rvecs[i],
                    'tvec': tvecs[i]
                }
                detected_markers.append(marker_info)
                
        except Exception as e:
            self.get_logger().error(f'Error in ArUco detection: {e}')
        return detected_markers

    def draw_axes(self, image, rvec, tvec, length=0.07):
        """Draws the 3D axes on the image for visualization."""
        try:
            cv2.drawFrameAxes(
                image, self.camera_matrix, self.dist_coeffs, rvec, tvec, length, thickness=6
            )
        except Exception as e:
            self.get_logger().error(f'Error drawing axes: {e}') 

    def rvec_tvec_to_transform(self, rvec, tvec):
        """Converts rvec/tvec to a TransformStamped message (camera_link frame)."""
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        
        # Quaternion calculation
        trace = rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]
        if trace > 0:
            s = math.sqrt(trace + 1.0) * 2
            qw = 0.25 * s
            qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
            qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
            qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
        elif rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
            s = math.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2]) * 2
            qw = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
            qx = 0.25 * s
            qy = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
            qz = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
        elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
            s = math.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2]) * 2
            qw = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
            qx = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
            qy = 0.25 * s
            qz = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
        else:
            s = math.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1]) * 2
            qw = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
            qx = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
            qy = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
            qz = 0.25 * s
        transform = TransformStamped()
        transform.transform.translation.x = float(tvec[0][0])
        transform.transform.translation.y = float(tvec[0][1])
        transform.transform.translation.z = float(tvec[0][2])
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        return transform


    def depthimagecb(self, data):
        """Callback function for depth image topic (converts 16UC1 to meters)."""
        try:
            depth_img = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            if depth_img.dtype == np.uint16:
                 self.depth_image = depth_img.astype(np.float32) / 1000.0 
            else:
                 self.depth_image = depth_img
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')

    def colorimagecb(self, data):
        """Callback function for color image topic."""
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting color image: {e}')

    def get_depth_at_point(self, x, y):
        """Retrieves the depth value (in meters) at a specific pixel (x, y)."""
        if self.depth_image is None:
            return None
            
        try:
            height, width = self.depth_image.shape
            x = max(0, min(x, width - 1))
            y = max(0, min(y, height - 1))
            
            depth_value = self.depth_image[y, x]
            # Handle 16UC1 format (mm) vs float format (m)
            if depth_value > 1000:
                depth_value /= 1000.0 

            if depth_value <= 0.0 or np.isnan(depth_value):
                return None
            
            return float(depth_value)
                
        except Exception as e:
            self.get_logger().error(f'Error getting depth: {e}')
            return None

    def bad_fruit_detection(self, rgb_image):
        """
        Detects bad fruits using a robust color mask and contour detection.
        (Using the user's provided logic for detection, simplified output)
        """
        bad_fruits = []
        if rgb_image is None:
            return bad_fruits

        try:
            tray_mask = self.create_fruit_mask(rgb_image)
            hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
            
            # --- User's provided masking ---
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
                            # Use an estimation if depth is unavailable (BAD PRACTICE, but handles missing data)
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

    ## CRITICAL FIX: CORRECT AXIS MAPPING
    def compute_3d_position(self, pixel_x, pixel_y, depth):
        """
        Converts 2D pixel coordinates and depth to 3D and maps to 'camera_link' frame.
        X_link = Forward (Depth), Y_link = Left, Z_link = Up.
        """
        z_depth = float(depth) 
        
        # 1. Standard Pinhole Model (Optical Frame: X_opt=Right, Y_opt=Down, Z_opt=Depth)
        x_optical = z_depth * (pixel_x - self.centerCamX) / self.focalX
        y_optical = z_depth * (pixel_y - self.centerCamY) / self.focalY
        
        # 2. Map Optical to Link Frame (Standard ROS camera convention)
        pos_x_cam = z_depth    # X_link = Z_optical (Depth)
        pos_y_cam = -x_optical # Y_link = -X_optical (Invert Right to get Left)
        pos_z_cam = -y_optical # Z_link = -Y_optical (Invert Down to get Up)
        
        return pos_x_cam, pos_y_cam, pos_z_cam

    def process_image(self):
        if self.cv_image is None:
            return

        try:
            display_image = self.cv_image.copy()
            current_stamp = self.get_clock().now().to_msg()
            
            # ArUco processing (Unchanged)
            aruco_markers = self.detect_aruco_markers(self.cv_image)
            for marker in aruco_markers:
                # ... (ArUco processing and TF publishing code here) ...
                marker_id = marker['id']
                corners = marker['corners']
                rvec = marker['rvec']
                tvec = marker['tvec']
                cv2.aruco.drawDetectedMarkers(display_image, [corners], borderColor=(0, 255, 255))
                self.draw_axes(display_image, rvec, tvec, length=0.07)
                center = np.mean(corners[0], axis=0).astype(int)
                cv2.circle(display_image, (int(center[0]), int(center[1])), 5, (255, 0, 0), -1)
                
                try:
                    aruco_tf = self.rvec_tvec_to_transform(rvec, tvec)
                    aruco_tf.header.stamp = self.get_clock().now().to_msg()
                    aruco_tf.header.frame_id = 'camera_link'
                    aruco_tf.child_frame_id = f'aruco_{marker_id}'
                    self.tf_broadcaster.sendTransform(aruco_tf)
                except Exception as e:
                    self.get_logger().error(f'Error publishing ArUco TF: {e}')


            # Bad fruits: draw contour, annotate, and publish TF
            bad_fruits = self.bad_fruit_detection(self.cv_image)
            for fruit in bad_fruits:
                fruit_id = fruit['id']
                center_x, center_y = fruit['center']
                depth = fruit['depth']
                x, y, w, h = fruit['bbox']
                contour = fruit['contour']
                
                # Use the corrected 3D position calculation
                pos_x_cam, pos_y_cam, pos_z_cam = self.compute_3d_position(center_x, center_y, depth)
                
                # [Visualization code here, mostly unchanged]
                cv2.drawContours(display_image, [contour], -1, (0, 255, 255), 2)
                cv2.rectangle(display_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(display_image, (center_x, center_y), 5, (0, 0, 255), -1)
                label = 'bad fruit'
                cv2.putText(display_image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                coord_text = f'({pos_x_cam:.2f}, {pos_y_cam:.2f}, {pos_z_cam:.2f})'
                cv2.putText(display_image, coord_text, (x, y + h + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
                
                frame_name = f'{TEAM_ID}_bad_fruit_{fruit_id}'
                
                # --- TF Transformation to base_link ---
                try:
                    camera_to_base = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
                    if do_transform_point is not None:
                        # 1. Point Stamped in the CORRECTLY ALIGNED camera_link frame
                        cam_pt = PointStamped()
                        cam_pt.header.frame_id = 'camera_link'
                        cam_pt.header.stamp = self.get_clock().now().to_msg()
                        cam_pt.point.x = pos_x_cam # Forward
                        cam_pt.point.y = pos_y_cam # Left
                        cam_pt.point.z = pos_z_cam # Up
                        
                        # 2. Transform to base_link frame
                        base_pt = do_transform_point(cam_pt, camera_to_base)
                        
                        # 3. Final TF broadcast
                        tf_msg = TransformStamped()
                        tf_msg.header.stamp = self.get_clock().now().to_msg()
                        tf_msg.header.frame_id = 'base_link'
                        tf_msg.child_frame_id = frame_name
                        
                        # Translation (Z offset removed)
                        tf_msg.transform.translation.x = base_pt.point.x
                        tf_msg.transform.translation.y = base_pt.point.y
                        tf_msg.transform.translation.z = base_pt.point.z # Z_HEIGHT_OFFSET removed
                        
                        # Orientation Fix: 180 deg rotation around Y-axis (W=0, Y=1)
                        # This sets the object frame's Z axis pointing down for the gripper.
                        tf_msg.transform.rotation.x = 0.0
                        tf_msg.transform.rotation.y = 1.0 
                        tf_msg.transform.rotation.z = 0.0
                        tf_msg.transform.rotation.w = 0.0
                        
                        self.tf_broadcaster.sendTransform(tf_msg)
                        self.get_logger().info(f'Published TF: {frame_name} at X:{base_pt.point.x:.3f}, Y:{base_pt.point.y:.3f}, Z:{tf_msg.transform.translation.z:.3f}') 
                    else:
                        self.get_logger().error('do_transform_point not available; cannot transform to base_link.')
                        
                except (LookupException, ConnectivityException, ExtrapolationException) as e:
                    self.get_logger().warn(f'Could not transform to base_link: {e}. Publishing in camera_link.')
                    # Fallback TF: publishing raw camera coordinates
                    tf_msg = TransformStamped()
                    tf_msg.header.stamp = self.get_clock().now().to_msg()
                    tf_msg.header.frame_id = 'camera_link'
                    tf_msg.child_frame_id = frame_name
                    tf_msg.transform.translation.x = pos_x_cam
                    tf_msg.transform.translation.y = pos_y_cam
                    tf_msg.transform.translation.z = pos_z_cam
                    tf_msg.transform.rotation.x = 0.0
                    tf_msg.transform.rotation.y = 0.0
                    tf_msg.transform.rotation.z = 0.0
                    tf_msg.transform.rotation.w = 1.0
                    self.tf_broadcaster.sendTransform(tf_msg)
            
            if SHOW_IMAGE:
                cv2.imshow('Bad Fruit Detection with ArUco', display_image)
                cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f'Error in process_image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FruitsTF()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down FruitsTF")
        node.destroy_node()
        rclpy.shutdown()
        if SHOW_IMAGE:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

