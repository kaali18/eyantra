#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool, Float64
import numpy as np
import cv2
import math

# =======================================================
# CONFIGURATION: Heading and Shape Logic
# =======================================================

# ----------------- Reverted Utility Constants -----------------
# X is the axis of travel (Blocks are spread along X)
# X is the axis of travel (Blocks are spread along X)
X_START = 1.05 
X_LENGTH = 0.73

# Lanes are horizontal rows (separated by Y-coordinates)
LANE_1_Y = -1.65
LANE_2_Y =  0.00
LANE_3_Y =  1.75

def get_plant_id_reverted(x, y, relative_y, robot_yaw):
    """
    Determines the plant_ID (1-8) based on world coordinates (x, y)
    using precise block transitions.
    """
    
    # ----------------- COORDINATE ADJUSTMENT -----------------
    # In the provided logs, we see X values like 3.21, 3.00, 2.73 etc.
    # The original logic used X_START = 1.05 and X_LENGTH = 0.73
    # Let's relax the boundaries a bit to account for real-world drift/noise.
    
    # Lanes are horizontal rows (separated by Y-coordinates)
    # LANE 1: Y approx -1.65
    # LANE 2: Y approx 0.00
    # LANE 3: Y approx 1.75
    
    lane = 0
    if abs(y - LANE_1_Y) <= 0.40: lane = 1
    elif abs(y - LANE_2_Y) <= 0.40: lane = 2
    elif abs(y - LANE_3_Y) <= 0.70: lane = 3
    
    if lane == 0: 
        # print(f"[DEBUG] Invalid Lane. Y={y:.2f}")
        return 0 

    # ----------------- BLOCK CALCULATION -----------------
    # Block 0: ~1.05 to ~1.78
    # Block 1: ~1.78 to ~2.51
    # Block 2: ~2.51 to ~3.24
    # Block 3: ~3.24 to ~4.00+
    
    x_rel = x - X_START
    
    # If x is less than start, it might be the dock station or approach
    if x < 1.05: 
        # Check if it is the dock station area (usually Lane 2 or special P1)
        return 0
    
    block = int(x_rel // 0.73)
    
    # Clamp block to 0-3 range just in case
    if block < 0: block = 0
    if block > 3: block = 3

    # ----------------- ID LOGIC -----------------
    plant_id = 0
    
    if lane == 1:   # Lane 1: IDs 1-4 (Ascending X ?)
        # Based on logs:
        # x=1.50 -> ID 1 (Block 0)
        # x=2.04 -> ID 2 (Block 1)
        # x=2.73 -> ID 3 (Block 2)
        # x=3.21 -> ID 3/4? 
        plant_id = block + 1
        
    elif lane == 3: # Lane 3: IDs 5-8
        plant_id = block + 5
        
    elif lane == 2: # Lane 2: IDs 1-8 based on side
        # Determine Direction of Travel along X axis
        # (Or determine side based on relative_y directly if available)
        
        # In the provided code, relative_y comes from the cluster centroid relative to robot center
        # relative_y > 0 is LEFT, relative_y < 0 is RIGHT
        
        x_direction_sign = math.cos(robot_yaw)
        is_left_side = relative_y > 0.01 
        
        # Case 1: X Increasing (Moving East / Forward) -> Heading ~0
        if x_direction_sign > 0.5:
            if is_left_side: plant_id = block + 5 # Left is Lane 3 side (IDs 5-8)
            else: plant_id = block + 1            # Right is Lane 1 side (IDs 1-4)
                
        # Case 2: X Decreasing (Moving West / Backward) -> Heading ~3.14 or -3.14
        elif x_direction_sign < -0.5:
            if is_left_side: plant_id = block + 1 # Left is Lane 1 side (IDs 1-4)
            else: plant_id = block + 5            # Right is Lane 3 side (IDs 5-8)
            
        else:
            # If turning, default to something reasonable or previous
            plant_id = block + 1

    # print(f"[DEBUG] Final ID: {plant_id} (Lane {lane}, Block {block}, x={x:.2f})")
    return plant_id

# Robot triggers detection at 0, 3.14, and -3.14 (Horizontal Lanes)
HORIZONTAL_TARGETS = [0.0, 3.14159, -3.14159] 
ORIENTATION_TOLERANCE = math.radians(10.0) 

# Shape Logic
TARGET_ANGLE_TRIANGLE = 2.356  
TOLERANCE_TRIANGLE = math.radians(25.0) # Increased to cover 120-degree observation 
TARGET_ANGLE_SQUARE = 1.5708
TOLERANCE_SQUARE = math.radians(20.0) 

class ShapeDetectorRANSAC(Node):
    def __init__(self):
        super().__init__('shape_detector_horizontal')
        
        # --- Publishers ---
        self.detection_pub = self.create_publisher(String, '/detection_status', 10)
        self.nav_command_pub = self.create_publisher(String, '/nav_command', 10)
        self.orientation_pub = self.create_publisher(Float64, '/orientation', 10)
        
        # --- Subscribers ---
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.report_ready_sub = self.create_subscription(
            Bool, '/advance_complete_signal', self.report_ready_callback, 10
        )
        
        # --- State Management ---
        self.robot_x, self.robot_y, self.robot_yaw = 0.0, 0.0, 0.0
        self.detection_active_by_heading = False 
        
        # --- RANSAC/Clustering Parameters ---
        self.ransac_threshold = 0.025      
        self.ransac_iterations = 500       
        self.min_line_points = 4           # Reduced to 4 for short lines
        self.max_lines = 6
        self.cluster_eps = 0.10
        self.cluster_min_samples = 4       # Reduced to 4 for short lines
        self.min_line_length = 0.05        # Kept at 0.05
        self.max_line_length = 0.40        
        self.detection_range = 1.75
        
        # --- History ---
        self.detected_shapes = []
        
        # --- Visualization ---
        self.viz_size = 800
        self.viz_scale = 100
        
        self.get_logger().info('=== Horizontal RANSAC Detector Started (Immediate Mode) ===')
        cv2.namedWindow('RANSAC Shape Detection', cv2.WINDOW_AUTOSIZE)
        
        # --- Debug Timer ---
        self.create_timer(1.0, self.print_odom_timer_callback)
        
        # --- State Tracking ---
        self.current_lane = None
        
        # --- Active Reporting State ---
        self.pending_report = {'shape': None, 'initial_center': None, 'relative_y': 0.0} 
        self.last_detection_time = 0.0

    def print_odom_timer_callback(self):
        # Calculate ID for debugging (assuming relative_y=0 for "center" estimation)
        debug_id = get_plant_id_reverted(self.robot_x, self.robot_y, 0.0, self.robot_yaw)
        self.get_logger().info(f"📍 ODOM: x={self.robot_x:.2f}, y={self.robot_y:.2f}, yaw={self.robot_yaw:.2f} | 🆔 Est. Plant ID: {debug_id}")

    # ==================== NAVIGATION & ODOMETRY ====================

    def odom_callback(self, msg):
        """Update pose and update the Heading Gate status."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Echo yaw for debugging
        self.orientation_pub.publish(Float64(data=self.robot_yaw))

        # Check Lane Entry
        new_lane = 0
        if abs(self.robot_y - LANE_1_Y) <= 0.40: new_lane = 1
        elif abs(self.robot_y - LANE_2_Y) <= 0.40: new_lane = 2
        elif abs(self.robot_y - LANE_3_Y) <= 0.70: new_lane = 3
        
        if new_lane != 0 and new_lane != self.current_lane:
            self.current_lane = new_lane
            self.get_logger().info(f"🛣️  >>> ENTERED LANE {new_lane} <<<")

        # Check if heading is 0, 3.14, or -3.14
        is_horizontal = any(abs(self.robot_yaw - target) <= ORIENTATION_TOLERANCE 
                          for target in HORIZONTAL_TARGETS)
        
        if is_horizontal != self.detection_active_by_heading:
            status = "ENABLED" if is_horizontal else "DISABLED (Turning)"
            self.get_logger().info(f"Heading {self.robot_yaw:.2f} -> Detection {status}")
            
        self.detection_active_by_heading = is_horizontal
    
    def report_ready_callback(self, msg: Bool):
        """Publishes the final status report when Nav Node signals it has arrived."""
        if msg.data and self.pending_report['shape'] is not None:
            shape = self.pending_report['shape']
            
            # Use the updated floor-division ID logic (Robot X, Robot Y)
            # User request: Report ID based on the final robot position after advancement/offsets
            plant_id = get_plant_id_reverted(self.robot_x, self.robot_y, self.pending_report['relative_y'], self.robot_yaw)
            
            status_map = {'TRIANGLE': 'FERTILIZER_REQUIRED', 'SQUARE': 'BAD_HEALTH'}
            status = status_map.get(shape, 'UNKNOWN')
            
            report_msg = String()
            report_msg.data = f"{status},{self.robot_x:.2f},{self.robot_y:.2f},{plant_id}"
            self.detection_pub.publish(report_msg)
            
            self.get_logger().info(f'✅ FINAL REPORT @ NEW POSE: {report_msg.data}')
            self.detected_shapes.append(self.pending_report['initial_center'])
            self.pending_report = {'shape': None, 'initial_center': None, 'relative_y': 0.0} 
            self.last_detection_time = self.get_clock().now().nanoseconds / 1e9

    # ==================== LIDAR PIPELINE ====================

    def scan_callback(self, msg):
        points = self.scan_to_points(msg)
        best_shape, best_lines, best_cluster = None, [], []
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        time_cooldown_active = (current_time - self.last_detection_time < 1.5) 
        pending_cooldown = self.pending_report['shape'] is not None
        cooldown_active = time_cooldown_active or pending_cooldown

        # Only process if Heading Gate is TRUE and points suffice
        if self.detection_active_by_heading and not pending_cooldown and len(points) >= 10:
            clusters = self.cluster_points(points)

            for cluster in clusters:
                if len(cluster) < self.cluster_min_samples: continue
                lines = self.fit_lines_ransac(cluster)
                if len(lines) < 2: continue
                
                shape = self.classify_shape(lines, cluster) 
                
                if shape:
                    # Calculate world coordinates for spatial filtering
                    centroid = np.mean(cluster, axis=0)
                    world_x = self.robot_x + centroid[0]*math.cos(self.robot_yaw) - centroid[1]*math.sin(self.robot_yaw)
                    world_y = self.robot_y + centroid[0]*math.sin(self.robot_yaw) + centroid[1]*math.cos(self.robot_yaw)
                    
                    if not time_cooldown_active and self.should_publish((world_x, world_y)):
                         # --- LANE 1 TRIANGLE FILTER ---
                         # User requirement: If a TRIANGLE is detected in Lane 1 (approx Y = -1.65), it must be IGNORED.
                         # This implies no advance, no stopping, and no reporting for Triangles in this lane.
                         self.get_logger().info(f"🧐 CHECKING FILTER: Shape={shape}, WorldY={world_y:.2f}, Lane1Diff={abs(world_y - LANE_1_Y):.2f}, CurrentLane={self.current_lane}")
                         
                         # --- FILTER 1: FRONT SECTOR EXCLUSION ---
                         # User requirement: Detection must be Left or Right, NOT Straight Front.
                         relative_x_local = centroid[0]
                         relative_y_local = centroid[1]
                         angle_local = math.atan2(relative_y_local, relative_x_local)
                         
                         if abs(angle_local) < 0.6: # Approx +/- 35 degrees
                             self.get_logger().warn(f"🚫 Ignoring FRONT detection at Angle {math.degrees(angle_local):.1f}°")
                             continue
                             
                         # --- FILTER 2: LANE 1 TRIANGLE ---
                         if shape == 'TRIANGLE' and (abs(world_y - LANE_1_Y) < 0.8 or self.current_lane == 1): 
                             self.get_logger().warn(f"🚫 Ignoring TRIANGLE at ({world_x:.2f}, {world_y:.2f}) - Inside Lane 1.")
                             continue
                         
                         # ACTIVE LOGIC: Send Stop Command
                         relative_y = centroid[1] 
                         self.send_nav_command(shape, (world_x, world_y), relative_y)
                         best_shape, best_lines, best_cluster = shape, lines, cluster
                         cooldown_active = True 
                         break
        
        self.visualize(points, best_cluster, best_lines, best_shape, self.detection_active_by_heading, cooldown_active)

    def scan_to_points(self, msg):
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max and r < self.detection_range:
                points.append([r * math.cos(angle), r * math.sin(angle)])
            angle += msg.angle_increment
        return np.array(points) if len(points) > 0 else np.array([]).reshape(0, 2)

    def cluster_points(self, points):
        """Custom DBSCAN clustering"""
        if len(points) == 0: return []
        n_points = len(points)
        visited = np.zeros(n_points, dtype=bool)
        labels = -np.ones(n_points, dtype=int)
        cluster_id = 0
        for i in range(n_points):
            if visited[i]: continue
            visited[i] = True
            neighbors = self.region_query(points, i, self.cluster_eps)
            if len(neighbors) < self.cluster_min_samples: continue
            labels[i] = cluster_id
            seed_set = list(neighbors)
            for j in seed_set:
                if not visited[j]:
                    visited[j] = True
                    new_neighbors = self.region_query(points, j, self.cluster_eps)
                    if len(new_neighbors) >= self.cluster_min_samples: seed_set.extend(new_neighbors)
                if labels[j] == -1: labels[j] = cluster_id
            cluster_id += 1
        return [points[labels == cid] for cid in range(cluster_id)]

    def region_query(self, points, idx, eps):
        distances = np.linalg.norm(points - points[idx], axis=1)
        return np.where(distances < eps)[0].tolist()

    def fit_lines_ransac(self, points):
        lines = []
        remaining = points.copy()
        for _ in range(self.max_lines):
            if len(remaining) < self.min_line_points: break
            line, inliers = self.ransac_line(remaining)
            if line is None or len(inliers) < self.min_line_points: break
            inlier_points = remaining[inliers]
            length, start, end = self.get_line_endpoints(inlier_points, line)
            if self.min_line_length <= length <= self.max_line_length:
                lines.append({'params': line, 'start': start, 'end': end, 'length': length})
            remaining = remaining[~inliers]
        return lines

    def ransac_line(self, points):
        n_points = len(points)
        best_line, best_inliers, max_inliers = None, None, 0
        for _ in range(self.ransac_iterations):
            idx = np.random.choice(n_points, 2, replace=False)
            line = self.fit_line_two_points(points[idx[0]], points[idx[1]])
            if line is None: continue
            dist = np.abs(line[0]*points[:,0] + line[1]*points[:,1] + line[2])
            inliers = dist < self.ransac_threshold
            if np.sum(inliers) > max_inliers:
                max_inliers, best_line, best_inliers = np.sum(inliers), line, inliers
        return best_line, best_inliers

    def fit_line_two_points(self, p1, p2):
        dx, dy = p2[0]-p1[0], p2[1]-p1[1]
        length = math.sqrt(dx**2 + dy**2)
        if length < 1e-6: return None
        return (-dy/length, dx/length, -((-dy/length)*p1[0] + (dx/length)*p1[1]))

    def get_line_endpoints(self, points, line):
        line_dir = np.array([line[1], -line[0]])
        projections = np.dot(points, line_dir)
        start, end = points[np.argmin(projections)], points[np.argmax(projections)]
        return np.linalg.norm(end - start), start, end

    def classify_shape(self, lines, cluster):
        n_lines = len(lines)
        if n_lines < 2: return None
        
        # Bounding Box Filter
        x_min, y_min = np.min(cluster, axis=0)
        x_max, y_max = np.max(cluster, axis=0)
        perimeter = 2 * ((x_max - x_min) + (y_max - y_min))
        if not (0.25 < perimeter < 2.0): return None
        
        angles = []
        for i in range(len(lines)):
            for j in range(i + 1, len(lines)):
                a1, b1, _ = lines[i]['params']
                a2, b2, _ = lines[j]['params']
                cos_ang = np.clip(np.dot([a1, b1], [a2, b2]), -1.0, 1.0)
                angle = math.acos(cos_ang)
                angles.append(angle)
                if TARGET_ANGLE_TRIANGLE > 1.57: angles.append(math.pi - angle)

        if n_lines == 2:
            if any(abs(a - TARGET_ANGLE_TRIANGLE) <= TOLERANCE_TRIANGLE for a in angles):
                return 'TRIANGLE'
        elif n_lines >= 3:
            if sum(1 for a in angles if abs(a - TARGET_ANGLE_SQUARE) <= TOLERANCE_SQUARE) >= 2:
                return 'SQUARE'
        return None

    def should_publish(self, center):
        for prev_x, prev_y in self.detected_shapes:
            if math.sqrt((center[0]-prev_x)**2 + (center[1]-prev_y)**2) < 0.5: return False
        return True

    def send_nav_command(self, shape, initial_center, relative_y=0.0):
        advance_dist = 0.65 if shape == 'SQUARE' else 0.80
        self.pending_report = {'shape': shape, 'initial_center': initial_center, 'relative_y': relative_y}
        self.nav_command_pub.publish(String(data=f'STOP,{advance_dist:.2f}'))
        self.get_logger().warn(f'🚦 {shape} Found. Advancing {advance_dist}m...')

    def visualize(self, all_points, cluster, lines, shape, active, cooldown):
        img = np.ones((self.viz_size, self.viz_size, 3), dtype=np.uint8) * 255
        center_px = (self.viz_size // 2, self.viz_size // 2)
        
        for pt in all_points:
            cv2.circle(img, (int(center_px[0] + pt[0]*self.viz_scale), int(center_px[1] - pt[1]*self.viz_scale)), 2, (180,180,180), -1)
        if len(cluster) > 0:
            for pt in cluster:
                cv2.circle(img, (int(center_px[0] + pt[0]*self.viz_scale), int(center_px[1] - pt[1]*self.viz_scale)), 4, (0,165,255), -1)
        for line in lines:
            p1, p2 = line['start'], line['end']
            cv2.line(img, (int(center_px[0] + p1[0]*self.viz_scale), int(center_px[1] - p1[1]*self.viz_scale)),
                          (int(center_px[0] + p2[0]*self.viz_scale), int(center_px[1] - p2[1]*self.viz_scale)), (0,255,0), 2)
        
        cv2.putText(img, f"ACTIVE: {active} | YAW: {self.robot_yaw:.2f}", (10, 30), 0, 0.7, (0,0,0), 2)
        if shape: cv2.putText(img, f"DETECTED: {shape}", (10, 90), 0, 0.9, (255,0,0), 3)
        cv2.imshow('RANSAC Shape Detection', img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ShapeDetectorRANSAC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()