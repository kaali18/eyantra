# Task 2B Manipulation Script - Complete Beginner's Guide
## Understanding the Robot Arm Control Code

**Team ID: 2203**  
**File: `task2b_manipulation.py`**

---

## Table of Contents
1. [What Does This Script Do?](#overview)
2. [Control Theory Basics](#control-theory)
3. [State Machine Concept](#state-machine)
4. [Line by Line Code Explanation](#code-explanation)
5. [How Servo Control Works](#servo-control)
6. [Complete Execution Flow](#execution-flow)

---

## What Does This Script Do? {#overview}

### The Mission
This script controls the UR5 robot arm to:
1. **Scan** for detected objects (reads TF transforms from detection script)
2. **Plan** a sequence of pick-and-place tasks
3. **Move** the arm to each object
4. **Pick** the object using magnetic gripper
5. **Place** it at the correct location
6. **Repeat** until all objects are handled

### The Challenge
- Robot arm has 6 joints (6 degrees of freedom)
- We want to control the end effector (gripper) position and orientation
- We use **visual servoing** - move gripper directly without calculating joint angles

### Control Method: Visual Servoing
Instead of calculating complex joint angles, we use **delta twist commands**:
- Tell the gripper: "Move 0.1m forward, rotate 15° clockwise"
- Controller figures out how to move all 6 joints
- Much simpler for us!

---

## Control Theory Basics {#control-theory}

### What is P-Control?
**P-Control (Proportional Control)** is the simplest control method:

```
Control Output = Gain × Error
```

**Example:**
- Current position: X = 0.5m
- Target position: X = 1.0m
- Error = 1.0 - 0.5 = 0.5m
- Gain = 0.5
- Control output = 0.5 × 0.5 = 0.25 m/s

**Characteristics:**
- Larger error → Faster movement
- Smaller error → Slower movement (smooth approach)
- Never perfect (always small error remains)

### Position vs Velocity Control
**Position Control:**
- You say: "Go to X = 1.0m"
- Controller figures out velocities

**Velocity Control:**
- You say: "Move at 0.25 m/s in X direction"
- More direct control
- What we use!

### Twist Message
A **Twist** message contains:
- **Linear velocity**: (x, y, z) in m/s
- **Angular velocity**: (roll, pitch, yaw) in rad/s

```
Twist:
  linear:
    x: 0.2  # Move forward 0.2 m/s
    y: 0.0  # No sideways motion
    z: 0.0  # No up/down motion
  angular:
    x: 0.0  # No roll
    y: 0.0  # No pitch
    z: 0.1  # Turn 0.1 rad/s
```

---

## State Machine Concept {#state-machine}

### What is a State Machine?
A state machine is like a flowchart that the robot follows:

```
    [INIT]
       ↓
   Wait 3 seconds
       ↓
 [SCAN FOR OBJECTS]
       ↓
  [EXECUTE TASK]
       ↓
[APPROACH OBJECT] → [PICK OBJECT] → [LIFT OBJECT] → [MOVE TO DROP] → [DROP OBJECT] → [RETURN HOME]
                                                                                           ↓
                                                                                    Next object?
                                                                                           ↓
                                                                                     Yes → Loop
                                                                                      No → [DONE]
```

### Why Use a State Machine?
- **Organized**: Each state has clear purpose
- **Predictable**: Always know what robot is doing
- **Debuggable**: Easy to find where things go wrong
- **Extensible**: Easy to add new states

### Our States

1. **INIT**: Wait for startup
2. **EXECUTE_TASK**: Start new pick-and-place task
3. **APPROACH_OBJECT**: Move above object
4. **PICK_OBJECT**: Descend and attach gripper
5. **LIFT_OBJECT**: Lift up after picking
6. **MOVE_TO_DROP**: Move to drop location
7. **DROP_OBJECT**: Release object
8. **RETURN_HOME**: Go back to home position
9. **DONE**: All tasks completed

---

## Line by Line Code Explanation {#code-explanation}

### Import Section (Lines 1-36)

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
```
Standard Python header (same as detection script)

```python
import rclpy
from rclpy.node import Node
```
ROS2 Python library

```python
from geometry_msgs.msg import Twist, Quaternion
```
**What it means:**
- `Twist`: Message for velocity commands (linear + angular)
- `Quaternion`: Message for orientation (4 numbers: x, y, z, w)

```python
from tf2_ros import TransformException, TransformStamped, Buffer, TransformListener
```
TF2 library for coordinate transforms

```python
from linkattacher_msgs.srv import AttachLink, DetachLink
```
**What it means:**
- Service message types for gripper control
- `AttachLink`: Service to attach object to gripper
- `DetachLink`: Service to release object

```python
import math
import time
from typing import List, Dict, Optional
```
**What it means:**
- `math`: Mathematical functions
- `time`: Time-related functions (not used much)
- `typing`: Type hints for better code documentation
  - `List`: List of items
  - `Dict`: Dictionary (key-value pairs)
  - `Optional`: Value that might be None

---

### Constants (Lines 38-68)

```python
# Control Parameters
LINEAR_P_GAIN = 0.5
ANGULAR_P_GAIN = 0.7
```
**What it means:**
- Proportional gains for control
- Higher gain = faster movement (but less stable)
- Lower gain = slower movement (but more stable)
- These values are tuned by trial and error

```python
MAX_LINEAR_VEL = 0.4
MAX_ANGULAR_VEL = 0.6
```
**What it means:**
- Maximum velocities (safety limits)
- Prevents arm from moving too fast
- 0.4 m/s linear = 40 cm/second
- 0.6 rad/s angular = ~34°/second

```python
POS_TOLERANCE = 0.08  # 8 cm
ORI_TOLERANCE = 0.15  # ~8.6 degrees
```
**What it means:**
- How close is "close enough"?
- Position within 8 cm = considered reached
- Orientation within 0.15 rad = considered reached

```python
APPROACH_OFFSET = 0.15  # 15 cm above
PICKUP_DISTANCE = 0.08  # 8 cm for attachment
```
**What it means:**
- `APPROACH_OFFSET`: How high to position gripper initially
  - Approach 15 cm above object (safe)
- `PICKUP_DISTANCE`: How close for magnetic attachment
  - Magnetic gripper needs < 10 cm
  - We use 8 cm to be safe

```python
WAYPOINT_HOLD_TIME = 1.0  # Wait 1 second
```
**What it means:**
- After reaching waypoint, wait 1 second
- Allows arm to settle before next action

```python
# TF Frames
EE_LINK = "tool0"
BASE_LINK = "base_link"
TEAM_ID = 2203
```
**What it means:**
- `EE_LINK`: End effector frame name
- `BASE_LINK`: Robot base frame name
- `TEAM_ID`: Your team number

---

### Predefined Locations (Lines 70-98)

```python
HOME_POSITION = {
    'name': 'HOME',
    'pos': [-0.214, -0.532, 0.557],
    'ori': [0.707, 0.028, 0.034, 0.707]
}
```
**What it means:**
- Dictionary defining home position
- `pos`: [X, Y, Z] in meters
- `ori`: [qx, qy, qz, qw] quaternion

**These positions were taught** by manually moving the arm and recording positions.

**Breakdown:**
- **HOME**: Starting position (safe, out of the way)
- **INTERMEDIATE**: Waypoint for safe travel (avoids collisions)
- **TRASHBIN**: Where to drop bad fruits
- **EBOT_DROP**: Where to drop fertilizer

---

### Class Definition (Lines 101-139)

```python
class Task2BManipulation(Node):
    """
    ROS2 Node for UR5 arm manipulation to pick and place objects.
    Uses visual servoing control from Task 1C.
    """
```
Class inheriting from Node

```python
    def __init__(self):
        super().__init__('task2b_manipulation')
        self.get_logger().info("Task 2B Manipulation Node Started - Team ID: 2203")
```
Initialize node with name 'task2b_manipulation'

```python
        # Publisher for arm control
        self.twist_pub = self.create_publisher(Twist, '/delta_twist_cmds', 10)
```
**What it means:**
- Create publisher to send twist commands
- Topic: `/delta_twist_cmds`
- Queue size: 10 messages
- This is how we control the arm!

```python
        # TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
```
Setup to receive transforms (object positions)

```python
        # Service clients for magnetic gripper
        self.attach_client = self.create_client(AttachLink, '/attach_link')
        self.detach_client = self.create_client(DetachLink, '/detach_link')
```
**What it means:**
- Create service clients (like phone contacts)
- `/attach_link`: Service to grab objects
- `/detach_link`: Service to release objects

```python
        # Wait for services
        self.get_logger().info("Waiting for gripper services...")
        self.attach_client.wait_for_service(timeout_sec=5.0)
        self.detach_client.wait_for_service(timeout_sec=5.0)
        self.get_logger().info("Gripper services ready!")
```
**What it means:**
- Wait up to 5 seconds for services to be available
- If services don't appear, will timeout

```python
        # State machine variables
        self.current_state = 'INIT'
        self.current_target = None
        self.is_holding = False
        self.hold_start_time = 0.0
        self.attached_object = None
```
**What it means:**
- `current_state`: Which state we're in
- `current_target`: Current waypoint we're moving to
- `is_holding`: Flag for waiting at waypoint
- `hold_start_time`: When we started waiting
- `attached_object`: Name of object currently attached

```python
        # Task sequence
        self.task_queue = []
        self.current_task_index = 0
```
**What it means:**
- `task_queue`: List of all pick-and-place tasks
- `current_task_index`: Which task we're currently doing

```python
        # Scanning control
        self.scan_timer = None
        self.scan_start_time = self.get_clock().now().nanoseconds / 1e9
        self.has_scanned = False
        self.scan_delay = 3.0  # Wait 3 seconds
```
**What it means:**
- Variables to control when to scan for objects
- Wait 3 seconds before scanning (give detection time to find objects)

```python
        # Control timer
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.control_loop)
```
**What it means:**
- Create timer that runs every 0.02 seconds (50 Hz)
- Calls `self.control_loop` function
- This is the heartbeat of the controller!

---

### Quaternion Math Functions (Lines 141-194)

**Why do we need quaternion math?**
To calculate orientation errors and generate rotation velocities!

```python
    def quaternion_normalize(self, q: Quaternion) -> Quaternion:
        """Normalizes a quaternion."""
        norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        if norm == 0.0:
            return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        return Quaternion(
            x=q.x / norm, y=q.y / norm, z=q.z / norm, w=q.w / norm
        )
```
**What it means:**
- Quaternions must have magnitude 1 (unit quaternion)
- `norm = sqrt(x² + y² + z² + w²)`
- Divide each component by norm
- If norm is 0, return identity quaternion (no rotation)

```python
    def quaternion_inverse(self, q: Quaternion) -> Quaternion:
        """Computes quaternion inverse."""
        return Quaternion(x=-q.x, y=-q.y, z=-q.z, w=q.w)
```
**What it means:**
- Inverse quaternion = opposite rotation
- For unit quaternions, inverse = conjugate
- Conjugate = negate x, y, z; keep w

```python
    def quaternion_multiply(self, q1: Quaternion, q2: Quaternion) -> Quaternion:
        """Multiplies two quaternions."""
```
**What it means:**
- Quaternion multiplication = combining rotations
- Order matters! q1 * q2 ≠ q2 * q1
- Uses Hamilton product formula (complex math)

```python
    def quat_error_to_rotation_vector(self, current_q: Quaternion, target_q: Quaternion):
        """Converts quaternion error to rotation vector for control."""
        q_curr_inv = self.quaternion_inverse(current_q)
        q_error = self.quaternion_multiply(target_q, q_curr_inv)
        q_error = self.quaternion_normalize(q_error)
```
**What it means:**
- Calculate error quaternion:
  - `q_error = target * current_inverse`
- This tells us how to rotate from current to target

```python
        w = q_error.w
        if abs(w) > 1.0:
            w = math.copysign(1.0, w)
        
        angle = 2.0 * math.acos(w)
```
**What it means:**
- Extract rotation angle from error quaternion
- `w` component relates to angle: `w = cos(angle/2)`
- `angle = 2 * acos(w)`

```python
        if angle < 1e-6:
            return [0.0, 0.0, 0.0], 0.0
```
If angle is tiny, no rotation needed

```python
        if angle > math.pi:
            angle = 2.0 * math.pi - angle
            # Flip the sign of the vector components
            q_error.x = -q_error.x
            q_error.y = -q_error.y
            q_error.z = -q_error.z
```
**What it means:**
- Take shortest rotation path
- Angles > 180° can be done as negative rotation
- Example: 270° clockwise = 90° counter-clockwise

```python
        sin_half_angle = math.sqrt(1.0 - w*w)
        
        if sin_half_angle < 1e-6:
            return [0.0, 0.0, 0.0], 0.0
        
        scale_factor = angle / sin_half_angle
        
        rotation_vector = [
            q_error.x * scale_factor,
            q_error.y * scale_factor,
            q_error.z * scale_factor
        ]
        
        return rotation_vector, angle
```
**What it means:**
- Convert to rotation vector (axis-angle representation)
- Rotation vector = axis of rotation × angle
- Used for velocity control

**In simple terms:**
This function answers: "How should I rotate to reach the target orientation?"
Output: Direction and amount of rotation

---

### Gripper Control (Lines 196-236)

```python
    def attach_object(self, object_name: str) -> bool:
        """Attaches object to gripper using magnetic attachment."""
        req = AttachLink.Request()
        req.model1_name = object_name
        req.link1_name = 'body'
        req.model2_name = 'ur5'
        req.link2_name = 'wrist_3_link'
```
**What it means:**
- Create service request
- `model1_name`: Object to attach (e.g., 'bad_fruit')
- `link1_name`: Which link of object ('body' is main link)
- `model2_name`: Robot model name ('ur5')
- `link2_name`: Which link of robot ('wrist_3_link' is near gripper)

**How magnetic gripper works:**
- Creates a virtual joint between object and robot
- Object becomes "welded" to gripper
- Moves with gripper

```python
        self.get_logger().info(f'Attaching {object_name}...')
        future = self.attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
```
**What it means:**
- Call service asynchronously (non-blocking)
- Wait up to 2 seconds for response
- `spin_until_future_complete`: Keep ROS running while waiting

```python
        if future.result() is not None:
            self.get_logger().info(f'Successfully attached {object_name}')
            self.attached_object = object_name
            return True
        else:
            self.get_logger().error(f'Failed to attach {object_name}')
            return False
```
**What it means:**
- Check if service call succeeded
- Update state and return result

**detach_object function** is similar but releases the object.

---

### TF Lookup (Lines 238-261)

```python
    def get_object_position(self, frame_name: str) -> Optional[Dict]:
        """Gets position and orientation of an object from TF."""
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                BASE_LINK, frame_name, rclpy.time.Time()
            )
```
**What it means:**
- Look up transform from base_link to object frame
- `rclpy.time.Time()`: Use latest available transform

```python
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
```
**What it means:**
- Extract position (X, Y, Z)
- Extract orientation (quaternion)
- Return as dictionary

```python
        except TransformException as ex:
            self.get_logger().warn(f'Could not get transform for {frame_name}: {ex}')
            return None
```
If transform not available, return None

---

### Object Scanning (Lines 263-301)

```python
    def scan_for_objects(self):
        """Scans for detected objects and builds task queue."""
        self.get_logger().info("Scanning for objects...")
```
This function runs after 3 seconds to find all detected objects

```python
        # Look for bad fruits
        for i in range(1, 5):  # Check for up to 4 bad fruits
            fruit_frame = f'{TEAM_ID}_bad_fruit_{i}'
            fruit_pos = self.get_object_position(fruit_frame)
            if fruit_pos:
                self.get_logger().info(f'Found bad fruit {i}')
                # Add pick and place task
                self.task_queue.append({
                    'action': 'PICK_AND_PLACE',
                    'object_frame': fruit_frame,
                    'object_name': 'bad_fruit',
                    'drop_location': TRASHBIN_POSITION
                })
```
**What it means:**
- Loop through possible fruit IDs (1-4)
- Try to find transform for each
- If found, add task to queue
- Task specifies: what to pick, where to drop

```python
        # Look for fertilizer cans
        for i in range(50):  # Check ArUco IDs 0-49
            fert_frame = f'{TEAM_ID}_fertilizer_{i}_base'
            fert_pos = self.get_object_position(fert_frame)
            if fert_pos:
                self.get_logger().info(f'Found fertilizer with ArUco ID {i}')
                self.task_queue.append({
                    'action': 'PICK_AND_PLACE',
                    'object_frame': fert_frame,
                    'object_name': 'fertiliser_can',
                    'drop_location': EBOT_DROP_POSITION
                })
```
Similar process for fertilizer

```python
        if len(self.task_queue) == 0:
            self.get_logger().warn("No objects detected!")
        else:
            self.get_logger().info(f'Found {len(self.task_queue)} objects to pick and place')
            self.current_state = 'EXECUTE_TASK'
```
**What it means:**
- If no objects found, warn user
- Otherwise, print count and start execution

---

### Movement Control (Lines 303-391)

```python
    def move_to_target(self, target_waypoint: Dict) -> bool:
        """Moves arm to target position. Returns True if reached."""
```
This is the core control function!

```python
        # Get current pose
        current_pose_tf = None
        try:
            current_pose_tf: TransformStamped = self.tf_buffer.lookup_transform(
                BASE_LINK, EE_LINK, rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f'Could not get current pose: {ex}')
            self.publish_zero_twist()
            return False
```
**What it means:**
- Look up current end effector position
- If can't get it, stop and return False

```python
        # Calculate position error
        current_p = current_pose_tf.transform.translation
        target_p = target_waypoint['pos']
        
        pos_error_vector = [
            target_p[0] - current_p.x,
            target_p[1] - current_p.y,
            target_p[2] - current_p.z
        ]
        pos_error_magnitude = math.sqrt(sum(e**2 for e in pos_error_vector))
```
**What it means:**
- Get current position
- Get target position
- Calculate error vector: target - current
- Calculate error magnitude (distance to target)
- `sqrt(x² + y² + z²)` = 3D distance formula

```python
        # Calculate orientation error
        current_q = current_pose_tf.transform.rotation
        target_q_list = target_waypoint['ori']
        target_q = Quaternion(
            x=target_q_list[0], y=target_q_list[1],
            z=target_q_list[2], w=target_q_list[3]
        )
        
        ori_error_vector, ori_error_angle = self.quat_error_to_rotation_vector(current_q, target_q)
```
**What it means:**
- Get current orientation
- Get target orientation
- Calculate rotation error

```python
        # Check if reached
        if pos_error_magnitude <= POS_TOLERANCE and ori_error_angle <= ORI_TOLERANCE:
            self.get_logger().info(
                f"Reached {target_waypoint['name']} | "
                f"P_Err: {pos_error_magnitude:.3f}m, R_Err: {ori_error_angle:.3f}rad"
            )
            self.publish_zero_twist()
            return True
```
**What it means:**
- If within tolerance, we're there!
- Stop moving (publish zero twist)
- Return True

```python
        # Generate control command
        twist_cmd = Twist()
        
        # Linear velocity (P-control)
        twist_cmd.linear.x = pos_error_vector[0] * LINEAR_P_GAIN
        twist_cmd.linear.y = pos_error_vector[1] * LINEAR_P_GAIN
        twist_cmd.linear.z = pos_error_vector[2] * LINEAR_P_GAIN
```
**What it means:**
- Create twist message
- Calculate linear velocities
- **P-Control**: velocity = error × gain
- Larger error → faster movement

```python
        # Angular velocity (P-control)
        twist_cmd.angular.x = ori_error_vector[0] * ANGULAR_P_GAIN
        twist_cmd.angular.y = ori_error_vector[1] * ANGULAR_P_GAIN
        twist_cmd.angular.z = ori_error_vector[2] * ANGULAR_P_GAIN
```
Same for angular velocities

```python
        # Clamp linear velocity
        linear_vel_mag = math.sqrt(
            twist_cmd.linear.x**2 + twist_cmd.linear.y**2 + twist_cmd.linear.z**2
        )
        if linear_vel_mag > MAX_LINEAR_VEL:
            ratio = MAX_LINEAR_VEL / linear_vel_mag
            twist_cmd.linear.x *= ratio
            twist_cmd.linear.y *= ratio
            twist_cmd.linear.z *= ratio
```
**What it means:**
- Calculate total speed
- If too fast, scale down proportionally
- Maintains direction but limits speed

**Example:**
- Calculated velocity: (0.6, 0.3, 0.0) → magnitude = 0.67 m/s
- Max allowed: 0.4 m/s
- Ratio: 0.4 / 0.67 = 0.6
- Scaled velocity: (0.36, 0.18, 0.0) → magnitude = 0.4 m/s ✓

Same clamping for angular velocity.

```python
        self.twist_pub.publish(twist_cmd)
        return False
```
Publish command and return False (not reached yet)

---

### State Machine Control Loop (Lines 393-566)

This is the brain! Controls the entire pick-and-place sequence.

```python
    def control_loop(self):
        """Main control loop implementing state machine."""
```
Runs 50 times per second (every 0.02 seconds)

#### INIT State

```python
        if self.current_state == 'INIT':
            # Check if enough time passed
            if not self.has_scanned:
                current_time = self.get_clock().now().nanoseconds / 1e9
                if current_time - self.scan_start_time >= self.scan_delay:
                    self.scan_for_objects()
                    self.has_scanned = True
            self.publish_zero_twist()
            return
```
**What it means:**
- Wait 3 seconds
- Scan for objects
- Stay still

#### HOLDING State

```python
        if self.is_holding:
            time_elapsed = self.get_clock().now().nanoseconds / 1e9 - self.hold_start_time
            if time_elapsed < WAYPOINT_HOLD_TIME:
                self.publish_zero_twist()
                return
            else:
                self.is_holding = False
                # Transition to next state based on current state
                if self.current_state == 'APPROACH_OBJECT':
                    self.current_state = 'PICK_OBJECT'
                # ... (more transitions)
                return
```
**What it means:**
- If holding at waypoint, check how long
- If not enough time, keep waiting
- If enough time, transition to next state

#### EXECUTE_TASK State

```python
        if self.current_state == 'EXECUTE_TASK':
            if self.current_task_index >= len(self.task_queue):
                self.current_state = 'DONE'
                return
            
            task = self.task_queue[self.current_task_index]
            self.get_logger().info(f"Starting task {self.current_task_index + 1}/{len(self.task_queue)}")
```
**What it means:**
- Check if all tasks done
- Get current task
- Print progress

```python
            # Get object position
            obj_pos = self.get_object_position(task['object_frame'])
            if obj_pos is None:
                self.get_logger().error(f"Cannot find object {task['object_frame']}")
                self.current_task_index += 1
                return
```
Look up object position from TF

```python
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
            self.current_state = 'APPROACH_OBJECT'
            return
```
**What it means:**
- Create target position ABOVE object
- `+ APPROACH_OFFSET` adds 15 cm to Z
- Orientation: gripper pointing down
- Transition to APPROACH_OBJECT state

#### APPROACH_OBJECT State

```python
        if self.current_state == 'APPROACH_OBJECT':
            if self.move_to_target(self.current_target):
                self.is_holding = True
                self.hold_start_time = self.get_clock().now().nanoseconds / 1e9
            return
```
**What it means:**
- Move to approach position
- If reached, start holding (wait)
- After holding, will automatically transition to PICK_OBJECT

#### PICK_OBJECT State

```python
        if self.current_state == 'PICK_OBJECT':
            task = self.task_queue[self.current_task_index]
            obj_pos = self.get_object_position(task['object_frame'])
            
            if obj_pos is None:
                self.get_logger().error("Lost object tracking!")
                self.current_task_index += 1
                self.current_state = 'EXECUTE_TASK'
                return
```
Get object position again (might have changed slightly)

```python
            # Move to pickup position (close to object)
            self.current_target = {
                'name': f"PICKUP_{task['object_name']}",
                'pos': [
                    obj_pos['pos'][0],
                    obj_pos['pos'][1],
                    obj_pos['pos'][2] + PICKUP_DISTANCE
                ],
                'ori': [0.0, 1.0, 0.0, 0.0]
            }
```
**What it means:**
- Create target 8 cm above object (close enough for magnet)
- `+ PICKUP_DISTANCE` = +0.08 m

```python
            if self.move_to_target(self.current_target):
                # Reached pickup position, attach!
                success = self.attach_object(task['object_name'])
                if success:
                    self.is_holding = True
                    self.hold_start_time = self.get_clock().now().nanoseconds / 1e9
                else:
                    self.get_logger().error("Failed to attach!")
                    self.current_task_index += 1
                    self.current_state = 'EXECUTE_TASK'
            return
```
**What it means:**
- When reached pickup position, call attach service
- If success, hold for 1 second (let magnet secure)
- If failed, skip this object and move to next

#### LIFT_OBJECT State

```python
        if self.current_state == 'LIFT_OBJECT':
            self.current_target = INTERMEDIATE_POSITION
            
            if self.move_to_target(self.current_target):
                self.is_holding = True
                self.hold_start_time = self.get_clock().now().nanoseconds / 1e9
                self.current_state = 'MOVE_TO_DROP'
            return
```
**What it means:**
- Move to intermediate position (safe waypoint)
- Avoids collisions during travel
- After reaching, will transition to MOVE_TO_DROP

#### MOVE_TO_DROP State

```python
        if self.current_state == 'MOVE_TO_DROP':
            task = self.task_queue[self.current_task_index]
            self.current_target = task['drop_location']
            
            if self.move_to_target(self.current_target):
                self.is_holding = True
                self.hold_start_time = self.get_clock().now().nanoseconds / 1e9
            return
```
**What it means:**
- Move to drop location (TRASHBIN or EBOT_DROP)
- When reached, hold for 1 second

#### DROP_OBJECT State

```python
        if self.current_state == 'DROP_OBJECT':
            task = self.task_queue[self.current_task_index]
            
            if self.attached_object:
                self.detach_object(self.attached_object)
            
            self.is_holding = True
            self.hold_start_time = self.get_clock().now().nanoseconds / 1e9
            return
```
**What it means:**
- Detach object (release from gripper)
- Hold for 1 second (let object settle)

#### RETURN_HOME State

```python
        if self.current_state == 'RETURN_HOME':
            self.current_target = HOME_POSITION
            
            if self.move_to_target(self.current_target):
                self.is_holding = True
                self.hold_start_time = self.get_clock().now().nanoseconds / 1e9
            return
```
**What it means:**
- Move back to home position
- After holding, will check if more tasks

#### DONE State

```python
        if self.current_state == 'DONE':
            self.publish_zero_twist()
            self.get_logger().info("Task 2B completed successfully!")
            self.timer.cancel()
            return
```
**What it means:**
- Stop moving
- Print completion message
- Cancel timer (stop control loop)

---

### Main Function (Lines 568-588)

```python
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
```
Standard ROS2 node startup and shutdown handling.

---

## How Servo Control Works {#servo-control}

### The Control Loop

```
1. Read current position (from TF)
2. Compare with target position
3. Calculate error
4. Generate velocity command (P-control)
5. Clamp to maximum velocity
6. Send command to robot
7. Wait 0.02 seconds
8. Repeat
```

### Visual Servoing Explained

Traditional robot control:
```
Target Position → Inverse Kinematics → Joint Angles → Robot moves
                   (Complex math!)
```

Visual servoing:
```
Target Position → Error → Velocity Command → Robot figures out joint angles
                          (Simple!)
```

### Why P-Control Works

As robot approaches target:
- Error gets smaller
- Velocity gets slower
- Smooth deceleration
- Gentle arrival

---

## Complete Execution Flow {#execution-flow}

### Full Sequence

1. **Node starts** → Wait 3 seconds
2. **Scan for objects** → Build task queue
3. **For each object:**
   - a. Read object position from TF
   - b. Move to approach position (15cm above)
   - c. Wait 1 second
   - d. Descend to pickup position (8cm above)
   - e. Attach gripper
   - f. Wait 1 second
   - g. Lift to intermediate position
   - h. Wait 1 second
   - i. Move to drop location
   - j. Wait 1 second
   - k. Detach gripper
   - l. Wait 1 second
   - m. Return to home
   - n. Wait 1 second
4. **Repeat for next object**
5. **All done!** → Stop

### Timing Example

For 1 object:
- Approach: ~5 seconds
- Wait: 1 second
- Pick: ~3 seconds
- Wait: 1 second
- Lift: ~4 seconds
- Wait: 1 second
- Move to drop: ~6 seconds
- Wait: 1 second
- Drop: instant
- Wait: 1 second
- Return home: ~5 seconds
- Wait: 1 second

**Total: ~30 seconds per object**

---

## Summary

**Key Concepts:**
1. **Visual Servoing**: Control gripper position directly with velocity commands
2. **P-Control**: Simple proportional control (velocity = gain × error)
3. **State Machine**: Organized sequence of actions
4. **Quaternion Math**: Represent and control 3D rotations
5. **TF Transforms**: Know where everything is in 3D space
6. **Service Calls**: Command gripper to attach/detach

**Control Strategy:**
- Read current position
- Calculate error to target
- Generate velocity proportional to error
- Clamp to maximum speed
- Repeat until reached

**State Machine Logic:**
- One state at a time
- Clear transitions
- Wait at waypoints
- Handle errors gracefully

---

**End of Manipulation Guide**

See `TASK2B_COMPLETE_GUIDE.md` for detection script explanation.
