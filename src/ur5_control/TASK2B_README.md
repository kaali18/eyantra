# Task 2B: Pick and Place - README

## Overview
Task 2B implements pick and place functionality for the UR5 robot arm to:
1. Pick up **bad fruits** and place them in the **trashbin**
2. Pick up **fertilizer cans** (detected via ArUco markers) and place them on the **eBot top**

## Files Created
- `src/ur5_control/src/task2b_detection.py` - Detects bad fruits and fertilizer cans, publishes TF transforms
- `src/ur5_control/src/task2b_manipulation.py` - Controls UR5 arm to pick and place objects

## Setup Instructions

### 1. Configure Link Attacher Plugin
Add this to your `~/.bashrc`:
```bash
echo 'export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/home/kasinath/colcon_ws/src:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH' >> ~/.bashrc
source ~/.bashrc
```

Verify:
```bash
echo $IGN_GAZEBO_SYSTEM_PLUGIN_PATH
```

### 2. Build the Package
```bash
cd ~/colcon_ws
colcon build --packages-select ur5_control
source install/setup.bash
```

## Running Task 2B

### Terminal 1: Launch Gazebo World
```bash
source ~/colcon_ws/install/setup.bash
ros2 launch eyantra_warehouse task2b.launch.py
```

### Terminal 2: Spawn UR5 Arm and Camera
```bash
source ~/colcon_ws/install/setup.bash
ros2 launch ur_simulation_gz spawn_arm.launch.py
```

### Terminal 3: Run Detection Script
```bash
source ~/colcon_ws/install/setup.bash
ros2 run ur5_control task2b_detection.py
```

This script will:
- Detect bad fruits using color-based detection (Task 1B method)
- Detect fertilizer cans using ArUco markers (Bonus Task 2 method)
- Publish TF transforms for detected objects
- **IMPORTANT**: After 5 seconds, detection will LOCK and positions become STATIC
- Wait for message: `[INFO] Detection LOCKED! Found X objects. Positions are now STATIC.`

### Terminal 4: Run Manipulation Script
**⚠️ IMPORTANT: Only start this AFTER detection locks (wait 5+ seconds)**

```bash
source ~/colcon_ws/install/setup.bash
ros2 run ur5_control task2b_manipulation.py
```

This script will:
- Scan for detected objects (waits 3 seconds after startup)
- For each bad fruit:
  1. Move to approach position (above fruit)
  2. Descend and attach using magnetic gripper
  3. Lift and move to trashbin
  4. Release object
  5. Return to home position
- For each fertilizer can:
  1. Move to approach position (above can)
  2. Descend and attach using magnetic gripper
  3. Lift and move to eBot top
  4. Release object
  5. Return to home position

## How It Works

### Detection Script (`task2b_detection.py`)
- **Inherits methods from Task 1B** for bad fruit detection
- **Inherits methods from Bonus Task 2** for ArUco marker detection
- **Static Position Locking** (NEW!):
  - Detects objects for 5 seconds
  - Saves all detected object positions in `base_link` frame
  - After 5 seconds, detection LOCKS
  - Only republishes saved static transforms (positions frozen)
  - This prevents objects from moving when camera/arm moves
- Publishes TF transforms with naming convention:
  - Bad fruits: `2203_bad_fruit_1`, `2203_bad_fruit_2`, etc.
  - Fertilizer: `2203_fertilizer_{aruco_id}_base`

### Manipulation Script (`task2b_manipulation.py`)
- **Uses servo control from Task 1C** for arm movement
- **State machine** controls pick and place sequence:
  1. `INIT` - Wait for object scan
  2. `EXECUTE_TASK` - Start new pick/place task
  3. `APPROACH_OBJECT` - Move above object
  4. `PICK_OBJECT` - Descend and attach
  5. `LIFT_OBJECT` - Lift up
  6. `MOVE_TO_DROP` - Move to drop location
  7. `DROP_OBJECT` - Release object
  8. `RETURN_HOME` - Return to home position
  9. `DONE` - All tasks completed

### Magnetic Gripper Control
- **Attach**: Uses `/attach_link` service
  - Bad fruit: `model1_name: 'bad_fruit'`
  - Fertilizer: `model1_name: 'fertiliser_can'`
- **Detach**: Uses `/detach_link` service
- **Distance requirement**: < 0.1 meters (10 cm) between object and `wrist_3_link`

## Key Parameters

### Control Gains (in `task2b_manipulation.py`)
```python
LINEAR_P_GAIN = 0.5
ANGULAR_P_GAIN = 0.7
MAX_LINEAR_VEL = 0.4
MAX_ANGULAR_VEL = 0.6
POS_TOLERANCE = 0.08  # 8 cm
APPROACH_OFFSET = 0.15  # 15 cm above object
PICKUP_DISTANCE = 0.08  # 8 cm for attachment
```

### Predefined Locations
- **HOME**: `[-0.214, -0.532, 0.557]`
- **INTERMEDIATE**: `[0.300, 0.250, 0.415]` (safe travel position)
- **TRASHBIN**: `[-0.806, 0.010, 0.182]`
- **EBOT_TOP**: `[-0.159, 0.501, 0.415]` (may need adjustment)

## Troubleshooting

### "No objects detected!"
- Ensure `task2b_detection.py` is running
- Check if objects are visible in camera view
- Wait for detection to lock (5 seconds)
- Verify TF transforms: `ros2 run tf2_ros tf2_echo base_link 2203_bad_fruit_1`

### Objects move/disappear when arm approaches
- **This is now FIXED with static detection locking**
- Make sure you waited for the "Detection LOCKED!" message before starting manipulation
- Detection locks after 5 seconds - positions become static
- If still seeing issues, increase `self.detection_timeout` in detection script

### "Failed to attach object"
- Ensure gripper is close enough (< 10 cm)
- Verify link attacher plugin is loaded (check `IGN_GAZEBO_SYSTEM_PLUGIN_PATH`)
- Check service availability: `ros2 service list | grep attach`

### Arm doesn't move
- Check if `/delta_twist_cmds` topic has subscribers
- Verify UR5 arm is spawned correctly
- Check for TF errors in logs

### Object positions incorrect
- Adjust camera intrinsic parameters in `task2b_detection.py`
- Verify depth camera is working: `ros2 topic echo /camera/depth/image_raw`
- Check TF tree: `ros2 run rqt_tf_tree rqt_tf_tree`

## Customization

### Adjust Drop Locations
Edit in `task2b_manipulation.py`:
```python
TRASHBIN_POSITION = {
    'name': 'TRASHBIN',
    'pos': [-0.806, 0.010, 0.182],  # Modify X, Y, Z
    'ori': [-0.684, 0.726, 0.05, 0.008]
}

EBOT_DROP_POSITION = {
    'name': 'EBOT_TOP',
    'pos': [-0.159, 0.501, 0.415],  # Modify X, Y, Z
    'ori': [0.029, 0.997, 0.045, 0.033]
}
```

### Change Detection Parameters
Edit in `task2b_detection.py`:
- Color ranges for bad fruit detection (lines 195-197)
- ArUco marker size: `self.marker_size = 0.1`
- Contour area thresholds: `if 800 < area < 25000:`

## Video Recording Tips
1. Start all 4 terminals in separate windows
2. Arrange windows so all are visible
3. Start recording before running manipulation script
4. Capture entire pick and place sequence for each object
5. Show final positions (trashbin full, fertilizer on eBot)

## Team Information
- **Team ID**: 2203
- **Task**: Task 2B - Pick and Place with UR5 Robot Arm
