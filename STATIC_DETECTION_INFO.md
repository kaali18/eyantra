# Static Detection Fix - Task 2B

## Problem
When the UR5 arm moved toward detected objects, the camera view changed, causing:
- Bounding boxes to move/disappear
- Object positions to shift continuously
- Manipulation to fail because target coordinates kept changing

## Solution: Detection Locking
The detection script now uses a **static latch mechanism**:

### How It Works

1. **Active Detection Phase (0-5 seconds)**
   - Script continuously detects bad fruits and fertilizer cans
   - Each detected object's position is calculated in `base_link` frame
   - Transforms are saved in a dictionary: `self.detected_objects`
   - First detection of each object is SAVED (latched)

2. **Automatic Lock (After 5 seconds)**
   - Detection stops processing new objects
   - Only the saved static transforms are republished
   - Object positions become FROZEN in base_link coordinates
   - Camera can move freely without affecting object positions

### Key Features

- **Detection Timeout**: 5 seconds (configurable via `self.detection_timeout`)
- **Static Storage**: All object transforms stored in `self.detected_objects` dictionary
- **Continuous TF Publishing**: Saved transforms are continuously republished with updated timestamps
- **Visual Feedback**: 
  - Before lock: Shows countdown timer
  - After lock: Shows "DETECTION LOCKED - Static Positions"

### Modified Code Locations

**In `task2b_detection.py`:**

1. **Line ~90** - Added storage variables:
```python
self.detected_objects = {}  # Store static transforms
self.detection_locked = False  # Lock flag
self.detection_timeout = 5.0  # Time before locking
self.start_time = self.get_clock().now().nanoseconds / 1e9
```

2. **Line ~390** - Modified `process_image()`:
   - Check if timeout elapsed → lock detection
   - If locked: only republish stored transforms
   - If not locked: detect and save new objects
   - Store transforms in `base_link` frame (not camera_link!)

## Benefits

✅ **Stable Target Positions**: Objects stay at fixed coordinates in world frame  
✅ **Reliable Manipulation**: Arm can approach objects without coordinates changing  
✅ **Camera Movement Safe**: Camera moving with arm doesn't affect saved positions  
✅ **Automatic Operation**: No manual intervention needed - locks after 5 seconds  

## Usage

### Normal Operation
```bash
# Terminal 1 - Gazebo
ros2 launch eyantra_warehouse task2b.launch.py

# Terminal 2 - UR5 Arm  
ros2 launch ur_simulation_gz spawn_arm.launch.py

# Terminal 3 - Detection (will auto-lock after 5 seconds)
ros2 run ur5_control task2b_detection.py

# Wait 5 seconds for detection lock message:
# [INFO] Detection LOCKED! Found X objects. Positions are now STATIC.

# Terminal 4 - Manipulation (start AFTER detection locks)
ros2 run ur5_control task2b_manipulation.py
```

### Adjusting Lock Time

Edit `task2b_detection.py` line ~92:
```python
self.detection_timeout = 5.0  # Change to desired seconds
```

## Verification

### Check Static Positions
After detection locks, verify object frames exist:
```bash
# List all TF frames
ros2 run tf2_tools view_frames

# Check specific object position (should not change)
ros2 run tf2_ros tf2_echo base_link 2203_bad_fruit_1
ros2 run tf2_ros tf2_echo base_link 2203_fertilizer_0_base
```

The position values should remain constant even as the arm/camera moves!

## Troubleshooting

### Objects disappear after arm moves
- Make sure detection locked BEFORE starting manipulation
- Check logs for "Detection LOCKED!" message
- Verify objects are stored in `base_link` frame (not camera_link)

### Detection never locks
- Check if objects are being detected (watch visualization window)
- Increase detection timeout if needed
- Ensure camera can see objects clearly in first 5 seconds

### Wrong object positions
- Detection must happen when arm is at HOME position
- Camera must have clear view of all objects during detection phase
- Check camera intrinsic parameters if positions are consistently off

## Technical Details

### Frame Transformation
- Objects detected in `camera_link` frame
- Immediately transformed to `base_link` frame using TF2
- Only `base_link` transforms are stored and republished
- This ensures positions are relative to robot base (fixed reference)

### TF Publishing Strategy
- During detection: Publish fresh transforms
- After lock: Republish stored transforms with updated timestamps
- Timestamp updates prevent TF tree from becoming stale
- Position/orientation data remains frozen

## Team Information
- **Team ID**: 2203
- **Fix Applied**: Static Detection Locking for Task 2B
- **Date**: 27 October 2025
