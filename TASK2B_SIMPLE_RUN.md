# Task 2B Simple Manipulation - Quick Start

## What Changed
Completely rewrote the manipulation script to follow Task 1C's simple pattern:
- No complex state machine
- Just a simple waypoint list (like Task 1C)
- Approach → Pick → Lift → Drop for each object
- Fertilizer cans placed at their ArUco bases
- Bad fruits placed at back tray

## How It Works
1. Wait 3 seconds after start
2. Scan for all objects (fertilizer + fruits)
3. Build waypoint sequence:
   - For each fertilizer: APPROACH → PICK → LIFT → DROP_AT_BASE
   - For each fruit: APPROACH → PICK → LIFT → DROP_AT_TRAY
4. Execute waypoints one by one (exactly like Task 1C)

## Run Instructions

### Terminal 1: Detection Node
```bash
cd /home/kasinath/colcon_ws
source install/setup.bash
ros2 run ur5_control task2b_detection.py
```

### Terminal 2: Manipulation Node
```bash
cd /home/kasinath/colcon_ws  
source install/setup.bash
ros2 run ur5_control task2b_manipulation.py
```

## Expected Output
```
[INFO] Task 2B Manipulation - Team 2203
[INFO] Services ready!
... wait 3 seconds ...
[INFO] Scanning for objects...
[INFO] Found fertilizer can 3
[INFO] Found fertilizer can 6
[INFO] Found bad fruit 1
[INFO] Found bad fruit 2
[INFO] Found bad fruit 3
[INFO] Built sequence with X waypoints
[INFO]   - 2 fertilizer cans
[INFO]   - 3 bad fruits
[INFO] Moving to: HOME
[INFO] Reached HOME
[INFO] Moving to: APPROACH_fertiliser_can_3
[INFO] Reached APPROACH_fertiliser_can_3
[INFO] Moving to: PICK_fertiliser_can_3
[INFO] Reached PICK_fertiliser_can_3
[INFO] Attaching fertiliser_can_3...
[INFO] Attached fertiliser_can_3!
[INFO] Moving to: LIFT_fertiliser_can_3
... continues for all objects ...
[INFO] All tasks completed!
```

## Key Features
- **Simple**: No complex state machine, just waypoints
- **Task 1C Pattern**: Same control loop structure
- **Correct Order**: Fertilizer first, then fruits
- **Direct Approach**: No intermediate waypoints causing confusion
- **Clear Logging**: See every step happening

## Troubleshooting
- **No objects found**: Check detection node is running
- **Can't attach**: Check gripper services are available
- **Joint limits**: Increase APPROACH_OFFSET (currently 0.20m)
