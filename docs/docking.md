---
title: Docking & Delivery Subsystem
---

# 🔗 Navigation

- [Home](index.md)
- [The Challenge](challenge.md)
- [General System](general-system.md)
- [Software Subsystem](software.md)
- [Mechanical Subsystem](mechanical.md)
- [Electrical Subsystem](electrical.md)
- [Docking & Delivery Subsystem](docking.md)
- [End User Documentation & BOM](user_docs.md)
- [Areas for Improvement](improvements.md)

---

# Docking & Delivery Subsystem

---

## System Overview and Design Rationale

The docking and delivery subsystem enables the robot to autonomously detect delivery stations, align to them with centimetre-level precision, and execute payload delivery sequences. This subsystem bridges the gap between exploration (where stations are discovered) and mission completion (where balls are delivered).

The system relies on two tightly integrated components: a standalone ArUco detection script (`aruco_live.py`) running directly on the Raspberry Pi, and the docking/firing logic within the mission controller (`exploration_fsm.py`). The detection script captures frames from the RPi Camera V2, estimates marker poses using `cv2.solvePnP`, and publishes them as a `MarkerArray` on `/aruco/markers`. The mission controller subscribes to these detections, transforms them into map-frame coordinates, persists them to YAML, and orchestrates the multi-stage docking approach and station-specific firing sequences.

This architecture was chosen because it separates the real-time computer vision workload (running at camera frame rate) from the mission logic (running event-driven), allowing each to be developed, tested, and tuned independently.

---

## ArUco Detection Pipeline

### Camera and Marker Configuration

| Parameter | Value |
|---|---|
| Camera | RPi Camera Module V2 (Sony IMX219, 8MP) |
| Resolution | 640 x 480 @ 30 FPS |
| ArUco Dictionary | DICT_6X6_250 |
| Marker Size | 5 cm (0.05 m) |
| Pose Solver | `cv2.solvePnP` with `SOLVEPNP_IPPE_SQUARE` |
| Calibration | Camera intrinsic matrix (K) and distortion coefficients loaded from `.npz` file |

### Detection Flow

The `aruco_live.py` script runs as a standalone process on the RPi and performs the following on each frame:

1. **Capture** - Read frame from the camera via V4L2
2. **Grayscale conversion** - Convert to grayscale for marker detection
3. **Marker detection** - `ArucoDetector.detectMarkers()` identifies marker corners and IDs
4. **Pose estimation** - For each detected marker, `cv2.solvePnP` computes the rotation vector (rvec) and translation vector (tvec) from the marker's four corner points and the camera calibration matrix
5. **Quaternion conversion** - The rvec is converted to a 3x3 rotation matrix via `cv2.Rodrigues`, then to a unit quaternion (x, y, z, w) using the Shepperd trace method
6. **ROS 2 publishing** - Each marker's pose is published as a `visualization_msgs/Marker` within a `MarkerArray` on `/aruco/markers`, with position in camera frame (x right, y down, z forward) and orientation as the computed quaternion

The script optionally initialises its own `rclpy` node for publishing, and gracefully degrades to display-only mode if ROS 2 is not available.

---

## Camera-to-Map Coordinate Transform

When the mission controller receives a marker detection on `/aruco/markers`, it applies quality gates before accepting the sighting:

| Gate | Condition | Purpose |
|---|---|---|
| Lateral offset | \|cam_x\| < 0.15 m | Reject oblique sightings with poor pose accuracy |
| Depth range | 0.15 m < cam_z < 2.0 m | Reject too-close (out of focus) or too-far (low resolution) readings |
| TF availability | Valid `map -> base_link` transform | Cannot compute map coordinates without robot pose |

Sightings that pass all gates are transformed to map coordinates through the `camera_to_map()` method:

1. Look up the robot's current position and yaw from the `map -> base_link` TF transform
2. Apply the camera X offset (4 cm lateral from robot centre) and project the camera-frame observation into map coordinates using the robot's heading
3. Extract the marker's **outward normal direction** from the orientation quaternion - this determines the heading the robot must face when docking (approaching perpendicular to the marker face)
4. Store the result as `(map_x, map_y, normal_yaw)` in the `detected_markers` dictionary, keyed by marker ID

The normal yaw is essential for docking - it defines the direction the robot must approach from to face the station head-on, ensuring the launcher is aimed at the receptacle.

---

## YAML State Persistence

Every time a marker pose is updated, it is serialised to `/tmp/aruco_dock_poses.yaml`. On startup, the controller loads any existing poses from this file.

This enables two important behaviours:
- **Cross-attempt persistence** - If a mission attempt fails, the robot can skip exploration on re-attempt since station locations are already known
- **Continuous refinement** - Each new sighting overwrites the stored pose, so the most recent observation (typically the most accurate) is always used for docking

A `clear_cache` parameter (default: True) clears the file on startup since the map frame changes between SLAM runs.

---

## Coarse-to-Fine Docking Strategy

Docking uses a three-stage approach that progressively transfers control from the Nav2 navigation stack to direct visual servoing. This handles the full distance range from initial discovery (~2 m) down to the final docking distance (0.10 m), passing through the camera's minimum reliable detection range (~0.20 m).

### Stage 1 - Coarse Nav2 Approach

Using the YAML-stored marker pose, the robot navigates to a standoff point **0.6 m** from the marker along its normal direction. This uses Nav2's global planner for collision-free routing through the maze. The approach heading is set to face toward the marker.

If the robot is already closer than 0.6 m (e.g., it discovered the marker at close range), this step is skipped.

### Stage 2 - Live Marker Acquisition and Runway

Upon reaching the approach area, the controller waits up to 5 seconds for a fresh live camera sighting of the target marker. If the marker is not immediately visible, a **360-degree spin search** is performed (rotating at 0.4 rad/s for up to 15 seconds).

Once a live sighting is acquired, the stored pose is updated with the fresh observation, correcting for any SLAM drift since the original detection. A new Nav2 goal is set at **0.35 m** from the live marker position - the "runway" point, oriented to face the marker.

### Stage 3 - Visual Servo Approach (cmd_vel)

The final approach bypasses Nav2 entirely and uses direct `cmd_vel` velocity commands with proportional control. This is necessary because Nav2's goal tolerance (~25 cm) is too coarse for the mission's docking requirements (~2 cm lateral).

The approach is split into three sub-phases:

**Phase 1 - Align:** The robot rotates in place, with angular velocity proportional to the marker's lateral offset (cam_x). The phase exits when lateral error drops below the alignment tolerance.

**Phase 2 - Drive:** The robot moves forward with velocity proportional to distance error (cam_z - target), while simultaneously correcting lateral offset. This continues until either the target distance is reached or cam_z drops below the blind threshold (0.20 m).

**Phase 3 - Dead-reckon:** Below 0.20 m the marker becomes too close to detect reliably. The robot drives the remaining distance at half speed with no camera feedback, based on the last known cam_z measurement.

### Control Parameters

| Parameter | Station A | Station B | Description |
|---|---|---|---|
| Alignment tolerance | 3 cm | 1 cm | Maximum lateral error before proceeding |
| Target distance | 0.10 m | 0.10 m | Final dock distance from marker |
| Blind threshold | 0.20 m | 0.20 m | cam_z below which camera is unreliable |
| Max linear speed | 0.08 m/s | 0.08 m/s | Forward speed cap during approach |
| Max angular speed | 0.50 rad/s | 0.50 rad/s | Rotation speed cap during alignment |
| Marker lost timeout | 3.0 s | 3.0 s | Tolerated gap before aborting |

Station B uses a tighter alignment tolerance (1 cm vs 3 cm) because the moving receptacle requires more precise lateral positioning to land balls consistently.

### Retry Logic

Each docking attempt allows up to **3 retries**. Between attempts, the robot returns to the approach area and re-acquires the live marker. This resolves transient failures caused by lighting changes, brief marker occlusion, or minor SLAM drift.

---

## Station A - Timed Fire Sequence

After successful docking at Station A:

1. Flywheel spin-up via `/start_flywheel` service call
2. Wait **6.0 s**, then fire ball 1 via `/fire_ball` service call
3. Wait **9.0 s**, then fire ball 2
4. Wait **1.0 s**, then fire ball 3
5. Flywheel stop via `/stop_flywheel` service call
6. **Undock** - reverse 0.30 m at 0.06 m/s to clear the station

The delays `[6.0, 9.0, 1.0]` are the team-specific timing pattern. The flywheel runs continuously across all three shots to maintain consistent RPM. Each delay represents the wait *before* that ball is fired, not after.

---

## Station B - Trigger-Based Fire Sequence

Station B uses a fundamentally different firing approach since the receptacle oscillates on a motorised rail:

1. Dock using the same coarse-to-fine pipeline (with 1 cm lateral tolerance)
2. Flywheel spin-up (stays running throughout)
3. **Wait for trigger marker** - A secondary ArUco marker (ID 2) is placed inside the moving receptacle's hole. When `marker_cb` detects this ID, it means the hole is aligned with the robot, and it sets a `threading.Event`
4. **Fire** - On trigger detection, immediately fire one ball
5. **Cooldown** - Wait for the trigger marker to leave the camera frame for at least **5.0 seconds** before accepting the next trigger. This prevents double-firing on the same pass of the receptacle
6. Repeat steps 3-5 for all 3 balls
7. Flywheel stop

If no trigger is detected within 30 seconds for any ball, the firing sequence aborts to prevent indefinite waiting.

### Marker ID Assignments

| ID | Purpose | Location |
|---|---|---|
| 0 | Station A dock marker | Fixed on Station A receptacle |
| 1 | Station B dock marker | Fixed on Station B rail assembly |
| 2 | Station B trigger marker | Inside the moving receptacle hole |

---

## Docking Testing and Performance

### Objective

Validate that the robot can reliably detect, approach, and dock at both station types with sufficient precision for successful ball delivery.

### Methodology

Testing was conducted in the lab environment with markers mounted at representative heights and angles:

- **Static docking tests**: Robot positioned at known distances (0.3 m to 2.0 m) from markers, measuring detection success rate and pose estimation accuracy
- **Full approach tests**: Complete coarse-to-fine docking sequence from exploration distance, measuring final lateral offset and angular error at dock position
- **Dynamic station tests**: Station B receptacle oscillating at operational speed, measuring trigger detection reliability and ball landing success rate

### Results

| Metric | Target | Result |
|---|---|---|
| Detection range | 0.3 - 2.0 m | Achieved |
| Detection success rate (static) | >= 95% | Achieved |
| Final lateral error (Station A) | < 3 cm | Achieved |
| Final lateral error (Station B) | < 1.5 cm | Achieved |
| Docking success rate (with retries) | >= 90% | Achieved |
| Trigger detection reliability | >= 95% | Achieved |
| Ball delivery success (Station A) | 3/3 balls | Achieved across multiple runs |
| Ball delivery success (Station B) | >= 2/3 balls | Achieved |

### Key Findings

- The 3-phase visual servo approach (align, drive, dead-reckon) reliably bridges the gap between Nav2's goal tolerance and the mission's precision requirements
- The dead-reckoning phase is critical - without it, the robot would stop at ~0.20 m (blind threshold) rather than reaching the 0.10 m target distance
- Station B's tighter alignment tolerance (1 cm) was necessary - at 3 cm lateral error, balls were missing the oscillating receptacle on roughly 40% of passes
- The trigger cooldown (5 s) effectively prevents double-fires, but requires the receptacle oscillation period to be longer than this cooldown

---

## Key Implementation References

| Feature | Location |
|---|---|
| ArUco detection and pose estimation | `aruco_live.py` |
| Camera-to-map coordinate transform | `exploration_fsm.py` - `camera_to_map()` |
| Quality gates for marker sightings | `exploration_fsm.py` - `marker_cb()` |
| YAML persistence | `exploration_fsm.py` - `save_poses()`, `load_poses()` |
| Coarse Nav2 approach | `exploration_fsm.py` - `execute_docking()` |
| Visual servo approach | `exploration_fsm.py` - `_cmd_vel_approach()` |
| Station A timed firing | `exploration_fsm.py` - `_fire_sequence()` |
| Station B trigger firing | `exploration_fsm.py` - `_fire_station_b_sequence()` |
| Spin search for lost markers | `exploration_fsm.py` - `_spin_until_marker()` |
| Undock reverse | `exploration_fsm.py` - `execute_undock()` |

---

## [End User Documentation & BOM →](user_docs.md)