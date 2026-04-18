---
title: Software Subsystem
---

# 🔗 Navigation

- [Home](index.md)
- [The Challenge](challenge.md)
- [General System](general-system.md)
- [Software Subsystem](software.md)
- [Mechanical Subsystem](mechanical.md)
- [Electrical Subsystem](electrical.md)
- [End User Documentation & BOM](user_docs.md)
- [Areas for Improvement](improvements.md)

---

# Software Subsystem

## Exploration Strategy

Our robot uses a **multi-layered exploration pipeline** to ensure all delivery station markers are found, even in challenging maze configurations where a single strategy might miss dead-end corridors or occluded markers.

The exploration proceeds through up to four stages, each more exhaustive than the last. The system exits early as soon as all required markers (Station A and Station B) have been detected:

| Stage | Strategy | Purpose |
|---|---|---|
| Phase 1a | Timed BFS Coverage Sweep | Rapid initial map seeding across known free space (15s budget) |
| Phase 1b | explore_lite (frontier-based) | Systematic frontier exploration with up to 600s timeout |
| Phase 1c | Frontier Cleanup (short-hop BFS) | Targeted cleanup of remaining frontiers that explore_lite missed |
| Phase 2 | Full Coverage Sweep | Exhaustive BFS visit of every reachable free cell as a last resort |

This layered approach ensures the robot does not waste time on exhaustive sweeps when markers are found early, but still has fallback strategies for difficult maze layouts.

---

### Phase 1a - Initial BFS Sweep

Before launching explore_lite, the controller runs a quick BFS coverage sweep over the current occupancy grid. Starting from the robot's position, it generates waypoints at regular grid spacing across all known free cells and navigates to them sequentially. This seeds the map rapidly and may detect markers before the heavier explore_lite process is even started.

The sweep terminates when either:
- The time budget (15 seconds) expires
- All required markers are found mid-transit

---

### Phase 1b - Frontier Exploration (explore_lite)

The primary exploration phase uses the `explore_lite` ROS 2 package, launched as a subprocess. explore_lite identifies frontiers (boundaries between mapped free space and unknown territory) in the occupancy grid and publishes navigation goals to the Nav2 action server.

The controller subscribes to `explore/status` and waits for the `EXPLORATION_COMPLETE` signal. A configurable timeout (600s) prevents indefinite waiting if explore_lite gets stuck.

Throughout this phase, the ArUco detection script (`aruco_live.py`) runs directly on the RPi as a standalone process, publishing detected markers to ROS 2. Any marker sighting is immediately transformed to map-frame coordinates and persisted.

---

### Phase 1c - Frontier Cleanup (Short-Hop BFS)

If explore_lite completes but required markers are still missing, an in-process frontier cleanup runs. This addresses a common failure mode where explore_lite declares completion while small unexplored pockets remain.

The cleanup algorithm:
1. BFS from the robot's current position across free cells to find the nearest frontier
2. Trace the BFS parent chain to get a path through known free space
3. Navigate along this path in short hops (0.6m increments)
4. After each hop, the map has updated from new LiDAR scans, so the BFS is recomputed

Key robustness features:
- **Time-decaying blacklist** (TTL = 90s, radius = 0.30m) prevents oscillation between unreachable frontiers
- **Costmap clearance checking** (0.22m) ensures hop targets are safely navigable
- **Recovery spins** (360°) are triggered after 3 consecutive navigation failures
- **Frontier clustering** groups connected frontier cells and snaps goals to reachable centroids

---

### Phase 2 - Coverage Sweep (Last Resort)

If markers remain undetected after all frontier strategies, a full BFS coverage sweep visits every reachable free cell in the occupancy grid. Once at least one marker is found, a partial timeout (120s) begins to avoid indefinite sweeping for the remaining marker.

---

## SLAM and Mapping

### Cartographer SLAM

The robot uses **Google Cartographer** for real-time SLAM, fusing data from three sources:

| Source | Purpose |
|---|---|
| LDS-02 LiDAR | 360° range measurements for scan matching and map construction |
| Wheel Encoders | Odometry estimates for inter-scan motion prediction |
| IMU | Inertial data for orientation refinement and drift correction |

Cartographer generates an occupancy grid map at 0.05 m/cell resolution, classifying each cell as free (0), occupied (100), or unknown (-1). The map is published on the `/map` topic and consumed by both Nav2 and the mission controller.

### Why Cartographer Over Alternatives

| Algorithm | ROS 2 Support | Loop Closure | CPU Load | Map Quality |
|---|---|---|---|---|
| Cartographer (selected) | Via port | Good | High | Very High |
| SLAM Toolbox | Native | Excellent | Medium | High |
| Gmapping | ROS 1 only | None | Low | Medium |

Cartographer was selected for its superior map quality, which is critical for reliable docking alignment. The higher CPU load is acceptable given the Raspberry Pi 4B's capacity for our workload.

---

## Path Planning and Control

### Nav2 Stack Configuration

Navigation uses the **Nav2** stack with the following component choices:

| Component | Selection | Role |
|---|---|---|
| Global Planner | NavFn (A*) | Computes shortest collision-free path through the global costmap |
| Local Controller | DWB (Dynamic Window Approach) | Generates real-time velocity commands for obstacle avoidance |
| Costmap | Global + Local | Inflated obstacle layers for safe path planning |
| Recovery Behaviors | Spin, BackUp, Wait | Automated recovery when the robot gets stuck |

### Why DWB Over Alternatives

DWB was selected over MPPI and Pure Pursuit for its proven reliability in indoor corridor environments and lower computational overhead on the Raspberry Pi 4B. While MPPI offers smoother trajectories in complex open spaces, DWB's simpler sampling approach provides sufficient performance in the structured maze corridors of this mission, with response latency consistently under 100ms.

### Frontier-Based Goal Selection

During exploration, frontier goals are generated by explore_lite and published directly to the Nav2 action server. The mission controller monitors goal outcomes:
- **Successful goals** allow the robot to continue exploring
- **Failed goals** trigger frontier blacklisting to prevent re-attempts at unreachable locations

By separating goal discovery (explore_lite / BFS) from path execution (Nav2 planner + controller), the system maintains a clean division of concerns and allows each component to be tuned independently.

---

## ArUco Detection Pipeline

### Marker Detection

A standalone Python script (`aruco_live.py`) runs directly on the Raspberry Pi, capturing frames from the RPi Camera V2 and publishing detected marker poses to ROS 2. Unlike the other ROS 2 nodes which are launched via the launch file, this script runs as an independent process and optionally initialises its own `rclpy` node to publish on `/aruco/markers`. The pipeline:

1. **Capture** - Frames at 640x480 resolution, 30 FPS
2. **Preprocessing** - Grayscale conversion for improved bit-pattern recognition under varying warehouse lighting
3. **Detection** - `cv2.aruco` with `DICT_6X6_250` dictionary identifies marker boundaries via `ArucoDetector`
4. **Pose Estimation** - `cv2.solvePnP` with the `IPPE_SQUARE` solver extracts the rotation vector (rvec) and translation vector (tvec) from the marker's four corner points and the camera's intrinsic calibration matrix (loaded from a `.npz` calibration file)
5. **Quaternion Conversion** - The rvec is converted to a rotation matrix via `cv2.Rodrigues`, then to a unit quaternion (x, y, z, w) for ROS 2 compatibility
6. **Publishing** - Detected markers are published on `/aruco/markers` as a `visualization_msgs/MarkerArray`, with each entry containing the marker ID, camera-frame position, and orientation quaternion

### Why DICT_6X6_250

The 6x6 dictionary provides a larger set of unique marker IDs (250) while maintaining robust detection at the distances used in this mission. The default marker size is 5cm, and detection is reliable within the camera's operational range at 640x480 resolution. The dictionary choice also allows us to use distinct IDs for Station A (ID 0), Station B (ID 1), and the Station B trigger marker (ID 2) without risk of cross-detection.

### Quality Gates

The mission controller applies quality gates before accepting a marker sighting:
- Lateral offset |cam_x| must be < 0.15m (rejects oblique sightings)
- Depth cam_z must be between 0.15m and 2.0m (rejects too-close or too-far readings)
- Valid TF transform must be available for coordinate conversion

---

## Camera-to-Map Coordinate Transform

When a marker passes quality gates, the controller converts the camera-frame observation to map-frame coordinates:

1. Look up the current `map -> base_link` TF transform to get the robot's world pose and yaw
2. Apply the camera X offset (4cm lateral) and project the camera-frame point (cam_x, cam_z) into the map frame using the robot's position and heading
3. Extract the marker's **outward normal direction** from its orientation quaternion - this gives the heading the robot should face when docking
4. Store the result as `(map_x, map_y, normal_yaw)` in the detected markers dictionary

The normal yaw is critical for docking - it defines the direction the robot must approach from to face the station head-on.

---

## YAML State Persistence

All detected marker poses are serialised to `/tmp/aruco_dock_poses.yaml` after every sighting update. On startup, the controller loads any existing poses from this file.

This serves two purposes:
- **Cross-attempt persistence** - If a mission attempt fails partway through, the robot can skip exploration entirely on re-attempt since it already knows where the stations are
- **Pose refinement** - Each new sighting overwrites the stored pose, so the most recent (and typically most accurate) observation is always used

A `clear_cache` parameter (default: True) controls whether the pose file is cleared on startup. This should be True for fresh SLAM runs since the map frame changes between runs.

---

## Docking Logic

### Coarse-to-Fine Approach

Both stations use a three-stage docking pipeline that progressively transfers control from Nav2 to direct visual servoing:

**Stage 1 - Coarse Nav2 Approach (YAML Pose)**
The robot navigates to a standoff point 0.6m from the stored marker position along the marker's normal direction. This uses Nav2's global planner for collision-free routing through the maze.

**Stage 2 - Refined Nav2 Runway (Live Marker)**
Upon arrival, the controller waits for a fresh live camera sighting of the marker. If not immediately visible, a 360° spin search is performed. The live pose corrects for any SLAM drift accumulated since the original sighting. A new Nav2 goal is set at 0.35m from the live marker position.

**Stage 3 - Visual Servo Approach (cmd_vel)**
The final approach uses direct `cmd_vel` velocity commands with proportional control, bypassing Nav2 entirely. This is split into three sub-phases:

| Sub-phase | Behaviour | Exit Condition |
|---|---|---|
| Align | Rotate in place, angular velocity proportional to cam_x error | \|cam_x\| < alignment tolerance |
| Drive | Forward velocity proportional to distance error, angular correction on lateral error | cam_z drops below blind threshold (0.20m) or target reached |
| Dead-reckon | Fixed forward velocity, no camera feedback | Remaining distance covered at half speed |

The dead-reckoning phase handles the camera's blind zone - below ~0.20m, the marker becomes too close to detect reliably, so the robot drives the remaining gap based on the last known distance.

### Control Parameters

| Parameter | Value | Description |
|---|---|---|
| `K_LINEAR` | 0.3 m/s per m | Proportional gain for forward velocity |
| `K_ANGULAR` | 1.5 rad/s per m | Proportional gain for angular correction |
| `MAX_LINEAR` | 0.08 m/s | Maximum forward speed during approach |
| `MAX_ANGULAR` | 0.50 rad/s | Maximum angular speed during alignment |
| `ALIGN_TOL_A` | 0.03 m | Lateral tolerance for Station A |
| `ALIGN_TOL_B` | 0.01 m | Lateral tolerance for Station B (tighter due to moving target) |
| `TARGET_DISTANCE` | 0.10 m | Final dock distance from marker |
| `BLIND_THRESHOLD` | 0.20 m | cam_z below which camera detection is unreliable |

### Retry Logic

Each docking attempt allows up to 3 retries (`MAX_DOCK_RETRIES`). Between attempts, the robot returns to the approach area and re-acquires the live marker, which often resolves transient alignment failures caused by lighting changes or brief marker occlusion.

---

## Station A - Timed Fire Sequence

After successful docking at Station A, the robot executes a timed firing sequence:

1. **Flywheel spin-up** via `/start_flywheel` service call
2. **Ball 1** - Wait 6.0s, then fire via `/fire_ball` service call
3. **Ball 2** - Wait 9.0s, then fire
4. **Ball 3** - Wait 1.0s, then fire
5. **Flywheel stop** via `/stop_flywheel` service call
6. **Undock** - Reverse 0.30m at 0.06 m/s to clear the station

The delays [6.0, 9.0, 1.0] are the team-specific timing pattern assigned in Week 7. The flywheel runs continuously across all three shots to maintain consistent RPM.

---

## Station B - Trigger-Based Fire Sequence

Station B uses a fundamentally different firing strategy since the receptacle is oscillating:

1. **Dock** using the same coarse-to-fine pipeline (with tighter 1cm lateral tolerance)
2. **Flywheel spin-up** via service call (stays running throughout)
3. **Wait for trigger** - A secondary ArUco marker (ID 2) is mounted on the moving receptacle. The controller's `marker_cb` sets a `threading.Event` when this trigger marker is detected
4. **Fire** - On trigger detection, immediately fire one ball
5. **Cooldown** - Wait for the trigger marker to leave the camera frame for at least 5.0s before accepting the next trigger (prevents double-firing on the same pass)
6. Repeat steps 3-5 for all 3 balls
7. **Flywheel stop**

If no trigger is detected within 30 seconds for any ball, the sequence aborts to prevent indefinite waiting.

---

## Mission Phase Ordering

The controller handles a common edge case where Station A is detected after Station B:

1. If Station A is detected during exploration, dock and deliver at Station A first, then Station B
2. If only Station B was found initially, dock and deliver at Station B first
3. After Station B, re-check if Station A has been detected (it may have been spotted during Station B transit)
4. If yes, execute Station A docking and delivery as a late retry

This ensures both stations are always attempted regardless of discovery order.

---

## System Launch Architecture

The entire robot system launches from a single entry point:

### `bringup_all.launch.py`

This top-level launch file starts three components in dependency order:

1. **Cartographer** - Begins SLAM and waits for LiDAR + IMU data
2. **Nav2** - Starts the navigation stack once Cartographer's map is available
3. **Mission FSM Node** - Launches the `UltimateMissionController` once Nav2 is ready

Additional nodes/processes:
- `aruco_live.py` (standalone script, run separately on RPi - captures camera frames and publishes marker detections to `/aruco/markers`)
- Launcher hardware controller (flywheel + servo services)

This single-file deployment satisfies the bonus scoring criteria and ensures all nodes start in the correct order with proper dependencies.

---

## Tuning

### Nav2 Parameter Configuration (`burger.yaml`)

Located at `param/burger.yaml`, this file configures:
- Global planner (NavFn) - tolerance, costmap usage
- Local controller (DWB) - velocity limits, trajectory scoring
- Costmap layers - obstacle marking, inflation radius, resolution
- Recovery behaviors - spin distance, backup distance, wait duration
- Robot footprint radius

After modifying parameters, rebuild with:
```bash
colcon build --packages-select <package_name>
```

### Key Nav2 Parameters

| Parameter | Description | Docs |
|---|---|---|
| NavFn tolerance | How close the planner needs to get to the goal | [NavFn guide](https://docs.nav2.org/configuration/packages/configuring-navfn.html) |
| DWB velocity limits | Max/min linear and angular speeds | [DWB guide](https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html) |
| Inflation radius | Safety buffer around obstacles | [Costmap guide](https://docs.nav2.org/configuration/packages/configuring-costmaps.html) |
| xy_goal_tolerance | Nav2 goal completion threshold (0.25m) | [BT Navigator](https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html) |

### Mission Controller Parameters

| Parameter | Value | Description |
|---|---|---|
| `TARGET_DISTANCE` | 0.10 m | Final dock distance from marker |
| `NAV2_APPROACH_DISTANCE` | 0.35 m | Runway distance for live marker approach |
| `YAML_APPROACH_DISTANCE` | 0.60 m | Coarse approach distance using stored pose |
| `MAX_DOCK_RETRIES` | 3 | Maximum docking attempts per station |
| `UNDOCK_DISTANCE` | 0.30 m | Reverse distance after Station A delivery |
| `STATION_A_FIRE_DELAYS` | [6.0, 9.0, 1.0] | Team-specific timing delays (seconds before each ball) |
| `STATION_B_BALLS` | 3 | Number of balls to fire at Station B |
| `TRIGGER_COOLDOWN` | 5.0 s | Wait after firing for trigger marker to clear |
| `EXPLORATION_TIMEOUT` | 600 s | Maximum time for explore_lite before forcing proceed |
| `INITIAL_BFS_DURATION` | 15 s | Time budget for initial BFS sweep |
| `FRONTIER_BLACKLIST_TTL` | 90 s | Time-decaying blacklist entry lifetime |
| `FRONTIER_BLACKLIST_RADIUS` | 0.30 m | Exclusion radius around failed frontier goals |
| `FRONTIER_HOP_DISTANCE` | 0.60 m | Maximum travel per frontier cleanup hop |

---

## Visualization and Debugging

### RViz Monitoring

During operation, the following data is visualized in RViz:
- Live occupancy grid map from Cartographer
- Robot pose and TF tree
- Nav2 planned paths and costmaps
- ArUco marker detections (when available)

### Mission Phase Logging

The controller publishes the current mission phase on `/mission_phase` as a String topic. Phase transitions are also logged to the ROS 2 logger, providing a complete timeline of the mission for post-analysis.

At Station B, additional diagnostic topics track:
- `/station_b_trigger_count` - Number of trigger marker detections
- `/station_b_balls_fired` - Number of balls successfully fired

---

## Safety Mechanisms

- **Immediate goal cancellation** - If a critical fault is detected (e.g., all markers found during a sweep), active Nav2 goals are cancelled immediately to avoid wasting time
- **Flywheel safety** - Flywheel activation only occurs after successful docking; the flywheel is always stopped via service call after each firing sequence
- **Marker loss handling** - If the tracked marker is lost during visual servo for more than 3 seconds, the approach is aborted and the robot stops
- **Mission abort** - If neither station marker is found after all exploration strategies, the mission aborts cleanly rather than entering an undefined state
- **Undock after delivery** - The robot always reverses after Station A to prevent blocking the receptacle

---

## [Mechanical Subsystem →](mechanical.md)