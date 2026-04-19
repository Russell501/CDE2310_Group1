---
title: Requirement Specifications
---

# 🔗 Navigation

- [Home](index.md)
- **Requirements** ← _You are here_
- [Con-Ops](conops.md)
- [High Level Design](high-level-design.md)
- [Sub System Design](subsystem-design.md)
- [Interface Control Documents](icd.md)
- [Software Development](software.md)
- [Testing](testing.md)
- [User Manual](user-manual.md)
- [Bill-Of-Materials](bill-of-materials.md)
- [Electrical Subsystem](electrical.md)
- [Mechanical Subsystem](mechanical.md)
- [Improvements](improvements.md)

---
# Requirement Specifications

---

## Problem Definition

As part of CDE2310 — Fundamentals of Systems Design (AY 25-26), each team must design an autonomous mobile robot (AMR) that navigates an unknown warehouse maze, detects delivery stations via ArUco markers, and dispenses ping pong balls into receptacles — all within a 25-minute window with no human teleoperation. The mission comprises three objectives of increasing difficulty:

| Objective | Station | Description |
|---|---|---|
| Primary | **Station A** — Static Delivery | Dock at a fixed receptacle and deliver 3 balls in a team-specific timing pattern; all balls must remain for ≥ 10 s |
| Primary | **Station B** — Dynamic Delivery | Track an oscillating receptacle on a motorised rail and deliver 3 balls with predictive timing |
| Bonus | **Station C** — Elevator Delivery | Navigate to a lift lobby, ascend to Level 2 via API calls, and complete a final delivery |

Line-following navigation and human-solved mapping are strictly prohibited.

---

## Mission Objectives

### Core Objectives

1. **Autonomous Maze Exploration & Mapping** — Explore the unknown maze using Cartographer SLAM (LiDAR + IMU + wheel encoders) and frontier-based exploration (explore_lite) to build a complete occupancy grid map without human input.

2. **Station Detection** — Detect ArUco markers (DICT_6X6_250) via RPi Camera V2 during exploration, extracting 6D pose (tvec/rvec) to determine each station's world-frame position and orientation.

3. **Precision Docking** — Align to each station within docking tolerances (lateral ±2 cm, angular ±3°, distance 0.2–1.0 m) using closed-loop visual feedback before initiating any delivery.

4. **Static Delivery (Station A)** — Spin up dual flywheels, dispense 3 balls via servo feeder with team-specific timing delays, and confirm each launch via IR break-beam. All balls must remain in the receptacle for ≥ 10 seconds.

5. **Dynamic Delivery (Station B)** — Track the oscillating receptacle's velocity over a sliding window of frames, compute a trigger offset compensating for ~300–500 ms system latency, and fire at the predicted intercept position.

6. **Full-System Integration** — Deploy all subsystems from a single ROS 2 launch file with autonomous state transitions (Exploration → Docking → Delivery → Transit), fault recovery (frontier blacklisting, marker search patterns), and continuous ROSBAG recording.

### Bonus Objective

7. **Elevator Traversal (Station C)** — Navigate to the lift lobby, enter via camera-based alignment (bypassing LiDAR metallic reflections), issue HTTP POST/GET API calls to ascend to Level 2, suspend and resume SLAM across the floor transition, and complete a final delivery.

---

## Project Deliverables

| Stakeholder Requirement | Project Deliverable |
|---|---|
| Traverse the maze autonomously from the start zone | Real-time LiDAR-based SLAM (Cartographer) with IMU and wheel encoder fusion to navigate from a fixed starting point |
| Explore and map an unknown maze layout | Frontier-based autonomous exploration (explore_lite) with Nav2 goal publication and frontier blacklisting on failure |
| Detect and identify delivery stations | ArUco marker detection pipeline (DICT_6X6_250) running concurrently during exploration via RPi Camera V2 |
| Localise station positions with precision | 6D pose estimation from ArUco markers providing translation and rotation vectors for docking alignment |
| Deliver payload to a static station | Timed flywheel launch sequence with servo-fed ball dispensing, IR break-beam confirmation, and team-specific delay pattern |
| Deliver payload to a dynamic (moving) station | Predictive lead-logic targeting with velocity estimation, latency compensation, and synchronised firing |
| Navigate a multi-level environment (bonus) | Lift API integration with SLAM suspension/resumption and global relocalisation on Level 2 |
| Operate as a fully autonomous system | Complete mission execution from a single ROS 2 launch file with no human intervention after activation |
| Record mission data for post-analysis | Continuous ROSBAG recording of all relevant topics throughout the mission |

---

## Functional Requirements

| ID | Requirement | Description |
|---|---|---|
| FR-01 | Autonomous Navigation | Navigate an unknown warehouse maze without human teleoperation or pre-mapped data |
| FR-02 | Station Detection | Detect and identify delivery stations via ArUco markers during autonomous exploration |
| FR-03 | Static Ball Delivery | Deliver 3 ping pong balls into Station A receptacle following a team-specific timing pattern (9,1); all balls must remain in the receptacle for ≥ 10 seconds |
| FR-04 | Dynamic Ball Delivery | Deliver 3 ping pong balls into the oscillating Station B receptacle using predictive timing; all balls must remain for ≥ 10 seconds |
| FR-05 | Obstacle Avoidance | Avoid all static and dynamic obstacles throughout the maze with zero collisions |
| FR-06 | Elevator Traversal | Autonomously enter lift, call API to ascend, and navigate Level 2 (bonus objective) |
| FR-07 | Mission Completion | Complete all primary tasks within the 25-minute execution window |
| FR-08 | Data Recording | Record full ROSBAG data for post-mission analysis and verification |

---

## Non-Functional Requirements

| Requirement Category | Non-Functional Requirement |
|---|---|
| Performance | Complete full primary mission (Stations A and B) within the 25-minute time window |
| Reliability | Tolerate varying lighting conditions, approach angles, and sensor noise across multiple runs |
| Safety | Avoid all collisions; activate flywheel launcher only when stationary at a validated docking position with alignment tolerances met |
| Power Efficiency | Operate within the 11.1V 1800mAh LiPo battery budget for at least 4 full trial runs on a single charge |
| Accuracy — Navigation | Maintain SLAM map resolution ≤ 0.05 m/cell with odometry drift corrected via loop closure |
| Accuracy — Docking | Achieve docking alignment within ±2 cm lateral, ±3° angular, and target firing distance (0.2–1.0 m range) |
| Accuracy — Detection | Detect ArUco markers reliably within 0.3–2.2 m operational range at 640×480 resolution |
| Latency | DWB local planner response latency < 100 ms from LiDAR scan to cmd_vel publication |
| Scalability | Support addition of sensors, stations, or logic modules without architectural changes |
| Maintainability | Modular ROS 2 node architecture allowing independent testing and replacement of any subsystem |
| Usability | Single launch file deployment; intuitive setup procedure completable within the setup period |
| Verification | Log all detection events, docking attempts, launch triggers, and navigation goals via ROSBAG |

---

## System Specifications

| ID | Parameter | Specification |
|---|---|---|
| SS-01 | SLAM Map Resolution | ≤ 0.05 m/cell |
| SS-02 | Obstacle Reaction Latency | DWB local planner < 100 ms (Nav2 default 20 Hz ≈ 50 ms) |
| SS-03 | Marker Detection Range | ArUco DICT_6X6_250 detectable at 0.3–2.2 m (640×480, 10 cm marker) |
| SS-04 | Docking Tolerance | Lateral ±2 cm, Angular ±3°, Distance 0.2–1.0 m |
| SS-05 | Flywheel Launch RPM | 2,100–14,350 RPM (variable via PWM for distance calibration) |
| SS-06 | Ball Feed Rate | Servo actuation ≤ 0.12 s/60° (SG90); inter-ball delay for RPM recovery and timing compliance |
| SS-07 | Power Envelope | All subsystems within 11.1V 1800mAh LiPo budget (~4 runs per charge) |
| SS-08 | Dynamic Delivery Latency | Total system latency Δt ≈ 300–500 ms compensated via predictive lead logic |

---

## Constraints

| Constraint | Description |
|---|---|
| Robot Platform | Must use the TurtleBot3 Burger platform as the base robot |
| Camera | Must use the provided RPi Camera V2 8MP for all vision-based detection |
| Navigation Method | Line-following navigation and human-solved mapping are strictly prohibited |
| Markers | Maximum of 6 markers allowed (2 per delivery zone), installed and removed within the 25-minute window |
| Physical Dimensions | Must be compact enough to navigate narrow maze corridors without contact |
| LiDAR Clearance | All mounted components must not obstruct the LDS-02 LiDAR's 360° field of view |
| Weight & Stability | Must maintain low centre of gravity for stable navigation; mounted payload must not shift during transit |
| Power | Single 11.1V 1800mAh 3S LiPo battery; RPi must remain above 4.75V operating threshold under all load conditions |
| Onboard Processing | All computation runs on Raspberry Pi 4B and OpenCR 1.0; no offboard compute permitted during mission |
| Budget | Total additional component cost must remain within department budget (estimated ~S$63 BOM) |
| Time | 25-minute window includes setup, mission execution, and cleanup |
| Autonomy | No manual teleoperation or physical intervention permitted after mission start declaration |

---

## Scoring Overview

Mission scoring is split into two components:

**Achievement-Based Scoring (up to 100 points + 20 bonus):**
Points are awarded for successful detection, alignment, and delivery at each station. Partial credit is available for landmark detection, frontal orientation, and partial delivery. Achievement objectives take precedence over timing — completing both stations slowly outscores completing one station quickly.

**Time-Based Competitive Scoring:**
Among teams with equal objective completion, ranking is determined by the time taken to deliver all payloads and the time to achieve full map closure. Both timings are recorded independently. No timing is recorded if milestones are not achieved.
