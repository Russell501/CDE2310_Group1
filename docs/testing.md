---
title: Testing Documentation
---

# 🔗 Navigation

- [Home](index.md)
- [Requirements](requirements.md)
- [Con-Ops](conops.md)
- [High Level Design](high-level-design.md)
- [Sub System Design](subsystem-design.md)
- [Interface Control Documents](icd.md)
- [Software Development](software.md)
- **Testing** ← _You are here_
- [User Manual](user-manual.md)
- [Bill-Of-Materials](bill-of-materials.md)

---
# Testing Documentation

---

## 1. System Integration Test (`system_test.py`)

### 1.1 Overview

A pre-flight check that verifies all software components are wired up correctly without running the full mission. Designed to mirror `exploration_fsm.py`'s ROS-side setup so that "if this passes, the FSM has what it needs to run."

The test runs on the laptop (after SSHing into the RPi to start `ball_launcher.py` and the camera/aruco/Nav2 stack) and presents results via a Tkinter GUI with live status updates.

### 1.2 Test Sequence

| # | Test | What It Checks | Pass Condition | Timeout |
|---|---|---|---|---|
| 1 | ROS 2 node initialised | `rclpy.init()` and node creation | Node created successfully | — |
| 2 | TF tree (map → base_link) | SLAM localisation active | Valid transform returned | 10 s |
| 3 | `/aruco/markers` topic publishing | Camera + ArUco pipeline alive | ≥ 1 message received | 8 s |
| 4 | ArUco marker visible | At least one marker detected | ≥ 1 marker ID in message | 15 s |
| 5 | Nav2 stack active | Nav2 lifecycle nodes ready | `waitUntilNav2Active()` returns | 20 s |
| 6 | `/start_flywheel` service | Flywheel service available and callable | Service responds with `success=True` | 5 s |
| 7 | `/fire_ball` service (1 ball) | Ball firing service callable (fires one real ball) | Service responds with `success=True` | 5 s |
| 8 | `/stop_flywheel` service | Flywheel stop service callable | Service responds with `success=True` | 5 s |

### 1.3 Safety Features

- If `/start_flywheel` fails, `/fire_ball` is **skipped** (never fire with motors off).
- If the test crashes or is interrupted, a safety-stop call is made to `/stop_flywheel` to prevent leaving motors running.
- On GUI quit or window close, a shutdown handler attempts to stop the flywheel before destroying the node.

### 1.4 Running the Test

```bash
# Prerequisites:
# 1. On RPi: ball_launcher.py running
# 2. On RPi: aruco_live.py running, camera pointed at a marker
# 3. On laptop: SLAM + Nav2 stack active

python3 system_test.py
```

The GUI displays each test in sequence with colour-coded status (PENDING → RUNNING → PASSED/FAILED/SKIPPED) and a final summary line.

---

## 2. Electrical Testing and Validation

### 2.1 Raspberry Pi Instability — Odometry and Push Capability Issues

One of the most significant issues encountered was instability in the Raspberry Pi's performance during operation. This manifested as erratic odometry readings and unreliable pushing behaviour of the TurtleBot. The root cause is suspected to be related to power supply noise or GPIO interference from shared power rails between the Raspberry Pi and other peripherals.

A strongly suspected contributing factor is the SG90 servo receiving unintended PWM signals even when it was not meant to be active. This was observed as random servo jitter or movement during phases where the servo should have been idle. The spurious PWM signals may have introduced noise onto the 5V rail, disrupting the Raspberry Pi's stable operation and indirectly affecting ROS2 processes responsible for odometry and motion control.

### 2.2 Servo PWM Interference

The SG90 servo was observed to occasionally receive unintended PWM signals during ROS2 startup and operation. This is consistent with GPIO noise during Raspberry Pi boot sequences. The issue was partially mitigated by manually repositioning the servo between runs. A longer-term fix would be to implement a hardware enable/disable circuit (e.g., a transistor gate) on the servo signal line, or to add a pull-down resistor on the PWM GPIO pin to suppress floating signals during boot.

### 2.3 Wiring Failures — Soldering Required

Multiple instances of wire breaks and loose connections were encountered throughout development and testing. Joints initially secured with jumper wires or breadboard connections proved unreliable under the mechanical stress of the TurtleBot's movement. Affected connections included motor driver signal wires and servo power leads. These were resolved by soldering the connections and reinforcing with heat shrink tubing. All critical connections in the final build should be soldered rather than relying on push-fit connectors.

### 2.4 Motor Voltage Adjustment

The original design routed flywheel motor power from the OpenCR's 11.1V output, which was found to be excessive for the RF300 Series motors. The design was revised to supply the motors at 5V via the L298 motor driver, powered from the Raspberry Pi's 5V rail. This change better matches the RF300's operating range (0.05–0.50W output) and reduced the risk of motor damage from overvoltage.

---

## 3. Docking and Delivery Testing

### 3.1 Test Methodology

Testing was conducted in the lab environment with markers mounted at representative heights and angles:

- **Static docking tests:** Robot positioned at known distances (0.3 m to 2.0 m) from markers, measuring detection success rate and pose estimation accuracy.
- **Full approach tests:** Complete coarse-to-fine docking sequence from exploration distance, measuring final lateral offset and angular error at dock position.
- **Dynamic station tests:** Station B receptacle oscillating at operational speed, measuring trigger detection reliability and ball landing success rate.

### 3.2 Docking Results

| Metric | Target | Result |
|---|---|---|
| Detection range | 0.3–2.0 m | Achieved |
| Detection success rate (static) | ≥ 95% | Achieved |
| Final lateral error (Station A) | < 3 cm | Achieved |
| Final lateral error (Station B) | < 1.5 cm | Achieved |
| Docking success rate (with retries) | ≥ 90% | Achieved |
| Trigger detection reliability | ≥ 95% | Achieved |
| Ball delivery success (Station A) | 3/3 balls | Achieved across multiple runs |
| Ball delivery success (Station B) | ≥ 2/3 balls | Achieved |

### 3.3 Key Findings

- The 3-phase visual servo approach (align, drive, dead-reckon) reliably bridges the gap between Nav2's goal tolerance and the mission's precision requirements.
- The dead-reckoning phase is critical — without it, the robot stops at ~0.20 m (blind threshold) rather than reaching the 0.10 m target distance.
- Station B's tighter alignment tolerance (1 cm) was necessary — at 3 cm lateral error, balls missed the oscillating receptacle on roughly 40% of passes.
- The trigger cooldown (5 s) effectively prevents double-fires, but requires the receptacle oscillation period to be longer than this cooldown.

---

## 4. Navigation Testing

### 4.1 Tight Space Navigation Issues

The frontier detection provided by `explore_lite` was reliable — it accurately identified all viable unmapped spaces as frontiers (confirmed by monitoring published frontier topics on RViz2). The issue lies with the Nav2 planning stack, which was unable to plot feasible paths through very tight spaces. When the planner fails to find a path, `explore_lite` blacklists the frontier, effectively giving up on reachable areas prematurely.

The root cause is an overly conservative inflation radius and cost scaling configuration in the Nav2 costmap, which makes the robot's effective footprint significantly larger than its physical size.

### 4.2 Suggested Navigation Improvements

- Conduct more testing on smaller and tighter maze configurations to tune `inflation_radius` and `cost_scaling_factor` parameters.
- Lower the `progress_timeout` to avoid excessive time on unreachable frontiers.
- Evaluate alternative planners such as `SmacPlanner2D` or `SmacPlannerHybrid`, which support full footprint parameters instead of circular radius — more accurate for the TurtleBot3 Burger's rectangular profile.

---

## 5. Factory Acceptance Test (FAT) Checklist

Pre-mission checklist of factory checks. All items must be ticked off and signed before the robot is presented for the final mission run.

| # | Check Item | Pass / Fail | Date / Initials |
|---|---|---|---|
| 1 | Battery 11.1V / 1800 mAh charged & load verified | | |
| 2 | All frame layers & payload secured | | |
| 3 | LiDAR 360° FOV unobstructed | | |
| 4 | Both Dynamixel motors respond to test commands | | |
| 5 | Flywheel motors spin freely, SG90 gate cycles without jam | | |
| 6 | Storage tube loaded with 9 balls; gravity feed confirmed | | |
| 7 | Camera: 640×480 @ 30 fps; ArUco detection active | | |
| 8 | SLAM: Cartographer launches & maps on power-on | | |
| 9 | explore_lite: frontiers identified & goals issued | | |
| 10 | Nav2 DWB: obstacle response < 100 ms | | |
| 11 | Docking: ±2 cm lateral, 0.2 m distance, ±3° angular | | |
| 12 | Station A: 3 ball sequence bench tested | | |
| 13 | Station B: Latency compensation rig tested | | |
| 14 | Lift API: HTTP POST/GET round trip confirmed | | |
| 15 | ROSBAG recording starts on launch | | |
| 16 | RPi Voltage ≥ 4.75V under worst-case load | | |
| 17 | Motor & signal wires routed separately; caps fitted | | |
| 18 | RPi and L298N not in contact | | |
| 19 | E-Stop functional | | |
| 20 | Document pack ready | | |

---

## 6. Acceptable Defect Log

Known imperfections judged acceptable for mission operation:

| ID | Description | Mission Impact | Operator Note |
|---|---|---|---|
| 1 | L298N missing one standard motor driver screw | None | Ensure wires are secured and not loose |

---

## 7. Maintenance & Part Replacement Log

| Date | Component | Action / Reason | By |
|---|---|---|---|
| 19/03/2026 | RPi Camera | Camera not working after testing with different RPi and FFC | Russell Ng |
| 07/04/2026 | 11.1V Li-ion Battery | Battery cable broken | Alex |
