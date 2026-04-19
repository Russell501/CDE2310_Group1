---
title: User Manual
---

# 🔗 Navigation

- [Home](index.md)
- [Requirements](requirements.md)
- [Con-Ops](conops.md)
- [High Level Design](high-level-design.md)
- [Sub System Design](subsystem-design.md)
- [Interface Control Documents](icd.md)
- [Software Development](software.md)
- [Testing](testing.md)
- [Bill-Of-Materials](bill-of-materials.md)
- [Electrical Subsystem](electrical.md)
- [Mechanical Subsystem](Mechanical.md)
- **User Manual** ← _You are here_

---
# User Manual

---

## 1. System Overview

![System Architecture Diagram](assets/images/general_system/system_architecture.png)
*System architecture overview.*


The Group 1 AMR is a TurtleBot3 Burger-based autonomous mobile robot with a custom launcher payload. Its mission is to autonomously navigate an unknown warehouse maze, detect ArUco-marked delivery stations, and deliver ping pong balls into static and oscillating receptacles — all within a 25-minute window with no human teleoperation.

| Parameter | Value |
|---|---|
| Platform | TurtleBot3 Burger + custom payload |
| Total Mass | 1402.24 g (1.40 kg) |
| Dimensions (L × W × H) | 138 mm × 178 mm × 192 mm |
| Battery | Li-Po 11.1V, 1800 mAh |
| Compute | Raspberry Pi 4B + OpenCR 1.0 |
| Sensors | LDS-02 LiDAR, RPi Camera V2 (8 MP) |
| Launcher | Dual counter-rotating flywheels + SG90 servo gate |
| Ball Capacity | 9 ping pong balls |
| Mission Window | 25 minutes |
| Estimated Runs per Charge | ~4–5 |

---

## 2. Prerequisites

### 2.1 Software Environment

A ROS 2 Humble environment must be installed on the operator's laptop. Follow the setup process at: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup

### 2.2 Software Installation

1. Back up and remove your existing ROS 2 src folder:

```bash
cp ~/[YOUR_ROS2_WORKSPACE]/src ~/[YOUR_DESIRED_BACKUP_PATH]
rm -rf ~/[YOUR_ROS2_WORKSPACE]/src
```

2. Clone the Group 1 repository into your ROS 2 workspace:

```bash
git clone https://github.com/Russell501/CDE2310_Group1.git
```

3. Install ROS 2 dependencies and rebuild the workspace:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## 3. Pre-Mission Setup

### 3.1 Physical Setup

1. Ensure the battery is fully charged (11.1V LiPo, 1800 mAh).
2. Verify all frame layers and the launcher payload are securely fastened.
3. Confirm the LiDAR has an unobstructed 360° field of view.
4. Load 9 ping pong balls into the gravity-feed storage tube; verify they feed freely.
5. Confirm the flywheel motors spin freely and the SG90 servo gate cycles without jamming.
6. Verify all wiring connections are secure (soldered connections preferred over jumper wires).
7. Check that the RPi Camera is properly connected via the CSI flat flex cable.

### 3.2 Pre-Flight Integration Test

Before running the full mission, execute the system integration test to verify all components:

1. On the RPi (via SSH): start `ball_launcher.py` and `aruco_live.py`.
2. On the RPi: start the SLAM and Nav2 stack.
3. Point the camera at an ArUco marker.
4. On the laptop: run `python3 system_test.py`.
5. All 8 tests should pass (the test will fire one real ball during test #7).

---

## 4. Mission Deployment

### Step 1 — Power On

Turn on the TurtleBot3 Burger. Wait for the Raspberry Pi to boot (indicated by the green LED activity on the Pi).

### Step 2 — SSH into the RPi

Open a terminal on your laptop and SSH into the TurtleBot3:

```bash
ssh ubuntu@[TURTLEBOT_IP]
```

### Step 3 — Start RPi Processes

In the SSH terminal, run the RPi launch script:

```bash
python3 main_launch.py
```

This starts the ArUco detection pipeline (`aruco_live.py`) and the launcher hardware controller (`ball_launcher.py`).

### Step 4 — Start Mission GUI

In a new terminal on your laptop:

```bash
ros2 run auto_nav mission_gui
```

This opens the Tkinter-based mission monitoring GUI showing real-time phase status, marker detections, and delivery logs.

### Step 5 — Launch the Mission

In another terminal on your laptop:

```bash
ros2 launch auto_nav bringup_all.launch.py
```

This starts the mission in dependency order:
1. Cartographer begins SLAM mapping.
2. Nav2 starts when Cartographer is ready.
3. The mission controller FSM starts when Nav2 is ready.

### Step 6 — Monitor

Observe the robot's progress through:
- **RViz2** — Live map, robot pose, planned paths, costmaps, and marker detections.
- **Mission GUI** — Current phase, marker status, docking attempts, and ball delivery logs.
- **Terminal output** — Detailed ROS 2 log messages from each node.

No physical intervention is permitted after the mission starts.

---

## 5. Mission Phases (What to Expect)

| Phase | What Happens | Approximate Duration |
|---|---|---|
| Boot & Init | SLAM starts mapping, Nav2 initialises | 1–2 min |
| Exploration | Robot explores the maze autonomously, detects station markers | 5–12 min |
| Station A | Robot docks at the static station and fires 3 balls with timed delays | 2–4 min |
| Transit | Robot navigates from Station A to Station B | 1–2 min |
| Station B | Robot docks and fires 3 balls synchronised to the moving receptacle | 2–4 min |
| Complete | Mission controller logs completion | < 1 min |

---

## 6. Troubleshooting

### 6.1 Missing base_link or odom topics in RViz2

Wait at least 30 seconds for the topics to be published. If still unavailable, terminate the launch files and run them again.

### 6.2 Servo not moving to fire balls

1. Check the logs in the mission GUI for ball launch messages.
2. Verify the messages were received in the SSH terminal (`ball_launcher.py` output).
3. If messages are present but servo is not moving, check that all 3 servo wires are connected (signal, power, ground).

### 6.3 Flywheel(s) not spinning when launching

1. Check logs in both the mission GUI and SSH terminal.
2. Inspect whether the flywheel disc is rubbing against the top waffle plate; if so, gently press it back down.
3. Check all connections: motors → motor driver, motor driver → Raspberry Pi GPIO pins.

### 6.4 Robot unable to navigate tight spaces

The Nav2 costmap inflation radius may be too conservative. Possible tuning (in `burger.yaml`):
- Reduce `inflation_radius` (currently 0.25 m).
- Increase `cost_scaling_factor` (currently 1.5).
- Consider switching to `SmacPlanner2D` for better tight-space path planning.

### 6.5 Erratic odometry or random servo jitter

Suspected cause: power supply noise on the 5V rail from shared peripherals, or unintended PWM signals on the servo GPIO during boot. Mitigations:
- Add a decoupling capacitor on the servo power line.
- Add a pull-down resistor on the servo PWM GPIO pin.
- Manually reposition the servo between runs if jitter is observed.

### 6.6 Wiring comes loose during operation

Solder all critical connections instead of using jumper wires. Apply heat shrink tubing over exposed wire joints. Minimise female-to-male pin connections on the Raspberry Pi — share 5V and GND on a proto board with soldered joints.

---

## 7. Areas for Improvement

### 7.1 Navigation of Tight Spaces

The frontier detection provided by `explore_lite` was reliable — it accurately identified all viable unmapped spaces as frontiers, as confirmed by monitoring the published frontier topics on RViz2. The issue lies with the Nav2 planning stack, which was unable to plot feasible paths through very tight spaces. When the planner fails to find a path, `explore_lite` blacklists the frontier, effectively giving up on reachable areas prematurely.

The root cause is an overly conservative inflation configuration in the Nav2 costmap. With `inflation_radius: 0.25` and `robot_radius: 0.135`, the effective no-go zone around every obstacle extends to 0.385m per side — meaning any gap narrower than ~0.77m is completely impassable to the planner, even though the robot physically only needs ~0.28m of clearance.

**Recommended improvements:**

- Conduct more testing on smaller and tighter maze configurations to properly tune `inflation_radius` and `cost_scaling_factor`. Reducing the inflation radius (e.g. to 0.15m) and increasing the cost scaling factor (e.g. to 5.0) would allow the planner to route through tight passages while still preferring open space when available.
- Lower `progress_timeout` (currently 60s) to 20–30s so the robot does not spend excessive time attempting to reach frontiers that may not yield useful mapping information, and moves on to more productive goals sooner.
- Evaluate alternative navigation planners such as `SmacPlanner2D` or `SmacPlannerHybrid`, which support a full footprint parameter instead of just a circular radius. This would provide more accurate path feasibility checks around tight passages, particularly for the non-uniform profile of the trutlebot with its launch payload.
- Add a retry mechanism to `explore_lite`'s frontier blacklisting logic. Currently, when Nav2 fails to reach a frontier (due to planner failure, progress timeout, or goal cancellation), `explore_lite` permanently blacklists it — the frontier is never attempted again for the rest of the session. This is overly aggressive because the costmap may have changed after the robot explores other areas, making previously unreachable frontiers viable. A better approach would be to replace the simple blacklist (`std::vector<Point>`) with a tracked attempt counter per frontier. On each failure, the counter increments rather than permanently blocking the frontier. `goalOnBlacklist()` would only filter frontiers that have reached a maximum retry count (e.g. 3 attempts). This allows the robot to move on to easier frontiers first, then circle back and retry previously failed ones — which may now succeed due to updated costmap data from exploring other parts of the maze. This change would be particularly effective when combined with the inflation and planner tuning above, as the retries would have a meaningful chance of producing a different planning outcome.

### 7.2 Electrical Stability

Stability of electrical components was a recurring issue throughout the project. Wires frequently came loose due to vibrations from robot movement, LiDAR motor operation, the launching sequence, and other repairs made on the robot. Additionally, unshielded wires with exposed metal risked unintentionally shorting other connections, leading to intermittent failures that were difficult to diagnose.

**Recommended improvements:**

- Solder as many intermediate wire connections as possible to eliminate loose contact points. Minimise the number of female wire to male pin connections on the Raspberry Pi — instead, share the 5V supply and common ground on a proto board where connections can be soldered securely.
- Apply heat shrink insulation over all exposed wire connections to prevent accidental shorts.
- Implement hardware filtering or a dedicated servo controller to prevent unintended PWM signals during boot, which caused erratic servo jitter.
- Measure and validate all estimated current draws with a multimeter during live operation to ensure components are operating within safe margins.
- Add decoupling capacitors on the 5V and 3.3V rails near sensitive components (servo, motor driver) to reduce power supply noise.
- Investigate a separate 5V BEC (Battery Eliminator Circuit) for the servo to isolate it from the RPi power rail.

---

## 8. Maintenance

### 8.1 Battery

- Charge the 11.1V 1800 mAh LiPo battery fully before each session.
- Estimated 4–5 full mission runs per charge (3.28 Wh per run, 17.98 Wh total capacity).
- Monitor RPi voltage under load — must remain ≥ 4.75V.

### 8.2 Mechanical

- Inspect all frame layer screws and standoffs before each run.
- Verify ball storage tube alignment and gravity feed.
- Check flywheel disc clearance from waffle plates.

### 8.3 Wiring

- Inspect all soldered connections for cold joints or breaks.
- Verify GPIO connections to L298N and servo are secure.
- Ensure signal and power wires are routed separately to minimise EMI.

### 8.4 Part Replacement Log

| Date | Component | Action / Reason | By |
|---|---|---|---|
| 19/03/2026 | RPi Camera | Camera not working after testing with different RPi and FFC | Russell Ng |
| 07/04/2026 | 11.1V Li-ion Battery | Battery cable broken | Alex |
