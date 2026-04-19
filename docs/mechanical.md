---
title: Mechanical Subsystem
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
- [User Manual](user-manual.md)
- [Bill-Of-Materials](bill-of-materials.md)
- [Electrical Subsystem](electrical.md)
- **Mechanical Subsystem** ← _You are here_
- [Improvements](improvements.md)

---

# Mechanical Subsystem Documentation

This document outlines the design, iteration, and assembly of the custom payload for the TurtleBot, detailing how we achieved a compact, reliable, and precise ping pong ball launching system.

---

## 1. System Overview

The mechanical subsystem is designed to securely transport and fire ping pong balls into a docking receptacle. The system consists of:
- A **compact, internally mounted launching mechanism** that securely holds the flywheel and feeder in between the robot's structural layers.
- A **gravity-assisted, servo-controlled feeder** with a rotating arm for precise, single-ball release.
- A **dual-flywheel launcher** integrated directly into the robot's internal structural layers to ensure a straight, spin-free launch.
- A low-profile design that maintains the LiDAR sensor unobstructed by any components


<img width="400" alt="System Overview CAD" src="https://github.com/user-attachments/assets/eac126f2-4600-4039-8f25-f959a80ce601" />

---

## 2. Problem Description & Design Requirements

To successfully complete the docking and firing tasks, the mechanical subsystem had to satisfy several strict constraints:
- **Payload Capacity:** Store exactly nine 2.7g ping pong balls securely without them escaping during movement or navigation
- **Dimensional Constraints:** Maintain a narrow overall footprint to ensure the robot can navigate tight maze environments without collision.
- **Sensor Clearance:** Ensure zero obstruction to the 360° field of view of the LDS-02 LiDAR without requiring external sensor mounts or height modifications.
- **Launch Kinematics:** Fire balls horizontally across a 15–25 cm docking gap with minimal vertical drop, ensuring they land cleanly inside a receptacle positioned at 149.5 mm from the ground.

---

## 3. Design Reasoning

### Compact Integration within TurtleBot Structure
The launcher system is designed to be integrated within the internal layers of the TurtleBot, rather than extending outward. By embedding the launcher between the hex supports and the existing robot layers, the overall footprint of the robot is minimised. This approach reduces the risk of collision and improves manoeuvrability within constrained maze environments.

In addition, increasing vertical stacking (height) was preferred over increasing width, as it maintains a narrower base. This design choice improves navigation through tight pathways while still allowing sufficient space for the launching mechanism.

### Aligned Launch Trajectory for Reliable Docking
The launcher is oriented to fire approximately along the central axis of the robot (not perfectly centered, but aligned with the robot’s forward direction). This alignment improves docking reliability by ensuring that the ball is consistently launched toward the receptacle’s expected position, reducing lateral error during dynamic movement.

### Flywheel-Based Launch Mechanism
A flywheel system was selected to achieve rapid and consistent ball launching. Unlike spring-based systems, flywheels allow continuous rotation and immediate successive launches without requiring reset time. This is particularly important for dynamic environments where the receptacle may be moving unpredictably, requiring fast response and repeated launch attempts. *(A dual-flywheel configuration was specifically chosen to provide balanced acceleration on both sides of the ball. This prevents the ball from adopting an unwanted lateral spin, ensuring a straighter horizontal trajectory).*

### Feeder Gate Control Mechanism
A servo-controlled feeder gate is implemented to regulate ball input into the flywheel system. This prevents accidental or unintended firing of balls and ensures that each ball is properly aligned before entering the flywheels. The mechanism also reduces the risk of jamming and improves consistency in launch timing.

| CAD Model | Real Life |
|:---------:|:---------:|
| <img width="400" alt="Feeder Gate CAD" src="https://github.com/user-attachments/assets/d6a81981-5d15-40c1-8332-270a2082e994" /> | ![feeder-gate-mechanism-pic](assets/images/mechanical/feeder-gate-mechanism.jpg) |

### Structural and Functional Integration
All components are designed to work within a compact 3D-printed housing system. The launcher, motors, and feeder mechanism are tightly integrated to maintain structural rigidity while minimising external protrusions. This improves both mechanical stability and spatial efficiency within the robot.

---

## 4. Iterative Design Changes

Our final design is the result of testing and refining several prototypes to solve critical stability and spatial issues.

### Iteration 1: Front-Mounted Launcher with Full Spiral Storage
- **Design:** A full spiral ball storage track feeding into a launcher mounted at the very front of the TurtleBot.
- **Problem:** The robot became heavily front-biased, risking tipping. Additionally, feeding the ball to the front required a sharp 90-degree bend in the internal tubing, which caused severe jamming and made the feeding mechanism overly complex.

<img width="400" alt="Iteration 1 CAD" src="https://github.com/user-attachments/assets/b394cdfa-895e-47f8-9301-56eaec3951d2" />

### Iteration 2: Side-Mounted Launcher with 0.75x Spiral
- **Design:** We reduced the spiral to 0.75 revolutions so the track ended in a straight path. We extended this path to a side-mounted launcher pointing forward.
- **Problem:** This resulted in an asymmetrical footprint (the robot was "fat" on one side). This extra width severely limited the robot's ability to navigate tight maze corridors and made aligning for docking highly inconsistent.

<img width="400" alt="Iteration 2 CAD" src="https://github.com/user-attachments/assets/ee062d10-3c7f-4f42-9b7f-687ff1c121d6" />

### Iteration 3: Centered & Layer-Integrated
- **Design:** We abandoned the spiral storage concept entirely. We centered the dual-flywheel launching mechanism and embedded a compact ball storage module tightly between the TurtleBot's structural plates. 
- **Result:** This eliminated the asymmetrical width, completely resolved the 90-degree feed jamming issue, and perfectly balanced the center of gravity. Furthermore, the internal embedding successfully prevented any vertical stacking that would block the LiDAR, eliminating the need to modify the sensor's stock height.

| CAD Model | Real Life |
|:---------:|:---------:|
| <img width="400" alt="Iteration 3 CAD" src="https://github.com/user-attachments/assets/eac60a02-d5e4-4e5c-a599-a366645a213d" /> | ![finalised-launcher-design](assets/images/mechanical/finalised-launcher-design.jpg) |

### Iteration 4: Stability Improvement
- **Design:** We extended the rear of the 3D-printed flywheel housing so that it sits flush against the TurtleBot's rear hex supports, allowing the housing to be directly bolted and tightened to the chassis.
- **Result:** This structural anchoring eliminated the vibrations caused by the rotational inertia of the spinning LiDAR. The robot's overall steadiness was restored, providing a rigid base for the launching mechanism and ensuring consistent docking and firing operations.

<div align="center">
  <table border="0">
    <tr>
      <td align="center">
        <b>Before</b><br>
        <img src="https://github.com/user-attachments/assets/68498098-e84f-4cf0-8534-6f6b0e04d588" width="300"><br>
        <sub>Vibration issues due to lack of support</sub>
      </td>
      <td align="center">
        <b>After</b><br>
        <img src="https://github.com/user-attachments/assets/27c9b4cc-286f-4e36-b1bc-f77a86a57490" width="300"><br>
        <sub>Rigid anchoring to hex supports</sub>
      </td>
    </tr>
  </table>
</div>
---

## 5. Kinematics & Launch Requirements

The flywheel system was engineered based on the exact physics required to cross the docking gap.

- **Launcher Exit Height:** 151.0 mm
- **Receptacle Height:** 149.5 mm
- **Vertical Drop ($\Delta y$):** 1.5 mm
- **Target Horizontal Range ($d$):** 15 cm to 25 cm

Assuming a perfectly horizontal launch, the time of flight ($t$) before the ball drops 1.5 mm is calculated using gravity ($g = 9.81 \text{ m/s}^2$):
$t = \sqrt{\frac{2\Delta y}{g}} \approx 0.0175 \text{ s}$

To cross the docking gap within this short time frame, the required horizontal velocity ($v_x = \frac{d}{t}$) is:
- **For 15 cm range:** $\approx 8.6 \text{ m/s}$
- **For 25 cm range:** $\approx 14.3 \text{ m/s}$

The dual DC motors and flywheel diameters were chosen and tuned specifically to impart an exit velocity within this 8.6 m/s to 14.3 m/s window.

---

## 6. Hardware & Components

### TurtleBot3 Subcomponents
- LiDAR (LDS-02)
- RaspberryPi & OpenCR 1.0
- 2x Dynamixel motors & LiPo Battery

### Custom Bill of Materials (BOM)
- 1x SG90 Servo Motor (for feeder gate)
- **2x** RF300FA-12350 DC Motors (for dual flywheels)
- 1x L298N Motor Driver
- Standard M4 Bolts, Nuts, and Threaded Inserts
- BambuLab PLA Filament (1kg)

### Custom 3D Printed Parts
- Flywheel Housing (includes dual motor mounts and servo gate mount)
- Compact Internal Ball Storage Module
- **2x** Flywheels
- Feeder Roller Arm

---

## 7. Assembly Instructions

### Step 1: Preparation
1.  **Insert Threads:** Heat-press threaded inserts into the designated holes on the flywheel housing and ball storage components.
2.  **Mount Feeder Roller:** Securely attach the custom 3D-printed feeder roller to the spline of the SG90 servo motor.
3.  **Attach Flywheels:** Firmly press-fit the two 3D-printed flywheels onto the shafts of the DC motors. Ensure they are fully seated to prevent wobble.
4.  **Disassemble TurtleBot Layer:** Remove the top waffle plate (Layer 4) of the TurtleBot, carefully disconnecting the LiDAR module.

![remove-lidar-layer](assets/images/mechanical/remove-lidar-layer.jpg)

### Step 2: Assembling the Launcher
1.  **Mount Servo Assembly:** Fasten the SG90 servo motor (with the attached feeder arm) onto its mounting bracket on the flywheel housing.
2.  **Install Flywheel Motors:** Press-fit both DC motors into the flywheel housing, ensuring the flywheels sit precisely alongside the shooting tube walls.
3. **Install ball storage:** Fasten the connecting joints

| Fitting Motors & Servo | Installing Ball Storage |
|:----------------------:|:-----------------------:|
| ![fit-motors-and-servo](assets/images/mechanical/fit-motors-and-servo.jpg) | ![install-ball-storage](assets/images/mechanical/install-ball-storage.jpg) |

### Step 3: Integration with TurtleBot
1.  **Position Launcher:** Place the assembled launcher module between the hex supports and the Layer 4 waffle plate.
2.  **Secure Mounts:** Fasten the top and rear of the flywheel housing to the waffle plate and hex supports using M4 bolts and the threaded inserts.
3.  **Install Storage Module:** Mount the compact internal ball storage directly above the launcher intake. 
4.  **Extend Supports:** Install additional 60 mm hex supports to the left and right sides of the chassis to accommodate the new hardware height.
5.  **Finalize Assembly:** Reattach the Layer 4 waffle plate onto the extended hex supports, ensuring the LiDAR sensor remains in its stock configuration on top.

---

## 8. Assembly Best Practices & Troubleshooting

- **Wiring & Routing:** Install the L298N motor driver, route the DC motor power cables, and manage the servo wiring *before* fully bolting down the launcher housing. Doing this after assembly is extremely difficult.
- **Dual Flywheel Tuning:** Both flywheels must spin at the exact same RPM. If one motor receives slightly more power than the other, the ball will curve out of the launcher (similar to a curveball in baseball). Use the L298N motor driver's PWM outputs to fine-tune and match the speeds.
- **Flywheel Alignment:** Ensure both flywheels are perfectly parallel to the shooting tube. Angular misalignment will introduce friction and cause inconsistent shots.
- **Servo Zeroing:** Before screwing the rotating arm onto the servo, write a quick test script to set the servo to its "neutral" (blocking) position. Mount the arm so it correctly blocks the ball in this state.
- **Clearances:** Double-check that no wires or custom parts are clipping into the LiDAR's rotational path.
