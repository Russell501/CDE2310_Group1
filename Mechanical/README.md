# TurtleBot Group 1 Mechanical Documentation

This document provides detailed instructions for printing, assembling, and integrating the custom payload onto your TurtleBot.

---

## Components

### TurtleBot3 Subcomponents
- LiDAR (LDS-02)
- RaspberryPi
- OpenCR 1.0
- 2x Dynamixel motors
- LiPo Battery
- USB2LDS

### Bill of Materials
- SG90 Servo Motor
- RF300FA-12350 DC Motor x2
- L298N Motor Driver
- RasberryPi Camera V2.1
- BambuLab PLA Filament 1kg

### Printed Parts
- Flywheel Housing (includes servo gate mount)
- Ball Storage
- Flywheels x2
- Feeder Roller
- RPi Camera Mount

### Hardware
- **DC Motor (2 pieces)**: Press-fitted onto flywheel housing using H7 Tolerance
- **Servo Motor (1 pieces)**: Connected onto the feeder roller and mounted onto the flywheel housing
- **Motor Driver**: To allow controlling motor speed due to running out of PWM pin 
- **RPi Camera**: Mounted onto the Rpi Camera Mount

---

## Assembly Instructions

### Step 1: Preparation
1. **Insert Threads**: Install threaded inserts into the flywheel housing and ball storage components.  
2. **Mount Feeder Roller**: Attach the feeder roller to the SG90 servo motor.  
3. **Attach Flywheels**: Press-fit each flywheel onto the shaft of the DC motors.  
4. **Disassemble TurtleBot Layer**: Remove the fourth layer of the TurtleBot, including the LiDAR module.  

---

### Step 2: Assembling the Launcher
1. **Mount Servo Assembly**: Secure the SG90 servo motor with the feeder roller onto the flywheel housing.  
2. **Install Flywheel Motors**: Press-fit the DC motors (with attached flywheels) into the flywheel housing.  

---

### Step 3: Mounting Launcher to TurtleBot
1. **Position Launcher**: Place the launcher between the hex supports and the fourth-layer waffle plate (ensure supports are flush with the 3D print).  
2. **Secure Top Mount**: Fasten the top of the flywheel housing to the fourth layer using bolts into the threaded inserts.  
3. **Secure Rear Mount**: Attach the back of the flywheel housing to the hex supports using bolts.  
4. **Install Ball Storage**: Mount the ball storage top surface to the waffle plate using bolts into the threaded inserts.  
5. **Extend Supports**: Add additional hex supports (60 mm) to both left and right sides.  
6. **Reattach Layer**: Mount the fourth layer of the TurtleBot onto the extended hex supports.  

---

---

## Important Installation Notes

### Wiring and Electronics
- **Power Components and Wiring**: Install all wiring, including DC motors and power connections, before fully mounting the launcher to ensure easier access and cable management
- **Ensure all wires are secured down and none are exposed**

### Servo Installation
- **Servo Placement**: Install the servo into the housing with the correct orientation and ensure that it won't jammed on to the flywheel or wires


### Flywheel Alignment
- **Motor and Flywheel Fit**: Ensure flywheels are firmly press-fitted onto the motor shafts and aligned properly within the housing. Misalignment may cause vibration or reduced launching performance
---

## System Specifications
- **Docking Distance**: ~15–25 cm  
- **Launcher Exit Height**: 151 mm from ground  
- **Receptacle Height**: 149.5 mm from ground  

---

## Launch Considerations
- **Mass of Ping Pong Ball**: 2.7 g  
- **Height Difference**: ~1.5 mm (approximately level trajectory)  
- **Launch Requirement**: Ball must travel horizontally across the docking gap with minimal vertical displacement  

---

## Motor Specifications (DC Flywheel Motors)
- **Voltage Range**: 1.5 – 6.0 V  
- **Nominal Voltage**: 3.0 V  
- **No-Load Speed**: ~3500 rpm  
- **Speed at Max Efficiency**: ~2830 rpm  
- **Current at Max Efficiency**: ~0.093 A  
- **Stall Torque**: ~4.90 mN·m  
- **Stall Current**: ~0.140 A  

---

## Launch Calculations

- **Launch Height**: 151 mm  
- **Receptacle Height**: 149.5 mm  
- **Vertical Drop**: 1.5 mm  

Assuming horizontal launch:

- **Time of Flight**: ~0.0175 s  

Required horizontal velocity:
- **For 15 cm range**: ~8.6 m/s  
- **For 25 cm range**: ~14.3 m/s  

The flywheel system must therefore impart an approximate exit velocity between **8.6 m/s and 14.3 m/s** for successful docking.
---

## Mechanical and Assembly Recommendations
- Ensure all components are secured and glue components that is loose
- Double-check servo alignments and wiring connections to ensure smooth operation.
- Conduct initial launch tests to verify mechanical stability and integrity.

---
## Design Reasoning

### Compact Integration within TurtleBot Structure
The launcher system is designed to be integrated within the internal layers of the TurtleBot, rather than extending outward. By embedding the launcher between the hex supports and the existing robot layers, the overall footprint of the robot is minimised. This approach reduces the risk of collision and improves manoeuvrability within constrained maze environments.

In addition, increasing vertical stacking (height) was preferred over increasing width, as it maintains a narrower base. This design choice improves navigation through tight pathways while still allowing sufficient space for the launching mechanism.

---

### Aligned Launch Trajectory for Reliable Docking
The launcher is oriented to fire approximately along the central axis of the robot (not perfectly centered, but aligned with the robot’s forward direction). This alignment improves docking reliability by ensuring that the ball is consistently launched toward the receptacle’s expected position, reducing lateral error during dynamic movement.

---

### Flywheel-Based Launch Mechanism
A flywheel system was selected to achieve rapid and consistent ball launching. Unlike spring-based systems, flywheels allow continuous rotation and immediate successive launches without requiring reset time. This is particularly important for dynamic environments where the receptacle may be moving unpredictably, requiring fast response and repeated launch attempts.

---

### Feeder Gate Control Mechanism
A servo-controlled feeder gate is implemented to regulate ball input into the flywheel system. This prevents accidental or unintended firing of balls and ensures that each ball is properly aligned before entering the flywheel. The mechanism also reduces the risk of jamming and improves consistency in launch timing.

---

### Structural and Functional Integration
All components are designed to work within a compact 3D-printed housing system. The launcher, motors, and feeder mechanism are tightly integrated to maintain structural rigidity while minimising external protrusions. This improves both mechanical stability and spatial efficiency within the robot.

---

For further support or inquiries, please contact CDE2310 Group 1 AY25/26 or refer to the provided project documentation.
```
