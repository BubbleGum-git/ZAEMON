# BE Capstone Project

**ZAEMON — Dynamically Balanced Wheeled Bipedal Robot**  
[Our Project proposal presentation](https://www.youtube.com/watch?v=3FdDc5E4eX4)

---

## Team Details

| Sr. No. | Name of Student | Roll No. | Branch                  | Email ID |
| ------- | --------------- | -------- | ----------------------- | -------- |
| 1       |Suraj Kumbhar|9| Automation and Robotics |2023.suraj.kumbhar@ves.ac.in|
| 2       |Baljeet Singh Labana|10| Automation and Robotics |2023.baljeet.labana@ves.ac.in|
| 3       |Aayush Maluste|13| Automation and Robotics |2023.aayush.maulste@ves.ac.in|
| 4       |Jai Bagul|35| Automation and Robotics |2023.jai.bagul@ves.ac.in|

---

## Guide Details

**Project Guide:** 
Madhumati Khuspe  
**Department:** 
Automation and Robotics  
**Institute:** 
Vivekanand Education Society's Institute of Technology, Mumbai

---

## Problem Statement

Conventional wheeled robots provide efficient locomotion but are generally limited to stable configurations and relatively simple movements. Legged robots offer greater mobility and dynamic capabilities but require more complex mechanical structures and control systems.

The aim of ZAEMON is to design and develop an open source dynamically balanced wheeled bipedal robot platform and hardware capable of maintaining balance, performing controlled wheeled locomotion, and executing dynamic movements such as jumping through real-time sensing, trajectory planning, dynamic modelling, and feedback control.

---

## Abstract

ZAEMON is a dynamically balanced wheeled bipedal robot developed to combine the efficiency of wheeled locomotion with the dynamic capabilities of a legged robotic system. Due to its variable center of mass, the robot behaves similarly to an inverted pendulum and requires continuous active stabilization to remain balanced.

The project focuses on the mechanical design, kinematic and dynamic modelling and real-time control of the robot.

An IMU provides real-time orientation feedback for estimating the robot's tilt and motion. PID control is used for balance stabilization and normal locomotion, while computed torque control is investigated for dynamic movements such as jumping.

The project aims to develop and experimentally validate a compact robotic platform capable of dynamic balancing, controlled locomotion, and agile movement.

---

## Objectives

1. To design and develop a dynamically balanced wheeled bipedal robot.
2. To study the kinematics and dynamics of the robotic system.
3. To implement inverse dynamics.
4. To develop smooth joint-space and Cartesian-space trajectories.
5. To implement real-time balance control using IMU feedback.
6. To implement PID-based stabilization for balancing and locomotion.
7. To investigate computed torque control for dynamic movements.
8. To integrate the mechanical, electrical, and software systems.
9. To experimentally test and validate the developed system.

---


## Existing System

Conventional wheeled robots provide efficient and relatively simple locomotion but have limited ability to dynamically change their body configuration or perform agile movements.

Legged robots provide greater mobility and dynamic capabilities but typically involve:

* Higher mechanical complexity
* Multiple actuators
* Complex control algorithms
* Higher computational requirements
* Increased power consumption

ZAEMON explores a hybrid approach that combines efficient wheeled locomotion with actively controlled leg joints.

---


## Current Hardware

The hardware architecture consists of:

* Microcontroller ESP32 S3
* IMU MPU6050
* Motors and actuators - BLDC Gimbal Motor
* Motor drivers - DRV8313 FOC driver
* Encoders - AS5600 Magnetic encoder
* Power supply and battery system
* Voltage regulation and power distribution circuitry

Detailed hardware specifications will be documented as the design is finalized.

---

## Design Files

Design files are maintained in the following directories:

```text
hardware/
├── CAD/
```

---

## Circuit Diagram

The circuit diagram will be added as the electronics architecture is finalized.

---

### Control Algorithm

1. Initialize the controller and sensors.
2. Read IMU and encoder measurements.
3. Estimate the current robot state.
4. Calculate the error from the desired state.
5. Generate the desired trajectory.
6. Calculate the required control action.
7. Generate motor commands.
8. Apply commands to the actuators.
9. Read the updated sensor state.
10. Repeat the control loop in real time.

---

## Implementation Details

### Hardware Implementation

The hardware implementation consists of the mechanical structure, actuators, motor drivers, embedded controller, IMU, encoders, and power system.

The mechanical structure is designed to provide the required degrees of freedom for balancing, locomotion, and dynamic movement.

ime feedback

---

## Repository Structure

```text
ZAEMON/
│
├── README.md
│
├── docs/
│   ├── progress/
│   ├── design/
│   └── reports/
│
├── hardware/
│   ├── CAD/
│   ├── PCB/
│   ├── schematics/
│   └── BOM/
│
├── software/
│   ├── firmware/
│   ├── control/
│   ├── simulation/
│   └── tools/
│
├── images/
│
└── reference/
```

---

## How to Run

Build, firmware upload, and simulation instructions will be added as the software and hardware platforms are finalized.

---

## Applications

Potential applications of ZAEMON include:

1. Research in dynamically balanced robotics.
2. Agile robotic locomotion research.
3. Balance and control system research.
4. Dynamic trajectory and model-based control research.
5. Educational and experimental robotics.

---

## Advantages

1. Combines wheeled locomotion with bipedal movement.
2. Enables active dynamic balancing.
3. Supports model-based motion planning and control.
4. Provides a platform for studying dynamic robotic movement.
5. Can be extended with advanced control and autonomous capabilities.

---

## Limitations

1. Requires continuous active control for maintaining balance.
2. Dynamic movement requires accurate state estimation and modelling.
3. Mechanical and control complexity is higher than conventional wheeled robots.
4. Actuator and battery performance constrain the robot's capabilities.
5. Initial testing is intended for controlled environments.

---

## Future Scope

Future development may include:

* Autonomous navigation
* Advanced sensor fusion and state estimation
* Adaptive and robust control
* Improved landing and impact control
* More complex dynamic movements
* Real-time optimization of the dynamic model
* Improved mechanical and actuator design

---

## License

This project is developed as part of the BE Capstone Project at VESIT, Mumbai.

**For academic and research purposes.**
