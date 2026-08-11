# BE Capstone Project

## Project Title

**ZAEMON — Dynamically Balanced Wheeled Bipedal Robot**

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

**Project Guide: **
**Department:** Automation and Robotics
**Institute:** VESIT, Mumbai

---

## Problem Statement

Conventional wheeled robots provide efficient locomotion but are generally limited to stable configurations and relatively simple movements. Legged robots offer greater mobility and dynamic capabilities but require more complex mechanical structures and control systems.

The aim of ZAEMON is to design and develop a dynamically balanced wheeled bipedal robot capable of maintaining balance, performing controlled wheeled locomotion, and executing dynamic movements such as jumping through real-time sensing, trajectory planning, dynamic modelling, and feedback control.

---

## Abstract

ZAEMON is a dynamically balanced wheeled bipedal robot developed to combine the efficiency of wheeled locomotion with the dynamic capabilities of a legged robotic system. Due to its elevated center of mass, the robot behaves similarly to an inverted pendulum and requires continuous active stabilization to remain balanced.

The project focuses on the mechanical design, kinematic and dynamic modelling, trajectory planning, and real-time control of the robot. Lagrangian mechanics is used to formulate the equations of motion, while the Newton-Euler method is used for efficient inverse dynamics computation. Joint-space and Cartesian-space trajectory planning are used to generate smooth and controlled movements for crouching, launching, landing, and wheeled locomotion.

An IMU provides real-time orientation feedback for estimating the robot's tilt and motion. PID control is used for balance stabilization and normal locomotion, while computed torque control is investigated for dynamic movements such as jumping. Cubic spline trajectories are used to generate smooth motion profiles and reduce sudden changes in motion.

The project aims to develop and experimentally validate a compact robotic platform capable of dynamic balancing, controlled locomotion, and agile movement.

---

## Objectives

1. To design and develop a dynamically balanced wheeled bipedal robot.
2. To study the kinematics and dynamics of the robotic system.
3. To develop a mathematical model using Lagrangian mechanics.
4. To implement inverse dynamics using the Newton-Euler method.
5. To develop smooth joint-space and Cartesian-space trajectories.
6. To implement real-time balance control using IMU feedback.
7. To implement PID-based stabilization for balancing and locomotion.
8. To investigate computed torque control for dynamic movements.
9. To integrate the mechanical, electrical, and software systems.
10. To experimentally test and validate the developed system.

---

## Scope of the Project

The project covers:

* Mechanical design and development
* Motor and actuator selection
* Embedded control system development
* IMU-based state estimation
* Kinematic modelling
* Dynamic modelling
* Lagrangian formulation
* Newton-Euler inverse dynamics
* Joint-space trajectory planning
* Cartesian-space trajectory planning
* Cubic spline trajectory generation
* PID-based balance control
* Computed torque control
* Wheeled locomotion
* Dynamic movement and jumping
* Simulation and experimental validation

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

## Proposed System

ZAEMON combines wheeled locomotion with an actively controlled bipedal structure.

The system uses:

* Wheels for efficient ground locomotion
* Actuated leg joints for posture and dynamic movement
* IMU feedback for real-time balance estimation
* Embedded control for real-time computation
* Kinematic and dynamic models for motion analysis
* Trajectory planning for controlled movement
* PID control for balance stabilization
* Computed torque control for dynamic movements

During balancing, ZAEMON is modelled as an inverted pendulum on wheels. The controller continuously estimates the robot's state and generates corrective motor commands to maintain stability.

---

## System Architecture

The overall system consists of the mechanical structure, actuators, sensors, motor drivers, embedded controller, and control algorithms.

![System Architecture](images/system_architecture.png)

### Control Flow

```text
             Sensors
          IMU / Encoders
                |
                v
        State Estimation
                |
                v
       Trajectory Planning
                |
                v
      Kinematics / Dynamics
                |
                v
            Control
          /         \
        PID     Computed Torque
          \         /
                |
                v
         Motor Commands
                |
                v
         Motor Drivers
                |
                v
            Actuators
                |
                v
          Robot Motion
                |
                └──────> Sensor Feedback
```

---

## Hardware Requirements

The hardware architecture consists of:

* Microcontroller
* IMU
* Motors and actuators
* Motor drivers
* Encoders
* Power supply and battery system
* Voltage regulation and power distribution circuitry

Detailed hardware specifications will be documented as the design is finalized.

---

## Software Requirements

The software development environment includes:

* Embedded firmware development tools
* MATLAB / Simulink for modelling and simulation
* Python for analysis and supporting tools
* CAD software for mechanical design
* Git and GitHub for version control and documentation

---

## Technologies Used

* Embedded Systems
* IMU-based State Estimation
* PID Control
* Computed Torque Control
* Kinematic Modelling
* Lagrangian Dynamics
* Newton-Euler Dynamics
* Inverse Dynamics
* Joint-Space Trajectory Planning
* Cartesian-Space Trajectory Planning
* Cubic Spline Trajectory Generation
* MATLAB / Python Simulation
* CAD Design
* PCB and Electronics Design
* Git and GitHub

---

## Methodology

The project follows an iterative development approach:

1. Literature survey and study of dynamically balanced robotic systems.
2. Identification of mechanical, electrical, and control requirements.
3. Mechanical and electrical system design.
4. Selection of actuators, sensors, controllers, and power components.
5. Development of the kinematic model.
6. Development of the dynamic model using Lagrangian mechanics.
7. Development of inverse dynamics using the Newton-Euler method.
8. Development of joint-space and Cartesian-space trajectories.
9. Implementation of PID-based balance control.
10. Development of computed torque control for dynamic movement.
11. Simulation and validation of the control system.
12. Hardware fabrication and system integration.
13. Experimental testing and performance evaluation.

---

## Project Timeline

| Phase | Task                                        | Status      |
| ----- | ------------------------------------------- | ----------- |
| 1     | Problem definition and literature survey    | In Progress |
| 2     | System and mechanical design                | Planned     |
| 3     | Kinematic and dynamic modelling             | Planned     |
| 4     | Electronics and embedded system development | Planned     |
| 5     | Trajectory planning and control development | Planned     |
| 6     | Prototype fabrication and integration       | Planned     |
| 7     | Testing and validation                      | Planned     |
| 8     | Final documentation                         | Planned     |

---

## Weekly Progress Updates

| Week   | Date | Work Completed | Next Steps | Issues / Challenges |
| ------ | ---- | -------------- | ---------- | ------------------- |
| Week 1 |      |                |            |                     |

---

## Design Files

Design files are maintained in the following directories:

```text
hardware/
├── CAD/
├── PCB/
├── schematics/
└── BOM/
```

---

## Circuit Diagram

The circuit diagram will be added as the electronics architecture is finalized.

---

## Flowchart / Algorithm

The control flow of ZAEMON follows a continuous feedback loop:

```text
Start
  |
  v
Initialize System
  |
  v
Read IMU / Encoder Data
  |
  v
Estimate Robot State
  |
  v
Generate Desired Trajectory
  |
  v
Calculate Control Action
  |
  v
Generate Motor Commands
  |
  v
Drive Motors
  |
  v
Read Updated State
  |
  └──────────────> Repeat
```

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

### Software Implementation

The software is organized into firmware, control algorithms, simulation, and supporting tools.

```text
software/
├── firmware/
├── control/
├── simulation/
└── tools/
```

The control software handles:

* Sensor acquisition
* State estimation
* Trajectory generation
* Dynamic calculations
* Controller execution
* Motor command generation
* Real-time feedback

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

## References

References, research papers, datasheets, and other technical resources used during development will be maintained in:

```text
reference/
```

---

## Repository Update Guidelines

The repository will be updated continuously throughout the development of ZAEMON.

* Keep the README updated with major project changes.
* Maintain weekly progress updates.
* Commit code and design changes regularly.
* Upload relevant CAD, PCB, schematic, and simulation files.
* Document important engineering decisions.
* Add experimental results as testing progresses.
* Use meaningful commit messages.
* Avoid committing temporary or unnecessary generated files.

### Example Commit Messages

```text
Add initial system architecture
Add ZAEMON mechanical design
Add IMU interface
Implement balance controller
Add Lagrangian dynamic model
Add Newton-Euler inverse dynamics
Add trajectory generation
Update weekly progress
Add prototype testing results
Update control documentation
```

---

## License

This project is developed as part of the BE Capstone Project at VESIT, Mumbai.

**For academic and research purposes.**
