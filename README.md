# Task Priority Kinematic Control of an Underwater Vehicle Manipulation System (UVMS) and Bimanipulation

**Description:** Task Priority Kinematic Control of an Underwater Vehicle Manipulation System (UVMS) and Bimanipulation.

[cite_start]**Authors:** Milad Rabiei, Arian Tavousi [cite: 2]

---

## Project Overview

[cite_start]This repository contains the MATLAB implementation of a Task-Priority Inverse Kinematics (iCAT) scheme applied to complex cooperative robotic systems[cite: 36, 418]. [cite_start]The project explores hierarchical control, null-space projections, and rigid-body constraints across three distinct robotic challenges[cite: 36, 465]. 

[cite_start]The primary control architecture relies on enforcing strict priorities where tasks earlier in the stack override lower-priority objectives, ensuring critical safety constraints (like joint limits and collision avoidance) are maintained while secondary operational tasks are executed[cite: 46, 465].

---

## Part 1: UVMS Complete Mission

[cite_start]This module simulates an Underwater Vehicle-Manipulator System (UVMS) performing a sequence of autonomous actions[cite: 25, 36]. [cite_start]The control vector is the UVMS generalized velocity $\dot{y}=[\dot{q}^T, \nu^T]^T \in \mathbb{R}^{13}$, encompassing both the 7-DoF arm and the 6-DoF vehicle base[cite: 37, 38, 39].

### Mission Phases

* [cite_start]**Safe Waypoint Navigation:** Drives the vehicle to a target waypoint [10.5, 37.5, -38] while guaranteeing safety via an altitude floor and keeping a stable attitude[cite: 27, 28, 51, 137].
* [cite_start]**Landing and Alignment:** The vehicle descends to the target nodule while aligning its yaw (z-axis) to the horizontal projection of the nodule's direction[cite: 29, 52].
* [cite_start]**Fixed-Base Manipulation:** The vehicle locks its pose while the manipulator arm scans the nodule[cite: 30, 31, 53]. 
* [cite_start]**Action Transitions:** Smooth transitions between mission phases are achieved by merging task activations using continuous bell-shaped functions to prevent velocity discontinuities and reduce jerk[cite: 48, 82].

---

## Part 2: Bimanual Manipulation (Centralized)

[cite_start]This exercise utilizes two 7-DoF Franka Panda manipulators, treating them as a single centralized 14-DoF system $\dot{y}=[\dot{q}_L^T, \dot{q}_R^T]^T \in \mathbb{R}^{14}$[cite: 418, 419, 456, 457]. 

### Mission Phases

* [cite_start]**Move-to Grasp Points:** Both end-effectors are driven to specific grasping frames on the object while rigorously enforcing joint limits and a minimum 15 cm altitude[cite: 431, 438, 440, 469].
* [cite_start]**Rigid Transport:** A 6D equality constraint $\dot{\overline{x}}_{RBC}=0_{6}$ is activated to ensure both arms induce the exact same object twist, carrying the object as a rigid body to the goal pose without generating internal tearing forces[cite: 442, 470, 583, 588].
* [cite_start]**Hold:** Joint velocities are set to zero, leaving only the minimum altitude safety task active[cite: 452, 471].

---

## Part 3: Cooperative Manipulation (Distributed)

[cite_start]Building on the bimanual setup, this advanced module treats the two Franka Panda arms as independent, decentralized agents executing a Distributed Coordination Policy[cite: 730, 774, 775].

### Distributed Coordination Algorithm

* [cite_start]**Independent Estimation (Solo TPIK):** Each agent calculates its ideal object velocity $\dot{x}_{t,i}$ considering its local safety constraints (like joint limit avoidance)[cite: 777].
* [cite_start]**Consensus (Negotiation):** The agents exchange proposed velocities and compute a weighted average $\hat{x}_{t}=\frac{\mu_{L}\dot{x}_{t,L}+\mu_{R}\dot{x}_{t,R}}{\mu_{L}+\mu_{R}}$[cite: 778, 779]. [cite_start]Weights dynamically adapt based on tracking error, naturally creating a leader-follower dynamic if one arm struggles[cite: 781, 782].
* [cite_start]**Kinematic Consistency (Projection):** To prevent internal forces on the object, the averaged velocity is projected onto the feasible subspace using the constraint matrix $C=H_{L}-H_{R}$ to generate the final cooperative velocity $\dot{\dot{x}}_{coop}$[cite: 783, 784, 785].
* [cite_start]**Coordinated Execution:** Both arms execute a secondary TPIK optimization specifically to track the negotiated cooperative velocity as their highest priority[cite: 788].

---

## Tools and Technologies

* [cite_start]**MATLAB:** Core computation, simulation loop, and iCAT matrix projections[cite: 7, 420].
* [cite_start]**ROBUST / Unity:** Visualization environment for the UVMS underwater simulation[cite: 7].
* [cite_start]**Python / RST:** Visualization and URDF kinematics processing for the bimanual Franka Panda simulations[cite: 420, 425].
