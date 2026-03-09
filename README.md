# Task Priority Kinematic Control of an Underwater Vehicle Manipulation System (UVMS) and Bimanipulation

**Description:** Task Priority Kinematic Control of an Underwater Vehicle Manipulation System (UVMS) and Bimanipulation.

**Authors:** Milad Rabiei, Arian Tavousi

---

## References

This project is based on the task-priority control framework presented in the following works:

**[10]** E. Simetti and G. Casalino,  
*A novel practical technique to integrate inequality control objectives and task transitions in priority based control*,  
Journal of Intelligent & Robotic Systems, vol. 84, no. 1, pp. 877–902, 2016.  
https://doi.org/10.1007/s10846-016-0363-4

**[11]** E. Simetti, G. Casalino, F. Wanderlingh, and M. Aicardi,  
*Task priority control of underwater intervention systems: Theory and applications*,  
Ocean Engineering, vol. 164, pp. 40–54, 2018.  
https://doi.org/10.1016/j.oceaneng.2018.06.061

---

# Project Overview

This repository contains the MATLAB implementation of a **Task-Priority Inverse Kinematics (iCAT)** scheme applied to complex cooperative robotic systems.

The project explores **hierarchical control**, **null-space projections**, and **rigid-body constraints** across three distinct robotic challenges.

The primary control architecture relies on enforcing **strict task priorities**, where higher-priority tasks override lower-priority objectives. This ensures that critical safety constraints—such as **joint limits, altitude constraints, and collision avoidance**—are always respected while secondary operational objectives are executed.

---

# Part 1: UVMS Complete Mission

This module simulates an **Underwater Vehicle-Manipulator System (UVMS)** performing a sequence of autonomous actions.

The control vector is the UVMS generalized velocity:

```
ẏ = [q̇ᵀ, νᵀ]ᵀ ∈ ℝ¹³
```

which includes:

- **7-DoF manipulator joint velocities**
- **6-DoF vehicle base velocities**

---

## Mission Phases

### Safe Waypoint Navigation

The vehicle navigates toward the target waypoint:

```
[10.5, 37.5, -38]
```

Safety constraints ensure:

- minimum altitude above the seabed
- stable vehicle attitude
- smooth trajectory tracking

---

### Landing and Alignment

The vehicle descends toward the target nodule while aligning its **yaw axis** with the horizontal projection of the nodule direction.

This phase ensures the manipulator is correctly oriented before interaction.

---

### Fixed-Base Manipulation

Once positioned, the vehicle **locks its pose** and the manipulator arm performs a scanning motion around the nodule.

This phase isolates arm motion from base disturbances.

---

### Action Transitions

Transitions between mission phases are implemented using **continuous bell-shaped activation functions**.

These functions smoothly blend task activation levels, preventing:

- velocity discontinuities
- high jerk
- unstable switching behavior

---

# Part 2: Bimanual Manipulation (Centralized)

This module uses **two 7-DoF Franka Panda manipulators**, modeled as a single centralized system.

The combined control vector is:

```
ẏ = [q̇_Lᵀ, q̇_Rᵀ]ᵀ ∈ ℝ¹⁴
```

where:

- `q_L` = left arm joint configuration  
- `q_R` = right arm joint configuration

---

## Mission Phases

### Move-to Grasp Points

Both end-effectors move to predefined grasp frames on the object while enforcing:

- joint limit avoidance
- a **minimum altitude constraint of 15 cm**

---

### Rigid Transport

A **6D rigid-body constraint**

```
ẋ_RBC = 0₆
```

is activated.

This ensures that both manipulators generate the **same object twist**, guaranteeing the object behaves as a **rigid body** during transport.

This avoids:

- internal stresses
- tearing forces between the arms

---

### Hold

At the goal pose:

- joint velocities are set to zero
- only the **minimum altitude safety task** remains active

---

# Part 3: Cooperative Manipulation (Distributed)

This advanced module treats the two Franka Panda arms as **independent agents** executing a **distributed coordination policy** rather than centralized control.

---

## Distributed Coordination Algorithm

### 1. Independent Estimation (Solo TPIK)

Each robot independently computes its preferred object velocity:

```
ẋ_t,i
```

while respecting its local safety constraints (such as joint limits).

---

### 2. Consensus (Negotiation)

The robots exchange their proposed velocities and compute a **weighted average**:

```
x̂_t = ( μ_L ẋ_t,L + μ_R ẋ_t,R ) / ( μ_L + μ_R )
```

The weights dynamically adapt based on tracking error, naturally producing a **leader–follower behavior** if one robot experiences difficulties.

---

### 3. Kinematic Consistency (Projection)

To avoid internal forces on the object, the negotiated velocity is projected onto the feasible subspace using the constraint matrix:

```
C = H_L - H_R
```

This produces the final cooperative velocity:

```
ẋ_coop
```

---

### 4. Coordinated Execution

Both robots then run a **secondary Task-Priority Inverse Kinematics (TPIK)** optimization where tracking the cooperative velocity becomes the **highest-priority task**.

---

# Tools and Technologies

### MATLAB
- Core numerical computations
- Task priority control implementation
- Simulation loops and matrix projections

### ROBUST / Unity
- Visualization environment for the **UVMS underwater mission**

### Python / RST
- Visualization tools
- URDF processing
- Kinematic simulation for the **Franka Panda bimanual system**
