---
layout: default
title: "Milestone 2"
parent: Project
nav_order: 2
---

# Report 2: Mid-Point Technical Proof

{: .no_toc }

This page presents the mid-point technical development of the project, detailing the implemented system architecture, the underlying kinematic model, the complete ROS 2 computational pipeline, and an analysis of real-world system behavior, including sensor uncertainty and run-time performance.

---

## Table of Contents

{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 1. Differential Drive Kinematics Model

The differential drive is one of the most common mobile robot configurations. It consists of two independently actuated wheels mounted on the same axis, with a passive caster wheel for balance. By varying the speeds of the left and right wheels, the robot can move forward, backward, and turn. The kinematics model below formally describes how the robot's motion is determined by its wheel velocities.

&nbsp;

### 1.1 State Vector

The robot's configuration in the environment is captured by a state vector that describes its position and orientation in the 2D world frame. The state vector is defined as:

$$
q = \begin{bmatrix} x \\\ y \\\ \theta \end{bmatrix}
$$

Where:
- $$x$$ and $$y$$ represent the robot's position in the world frame.
- $$\theta$$ represents the robot's heading angle (orientation) measured counterclockwise from the world x-axis.

&nbsp;

### 1.2 Control Inputs

The robot is controlled by independently commanding the angular velocities of its two drive wheels. These are the sole inputs that determine the robot's motion, making the differential drive an underactuated system — it can only directly control two degrees of freedom (linear and angular velocity) despite operating in a 3-DOF space $$(x, y, \theta)$$.

The control inputs are the angular velocities of the right and left wheels: $$​\dot{\phi}_R$$ and $$\dot{\phi}_L$$​, where wheel radius is $$r$$ and track width (wheelbase) is $$L$$.

&nbsp;

### 1.3 Forward Kinematics — Mapping from Wheel Velocities to Body/World Velocity

Forward kinematics describes how low-level wheel commands translate into the robot's motion. Each wheel's angular velocity is first converted into a tangential linear speed at the wheel's contact point with the ground:

$$
v_{right} = r\dot{\phi}_R \quad \quad v_{left} = r\dot{\phi}_L
$$

Where $$r$$ is the wheel radius. Since both wheels are rigidly coupled to the same chassis, their individual speeds can be combined to determine the robot's overall linear velocity $$^xv$$ (the average of both wheel speeds) and angular velocity $$\omega$$ (proportional to their difference). When both wheels spin at equal speeds the robot moves in a straight line; when they differ, the robot turns:

$$
^xv = \frac{r}{2} (\dot{\phi}_R + \dot{\phi}_L) \quad \quad \omega = \dot{\theta} = \frac{r}{L} (\dot{\phi}_R - \dot{\phi}_L)
$$

Where $$L$$ is the track width (distance between the two wheels).

&nbsp;

### 1.4 Full World-Frame State Update (the kinematic model)

The robot's body-frame velocities onto the world frame using the heading angle $$\theta$$. This yields the complete kinematic model, a direct mapping from wheel control inputs $$(\dot{\phi}_R, \dot{\phi}_L)$$ to the time derivatives of the robot's state $$(x, y, \theta)$$:

$$
\dot{q} = v_{world} = \begin{bmatrix} ^xv cos(\theta) \\\ ^xvsin(\theta) \\\ \dot{\theta} \end{bmatrix} = \begin{bmatrix} \frac{r}{2} (\dot{\phi}_R + \dot{\phi}_L) cos(\theta) \\\ \frac{r}{2} (\dot{\phi}_R + \dot{\phi}_L) sin(\theta) \\\ \frac{r}{L} (\dot{\phi}_R - \dot{\phi}_L) \end{bmatrix}
$$

This model assumes planar motion on a flat surface, pure rolling contact (no wheel slip), and a rigid chassis, which are all consistent with the standard differential drive assumptions. These equations serve as the foundation for odometry estimation and motion planning in the system's implementation.

&nbsp;

---

## 2. System Architecture

### 2.1 Detailed Computational Map

TODO

---

## 3. Experimental Analysis & Validation

### 3.1 Noise & Uncertainty Analysis

### 3.2 Run-Time Issues

### 3.3 Milestone Video

TODO

---

## 4. Project Management

### 4.1 Instructor Feedback Integration

| Critiques/Questions | Specific Technical Actions Taken |
|---------------------|----------------------------------|
| 1. Do you plan to add more objects to the environment? How will you handle objects added or removed by other students in a common space during operation? |  |
| 2. How exactly are you calculating positional error? |  |
| 3. What specifically are "suggested actions" under the VLN integration? |  |
| 4. I’m not quite understanding the need for the YOLO model if the VLN model already takes camera images and instructions as inputs. Could you clarify? |  |
| 5. If the VLN outputs waypoints in the SLAM map, what custom work is required to convert that to a Nav2 goal? I see that as a very short task, is the bulk of the custom work in the semantic map builder? |  |

### 4.2 Individual Contribution

| Team Member | Primary Technical Role | Key Git Commits/PRs | Specific File(s) Authorship (Direct Links) |
|-------------|------------------------|---------------------|--------------------------------------------|
| Akshaya J |  |  |  |  |
| Moss Barnett |  |  |  |
| Nivas Piduru |  |  |  |

---