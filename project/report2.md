---
layout: default
title: "Milestone 2"
parent: Project
nav_order: 2
---
# Milestone 2

{: .no_toc }

---

## 1. Differential Drive Kinematics Model

### State Vector

The robot's state in the world frame is:

```math
q = \begin{bmatrix} x \\\ y \\\ \theta \end{bmatrix}
```
<br/><br/>

### Control Inputs

The control inputs are the angular velocities of the right and left wheels: $​\dot{\phi}_R$ and $\dot{\phi}_L$​, where wheel radius is $r$ and track width (wheelbase) is $L$.
<br/><br/>

### Forward Kinematics — Mapping from Wheel Velocities to Body/World Velocity

First, wheel angular velocities map to linear wheel speeds:

```math
v_{right} = r\dot{\phi}_R \quad \quad v_{left} = r\dot{\phi}_L
```
<br/><br/>

These combine to give the robot's linear and angular velocity:

```math
^xv = \frac{r}{2} (\dot{\phi}_R + \dot{\phi}_L) \quad \quad \omega = \dot{\theta} = \frac{r}{L} (\dot{\phi}_R - \dot{\phi}_L)
```
<br/><br/>

### Full World-Frame State Update (the kinematic model)

```math
\dot{q} = v_{world} = \begin{bmatrix} ^xv cos(\theta) \\\ ^xvsin(\theta) \\\ \dot{\theta} \end{bmatrix} = \begin{bmatrix} \frac{r}{2} (\dot{\phi}_R + \dot{\phi}_L) cos(\theta) \\\ \frac{r}{2} (\dot{\phi}_R + \dot{\phi}_L) sin(\theta) \\\ \frac{r}{L} (\dot{\phi}_R - \dot{\phi}_L) \end{bmatrix}
```
<br/><br/>


---