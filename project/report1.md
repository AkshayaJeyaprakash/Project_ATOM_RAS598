---
layout: default
title: "Report 1"
parent: Project
nav_order: 1
---

# Report 1: Proposal & Architecture

{: .no_toc }

This page breaks down our proposed project into the mission and scope of the project, the specifications of the robot used, architecture, protocol, and git infestructure.

---

## Table of Contents

{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 1. Missions Statement & Scope

Define the mission statement, scope, and the "success state" of your project. Identify the specific environment (e.g., cluttered warehouse, narrow hallway) and the primary problem your robot is designed to solve.

---

## 2. Technical Specifications

  - Robot Platform: Hardware (TurtleBot 4, etc.) or specific simulation environment.
  - Kinematic Model: Declare the model (Differential Drive, Ackermann, or Holonomic).
  - Preception Stack: List all sensors used (LiDAR, RGB-D, IMU).

--- 

## 3. High Level System Architecture

  - Mermaid Diagram
  Provide a visual flow following the Perception, Estimation, Planning and Actuation convention. This diagram must illustrate how data moves through various modules in the system.

  - Module Declaration Table
  A table listing every module in your diagram, explicitly labeled as either Library (existing ROS 2 packages) or Custom (code you will write).

  - Module Intent

    - Libraries
    Provide a 50-150 word writeup describing the intent behind choosing a specific package for module and the configuration you intend to tune (e.g. max velocity of the differential drive controller).

    - Custom Libraries
    Provide a 100-200 word writeup describing the specific algorithm you intend to implement from scratch (e.g., "Implementing a custom RRT* to navigate narrow passages"). Note: This abstract forms the "contract" for your Algorithmic Factor grade.

---

## 4. Safety & Operational Protocol
Define the software Deadman Switch or any timeout logic to prevent hardware damage in the event of communication loss. Describe the specific conditions that trigger a system-wide "E-Stop."

---

## 5. Git Infrastructure
Link to your shared team repository and confirm the Git Submodule setup is active on your individual site.

---

- [ ] Akshaya
- [ ] Moss
- [ ] Nivas