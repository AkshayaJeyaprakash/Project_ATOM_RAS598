---
layout: default
title: "Report 1"
parent: Project
nav_order: 1
---

# Report 1: Proposal & Architecture

{: .no_toc }

This page breaks down our proposed project into the mission and scope of the project, the specifications of the robot used, architecture, protocol, and git infestructure.

- [ ] Akshaya
- [ ] Moss
- [ ] Nivas

---

## Table of Contents

{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 1. Missions Statement & Scope

### 1.1 Mission statement
To create a robot that can be given simple spoken instructions that it then uses to determin the best corse of action and successfully completes the instructions

### 1.2 Scope 
- understanding spoken instructions
- autonomously exploring a room
- detecting and cataloging verbally specified objects
- creating a map of all the cataloged objects
- presenting the map to the user

### 1.3 Success state 
The robot can successfully be verbally instructed what objects to find then navigate the room with no additional user input while detecting and mapping the location of all specified objects

### 1.4 Enviornment
Where will the robot be used?

### 1.5 Primary problem
What is the project solving?

---

## 2. Technical Specifications

Robot Platform: TurtleBot 4

Kinematic Model: Declare the model (Differential Drive, Ackermann, or Holonomic)

Preception Stack: RPLIDAR A1M8, OAK-D-LITE, IMU

--- 

## 3. High Level System Architecture

### 3.1 Mermaid Diagram

```mermaid
flowchart TD;
    A["User"];
    A --> B(["User Input"]);
    B --> C["VLN Model"]
    C --> D(["VLN Model Interpretation"]);
    D --> E(["VLN Desicion"]);
    E --> F["Robot"];
    F --> G["Motors"];
    G --> H(["Navigation"]);
    F --> I["Camera"];
    F --> J["SLAM"];
    J --> M(["Robot Location"]);
    I --> K(["Object Detection"]);
    K --> Q(["Location of Object"]);
    M --> Q;
    Q --> N{"Finished Mapping Room"};
    N -->|Yes| O["Display Map With Object Locations"]
    N -->|No| P["Continue Search"]
```

### 3.2 Module Declaration Table

  | Module              | Library or Custom |
  |:----------------    |:------------------|
  | VLN Model           | Library           |
  | SLAM                | Library           |
  | Nav2                | Library           |
  | YOLO                | Library           |
  | VLN Integration     | Custom            |
  | Map Builder         | Custom            |
  | Exploration Manager | Custom            |
  | Action Translator   | Custom            |

### 3.3 Module Intent

  #### Libraries: 
    Provide a 50-150 word writeup describing the intent behind choosing a specific package for module and the configuration you intend to tune (e.g. max velocity of the differential drive controller).
    
VLN Model
        
SLAM
        
Nav2
        
YOLO
        

  #### Custom Libraries: 
    Provide a 100-200 word writeup describing the specific algorithm you intend to implement from scratch (e.g., "Implementing a custom RRT* to navigate narrow passages"). Note: This abstract forms the "contract" for your Algorithmic Factor grade.
    
VLN Integration
        
Map Builder
        
Exploration Manager
        
Action Translator
        

---

## 4. Safety & Operational Protocol
Define the software Deadman Switch or any timeout logic to prevent hardware damage in the event of communication loss. Describe the specific conditions that trigger a system-wide "E-Stop."

Speed limit

Stop and back up if robot runs into object

---

## 5. Git Infrastructure
Link to your shared team repository and confirm the Git Submodule setup is active on your individual site.
