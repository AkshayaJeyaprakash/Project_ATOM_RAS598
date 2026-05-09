---
layout: default
title: "Milestone 3"
parent: Project
nav_order: 3
---

# Milestone 3

{: .no_toc }

---

## 1. Abstract

ATOM (Autonomous Task & Object Management) is a semantic robot object-retrieval system built on a TurtleBot4 Lite. A user types or speaks a natural language command such as *"find the red bottle"* through a web interface. The system parses the command using a Large Language Model, recalls where similar objects were seen before from a persistent spatial memory, navigates to those locations using Nav2, visually detects and validates the object using a three-model ensemble of YOLO, CLIP, and LLaVA, drives to within arm's reach using closed-loop depth control, and celebrates success with an ascending audio sweep and LED animation. The entire system requires no environment-specific retraining — it works zero-shot in any mapped indoor space.

This report describes how the system works end-to-end, the mathematics behind each custom module, and an evaluation of its performance and ethical implications.

---

## 2. System Architecture & Pipeline Flow

ATOM is a distributed system spanning three physical compute nodes: the TurtleBot4 Lite robot, an Ubuntu VM running the ROS2 middleware and Nav2, and a host laptop running the AI inference server. The diagram below shows both the architecture (which node handles what) and the data flow between them.

```mermaid
flowchart TD
    USER(["👤 User Command\n'find the red bottle'"])
    UI["🖥️ Streamlit UI\nstreamlit_app.py\n/atom/resolved_target"]

    subgraph VM ["Ubuntu VM — ROS2 Jazzy"]
        direction TB
        EC["🧠 exploration_coordinator.py\nState Machine · LLM Parser\nMemory Nav · Centering · Approach"]
        OD["👁️ object_detector.py\nYOLO Inference · Target Matching\nOpenCV Display"]
        GP["🗺️ goal_publisher.py\nNav2 Action Client\nCostmap Manager"]
        MM["💾 memory_mapper.py\nJSON Spatial Memory\n~/maps/memory.json"]
        SM["🛡️ safety_monitor.py\nEmergency Stop\nBattery Autodock"]
        NAV2["⚙️ Nav2 Stack\nAMCL · MPPI · BT · Costmaps"]
    end

    subgraph HOST ["Host Laptop — Flask :5000"]
        direction TB
        YOLO["YOLOv8n\n/detect"]
        CLIP["CLIP ViT-B/32\n/clip_score"]
        LLAVA["LLaVA via Ollama\n/llava_parse_task\n/llava_reason"]
    end

    subgraph ROBOT ["TurtleBot4 Lite — robot_03"]
        direction TB
        CAM["OAK-D RGB\n250×250 30Hz"]
        DEPTH["OAK-D Stereo\n640×400 14Hz"]
        LIDAR["RPLiDAR A1\n720pts 7.7Hz"]
        MOTORS["cmd_vel\nDiffDrive"]
        CELEB["🎉 celebration_node.py\nAudio + LED"]
    end

    USER --> UI
    UI -->|"/atom/resolved_target"| EC

    EC -->|"POST /llava_parse_task\ntask + memory_list"| LLAVA
    LLAVA -->|"yolo_class, memory_object\ndescription"| EC

    EC -->|"Load memory.json\nSort by confidence"| MM
    EC -->|"/atom/resolved_class\nTRANSIENT_LOCAL"| OD
    EC -->|"/exploration_goal\n{x, y, final}"| GP

    GP -->|"NavigateToPose\naction goal"| NAV2
    NAV2 -->|"cmd_vel"| MOTORS
    NAV2 -->|"/atom/nav_status\nGOAL_REACHED"| EC

    CAM -->|"/oakd/rgb"| OD
    OD -->|"POST /detect\nbase64 JPEG"| YOLO
    YOLO -->|"detections[]"| OD
    OD -->|"/atom/object_spotted\n{class, conf, bbox}"| EC
    OD -->|"/atom/detections"| MM

    EC -->|"POST /clip_score\nimage + text"| CLIP
    EC -->|"POST /llava_reason\nimage + target"| LLAVA
    CLIP -->|"cosine score"| EC
    LLAVA -->|"present: true/false"| EC

    EC -->|"/atom/depth_bbox\n{bbox, class}"| DEPTH
    DEPTH -->|"/atom/get_depth service\ndistance_m"| EC

    EC -->|"/cmd_vel_unstamped\nTwist"| MOTORS
    EC -->|"/atom/task_status\nGOAL COMPLETED"| CELEB
    CELEB -->|"/cmd_audio\nAudioNoteVector"| ROBOT
    CELEB -->|"/led_animation\nLedAnimation"| ROBOT

    SM -->|"10Hz zero vel\nemergency override"| MOTORS
    SM -->|"/atom/emergency_stop"| EC
    SM -->|"/atom/emergency_stop"| GP

    LIDAR -->|"/scan"| NAV2

    style EC fill:#1e3a5f,color:#fff
    style OD fill:#1e3a5f,color:#fff
    style GP fill:#1e3a5f,color:#fff
    style MM fill:#1e3a5f,color:#fff
    style SM fill:#7b1e1e,color:#fff
    style CELEB fill:#1e5f3a,color:#fff
    style YOLO fill:#5f3a1e,color:#fff
    style CLIP fill:#5f3a1e,color:#fff
    style LLAVA fill:#5f3a1e,color:#fff
```

When the user issues a command, it flows to the `exploration_coordinator` — the brain of the system. The coordinator parses the command using LLaVA, checks its spatial memory for known object locations, and begins navigating. While the robot moves, `object_detector` continuously runs YOLO on camera frames looking for the target. When it spots something, the coordinator stops the robot, centers it visually, checks depth, validates the detection with CLIP and LLaVA, and drives to the object. On success, the `celebration_node` on the robot plays a sound and flashes the LEDs. The `safety_monitor` runs in parallel at all times, watching for emergency stops and low battery. The `memory_mapper` silently records everything the robot sees into a JSON file, which is consulted on future commands.

---

## 4. Ethical Impact Statement

Our autonomous TurtleBot 4 system integrates natural language understanding, computer vision-based object detection, and semantic mapping to explore environments and identify user-specified objects. This combination introduces important ethical considerations in privacy, safety, and bias that must be addressed both in current implementation and future iterations.

From a privacy standpoint, the robot uses onboard RGB and depth sensing for object detection and mapping. These sensors may capture incidental visual data such as people, personal belongings, or sensitive environments during exploration. Although our current system does not explicitly store or transmit personally identifiable information, future versions that log semantic maps or annotated images could unintentionally retain sensitive data. To mitigate this, we should implement data minimization strategies, such as discarding raw images after inference, applying real-time anonymization techniques like face or text blurring, and restricting long-term storage of environment data.

In terms of safety, the robot operates using a differential drive model and autonomous exploration strategies in potentially dynamic environments. Motion is governed by wheel velocities, meaning unsafe velocity commands or perception failures could result in collisions. This is especially relevant when combining exploration with object-seeking behavior, which may bias the robot toward goal completion over obstacle avoidance. Safety can be improved through strict velocity limits, reliable obstacle detection using depth sensing, and fail-safe mechanisms such as emergency stops. Future iterations should include redundancy across sensing modalities and more robust runtime validation of navigation decisions.

Bias in the system primarily stems from hardware and perception limitations. Vision-based object detection models such as YOLO may exhibit uneven performance depending on lighting, object appearance, or dataset bias. Additionally, depth sensors and LiDAR struggle with reflective or transparent surfaces such as glass, leading to incomplete environmental understanding. These limitations can cause the robot to perform inconsistently across environments, creating unequal reliability. Sensor fusion, combining vision, depth, and potentially LiDAR, can help reduce these biases.

Applying the Utilitarian Test, the system provides clear benefits by enabling intuitive human-robot interaction and autonomous exploration, but these benefits must be balanced against risks such as privacy exposure and physical harm. The Justice Test highlights the need for consistent performance across environments, ensuring that failures do not disproportionately affect certain users or settings. The Virtue Test emphasizes responsible engineering practices, requiring us to prioritize safety, transparency, and continuous system improvement.

Overall, ethical deployment of this system requires anticipating these challenges and proactively designing safeguards that scale with system capability.

---

## 5. Custom Module Code Links

Camera Processor ([camera_processor.py](https://github.com/AkshayaJeyaprakash/Project_ATOM_RAS598/blob/Milestone_3/atom_ws/src/atom_bringup/atom_bringup/camera_processor.py))

Object Detector ([object_detector.py](https://github.com/AkshayaJeyaprakash/Project_ATOM_RAS598/blob/Milestone_3/atom_ws/src/atom_bringup/atom_bringup/object_detector.py))

Exploration Coordinator ([exploration_coordinator.py](https://github.com/AkshayaJeyaprakash/Project_ATOM_RAS598/blob/Milestone_3/atom_ws/src/atom_bringup/atom_bringup/exploration_coordinator.py))

Goal Publisher ([goal_publisher.py](https://github.com/AkshayaJeyaprakash/Project_ATOM_RAS598/blob/Milestone_3/atom_ws/src/atom_bringup/atom_bringup/goal_publisher.py))

Memory Mapper ([memory_mapper.py](https://github.com/AkshayaJeyaprakash/Project_ATOM_RAS598/blob/Milestone_3/atom_ws/src/atom_bringup/atom_bringup/memory_mapper.py))

Safety Monitor ([safety_monitor.py](https://github.com/AkshayaJeyaprakash/Project_ATOM_RAS598/blob/Milestone_3/atom_ws/src/atom_bringup/atom_bringup/safety_monitor.py))

Streamlit ([streamlit.py](https://github.com/AkshayaJeyaprakash/Project_ATOM_RAS598/blob/Milestone_3/atom_ws/src/atom_bringup/atom_bringup/streamlit.py))

--- 

## 6. Individual Contribution & Audit Appendix

TODO: Fill out the team members' contributions in a table

| Team Member | Primary Technical Role | Key Commits | Specific File(s) / Components |
|-------------|----------------------|-------------|-------------------------------|
| Akshaya J |  | [](), [](), [](), []() |  |
| Nivas Piduru |  | [](), [](), [](), []() |  |
| Moss Barnett |  | [](), [](), [](), []() |  |
---
