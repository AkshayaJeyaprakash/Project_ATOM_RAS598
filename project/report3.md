---
layout: default
title: "Milestone 3"
parent: Project
nav_order: 3
---

# Milestone 3

{: .no_toc }

---

{: .warning }
> ⚠️ **This milestone is not yet complete.**  
> The content in this page is a placeholder and will be updated as the project progresses.

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
