---
title: Automatic Delivery Turtlebot Systems
tags:
  - tag1
  - tag2
---

# Introduction

- **Project**: Automatic Delivery Turtlebot Systems  
- **Team**: 12  
- **Members**: Sandy Lin, Alan Cheng, Yibo Yuan  
- **Academic Term**: Spring 2025  
- **University**: Arizona State University  
- **Course**: RAS598 — Experimentation and Deployment of Robotic Systems  
- **Instructor**: Prof. Daniel M. Aukes  

---

# Project Overview

## Scope
Develop an automated delivery workflow by integrating a UR5 robotic arm with a TurtleBot 4. Two workstations (A & B) hold red and blue blocks. UR5 places blocks on the TurtleBot; the robot then classifies the colour and autonomously delivers each block to the correct workstation, optimising sorting and intra‑lab logistics.

## Data Acquisition & Filtering
* **LiDAR** → 3‑D point‑cloud mapping and obstacle detection.  
* **IMU** → Motion & stability feedback.  
* **Camera (RGB)** → Colour recognition under variable lighting.

During system tests we verify sensor accuracy, ensure real‑time mapping, and confirm that navigation avoids collisions.

## Interaction (RViz)
RViz displays TurtleBot state, LiDAR scan, map, TF frames, navigation goals, and the Nav2‑generated path for live debugging.

## Control & Autonomy
Refined sensor data feeds low‑level controllers for collision avoidance and stability, while a high‑level planner (Nav2) updates paths so the TurtleBot can deliver blocks based on vision‑detected colour.

## Development Prerequisites
1. Configure UR5 ⇌ TurtleBot communication (ROS 2).  
2. Select an efficient path‑planning algorithm (A*, Dijkstra, or alternative).  
3. Integrate Python libraries such as OpenCV for object detection.

---

# Final Demonstration

## Resources Required
- UR5 arm & TurtleBot 4
- IMU, LiDAR, wheel encoders
- High‑performance workstation
- Reference projects (e.g., GitHub) & course material

## Classroom Setup
- UR5 on table adjacent to obstacle course
- Small blocks (red/blue)
- Modular boards for configurable obstacles
- Clear safety perimeter for mobile robot

## Test Matrix
Robustness is evaluated under varied obstacle layouts. Accuracy metrics compare sensor data against ground truth to validate adaptability.

---

# Impact
This project deepens our skills in multi‑robot coordination, ROS 2 middleware, RViz/Digital‑Twin visualisation, custom GUI design, and advanced path planning.

# Advising
Prof. Aukes provides ROS 2 expertise, motion‑planning advice, and hardware access. We will also consult domain specialists as needed.

---

# Project Progress

## Achievements

### 1 UR5 Pick‑and‑Place
<iframe width="560" height="315" src="https://www.youtube.com/embed/wdnD8_uXcG0" title="UR5 Robotic Arm Demo" frameborder="0" allowfullscreen></iframe>

### 2 GUI Enhancement
*Added **Origin** button to send TurtleBot to the staging area.*

<img src="./images/GUI.jpg" alt="GUI Interface" width="600" />

### 3 Pre‑defined Motions
<iframe width="560" height="315" src="https://www.youtube.com/embed/AvNOWas0qkQ" title="Pre‑defined Actions" frameborder="0" allowfullscreen></iframe>

### 4 Initial Path Planning (Nav2)
Implemented obstacle‑aware path planning in `ttb_nav.py`.

---

## Upcoming Tasks
1. Continuous pick–deliver loop controller.  
2. Optimise path planning (A*, Dijkstra).  
3. Integrate RViz digital‑twin visualisation.  
4. Full system validation.

---

# Code Breakdown

## GUI (`gui.py`)
PyQt5 interface + ROS 2 node publishing TurtleBot state, velocity, and goals; plots IMU linear acceleration with a moving‑average filter.

## UR5 Control Nodes
- `move_to_position.py` → Joint & Cartesian trajectories.  
- `get_position.py` → Joint/pose feedback.

## URSim & Physical Robot
Validated control nodes in URSim; hardware constraints handled with Dashboard API and a URP script (`load_and_run_script.py`).

## TurtleBot State Machine (`turtlebot_state.py`)
Publishes `/turtlebot_state`; detects arrival by monitoring zero velocity.

## Navigation State Machine (`ttb_nav.py`)
Translates high‑level states into `navigate_to_pose` goals; feedback‑aware.

---

# Unresolved Tasks

| # | Issue | Attempted / Proposed Solution |
|---|-------|--------------------------------|
| 1 | **Create 3 Sensor Failure** – `/odom`, `/imu`, `/scan` absent | Launched `rf2o_laser_odometry` to compute odom from LiDAR. |
| 2 | **Missing `/odom` TF Frame** | Broadcast static transform: `static_transform_publisher 0 0 0 0 0 0 odom base_link`; frame still missing. |
| 3 | **SLAM Frame Drift** | Suspected LiDAR‑only odom causing drift; plan to filter LiDAR data and fuse encoder/IMU inputs. |

---

> **Repository:** <https://ras598-2025-s-team12.github.io/>
