---
title: Automatic Delivery Turtlebot Systems
tags:
  - tag1
  - tag2
---

[TOC]

# I. Introduction

- **Project**: Automatic Delivery Turtlebot Systems  
- **Team**: 12  
- **Members**: Sandy Lin, Alan Cheng, Yibo Yuan  
- **Academic Term**: Spring 2025  
- **University**: Arizona State University  
- **Course**: RAS598 — Experimentation and Deployment of Robotic Systems  
- **Instructor**: Prof. Daniel M. Aukes  

---

# II. Project Overview Update

## Scope
This project aims to develop an automated delivery system by integrating a UR5 robotic arm with a TurtleBot. Two workstations, A and B, will be set up alongside red and blue blocks. The UR5 arm will transfer the colored blocks onto the TurtleBot, which will then identify each block’s color and autonomously navigate to deliver it to the designated workstation. Through this integration, we seek to optimize the sorting and delivery workflow.

## Data Acquisition & Filtering
* **LiDAR** → 3‑D point‑cloud mapping and obstacle detection.  
* **IMU** → Motion & stability feedback.  
* **Camera (RGB)** → Colour recognition under variable lighting.

The LiDAR will be used to collect a 3D point cloud and convert it into a map. During testing, we ensure that the LiDAR accurately senses the environment with real-time data to generate the complete map. The IMU is evaluated to confirm that it precisely measures movement and maintains stability, which is crucial for the robot's mobility. Additionally, the camera’s ability to recognize colors under various conditions will be tested. In the final demonstration, the LiDAR will be fully integrated to enable smooth navigation, ensuring that the Turtlebot moves efficiently towards workstations with the ability to avoid obstacles without collision. The IMU will be used to maintain the Turtlebot's balance and ensure stable movement during delivery tasks.

## Interaction (RViz)
We will use RViz to visualize the turtlebot states, LiDAR data, environment map, tf frames, navigation goals (two workstations), the planned path generated by Nav2, and debugging in real-time.

## Control & Autonomy
The processed sensor data enables real-time responses such as collision avoidance, stability adjustments, and immediate path corrections. For instance, the LiDAR data helps detect obstacles, while the IMU provides vital motion and balance information. At the higher level, the controller uses the refined sensor data to update path planning, ensuring the Turtlebot navigates efficiently and delivers blocks to the designated workstation based on color recognition.

## Development Prerequisites
1. Configure UR5 ⇌ TurtleBot communication (ROS 2).  
2. Select an efficient path‑planning algorithm (A*, Dijkstra, or alternative).  
3. Integrate Python libraries such as OpenCV for object detection.

To build the system, we need to understand how communication between UR5 and TurtleBot 4 works. Choosing the most suitable path-planning algorithm is crucial for efficient TurtleBot 4 navigation and minimizing delivery time. Lastly, integrating useful Python packages like OpenCV for object detection will enhance the system's capabilities.

---

# III. Resources & Evaluation Plan

## Resources Required
- UR5 arm & TurtleBot 4
- IMU, LiDAR, wheel encoders
- High‑performance workstation
- Reference projects (e.g., GitHub) & course material given by Prof. Aukes.

## Classroom Setup
- UR5 robot arm and gripper
- Small blocks (red/blue)
- Several boards make up the obstacle course. 
- Enough space for robotic operation

## Test Matrix
- Robustness is evaluated under varied obstacle layouts. Accuracy metrics compare sensor data against ground truth to validate adaptability.

We will test under several different obstacle conditions to ensure that the robot are robust enough to handle variability. Several tests will be conducted to see if the robot works accurately. Comparing the sensor data and analog outputs to real-world conditions to ensure that our designed algorithms can accurately adapt to environmental changes.

## Impact
This project will enhance our understanding of multi-robot communication by coordinating the UR5 robotic arm and TurtleBot 4. Building an Rviz simulation will help us simulate and test the system. Developing our own user interface will improve interaction and control. Lastly, exploring different path-planning algorithms will deepen our knowledge of autonomous navigation.

## Advising
Dr. Aukes serves as our advisor for this project, providing technical guidance and hardware support. With his expertise in ROS2 development, robotic motion planning, and control systems, he plays a crucial role in helping us navigate the technical challenges of integrating the UR5 robotic arm and Turtlebot. Additionally, we plan to seek further technical guidance from other experts in the field to ensure the success of our project.

---

# IV. Project Progress

## Achievements

### 1 UR5 Pick‑and‑Place
*We successfully implemented control over the UR5 robotic arm, enabling it to perform pick-and-place tasks from point A to point B.*

<iframe width="560" height="315" src="https://www.youtube.com/embed/wdnD8_uXcG0" title="UR5 Robotic Arm Demo" frameborder="0" allowfullscreen></iframe>

### 2 GUI Enhancement
*Added an "Origin" button based on the previously developed GUI. After initiating the system with the "Start" button, pressing "Origin" commands the Turtlebot to move to the designated position and wait to receive colored blocks.*

<img src="./images/GUI.jpg" alt="GUI Interface" width="600" />

### 3 RQT graph

<img src="./images/rqt_graph.jpg" alt="rqt_graph" width="600">

<img src="./images/gui_cmd_flowchart.png" alt="gui_cmd_flowchart" width="600">

### 4 Pre‑defined Actions
*We successfully enabled the TurtleBot to execute predefined actions. When button A is pressed, it moves forward 3 meters and then turns left for 1 meter; when button B is pressed, it moves forward 3 meters and then turns right for 1 meter.*

<iframe width="560" height="315" src="https://www.youtube.com/embed/AvNOWas0qkQ" title="Pre‑defined Actions" frameborder="0" allowfullscreen></iframe>

<img src="./images/gui_program_flowchart.png" alt="gui_program_flowchart" width="600">

### 5 Initial Path Planning (Nav2)
Created an initial version of path planning within ttb_nav.py, incorporating obstacle detection and avoidance using ROS2's Nav2 framework.

---

## Upcoming Tasks
1. Continuous pick–deliver loop controller.<br>
Develop a controller that enables continuous round-trip operations between the UR5, the color detection positions, and the drop-off block position.
2. Optimise path planning (A*, Dijkstra).<br>
Optimize the current path planning logic with more advanced algorithms such as A* or Dijkstra to enhance navigation efficiency and stability.
3. Integrate RViz digital‑twin visualisation.<br>
Add RViz visualization to support real-time monitoring and build a digital twin for the delivery system.
4. Full system validation.

---

# V. Code Breakdown

## GUI (`gui.py`)
1. The [`gui.py`](https://github.com/RAS598-2025-S-Team12/RAS598-2025-S-Team12.github.io/blob/main/src/t12_prj/t12_prj/gui.py), defines a TurtleBotGUI class that inherits from both rclpy.Node and QtWidgets.QMainWindow, integrating a PyQt5 interface with ROS2 communication.<br>
2. It creates publishers for `/turtlebot_state`, `/gui_cmd_vel`, and `/simple_goal`, and subscribers for `/default_vel`, `/c3_14/odom`, and `/c3_14/imu`, handling state, velocity commands, odometry, and IMU data.<br>
3. The GUI layout comprises a read-only logging panel, a Matplotlib canvas plotting raw and FIR-filtered linear acceleration over time, and text fields displaying current linear and angular velocities.<br>
4. Spin-box controls with Reset/Send buttons enable manual velocity entry, while large START/STOP and A/B/Origin buttons toggle operation and dispatch predefined navigation goals, each action logged with a timestamp.<br>
5. Callback methods (odom_callback, imu_callback) update velocity displays and append IMU samples to a rolling buffer; update_plot applies a moving-average filter to the latest N samples before redrawing the acceleration graph.<br>

## UR5 Control Nodes
<ol>
  <li>The ROS2 nodes, implemented in Python, include:</li>
    <ul>
    <li>
      <a href="https://github.com/RAS598-2025-S-Team12/RAS598-2025-S-Team12.github.io/blob/main/src/ur5_control/ur5_control/move_to_position.py">
        <code>move_to_position.py</code>
      </a>：sends trajectory commands to control robot motion.
    </li>
    <li>
      <a href="https://github.com/RAS598-2025-S-Team12/RAS598-2025-S-Team12.github.io/blob/main/src/ur5_control/ur5_control/get_position.py">
        <code>get_position.py</code>
      </a>：subscribes and monitors robot joint states and end‑effector poses.
    </li>
    </ul>
  <li>Publishers and subscribers involved:</li>
    <ul>
      <li>Publishes JointTrajectory messages to `/scaled_joint_trajectory_controller/joint_trajectory`.</li>
      <li>Subscribes to `/joint_states for joint angles`.</li>
      <li>Subscribes to `/tcp_pose_broadcaster/pose` for end-effector position feedback.</li>
    </ul>
  <li>Motion strategy:</li>
    <ul>
      <li>Vertical pick-and-place movements utilize linear trajectory commands (moveL) for precision.</li>
      <li>Horizontal movements employ joint-space trajectory commands (moveJ) for efficient operation.</li>
    </ul>
  <li>Feedback system:</li>
    <ul>
      <li>Continuously monitors joint angles and end-effector positions.</li>
      <li>Ensures closed-loop accuracy for task execution.</li>
    </ul>
</ol>

## URSim & Physical Robot
<ol><li>Validation environment:
<ul><li>Official Universal Robots ursim_e-series simulation software validates the ROS2 node performance.<br>

<iframe width="560" height="315" src="https://www.youtube.com/embed/uVdeHsFNLeA?si=uVlgHfMbPelUGMhy" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe></li></ul></li>

<li>Physical hardware constraints:
<ul><li>Simultaneous ROS2 operation of robot and gripper not feasible due to hardware limitations.</li></ul></li>

<li>Alternative solution implemented:
<ul><li>URP control script (ur5_control.urp) pre-developed.</li>
<li>
  <a href="https://github.com/RAS598-2025-S-Team12/RAS598-2025-S-Team12.github.io/blob/main/src/ur5_programs/load_and_run_script.py">
    <code>load_and_run_script.py</code>
  </a>：communicates with robot via Dashboard&nbsp;server.<br>

<iframe width="560" height="315" src="https://www.youtube.com/embed/_y8J7vZQI5s?si=MiuTpfxab6QTnRho" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe></li></ul></li>

<li>Python script functionality:
<ul><li>Establishes TCP socket connection to robot Dashboard server.</li>
<li>Loads and executes predefined URP program.</li>
<li>Enables coordinated robotic arm and gripper actions.<br>

<iframe width="560" height="315" src="https://www.youtube.com/embed/RPhDaaa79vg" title="UR5 Robotic Arm Demo" frameborder="0" allowfullscreen></iframe></li></ul></li>
</ol>


## TurtleBot State Machine (`turtlebot_state.py`)
1. The [`turtlebot_state.py`](https://github.com/RAS598-2025-S-Team12/RAS598-2025-S-Team12.github.io/blob/main/src/t12_prj/t12_prj/turtlebot_state.py) defines a TurtleBotState class that inherits from rclpy.Node, containing the logic for monitoring and publishing the robot’s current action state.
2. It creates a publisher on `/turtlebot_state` and two subscribers—one to the same topic for echoing state updates (state_callback), and one to `/c3_14/cmd_vel` for velocity commands (vel_callback)—then initializes action_in_progress, zero_cmd_time, and a 1 Hz timer (check_arrival_condition).
3. The publish_state(text) method wraps text into a String message, publishes it, and logs the update; state_callback sets action_in_progress (“A”, “B” or None) based on incoming state strings.
4. The vel_callback watches for consecutive zero-velocity Twist messages during an active action, stamping the first zero-command time, while non-zero commands reset that timer.
5. Every second, check_arrival_condition checks if the robot has held zero velocity for more than 3 seconds during an action—if so, it publishes either “Arrived at A” or “Arrived at B” and then clears the action state.

## Navigation State Machine (`ttb_nav.py`)
1. The [`ttb_nav.py`](https://github.com/RAS598-2025-S-Team12/RAS598-2025-S-Team12.github.io/blob/main/src/t12_prj/t12_prj/ttb_nav.py) defines a TtbNav class that inherits from rclpy.Node, implementing a navigation state machine which listens to `/turtlebot_state` commands and translates them into Nav2 NavigateToPose goals.
2. It retrieves three parameters `origin_pos`, `ws1_pos`, `ws2_pos` as [x, y, yaw_deg] from [`ttb_pos_point.yaml`](https://github.com/RAS598-2025-S-Team12/RAS598-2025-S-Team12.github.io/blob/main/src/t12_prj/config/ttb_pos_point.yaml), logs these goal positions, initializes an ActionClient for the `navigate_to_pose` action server, and subscribes to `/turtlebot_state`, while tracking the last sent goal with `current_goal_tag`.
3. The state_cb callback strips and logs each incoming state string, ignores `Idle` or `AtLoad`, then maps "StartReturn"/"Origin" → `origin`, "StartDelivery1" → `ws1`, "StartDelivery2" → `ws2`, invoking `send_goal` only if the requested tag differs from `current_goal_tag`.
4. The send_goal(pos_xyz, tag) method converts the [x, y, yaw_deg] tuple into a PoseStamped (using quaternion_from_euler for the yaw), stamps it in the map frame with the current time, logs the outgoing goal, updates `current_goal_tag`, and calls `send_goal_async` with `feedback_cb` attached.
5. The feedback and result callbacks handle the rest: `feedback_cb` logs the remaining distance, `goal_resp_cb` checks acceptance (resetting the tag on rejection and chaining `result_cb`), and `result_cb` logs success, cancellation or failure, then clears `current_goal_tag` so new goals can be sent.

---
# VI. Topic learned

## UR5 Troubleshooting
<ol>
  <li>Controller initialization failure</li>
    <ul>
    <li>
      Problem: Launching ur_control.launch.py resulted in the spawners being unable to contact /controller_manager/list_controllers, causing controller loading to fail.
    </li>
    <li>
      Cause:
      <ol>
        <li>
          ROS 2 relies on DDS discovery
          <ul>
            <li>
              By default Cyclone DDS advertises every participant via UDP multicast (239.255.0.1:7400). When all processes can hear this packet they automatically discover each other and open the required service channels.
            </li>
          </ul>
        </li>
        <li>
          <AllowMulticast>false disables all multicast traffic
            <ul>
              <li>
                With multicast blocked, discovery packets are neither sent nor received. Unless every peer (including the local host) is listed manually, different ROS 2 processes – even on the same machine – cannot see each other, so the spawner never finds /controller_manager/list_controllers.
              </li>
            </ul>
          </li>
          <li>
            <DontRoute>true sets the SO_DONTROUTE socket flag
            <ul>
              <li>
                This forces Cyclone DDS to send only to directly-connected networks and to ignore any address that requires routing. In a VM or a host with several interfaces (loopback, bridge, etc.) this further prevents the participants from discovering each other.
              </li>
            </ul>
          </li>
          <li>
            After both lines were commented out
            <ul>
              <li>
                Multicast packets are allowed again → automatic discovery works.
              </li>
              <li>
                Routing is permitted → packets can travel through the VM bridge and loopback.
              </li>
              <li>
                Consequently, the spawner can reach /controller_manager/list_controllers, and the launch sequence completes successfully.
              </li>
            </ul>
          </li>
        </ol>
      </li>
      <li>
        Solution: These two lines were commented out to restore default multicast-based discovery, allowing the controllers to load successfully.
      </li>  
    </li>
    </ul>

  <li>Real-time scheduling not enabled</li>
    <ul>
      <li>Warning: Your system/user seems not to be setup for FIFO scheduling.</li>
      <li>Cause: By default, real-time scheduling is not enabled in Ubuntu, and the real-time group and related permissions are not set.</li>
      <li>Solution:
        <ol>
          <li>
            Add user to proper groups:
            <ul>
              <li>
                sudo groupadd realtime<br>
                sudo usermod -aG realtime $(whoami)<br>
                sudo usermod -aG rtkit $(whoami)
              </li>
            </ul>
          </li>
          <li>
            Create /etc/security/limits.d/99-realtime.conf with:
            <ul>
              <li>
                @realtime   - rtprio     99  <br>
                @realtime   - memlock    unlimited  <br>
                @realtime   - nice      -20  
              </li>
            </ul>
          </li>
          <li>
            Reboot system
            <ul><li>sudo reboot</li></ul>
          </li>
          <li>
            Verify group membership
            <ul><li>groups $(whoami)</li></ul>
          </li>
        </ol>
      </li>
      </li>
    </ul>
  
  <li>Calibration data not applied</li>
    <ul>
      <li>Problem: The robot driver starts, but the real robot’s pose may deviate from the MoveIt visualization.</li>
      <li>Cause: The robot_calibration.yaml file is missing or not applied.</li>
      <li>Solution:
        <ol>
          <li>
            Run the calibration launch file:
            <ul><li>ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=<robot_ip>
            </li></ul>
          </li>
        </ol>
      </li>
    </ul>
              

</ol>



---

# VII. Unresolved Tasks

| # | Issue | Problem Description | Attempted / Proposed Solution |
|---|-------|--------------------|--------------------------------|
| 1 | **Create 3 Platform Sensor Failure** – `/odom`, `/imu`, `/scan` not published | The iRobot Create 3 platform failed to initialize properly. As a result, essential ROS 2 topics such as `/odom`, `/imu`, and `/scan` were not published, thereby disabling key navigation and localization functionalities. | Integrated the `rf2o_laser_odometry` package to estimate odometry from LiDAR data:<br>`ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py laser_scan_topic:=/rpi_14/scan` |
| 2 | **Missing `/odom` TF Frame** | Despite launching the odometry node, the `/odom` frame did not appear in the TF tree, which is essential for many localization and navigation stacks. | Added static transform:<br>`ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link`<br>However, `/odom` still missing when verifying with `ros2 run tf2_tools view_frames`. |
| 3 | **SLAM Frame Drift in RViz** | When executing SLAM through the `turtlebot4_navigation` stack, RViz showed growing drift between frames over time. This led to inconsistencies between the robot's estimated and actual positions, making localization unreliable. | The drift likely stems from the fact that /odom was derived entirely from LiDAR-based odometry without integration of wheel encoders or IMU data. A potential remedy is to apply filtering to the LiDAR measurements—removing spurious (“garbage”) data—to improve the accuracy of the /odom estimation. |

---

> **Repository:** <https://github.com/RAS598-2025-S-Team12/RAS598-2025-S-Team12.github.io/tree/main/src>
