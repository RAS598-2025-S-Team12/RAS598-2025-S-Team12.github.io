---
title: Project Timeline
---


``` mermaid

graph TD
    Start[System Start] --> UR5_Pickup[UR5 randomly picks up object]
    UR5_Pickup --> Wait_Turtlebot[Wait for Turtlebot arrival]

    Wait_Turtlebot -->|Turtlebot arrived| Place_Object[Place object on Turtlebot platform]
    Wait_Turtlebot -->|Turtlebot not arrived| Wait_Turtlebot

    Place_Object --> UR5_Reset[UR5 returns to standby position]
    UR5_Reset --> Wait_Turtlebot

    Place_Object --> Turtlebot_Identify[Turtlebot identifies Aruco marker on object]
    Turtlebot_Identify --> Path_Planning[Turtlebot plans path to workstation based on Aruco marker]
    Path_Planning --> Navigate_Workstation[Turtlebot navigates to workstation]

    Navigate_Workstation --> Arrive_Workstation[Arrives at workstation]
    Arrive_Workstation --> Turtlebot_Arm_Pickup[Turtlebot's arm picks up object]
    Turtlebot_Arm_Pickup --> Place_Workstation[Object placed onto workstation]

    Place_Workstation --> Turtlebot_Return[Turtlebot returns to predefined location]
    Turtlebot_Return --> Wait_For_Object[Turtlebot waits for next object]

    Wait_For_Object -->|Next object ready| Wait_Turtlebot
    Wait_Turtlebot -->|Process completed| End[Process End]

```
