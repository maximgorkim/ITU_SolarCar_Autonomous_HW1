# ITU ZES Solar Car Team – Autonomous Systems Homework 1

## Project Overview
This repository contains the implementation of Homework 1 from the ITU ZES Solar Car Team Autonomous Systems Group. 
The project demonstrates basic ROS 2 Humble functionalities, publisher–subscriber communication, and trajectory tracking control using the **Stanley Method**.

---

## Repository Structure
```
📂 turtle_controller/
 ├── keyboard_controller.cpp   # Manual/Autonomous keyboard teleop
 ├── path_publisher.cpp        # Publishes predefined square path
 ├── stanley_controller.cpp    # Path-following controller (Stanley)
 └── twist_demo.cpp            # Simple Twist message demo
```

---

## 1️⃣ Keyboard Controller
A C++ node that allows manual control of the turtle via keyboard inputs (W, A, S, D). 
- Press **X** to toggle between *Manual* and *Autonomous* mode. 
- Press **Space** to stop immediately. 
- Publishes `geometry_msgs/Twist` messages to `/turtle1/cmd_vel`. 

**Explanation:** 
> The node uses SDL to capture keyboard events and publish real-time velocity commands to control the turtle. It supports simultaneous key presses for smooth motion and allows switching between manual and automatic control modes.

---

## 2️⃣ Path Publisher
Publishes a continuous **square-shaped path** centered near `(5.5, 5.5)` as a `nav_msgs/Path` message on the topic `/plan`. 
Used as a reference trajectory for the Stanley Controller.

**Explanation:** 
> This node creates a dense sequence of waypoints forming a closed square and republishes it periodically so that late subscribers (like RViz or the controller) can still visualize the path.

---

## 3️⃣ Stanley Controller
Implements the **Stanley Path-Following Algorithm** in C++. 
- Subscribes to the path and the turtle’s pose. 
- Computes cross-track and heading errors. 
- Combines feedback and feedforward curvature terms to generate steering commands. 
- Publishes adaptive velocity commands to `/turtle1/cmd_vel`. 
- Prints accumulated total lateral error in the terminal.

**Explanation:** 
> The controller aligns the turtle with the reference path using the Stanley control law. It calculates steering based on both geometric errors and curvature prediction, adjusting linear velocity dynamically for stable and smooth path tracking.

---

## 4️⃣ Twist Demo
A simple example node that continuously publishes fixed velocity commands to demonstrate ROS 2 message publishing.

---

## How to Build & Run
```bash
cd ~/ws_hw
colcon build
source install/setup.bash
```

Then run the nodes in separate terminals:
```bash
ros2 run turtle_controller keyboard_controller
ros2 run turtle_controller path_publisher
ros2 run turtle_controller stanley_controller
```

---

## Deliverables
- Screenshots of successful node operation 
- Visualization of the followed path in RViz 
- Terminal output showing total lateral error 
- Short report (PDF) with explanations and GitHub link 

---

## Author
**Mehmet Görkem Keskin** 
Istanbul Technical University – Control and Automation Engineering 
GitHub: [@maximgorkim](https://github.com/maximgorkim)
