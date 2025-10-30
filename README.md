# Project 2 – Reactive TurtleBot on Physical Robot  
**CS 5023: Intro to Intelligent Robotics**

---

## 📌 Overview
This project runs a **TurtleBot 2** (Kobuki base + 3D/depth sensor) on the **real robot in REPF B4** using ROS Melodic.  
The robot implements the **same reactive control behaviors from Project 1**, but now on actual hardware instead of Gazebo.  
Depth data from the camera is converted to a 2D laser scan on `/scan`, bumpers are read from `/mobile_base/events/bumper`, and odometry from `/odom`.  
The behavior node arbitrates among these inputs and publishes the final velocity to **`/mobile_base/commands/velocity`**.

**Behaviors implemented (priority order):**
1. **Halt** – Stop when bumpers detect collision.  
2. **Keyboard teleoperation** – Accept manual `/cmd_vel` inputs when safe.  
3. **Escape** – If symmetric obstacles (< ~1 ft ahead, left ≈ right) are detected, back up + turn ~180° (±30°).  
4. **Avoid** – Turn away from asymmetric obstacles within ~1 ft in front.  
5. **Random turn** – After every ~1 ft forward movement (from `/odom`), turn randomly ±15° (tunable).  
6. **Drive forward** – Default motion.  

This matches the assignment requirement: “The details of these behaviors and the overall reactive architecture are the same as for Project 1, so it should be possible to simply use the software you implemented for Project 1 to control the robot in Project 2.”

---

## 📂 Repository Structure

```text
catkin_wsp/

├── src/

│   └── project2/

│       ├── launch/

│       │   └── behaviors_turtlebot.launch

│       ├── scripts/

│       │   └── Behaviors_Full_Turtlebot.py

│       ├── CMakeLists.txt

│       └── package.xml
