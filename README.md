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

catkin_wsp1/

├── src/

│ ├── project2/

│ │ ├── launch/

│ │ │ ├── behaviors_turtlebot.launch

│ │ ├── scripts/

│ │ │ └── Behaviors_Full_Turtlebot.py

│ │ ├── CMakeLists.txt

│ │ └── package.xml

## ⚙️ Installation

### 1. Clone Repository

cd ~/catkin_wsp/src
git clone https://github.com/bran-aviles/Project-2-ROS.git project2
cd ~/catkin_wsp
catkin_make


Source Workspace
source devel/setup.bash


Ensure Dependencies Installed

On CNS Linux machines, make sure the following packages are available (most preinstalled):

ROS Melodic

TurtleBot 2 packages:

turtlebot_bringup

kobuki_keyop

depthimage_to_laserscan

turtlebot_description

kobukie_description

yocs_controllers, yocs_cmd_vel_mux

If any are missing, copy them into ~/catkin_wsp/src/ from the default workspace:

cp -r ~/catkin_ws/src/<package-name> ~/catkin_wsp/src/


🚀 Running the Project
1. Set up the robot laptop (Turtlebot side)
  - source /opt/ros/melodic/setup.bash
  - source ~/catkin_wsp/devel/setup.bash
  - export ROS_MASTER_URI=http://localhost:11311
  - export ROS_HOSTNAME=IP_OF_TURTLEBOT

    ROS_MASTER_URI points to the master running on the robot.
    ROS_HOSTNAME must be the robot’s IP/hostname so other machines can reach it.

2. Set up the desktop (CSN machine)
  - source /opt/ros/melodic/setup.bash
  - source ~/catkin_wsp/devel/setup.bash
  - export ROS_MASTER_URI=http://IP_OF_TURTLEBOT:11311
  - export ROS_HOSTNAME=IP_OF_PC

    Here the desktop tells ROS: “the master is on the TurtleBot.”

3. SSH into the TurtleBot (to run bringup there)
- From the desktop (or another machine on the same network), open an SSH session to the TurtleBot:
  - ssh -Y IP_OF_TURTLEBOT

  - Inside that SSH shell on the robot, source again:
  - source /opt/ros/melodic/setup.bash
  - source ~/catkin_wsp/devel/setup.bash

    Now this SSH terminal is controlling the robot’s ROS master.

4. Start the 3D sensing (depth camera)
- Open a new terminal, then repeat from steps 2-3 in that terminal
  - roslaunch turtlebot_bringup 3dsensor.launch

    This launches the depth/3D sensor, so the system has /camera/depth/image_raw, which our launch will convert to /scan.

5. Run the behavior launch
  - roslaunch project2 behaviors_turtlebot.launch

    This launch file:
  - starts keyboard teleop (kobuki_keyop) → /cmd_vel

  - runs depthimage_to_laserscan → /scan

  - starts Behaviors_Full_Turtlebot.py → subscribes to /cmd_vel, /odom, /scan, and publishes to /mobile_base/commands/velocity


🤖 Reactive Controller (reactive_controller.py)

Bumper: Stops robot + triggers escape.

Laser scan: Used for symmetric escape, wall avoidance, and obstacle avoidance.

Odometry: Tracks distance → triggers random turn after ~0.3 m (~1 ft).

Teleop override: Accepts keyboard inputs but blocks unsafe ones.


🧾 Code Source / Academic Integrity Note

This Project 2 code did not start 100% from scratch. It is adapted from:

  1. Our Project 1 solution for CS 5023 (reactive controller in simulation, Gazebo + TurtleBot 2).

  2. The publicly available ROS workspace:
     - https://github.com/subhashchandra001/catkin_wsp1.git

Code from this repository was used as a starting point/reference for:
  - the structure of the reactive controller node (publisher/subscribers),
  - the ordering and logic of the behaviors (bumper halt → teleop → symmetric escape → asymmetric avoid → random turn → forward),
  - the use of /scan sectors to decide symmetric vs. asymmetric obstacles,
  - and the general ROS launch pattern for TurtleBot + depthimage_to_laserscan.

Original / modified parts in this project:

- Original to Project 2: topic names and remaps for the physical TurtleBot (e.g. publishing to /mobile_base/commands/velocity), SSH + bringup instructions for CSN, updated launch to include 3D sensor and teleop, and adjustments to angle-based random turn and escape thresholds for the real robot.

- Modified from Project 1 / GitHub source: scan callback sectoring, symmetric/asymmetric checks, and the priority ordering were kept but adapted to the real /scan produced by turtlebot_bringup/3dsensor.launch.

- Unmodified / referenced: the overall behavior description and ROS node structure (publisher/subscriber pattern) from Project 1.

This credit is provided to comply with the assignment statement: if you start from code that is freely available (web, friends, student organizations, past projects), you must give “a complete and accurate accounting of where all of your code came from and indicate which parts are original or changed.”

🧪 Testing

Bumper test: press a bumper → robot should stop immediately.

Teleop test: teleop should drive the robot when the path is clear, but should be blocked if an obstacle is too close.

Symmetric obstacle test: obstacle centered in front → robot should reverse and do ~180° turn.

Asymmetric obstacle test: obstacle only on left → robot should turn right (and vice versa).

Random turn test: let it drive ~1 ft → robot should do a small turn.

Default forward: in open space, it should go forward.

👥 Team Members
Brandon Aviles

Subhash Chandra

References:

[1]University of Oklahoma, School of Computer Science, CS 4023/5023 Intelligent Robotics – Fall 2024 Project 1: Reactive Robotics Using ROS and TurtleBot (Code Base), Norman, OK, USA, 2024.

[2] Open Robotics, Robot Operating System (ROS) Documentation. [Online]. Available: https://wiki.ros.org

[3] Open Robotics, Gazebo Simulation Environment Documentation. [Online]. Available: https://gazebosim.org

[4] R. A. Brooks, “A Robust Layered Control System for a Mobile Robot,” IEEE J. Robotics and Automation, vol. 2, no. 1, pp. 14–23, 1986.

[5] J. L. Jones, A. M. Flynn, and B. A. Seiger, Mobile Robots: Inspiration to Implementation, 2nd ed. Natick, MA, USA: A K Peters, 1999.

[6] R. R. Murphy, Introduction to AI Robotics. Cambridge, MA, USA: MIT Press, 2000.


