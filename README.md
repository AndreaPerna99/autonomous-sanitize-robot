# TurtleBot3 Complete Sanitization System (ROS2 Project)

[![Python](https://img.shields.io/badge/Python-3.10-blue?logo=python)](https://www.python.org/)  
[![ROS2](https://img.shields.io/badge/Framework-ROS2-orange?logo=ros)](https://docs.ros.org/en/humble/index.html)

Autonomous exploration, mapping, navigation, and sanitization of unknown environments using TurtleBot3 Burger and ROS2 Humble.

---

## 📌 Project Info
- 🎓 Course: Autonomous and Mobile Robotics
- 🏫 University of Bologna
- 📅 Year: 2023/2024

---

## 🧠 About the Project

This project develops a complete **autonomous robotic system** for indoor **exploration, mapping, navigation**, and **sanitization**.  
It is based on **TurtleBot3 Burger** simulated in Gazebo and completely developed in **Python** for ROS2 Humble.

The robot autonomously explores an **unknown environment**, builds a **map**, localizes itself with **AMCL**, navigates sequentially through **room centers**, and performs **systematic sanitization** using **Boustrophedon decomposition**.

The system features:
- Fully autonomous pipeline, from unknown map to full sanitization.
- Real-time user interaction for task setup.
- Modular and parameterized architecture for multiple environments.
- Energy-aware UV lamps control during navigation and sanitization.

---

## 🗂 Project Structure

```
📦 ros2_ws/
┣ 📁 src/my_robot_controller/
┃ ┣ 💻 complete_sanitization.py       # Core controller node
┃ ┣ 💻 velocity_control_FSM.py        # FSM-based low-level obstacle avoidance
┃ ┣ 📁 maps/                          # Stored PGM + YAML maps
┃ ┗ 📄 README.md                       # You are here!
┣ 📄 install/
┣ 📄 build/
┣ 📄 log/
┣ 📄 Colcon configuration
```

---

## ⚙️ How to Run

### Setup Workspace
```bash
cd ros2_ws/
colcon build --symlink-install
. install/setup.bash
```

---

## 🎯 Main Tasks

### 1. Obstacle Avoidance (FSM Control)
- **Gazebo launch** (Terminal 1):
  ```bash
  export TURTLEBOT3_MODEL=burger
  ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py x_pose:=-2.0 y_pose:=1.0
  ```
- **Launch FSM** (Terminal 2):
  ```bash
  ros2 run my_robot_controller velocity_control_FSM
  ```

---

### 2. Simultaneous Localization and Mapping (SLAM)

- **Gazebo launch** (Terminal 1)
- **RVIZ and SLAM launch** (Terminal 2):
  ```bash
  ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True
  ```
- **Start Exploration (Explore Lite)** (Terminal 3):
  ```bash
  ros2 launch explore_lite explore.launch.py return_to_init:=True
  ```
- **Save map once finished** (Terminal 4):
  ```bash
  ros2 run nav2_map_server map_saver_cli -f maps/house_map
  ```

---

### 3. Autonomous Navigation on Known Map

- **Gazebo launch** (Terminal 1)
- **Load Map and Navigation stack** (Terminal 2):
  ```bash
  ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=<path_to_saved_map>.yaml
  ```
- **Launch complete_sanitization.py** (Terminal 3):
  ```bash
  ros2 run my_robot_controller complete_sanitization
  ```
  Then **type**:
  ```
  Navigation
  House
  ```

The robot will autonomously navigate through the map by moving through the centers of the specified rooms.

---

### 4. Autonomous Sanitization

- **Gazebo + RVIZ launch** as above.
- **Launch complete_sanitization.py**:
  ```bash
  ros2 run my_robot_controller complete_sanitization
  ```
  Then **type**:
  ```
  Sanitization
  House
  ```

The robot will navigate to each room and perform a **Boustrophedon Decomposition** to sanitize it completely with UV lamps.

---

## 🛠 Technologies and Tools

- **ROS2 Humble**
- **Gazebo Simulator**
- **TurtleBot3 Burger**
- **Python**
- **Explore Lite Package** for exploration
- **Navigation2 (Nav2)** for AMCL localization and path planning
- **Boustrophedon Decomposition** for sanitization

---

## 📊 Features

- **Interactive Experience**:  
  User selects map and task type at runtime.

- **Map Independence**:  
  Supports any custom map through parameterized config files.

- **Autonomous Localization**:  
  Robot automatically localizes using **Adaptive Monte Carlo Localization (AMCL)** and monitors covariance in real-time.

- **Navigation**:  
  Rooms are visited sequentially based on their center coordinates extracted from map data.

- **Sanitization**:  
  UV lamps activated only during sanitization, following **Boustrophedon decomposition** to guarantee full area coverage.

- **Energy Awareness**:  
  UV lamps off during navigation, energy statistics updated in real-time.

- **Visualization**:  
  Sanitization progress published on `/map` topic and visualized in **Rviz** using custom OccupancyGrid maps.

---

## 🖼️ Project Snapshots

- **Robot Sanitization in BigHouse Map**  
  ![Sanitization](./images/sanitization_example.png)

- **Dynamic Path Planning Visualization**  
  ![Exploration](./images/path_planning.png)

- **Occupancy Grid Update**  
  ![Occupancy Grid](./images/occupancy_grid.png)

---

## 📎 Resources

- 📘 Complete project report (coming soon)

---

## 👨‍🎓 Authors

Andrea Perna  
Giuseppe Speciale  
Riccardo Marras  

MSc Automation Engineering – University of Bologna

📧 and.perna99@gmail.com

---

## 👩‍🏫 Supervisors

- Prof. Gianluca Palli  
- Dr. Kevin Galassi  

---

## 📜 License

All rights reserved. Educational and demonstrative use only.

---
