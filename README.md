
<!-- @import "[TOC]" {cmd="toc" depthFrom=1 depthTo=6 orderedList=false} -->
# Military Drones Simulation

A ROS 2 Jazzy + Gazebo Harmonic based multi-drone surveillance simulation. The system implements a **swarm of three modified X3 quadcopters** equipped with cameras and onboard object detection using YOLOv11.

---

## ✨ Features

* **Three surveillance drones** (`X3_1`, `X3_2`, `X3_3`) modeled in Gazebo.
* **Custom onboard cameras** for live video feeds.
* **YOLOv11 TFLite-based object recognition** via `object_recognizer.py`.
* **Centralized swarm planner** for multi-drone coordination.
* **GUI (`drone_gui.py`)** to monitor and interact with drones.
* **ROS 2 ↔ Gazebo bridge** for control and sensor integration.
* Predefined **environment objects** (car, horse, man, zebra, etc.).

---

## 📂 Repository Structure

```
military_drones/
├── LICENSE                         # License file
├── military_drones_bringup          # Launch and configs
├── military_drones_control          # Control nodes, GUI, object recognition
│   └── resources                    # YOLO model + dataset configs
├── military_drones_description      # Drone and environment models
├── military_drones_gazebo           # Gazebo plugins and worlds
└── README.md                        # Project documentation
```

---

## 🚀 Installation

Tested on **Ubuntu 24.04 (Noble Numbat)** with **ROS 2 Jazzy** on **Gazebo Harmonic**.

```bash
# Clone the repository
cd ~/ros_ws/src
git clone github.com/Manohara-Ai/Military_Drones military_drones

# Install dependencies
sudo apt update
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
cd ~/ros_ws
colcon build
source install/setup.bash
```

---

## ▶️ Running the Simulation

Launch the full system:

```bash
ros2 launch military_drones_bringup military_drones.launch.py
```

This will:

* Spawn the **3 X3 quadcopters** in Gazebo.
* Start swarm control nodes (`central_planner`, `flight_controller`).
* Run **YOLOv11 object recognition** (`object_recognizer`).
* Open the **GUI** and RViz visualization.
* Establish ROS 2 ↔ Gazebo bridges.

---

## 📦 Packages

### `military_drones_bringup`

* Launch files & configs.
* RViz visualization setup.

### `military_drones_control`

* `central_planner.py` → swarm mission coordination.
* `flight_controller.py` → per-drone control.
* `object_recognizer.py` → YOLOv11 inference.
* `drone_gui.py` → live monitoring GUI.
* `resources/` → contains YOLO model & dataset labels.

### `military_drones_description`

* Models of drones & environment objects.
* Textures, meshes, SDF configs.

### `military_drones_gazebo`

* Gazebo world (`world.sdf`).
* Plugins for system simulation.

---

## 📌 Future Work

* Enhance multi-drone autonomy with reinforcement learning.
* Implement SLAM-based navigation.
* Add real-time communication between drones.
* Improve GUI with mission planning tools.

---

## 👨‍💻 Authors

Developed by **[Manohara B M](https://github.com/Manohara-Ai)** & **[Vibhashree Vasuki](https://github.com/paaduka32)**.

---

## 📜 License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
