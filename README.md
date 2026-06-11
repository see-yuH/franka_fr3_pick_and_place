# FR3 Pick-and-Place Simulation

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://releases.ubuntu.com/22.04/)
[![License](https://img.shields.io/badge/License-Apache--2.0-green)](LICENSE)

A ROS 2 package (`fr3_delivery_sim`) that simulates a **Franka Research 3 (FR3)** robotic arm performing autonomous pick-and-place and block-sorting tasks in Gazebo. The pipeline integrates MoveIt 2 for collision-aware motion planning, an overhead camera for object detection, and `ros2_control` with fake hardware for full simulation fidelity — no physical robot required.

---

## Table of Contents

- [Overview](#overview)
- [Repository Structure](#repository-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Running the Simulation](#running-the-simulation)
- [System Architecture](#system-architecture)
- [Troubleshooting](#troubleshooting)

---

## Overview

This project demonstrates a complete end-to-end robotic manipulation pipeline in simulation:

1. Objects are spawned dynamically in a Gazebo world.
2. An overhead vision node detects their 3D coordinates and publishes them.
3. MoveIt 2 plans a collision-free trajectory to the object.
4. The FR3 arm executes the grasp, transports the object, and releases it at the target zone.

All motion is executed using MoveIt 2's `MoveGroupInterface` over the `joint_trajectory_controller`, with the robot hardware handled by `ros2_control` in fake mode.

---

## Repository Structure

```
franka_fr3_pick_and_place/
├── config/           # MoveIt 2, ros2_control, and controller configuration files
├── launch/           # Launch files for bring-up
├── src/              # Python nodes (vision detector, pick-and-place executor)
├── urdf/             # Robot description and Gazebo world files
├── CMakeLists.txt
└── package.xml
```

---

## Prerequisites

Ensure the following are installed and configured before building:

| Requirement | Version / Notes |
|---|---|
| Operating System | Ubuntu 22.04 |
| ROS 2 | Humble Hawksbill |
| Gazebo | Classic (Gazebo 11) or Ignition Fortress |
| MoveIt 2 | Humble release |
| ros2_control | Humble release |
| Git, Colcon | Latest available |

> This package is built on top of the official [franka_ros2](https://github.com/frankarobotics/franka_ros2) workspace. **Complete that setup first** before proceeding with the installation steps below.

---

<img width="800" height="446" alt="Image" src="https://github.com/user-attachments/assets/176bd0e8-38c0-40bc-801d-b0140cc6debb" />

<img width="1047" height="241" alt="Image" src="https://github.com/user-attachments/assets/10245363-93ff-439e-9c43-f25d26f0d82d" />

---

## Installation

### 1. Set up the Franka ROS 2 workspace

Follow the official `franka_ros2` setup guide completely before proceeding:

> https://github.com/frankarobotics/franka_ros2

This sets up the base workspace at `~/ros2_ws` with all Franka dependencies, URDF descriptions, mesh files, and MoveIt configuration already built.

### 2. Clone this package into the existing workspace

```bash
cd ~/ros2_ws/src
git clone https://github.com/see-yuH/franka_fr3_pick_and_place.git
```

### 3. Install any additional dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Rebuild the workspace

```bash
colcon build --symlink-install
```

### 5. Source the workspace

```bash
source install/setup.bash
```

> **Tip:** Add this line to your `~/.bashrc` so you don't need to source manually in each new terminal:
> ```bash
> echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
> ```

---

## Running the Simulation

The simulation requires three separate terminals. Source the workspace in each one before running.

### Terminal 1 — Launch Gazebo and spawn objects

```bash
ros2 launch fr3_delivery_sim sim.launch.py
```

This brings up the Gazebo world, spawns the FR3 robot with `ros2_control`, and dynamically places objects in the scene.

### Terminal 2 — Start the vision detector

```bash
ros2 run fr3_delivery_sim vision_detector.py
```

Processes the overhead camera feed and publishes the `[x, y, z]` coordinates of detected objects as ROS topics.

### Terminal 3 — Execute pick-and-place

Default run:
```bash
ros2 run fr3_delivery_sim pick_and_place.py
```

To increase movement speed:
```bash
ros2 run fr3_delivery_sim pick_and_place.py --ros-args -p use_sim_time:=true -p vel_scale:=0.3 -p acc_scale:=0.3 -p cart_speed:=0.06
```

To change the drop location:
```bash
ros2 run fr3_delivery_sim pick_and_place.py --ros-args -p use_sim_time:=true -p drop_x:=0.55 -p drop_y:=0.20
```

Subscribes to the detected object positions, calls MoveIt 2 to plan trajectories, and commands the FR3 arm to grasp and sort each object.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                     Gazebo Simulation                   │
│   FR3 Robot (ros2_control / fake hardware)              │
│   Overhead Camera  │  Spawned Objects                   │
└────────┬───────────┴──────────────────┬─────────────────┘
         │ /image_raw                   │ object poses
         ▼                              ▼
  [vision_detector.py]         publishes /detected_object
         │
         └──────────────────────────────┐
                                        ▼
                           [pick_and_place.py]
                                        │
                           MoveIt 2 MoveGroupInterface
                                        │
                           joint_trajectory_controller
                                        │
                              FR3 executes motion
```

**Node summary:**

- **`vision_detector.py`** — Subscribes to the simulated camera, segments objects by colour/depth, and publishes target coordinates.
- **`pick_and_place.py`** — Reads target coordinates, calls MoveIt 2 to plan to a pre-grasp pose, closes the gripper, lifts, transports, and drops the object at the delivery zone.

---

## Troubleshooting

**Gazebo loads but the robot model is missing**
Verify that the `franka_ros2` workspace was set up and built successfully — it provides the `.dae` and `.stl` mesh files required by the FR3 URDF. Re-run `colcon build` if needed.

**Controllers fail to load on startup**
Check that the controller names in `config/` match those defined in your `ros2_control` YAML. Run `ros2 control list_controllers` to inspect the active state.

**MoveIt 2 reports no valid plan**
The spawned object may be outside the FR3's reachable workspace. Check the object spawn coordinates in the launch file and confirm they fall within the arm's kinematic reach.

**Gazebo physics is unstable or running slowly**
Hardware-accelerated rendering is recommended. Ensure your GPU drivers are active and that `GAZEBO_MODEL_PATH` is set correctly if custom meshes are not loading.

**`vision_detector.py` exits immediately or publishes no detections**
Confirm the camera topic name matches what is published by the Gazebo camera plugin. Use `ros2 topic list` after launching the simulation to verify.

---

## Credits

**Adithya Raj**
[github.com/see-yuH](https://github.com/see-yuH)
**Sanjeet Sathiyamoorthy**
[github.com/sanjeetsathiyamoorthy](https://github.com/sanjeetsathiyamoorthy)
