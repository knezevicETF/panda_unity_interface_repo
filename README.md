
# Franka Panda Teaching and Motion Framework

A ROS 2-based motion control and teaching system for the **Franka Emika Panda** robot using **MoveIt 2**, **pymoveit2**, and **Unity**.

This framework supports:

- Motion planning and execution (joint & Cartesian)  
- Real‑time Unity → ROS teleoperation of the end‑effector  
- Path recording and playback (teaching by demonstration)  
- Bidirectional ROS ↔ Unity communication

## 📦 Features

### 🤖 Robot Simulation & Control

- Based on `panda_gz_moveit2`: Franka Panda robot model, MoveIt 2 config, and Gazebo integration  
- Full motion planning with MoveIt 2 (joint & Cartesian) via Python  
- Gripper control interface

### 🐍 Python Interface

- Uses **pymoveit2**: high‑level Python binding for MoveIt 2 actions/services  
- Easy scripting: joint space, pose goals, Cartesian paths  
- Gripper and servo examples included

### 🎮 Unity Integration 

- Unity shoud publishes EE poses and control commands to ROS  
- ROS publishes robot state back to Unity "/joint_states"
- Soft real‑time teleoperation suitable for teaching and visualization


## 🚀 Getting Started

### 🛠 Dependencies

- ROS 2 **Humble (recommended)**  
- MoveIt 2 for Humble  
- Gazebo Ignition Fortress  
- Python 3.10+ (ROS2 default)  
- `pymoveit2` package in workspace

### 📥 Clone

```bash
cd ~/ros2_ws/src
git clone -b humble_devel https://github.com/AndrejOrsula/panda_gz_moveit2.git
git clone https://github.com/AndrejOrsula/pymoveit2.git
git clone https://github.com/knezevicETF/panda_unity_interface_repo.git)
# Add your Unity‑ROS bridge
```

## 🧠 Build & Source

```bash
cd ~/ros2_ws
rosdep update
rosdep install -y -r -i --from-paths src
colcon build --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
source install/setup.bash
```

## 🌀 Run Simulation & Controllers

### Launch RViz + MoveIt 2

```bash
ros2 launch panda_moveit_config ex_fake_control.launch.py 
```

### Run Python Controller

```bash
ros2 run panda_unity_interface moveit_controller
```

### Run Dummy Mission Builder

```bash
ros2 run panda_unity_interface dummy_mission
```

## 📡 Unity Communication

### Unity → ROS

| Topic | Type | Description |
|-------|------|-------------|
| `/mission_command` | `panda_unity_interface_msgs/Mission` | Full mission path (Waypoint[]) |

### ROS → Unity

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Current robot joints pose |


## 📄 License

This project uses the **BSD‑3‑Clause License**.
