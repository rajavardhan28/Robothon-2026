# 🤖 SO101 Pick-and-Place Hackathon Project

This project implements a **ROS2 + MoveIt + Gazebo pipeline** for a robotic arm (SO101) to perform **pick-and-place tasks**.

---

# 📁 Project Structure

```
workspace/
 ├── src/
 │   ├── so101_description/
 │   ├── so101_unified_bringup/
 │   ├── so101_moveit_config/
 │   ├── pick_place_pkg/
 ├── build/
 ├── install/
 ├── log/
```

---

# ⚙️ Requirements

* Ubuntu 22.04 (WSL or native)
* ROS2 Humble
* Gazebo (Classic)
* MoveIt2

---

# 📦 Install Dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  python3-colcon-common-extensions
```

---

# 🛠️ Build the Workspace

```bash
cd ~/workspace
colcon build
```

---

# 🔁 Source Environment

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/install/setup.bash
```

👉 (Do this in every new terminal)

---

# 🚀 Run Simulation

```bash
ros2 launch so101_unified_bringup main.launch.py
```

---

# 🔍 Verify Controllers

```bash
ros2 control list_controllers
```

Expected output:

```
joint_state_broadcaster [active]
joint_trajectory_controller [active]
```

---

# 🎮 Manual Control (RViz)

1. Open RViz (auto-launches or run `rviz2`)
2. Go to **MotionPlanning panel**
3. Move robot using markers
4. Click:

```
Plan → Execute
```

or

```
Plan & Execute
```

---

# 🤖 Run Pick-and-Place Pipeline

Open a new terminal:

```bash
cd ~/workspace
source install/setup.bash

ros2 run pick_place_pkg pick_place_node
```

---

# 🔄 Pipeline Flow

```
Pick → Attach → Move → Place → Detach
```

---

# 🧠 Notes

* Ensure `ros2_control` is correctly configured in URDF
* Controllers must be active before execution
* Gazebo must be running before launching pipeline

---

# 🐞 Troubleshooting

### ❌ Controllers not loading

```bash
ros2 node list
```

Check for:

```
/controller_manager
```

---

### ❌ Robot not moving in RViz

* Click **Execute** (not just Plan)
* Ensure controllers are active

---

### ❌ Gazebo not opening (WSL)

Run:

```bash
gazebo
```

If it fails, install OpenGL support or run headless.

---

# 🏆 Final Goal

✔ Robot moves in simulation
✔ Controllers active
✔ Pick-and-place pipeline works
✔ Ready for hackathon submission

---

# 🚀 Author
Ashwin Anil
Advika Kavya Kumar
Rajavardhan R
Sumant A Gunagi
Udhith Narayan
Physical AI Hackathon 2026

---
