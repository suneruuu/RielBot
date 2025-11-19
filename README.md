## RielBot

RielBot is a ROS 2 workspace that contains two packages:

- `rielbot_bringup` – a convenience launch file that brings up visualization and simulation together.
- `rielbot_description` – a complete URDF/Xacro model of a mecanum-style base, plus launch files for RViz2 visualization and Gazebo (via `ros_gz`) simulation.

Use this repository as a starting point for experimenting with mobile robot modeling, visualization, and sim-to-real workflows.

---

### Prerequisites

Install the following on Ubuntu 22.04 LTS:

- ROS 2 Jazzy Jalisco desktop install (follow the [official docs](https://docs.ros.org/en/jazzy/Installation.html))
- colcon and vcstool (`sudo apt install python3-colcon-common-extensions python3-vcstool`)
- Gazebo via `ros_gz` meta-package (`sudo apt install ros-jazzy-ros-gz`)
- Visualization utilities (`sudo apt install ros-jazzy-joint-state-publisher-gui`)

> All instructions below assume `ROS_DISTRO=jazzy`.

---

### Workspace Setup

```bash
# Create a workspace and clone the repo
mkdir -p ~/rielbot_ws
cd ~/rielbot_ws
git clone https://github.com/suneruuu/RielBot.git

# Install package dependencies (optional if using rosdep)
rosdep install --from-paths . --ignore-src --rosdistro jazzy -y
```

This repository already contains `src/rielbot_description` and `src/rielbot_bringup`. No additional overlays are required.

---

### Build & Source

```bash
cd ~/rielbot_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source ~/rielbot_ws/install/setup.bash
```

If you want every new terminal to pick up ROS 2 and this workspace automatically, append both setup files to your `~/.bashrc` (adjust paths if your installs differ):

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/rielbot_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### Launching the Robot

- **RViz2 visualization only**

  ```bash
  ros2 launch rielbot_description display.launch.py
  ```

  This starts `robot_state_publisher`, `joint_state_publisher_gui`, and RViz2 with the pre-configured view located at `src/rielbot_description/rviz/display.rviz`.

- **Gazebo simulation only**

  ```bash
  ros2 launch rielbot_description gazebo.launch.py
  ```

  The launch file sets `GZ_SIM_RESOURCE_PATH`, starts Gazebo (`ros_gz_sim`), spawns the URDF, and bridges the `/clock` topic back into ROS 2 via `ros_gz_bridge`.

- **Full bringup (RViz2 + Gazebo)**

  ```bash
  ros2 launch rielbot_bringup rielbot.launch.py
  ```

  This orchestrates both description launch files so you can visualize the robot in RViz2 while the simulation runs.

---

### Repository Layout

```
src/
├── rielbot_bringup/        # launch orchestration package
└── rielbot_description/    # URDF, meshes, RViz and Gazebo launch files
```

Key assets:

- `urdf/rielbot.urdf.xacro` – includes `rielbot_platform.xacro` (geometry, joints, inertias) and `rielbot_gazebo.xacro` (Gazebo friction tuning for mecanum-like strafing).
- `meshes/` – STL files for the base, wheels, casters, and IMU.
- `launch/display.launch.py` – visualization stack for RViz2.
- `launch/gazebo.launch.py` – Gazebo/`ros_gz` pipeline and ROS bridge.

---

### Customizing the Robot

1. Edit `urdf/rielbot_platform.xacro` to modify geometry, masses, joints, or materials.
2. Adjust Gazebo physics (frictions, joints, plugins) in `urdf/rielbot_gazebo.xacro`.
3. Update RViz visuals or TF displays in `rviz/display.rviz`.
4. Rebuild (`colcon build`) after changes, or run `ros2 launch` directly if only Xacro or RViz configs changed.