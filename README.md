# Sub-Terrain Challenge ROS2 Workspace

This repository contains the ROS2 workspace used for the Sub-Terrain Challenge simulation.

## System Requirements

- **Operating System:** Ubuntu 24.04  
- **ROS Version:** ROS2 Jazzy  
- **Build System:** colcon  

---

## Setup

This section describes how to set up the environment required to build and run the project.

### 1. Install ROS2 Jazzy

Install ROS2 Jazzy by following the official installation guide:

https://docs.ros.org/en/jazzy/Installation.html

Make sure to install the **Desktop** version.

After installation, source ROS2:

```bash
source /opt/ros/jazzy/setup.bash
```

### 2. Install Build Tools and ROS Development Tools

Install the required build tools and ROS development tools:

```bash
sudo apt update
sudo apt install colcon cmake ros-dev-tools
```

### 3. Install Required ROS Packages

Install additional ROS packages used by this workspace:

```bash
sudo apt install ros-$ROS_DISTRO-depth-image-proc
sudo apt install ros-jazzy-octomap-server ros-jazzy-octomap-msgs ros-jazzy-octomap-rviz-plugins
```

### 4. Build the Workspace

Navigate to the ROS2 workspace:

```bash
cd ros2_ws
```

Build the workspace:

```bash
colcon build
```

After building, source the workspace:

```bash
source install/setup.bash
```

### 5. Setup Simulation Executable

Unpack `simulation.zip` and move the extracted files to `sub-terrain-challange/ros2_ws/install/simulation/lib/simulation`:

Then make the Unity simulation binary executable:

```bash
chmod +x install/simulation/lib/simulation/Simulation.x86_64
```

---

## Running the Simulation

After building the workspace, start the simulation using the provided bash script.

```bash
bash ros2_ws/src/mapping/scripts/start_simulation_with_rviz.sh
```

If you get a permission error, make the script executable once:

```bash
chmod +x ros2_ws/src/mapping/scripts/start_simulation_with_rviz.sh
```

If you prefer to launch manually (after sourcing `ros2_ws/install/setup.bash`), you can still run:

```bash
ros2 launch simulation simulation.launch.py
```
