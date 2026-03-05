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

### 3. Build the Workspace

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

### 4. Setup Simulation Executable

Unpack `simulation.zip` and move the extracted files to `sub-terrain-challange/ros2_ws/install/simulation/lib/simulation`:

Then make the Unity simulation binary executable:

```bash
chmod +x install/simulation/lib/simulation/Simulation.x86_64
```

---

## Running the Simulation

After building and sourcing the workspace, run the simulation using the provided launch files.

```bash
ros2 launch simulation simulation.launch.py
```
