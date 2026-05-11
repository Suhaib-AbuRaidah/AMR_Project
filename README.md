# AMR Project

This repository is a ROS 2 workspace for an autonomous mobile robot simulation. It includes a Gazebo town world, the robot model, mapping with SLAM Toolbox, Nav2 navigation, traffic robots, mission execution, and a simple GUI for running service missions between landmarks.

## Requirements

Install ROS 2 first, then install the common build and dependency tools:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep
```

The project also uses Gazebo/Ignition, `ros_gz_bridge`, Nav2, SLAM Toolbox, and standard ROS 2 message packages. From the workspace root, install package dependencies with:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Clone And Build

```bash
git clone https://github.com/Suhaib-AbuRaidah/AMR_Project.git
cd AMR_Project
source /opt/ros/<ros-distro>/setup.bash
colcon build
source install/setup.bash
```

Replace `<ros-distro>` with your ROS 2 distribution, for example `humble` or `jazzy`.

Every new terminal should source ROS 2 and the workspace:

```bash
source /opt/ros/<ros-distro>/setup.bash
source install/setup.bash
```

## Mapping

Use mapping mode when you want to launch the simulation, bridge, SLAM, exploration, and RViz together:

```bash
ros2 launch amr_mapping mapping_full.launch.py
```

After the map is complete, save it from another sourced terminal:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_town_map
```

To use the new map for navigation, place the generated `.yaml` and `.pgm` files in:

```text
src/amr_navigation/maps/
```

## Navigation

Use navigation mode when you want to run the full robot system with Gazebo, bridge, Nav2, mission executor, GUI, RViz, and traffic robots:

```bash
ros2 launch amr_navigation full_system.launch.py use_rviz:=true use_traffic:=true
```

If you want to run without traffic robots:

```bash
ros2 launch amr_navigation full_system.launch.py use_rviz:=true use_traffic:=false
```

## Useful Notes

- Rebuild after changing launch files, package files, or installed resources:

```bash
colcon build
source install/setup.bash
```

- The main Gazebo world is:

```text
src/amr_gazebo/worlds/town.world
```

- The navigation map config is:

```text
src/amr_navigation/maps/my_town_map1.yaml
```
