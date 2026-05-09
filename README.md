# AMR Project

ROS 2 workspace for an autonomous mobile robot simulation. The repository includes Gazebo world assets, robot simulation launch files, mapping, exploration, traffic, mission manager, GUI, navigation, landmarks, and description packages.

## Packages

- `amr_gazebo`: Gazebo / Ignition world, models, and launch files.
- `amr_mapping`: SLAM Toolbox launch and odometry-to-TF helper.
- `amr_exploration`: Exploration node and launch file.
- `amr_description`: Robot description package.
- `amr_navigation`: Navigation package.
- `amr_mission_manager`: Mission manager package.
- `amr_traffic`: Traffic/control helper package.
- `amr_landmarks`: Landmark-related package.
- `amr_gui`: GUI package.

## Requirements

Install ROS 2 and the packages needed by this workspace. The project uses `ros2`, `colcon`, Ignition Gazebo, `ros_gz_bridge`, and `slam_toolbox`.

Example for a ROS 2 environment:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep
```

Install package dependencies from the workspace root:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Clone And Build

Clone the repository:

```bash
git clone https://github.com/Suhaib-AbuRaidah/AMR_Project.git
cd AMR_Project
```

Source ROS 2, then build:

```bash
source /opt/ros/<ros-distro>/setup.bash
colcon build
```

Replace `<ros-distro>` with your installed ROS 2 distribution, for example `humble` or `jazzy`.

After every new terminal, source the workspace:

```bash
source install/setup.bash
```

## Run The Simulation

Terminal 1: launch the town world.

```bash
source /opt/ros/<ros-distro>/setup.bash
source install/setup.bash
ros2 launch amr_gazebo town.launch.py
```

Terminal 2: start the ROS-Gazebo bridge.

```bash
source /opt/ros/<ros-distro>/setup.bash
source install/setup.bash
ros2 launch amr_gazebo bridge.launch.py
```

Terminal 3: launch SLAM mapping.

```bash
source /opt/ros/<ros-distro>/setup.bash
source install/setup.bash
ros2 launch amr_mapping slam_mapping.launch.py
```

Terminal 4: launch exploration.

```bash
source /opt/ros/<ros-distro>/setup.bash
source install/setup.bash
ros2 launch amr_exploration exploration.launch.py
```

## Useful Commands

List packages in the workspace:

```bash
ros2 pkg list | grep amr
```

Rebuild one package:

```bash
colcon build --packages-select amr_gazebo
```

Run a package launch file:

```bash
ros2 launch <package_name> <launch_file.py>
```

Clean generated build files:

```bash
rm -rf build install log
```

Then rebuild with:

```bash
colcon build
source install/setup.bash
```
