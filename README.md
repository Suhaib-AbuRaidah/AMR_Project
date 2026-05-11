# AMR Project

ROS 2 workspace for an autonomous mobile robot simulation. The repository includes Gazebo world assets, robot simulation launch files, mapping, exploration, traffic, mission manager, GUI, navigation, landmarks, and description packages.

## Packages

- `amr_gazebo`: Gazebo / Ignition world, models, and launch files.
- `amr_mapping`: SLAM Toolbox launch and odometry-to-TF helper.
- `amr_exploration`: Exploration node and launch file.
- `amr_description`: Robot description package.
- `amr_navigation`: Navigation package.
- `amr_mission_manager`: Mission manager package.
- `amr_mission_executor`: Mission executor package.
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

## Run The System

Open a new terminal and source ROS 2 plus the workspace before running any command:

```bash
source /opt/ros/<ros-distro>/setup.bash
source install/setup.bash
```

### Full Navigation System

This is the main command for running the service-robot stack. It launches Gazebo, the ROS-Gazebo bridge, Nav2, the mission executor, and the GUI:

```bash
ros2 launch amr_navigation full_system.launch.py
```

Useful options:

```bash
ros2 launch amr_navigation full_system.launch.py use_rviz:=true
ros2 launch amr_navigation full_system.launch.py use_traffic:=true
ros2 launch amr_navigation full_system.launch.py auto_init_pose:=false
```

You can combine options:

```bash
ros2 launch amr_navigation full_system.launch.py use_rviz:=true use_traffic:=true
```

### Full Mapping System

This launches Gazebo, the bridge, SLAM Toolbox, exploration, and RViz:

```bash
ros2 launch amr_mapping mapping_full.launch.py
```

Choose an exploration mode:

```bash
ros2 launch amr_mapping mapping_full.launch.py exploration_mode:=straight
ros2 launch amr_mapping mapping_full.launch.py exploration_mode:=free_space
ros2 launch amr_mapping mapping_full.launch.py exploration_mode:=random
ros2 launch amr_mapping mapping_full.launch.py exploration_mode:=random_small_rotation
ros2 launch amr_mapping mapping_full.launch.py exploration_mode:=landmark_search
```

Disable RViz if you only want the mapping/exploration nodes:

```bash
ros2 launch amr_mapping mapping_full.launch.py use_rviz:=false
```

Save the generated map from another sourced terminal:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_town_map
```

This creates `~/my_town_map.yaml` and `~/my_town_map.pgm`. Copy them into `src/amr_navigation/maps/` if you want Nav2 to use the new map.

### Manual Mapping Bringup

Use this flow when debugging one part of the mapping stack at a time.

Terminal 1: launch the town world.

```bash
ros2 launch amr_gazebo town.launch.py
```

Terminal 2: start the ROS-Gazebo bridge.

```bash
ros2 launch amr_gazebo bridge.launch.py
```

Terminal 3: launch SLAM mapping.

```bash
ros2 launch amr_mapping slam_mapping.launch.py
```

Terminal 4: launch exploration.

```bash
ros2 launch amr_exploration exploration.launch.py
```

Exploration modes can also be selected manually:

```bash
ros2 launch amr_exploration exploration.launch.py exploration_mode:=straight
ros2 launch amr_exploration exploration.launch.py exploration_mode:=free_space
ros2 launch amr_exploration exploration.launch.py exploration_mode:=random
ros2 launch amr_exploration exploration.launch.py exploration_mode:=random_small_rotation
ros2 launch amr_exploration exploration.launch.py exploration_mode:=landmark_search
```

### Navigation Only

If Gazebo and the bridge are already running, launch Nav2 only:

```bash
ros2 launch amr_navigation navigation.launch.py
```

### Mission Executor And GUI Only

If Nav2 is already running, launch the mission executor:

```bash
ros2 launch amr_mission_executor mission_executor.launch.py
```

Launch the GUI:

```bash
ros2 launch amr_gui gui.launch.py
```

For standalone GUI testing with a mock mission server:

```bash
ros2 launch amr_gui gui.launch.py use_mock_server:=true
```

### Traffic Robots

Spawn two traffic robots into the existing Gazebo world:

```bash
ros2 launch amr_traffic two_traffic_robots.launch.py world_name:=default
```

Or let the full navigation system spawn them:

```bash
ros2 launch amr_navigation full_system.launch.py use_traffic:=true
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
