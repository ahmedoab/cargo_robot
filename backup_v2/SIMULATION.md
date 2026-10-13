# Simulation guide (ROS 2 Jazzy + Gazebo Harmonic)

This runs the cargo robot in Gazebo with the **same ROS 2 package, topics, frames and Nav2/SLAM settings** as the real robot. Anything that works in simulation should work on the robot with only `use_sim_time` changed.

## What is simulated

| Real robot | In simulation |
|---|---|
| `base_driver` (motors, encoders, odometry) | Gazebo **DiffDrive** plugin: same `/odom`, same TF `odom → base_footprint`, same `/joint_states` |
| RPLIDAR A1M8 on `/scan` | Gazebo `gpu_lidar`: 360 samples, 0.15–12 m, 7 Hz, on `/scan` |
| USB camera (read directly by `vision_node`) | Gazebo camera on `/camera/image_raw` (640 × 480, 15 Hz) |
| `speed_limiter`, laser filter, SLAM, Nav2 | exactly the same nodes and config files |
| Load cells, E-stop, GPIO | **not simulated**. There's no `/payload_weight`, so `speed_limiter` lets the full speed through. |

## 1. Install (Ubuntu 24.04 + ROS 2 Jazzy)

```bash
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
     ros-jazzy-slam-toolbox ros-jazzy-laser-filters ros-jazzy-robot-state-publisher \
     ros-jazzy-xacro ros-jazzy-teleop-twist-keyboard python3-yaml
```

`ros-jazzy-ros-gz` installs Gazebo Harmonic, the version that goes with ROS 2 Jazzy.

## 2. Build

```bash
git clone <repo url> ~/cargo_robot
cd ~/cargo_robot
python3 tools/make_configs.py          # writes config/nav2_params.yaml + config/slam.yaml from the installed defaults
cd robot_ws
colcon build --symlink-install
source install/setup.bash              # add this line to ~/.bashrc
```

`robot_ws/src/cargo_bot` is a normal ament_python package. You can also copy it into your own workspace.

## 3. Run

Terminal 1, Gazebo with the robot in a test room (8 × 6 m, with a doorway and a few obstacles):

```bash
ros2 launch cargo_bot sim.launch.py            # add headless:=true for no Gazebo window
```

Terminal 2, drive it:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Terminal 3, make a map (drive slowly around the room), then save it:

```bash
ros2 launch cargo_bot slam.launch.py use_sim_time:=true
mkdir -p ~/maps && ros2 run nav2_map_server map_saver_cli -f ~/maps/sim
```

Then navigate on the saved map (stop SLAM first):

```bash
ros2 launch cargo_bot nav.launch.py map:=$HOME/maps/sim.yaml use_sim_time:=true
ros2 launch nav2_bringup rviz_launch.py        # 2D Pose Estimate, then Nav2 Goal
```

The delivery mission also works in simulation: `ros2 run cargo_bot mission --stations <file>`. It waits for cargo on `/payload_weight`, which you can fake with `ros2 topic pub /payload_weight std_msgs/msg/Float32 "{data: 2.0}"`.

## Topics and frames (identical to the real robot)

| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | geometry_msgs/Twist | commands from teleop / Nav2 / vision |
| `/cmd_vel_safe` | geometry_msgs/Twist | after `speed_limiter`; drives the wheels |
| `/odom` | nav_msgs/Odometry | wheel odometry |
| `/scan`, `/scan_filtered` | sensor_msgs/LaserScan | raw lidar / with the robot body removed (Nav2 + SLAM use this) |
| `/joint_states` | sensor_msgs/JointState | wheel angles |
| `/camera/image_raw` | sensor_msgs/Image | simulation only |
| `/payload_weight`, `/speed_factor`, `/estop` | std_msgs | real robot only (`/speed_factor` also in sim) |

TF tree: `map → odom → base_footprint → base_link → {left_wheel_link, right_wheel_link, front_caster_link, rear_caster_link, laser, camera_link}`

## Robot numbers

| | |
|---|---|
| Size | 434 × 360 × 263 mm; Nav2 footprint 0.45 × 0.38 m |
| Drive | differential, wheel radius 0.05 m, wheel separation 0.33 m, max 0.35 m/s in Nav2 |
| Mass | ~5 kg without cargo (estimate in the URDF), max payload 15 kg |
| Lidar | `laser` at (0.153, 0, 0.20) m from `base_link`, rotated 180° (RPLIDAR's 0° points backwards) |
| Camera | `camera_link` at (0.212, 0, 0.048) m, tilted 35° down |
| `base_link` | centre of the drive axle, 0.05 m above the floor (`base_footprint`) |

## Files

| File | What it is |
|---|---|
| `robot_ws/src/cargo_bot/urdf/cargo_bot.urdf.xacro` | robot model (real + sim), with masses and collision shapes. `sim:=true` adds the Gazebo parts. |
| `robot_ws/src/cargo_bot/urdf/cargo_bot.gazebo.xacro` | DiffDrive + JointStatePublisher plugins, lidar and camera sensors, wheel friction |
| `robot_ws/src/cargo_bot/meshes/cargo_bot_body.stl` | the real robot shape, exported from the CAD model |
| `robot_ws/src/cargo_bot/worlds/test_area.sdf` | the test room |
| `robot_ws/src/cargo_bot/config/gz_bridge.yaml` | Gazebo ↔ ROS topic bridge |
| `robot_ws/src/cargo_bot/launch/sim.launch.py` | starts all of the above |
| `cad/autocad/CargoRobot_v2_drawings.pdf` | dimensioned drawings of the robot |
| `cad/autocad/CargoRobot_v2_schematics.pdf` | electrical schematics |
| `cad/output/robot_assembly.step` | full 3D CAD assembly |

## If something doesn't work

- **Robot doesn't move:** check `ros2 topic echo /cmd_vel_safe` while pressing teleop keys. If that works, run `gz topic -e -t /cmd_vel` to see whether the bridge passes it on.
- **No `/scan` or camera image:** Gazebo sensors need a GPU with OpenGL (`ogre2`). In a virtual machine, enable 3D acceleration, or run on a real Ubuntu install.
- **Robot shows as a plain grey box, or "mesh not found":** check that `colcon build` installed `share/cargo_bot/meshes/cargo_bot_body.stl`. `sim.launch.py` adds that folder to `GZ_SIM_RESOURCE_PATH`.
- **TF errors in RViz:** every node must use sim time. Start SLAM/Nav2 with `use_sim_time:=true` and set RViz to use sim time too.
- **Robot drifts or slides:** wheel friction is in `cargo_bot.gazebo.xacro` (`mu1`/`mu2`), and masses are in `cargo_bot.urdf.xacro`.
