# ANVRIT

ANVRIT is a ROS 2 (Humble) stack for an outdoor, GPS-guided differential-drive robot. It covers the robot's URDF/mesh description, Gazebo simulation worlds, wheel-odometry generation from encoder ticks, an Arduino motor/encoder firmware, and a GPS waypoint-following navigation stack built on Nav2 + `robot_localization`.

## Packages

| Package | Type | Purpose |
|---|---|---|
| [anvrit_description](anvrit_description) | `ament_cmake` | Robot URDF/xacro, STL meshes, Gazebo world files, and launch files to spawn the robot in Gazebo with `robot_state_publisher`. |
| [anvrit_gazebo](anvrit_gazebo) | `ament_cmake` | Simulation environments: a Sonoma Raceway outdoor world (GPS testing) and an AWS RoboMaker warehouse world, with launch files to bring up Gazebo. |
| [anvrit_navigation](anvrit_navigation) | `ament_python` | GPS waypoint navigation: dual EKF sensor fusion (`robot_localization`), Nav2 GPS waypoint follower, a Tkinter GUI to log GPS waypoints, an interactive `mapviz`-click commander, and a YAML-waypoint-file commander. |
| [anvrit_odometry](anvrit_odometry) | `ament_python` | Converts wheel encoder ticks (`/wheel_ticks`) into wheel RPM and `nav_msgs/Odometry`, and republishes raw ticks read from the Arduino over serial. |
| [odometry_generator](odometry_generator) | `ament_python` | Standalone serial encoder reader with a 1D Kalman filter for smoothing a single encoder's position estimate. |
| [arduino.ino](arduino.ino) | Arduino firmware | Reads two quadrature encoders via pin-change interrupts, drives two DC motors over an H-bridge, and exchanges tick counts / motion commands with the ROS 2 side over USB serial. |

## Architecture

```
Arduino (arduino.ino)
  - quadrature encoder ISRs -> tick counts (Serial, 9600 baud)
  - serial commands 'F','B','L','R','S' + speed digit -> motor PWM
        │  USB serial ("<left> <right>\n" every 100 ms)
        ▼
anvrit_odometry/ticks_pub.py  ──publishes──▶  /wheel_ticks (Int32MultiArray)
        │
        ▼
anvrit_odometry/odometry_gen.py
  - differential-drive odometry + RPM estimate
  - publishes /odometry (nav_msgs/Odometry), /rpm
        │
        ▼
anvrit_navigation: dual_ekf_navstat.launch.py (robot_localization)
  - fuses wheel odometry + IMU + GPS (navsat_transform)
  - publishes odometry/local, odometry/global
        │
        ▼
Nav2 GPS waypoint follower (gps_waypoint_follower.launch.py)
  - consumes waypoints from gps_waypoint_logger.py (GUI, → YAML)
    or logged_waypoint_follower.py (YAML file)
    or interactive_waypoint_follower.py (mapviz clicked point)
  - drives the robot to each GPS waypoint
```

`odometry_generator` is an alternate/experimental encoder pipeline: it reads a single encoder value directly over serial and smooths it with a Kalman filter, independent of the `anvrit_odometry` tick pipeline above.

## Prerequisites

- Ubuntu 22.04 with **ROS 2 Humble**
- Gazebo (classic) + `gazebo_ros_pkgs`
- Nav2, including `nav2_simple_commander` and the GPS waypoint follower demo (`nav2_gps_waypoint_follower_demo`)
- `robot_localization`
- `mapviz` and `swri_transform_util` (for the interactive GPS UI)
- Python: `pyserial`, `numpy`, `pyyaml` (`pip install pyserial numpy pyyaml`)
- Arduino IDE (or `arduino-cli`) to flash [arduino.ino](arduino.ino)

## Building

Clone this repository into the `src` folder of a ROS 2 workspace, then build with `colcon`:

```bash
mkdir -p ~/anvrit_ws/src
cd ~/anvrit_ws/src
git clone https://github.com/HrushikeshAnandSarangi/anvrit_repo24.git .
cd ~/anvrit_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Running the simulation

Launch the robot in the Sonoma Raceway Gazebo world:

```bash
ros2 launch anvrit_description sonoma_world.launch.py
```

Or bring up the full GPS navigation stack (Gazebo + dual EKF + Nav2 + RViz + mapviz):

```bash
ros2 launch anvrit_navigation gps_waypoint_follower.launch.py
```

Follow a set of waypoints stored in a YAML file:

```bash
ros2 run anvrit_navigation logged_waypoint_follower /path/to/waypoints.yaml
```

Log new GPS waypoints interactively from a running robot:

```bash
ros2 run anvrit_navigation gps_waypoint_logger /path/to/waypoints.yaml
```

## Running on real hardware

1. Flash [arduino.ino](arduino.ino) to the motor controller board (2 DC motors on an H-bridge, 2 quadrature encoders on pins 2/4 and 3/5).
2. Connect the board over USB (default `/dev/ttyUSB0`, 9600 baud).
3. Start the tick reader and odometry generator:

```bash
ros2 run anvrit_odometry ticks_publisher
ros2 run anvrit_odometry odometry_gen
```

Waypoint YAML files use this schema (as written by `gps_waypoint_logger`):

```yaml
waypoints:
  - latitude: 38.161491
    longitude: -122.454503
    yaw: 0.0
```

## Notes

- Package metadata (`package.xml` descriptions/licenses) is still using ROS 2's generated `TODO` placeholders and should be filled in per package.
- No license file is currently present in this repository.
