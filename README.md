# Autonomous Path Planning

ROS 2 Humble rover simulation, localization, navigation, and hardware sensor interfaces.
The active simulation uses Gazebo Classic 11, wheel odometry and a simulated ZED IMU,
upstream `robot_localization` local/global EKFs, GPS conversion, and Nav2 depth costmaps.
The simulated camera supplies depth/point clouds; visual odometry is not implemented
by this simulation.

## Run on a Mac through Docker

Start Docker Desktop. Native ROS/Gazebo installation is unnecessary for this workflow.
The image uses Linux AMD64 emulation on Apple Silicon to run the existing Classic plugins.
The first build downloads ROS, Gazebo and Nav2 dependencies and can take several minutes.

```bash
bash scripts/run_simulation_container.sh --build bash -lc \
  'source /opt/ros/humble/setup.bash; python3 /source/docker/gazebo_environment_smoke.py'
```

The repository is mounted read-only at `/source`. Build outputs and ROS logs live
inside the temporary container. To build and launch the complete simulation:

```bash
bash scripts/run_simulation_container.sh bash -lc '
  source /opt/ros/humble/setup.bash &&
  colcon build --base-paths /source/src/rover_description /source/src/rover_localization /source/src/rover_navigation /source/src/rover_bringup --cmake-args -DBUILD_TESTING=OFF &&
  source install/setup.bash &&
  xvfb-run -a ros2 launch rover_description simulation.launch.py
'
```

`xvfb-run` provides a virtual display for depth-camera rendering. This command runs
headlessly; it does not open a Gazebo window on macOS. The container is removed on
exit, so copy logs out before stopping a debugging container if they are needed.

`simulation.launch.py` starts Gazebo, the rover, controllers, localization, and Nav2.
Use `enable_nav2:=false` for isolated motion/localization testing. The default
`flat.world` has a ground plane and geographic origin for repeatable baseline tests.
To select park terrain, pass `world:=<installed-package-path>/world/park.world`.
On a Linux desktop with a display, `gui:=true` also starts the Gazebo client.
Nav2 starts after bounded localization readiness checks; `startup_timeout:=120.0`
controls that deadline in wall seconds.

The default `simp_rover.urdf` is a rigid suspension simulation fixture. Its
formerly unconstrained rocker joints caused invalid physical/sensor states on
ground contact; the fixture fixes those joints. Wheel friction is anisotropic
(lower lateral friction for skid steering), expressed along the collision-frame
axle. These friction coefficients and suspension constraints are simulation
assumptions, not measured rover dynamics. `rover.urdf` retains the original CAD
model for inspection; it is not the tested Gazebo entry point.

The simplified model's original wheel axes have been restored by request. A
positive velocity command moves the model backward in the `base_link` frame.
This differs from the controller/odometry convention and can affect localization
and Nav2. Previous passing navigation results predate the axis restoration.

## Active packages

| Package | Role |
| --- | --- |
| `rover_description` | URDF, Gazebo worlds/sensors/controllers, simulation launches, Nav2 parameters, measured-mount launcher |
| `rover_localization` | Simulation dual EKFs and hardware-specific sensor adapters/configuration using upstream `robot_localization` |
| `rover_navigation` | GPS-to-map navigation client, command relay, localization readiness check |
| `rover_bringup` | Hardware driver selection and optional hardware localization |
| `l76x_gps_interface` | L76X GPS interface |
| `icm20948_imu_interface` | ICM-20948 IMU interface |

`src/archives/COLCON_IGNORE` excludes historical estimators, drivers, and tests.
Do not remove it to make a stale command work. The upstream Humble
`robot_localization` binary supplies EKF/navsat executables; rover-specific
launches and adapters belong in `rover_localization`.

## Ubuntu build

For native execution use Ubuntu 22.04 with ROS 2 Humble, Colcon and rosdep.
Install dependencies using package manifests, then build and source the workspace:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch rover_description simulation.launch.py
```

The ZED hardware driver additionally requires the initialized
`src/zed-ros2-wrapper` submodule, compatible ZED SDK/CUDA, and a supported camera.
Those hardware dependencies are not required for the Docker simulation.

## Hardware bringup

After installing the drivers, building and sourcing the hardware workspace:

```bash
ros2 launch rover_bringup simple.launch.py \
  zed:=yes icm:=no wheel:=yes gps:=no localization:=yes drivers:=yes
```

`wheel` selects an existing encoder odometry topic; it does not launch a motor
controller. `drivers:=no` uses sensors already running elsewhere. `zed` selects
ZED optical odometry and gyro, `icm` selects the external gyro, and `gps` enables
GPS/global estimation when localization is selected. The hardware profile is
separate from the simulation's fixed sensor configuration.

Hardware localization can also launch directly:

```bash
ros2 launch rover_localization hardware_localization.launch.py
```

GPS/global estimation requires measured sensor transforms and an earth-referenced
heading. With a measured startup compass bearing use `heading_mode:=startup
initial_heading_deg:=<bearing>` (north=0, east=90). The alternative
`heading_mode:=imu heading_topic:=<topic>` requires a verified ENU heading source.
A URDF or `rover_description sensor_mounts.launch.py mounts_file:=<JSON>` must
publish the measured mounts. See `config/sensor_mounts.example.json`; its entries
are deliberately unverified. `publish_approximate_mounts:=true` is an explicit
bench fixture, not hardware calibration, and must not run alongside URDF mount TF.

## Interfaces and tests

See [the integration plan](docs/SIMULATION_INTEGRATION_PLAN.md) for the topic/TF
contract and staged checks, and [verification results](docs/SIMULATION_VERIFICATION.md)
for commands actually run, measured results and remaining limitations.

Run `bash scripts/test_localization_container.sh` for active localization tests.
Run the complete clean build, Colcon tests, and real Gazebo motion/navigation
checks with:

```bash
bash scripts/test_simulation_container.sh
```

The simulation probe in `scripts/probe_simulation.py` checks a running stack;
`--motion` requires Nav2 disabled and `--navigation` tests the active Nav2 stack.
A probe must run in the same container/network and ROS domain as its stack.
Use different ROS domains for concurrent test stacks to prevent data interference.

The local EKF alone owns `odom -> base_link`; the global EKF owns `map -> odom`.
The URDF owns sensor mounts. `/simulation/ground_truth` is only a test reference.
Simulation results do not establish physical rover accuracy or measured hardware
mounts. Park terrain, desktop GUI rendering and real sensors require their own
runtime verification.
