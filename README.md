# theodwyn-ros
Collection of ROS2 Middleware Packages for the *Theodwyn* Robots, Eomer and Eowyn unmanned ground vehicles (UGVs) 

---
## Content

- [theodwyn-ros](#theodwyn-ros)
  - [Content](#Content)
  - [Dependencies](#Dependencies)
  - [Build](#Build)
  - [Configuration](#Configuration)
  - [Execution](#Execution)


---

## Dependencies
###
This repository contains a collection of ROS packages, evidently dependent on the [ROS2 Middleware libraries](https://www.ros.org/). The associated installation instructions can be found on their website. 

In addition, the ROS development tools will be necessary to build this package from the provided source files. The tools may be installed via `apt`, after completing the aforementioned installation
```bash
sudo apt install ros-dev-tools
```
### Eigen3
To manage various aspects of linear algerbra within this repository, the [Eigen library](https://eigen.tuxfamily.org) is utilized. The associated installation instructions can be found on their website.
### Boost C++
To manage various aspects of hardware communications and I/O, the [Boost libraries](https://www.boost.org/doc/user-guide/intro.html) are utilized. The libraries may be nominally installed via `apt` in the following
```bash
sudo apt-get install libboost-all-dev
```
### ros2-vicon-receiver
To communicate with the local vicon system, the ros2-vicon-receiver package is used to publish localization data to the ROS network
```bash
git submodule update --init --recursive
```
> [!NOTE]
> This is a fork of the original repository, making necessary changes for consistent operations. Located at [PeteLealiieeJ/ros2-vicon-receiver](https://github.com/PeteLealiieeJ/ros2-vicon-receiver)

## Build
The Pacakge and CMake files provided allow this repository to be built simply with `colcon` in the following.
### Install ROS packages' dependencies
```bash
rosdep install --from-paths src --rosdistro $ROS_DISTRO -y --ignore-src
```
### Build ROS packages
In the top-level directory of this repository
```bash
colcon build --packages-select theo_msgs theo_srvs theo_comm theo_core theo_teleop theo_autoop
```
> [!WARNING]
> An additional package, `theo_recorder` is provided in this repository; However, the services it depends upon requires a ROS2 version higher than *Jazzy*. All other packages provided in this repository were originally developed and tested using ROS2 *Humble*. To build the aforementioned package, include `theo_recorder` in the packages selected above.

```bash
colcon build --packages-select vicon_receiver --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```
These commands will spawn `build/`, `install/`, `log/` folders in the top-level directory of this repository. To use the built packages, **ensure the libraries have been sourced** in the shell session, whether that process is automated via the `~/.bashrc` or manually performed in every new shell session with the following command.
```bash
source install/setup.bash
```

---

## Configuration
Each of the provided packages has respective configuration files, which may be utilized to change the behavior of the respective nodes. The parameters of the nodes, categorized to each of the provided pacakages, is listed below.
> [!NOTE]
> It is important to note that, by default, launch files will deploy nodes in the /eowyn/ namespace (/vicon/eowyn/ for vicon receiver).

### theo_core

#### Parameters for `mixer_node`

| Parameter            | Type     | Default           | Description |
|---|---:|---|---|
| `r_wheel`     | float | `1`     | radius of wheel in meters |
| `lx`          | float | `1`     | x-horizontal distance between wheel in meters |
| `ly`          | float | `1`     | y-horizontal distance between wheel in meters |
| `omega_ref`   | float | `1`     | reference wheel speed for max throttle (calibrated) |
| `flipdir_0`   | bool  | `false` | flip direction of wheel 0 in mixer |
| `flipdir_1`   | bool  | `false` | flip direction of wheel 1 in mixer |
| `flipdir_2`   | bool  | `false` | flip direction of wheel 2 in mixer |
| `flipdir_3`   | bool  | `false` | flip direction of wheel 3 in mixer |

#### Parameters for `servo_handler_node`

| Parameter            | Type     | Default           | Description |
|---|---:|---|---|
| `address`             | int     | `0x40`        | PCA9685 address on I2C bus associated with the `board.SCL_1` and `board.SDA_1` pins (see warning below)  |
| `channels`            | int     | `16`          | PCA9685 channels |
| `pan_init_cmd`        | float   | `0`           | initial command for pan servo sent during spin-up and spin-down |
| `tilt_init_cmd`       | float   | `0`           | initial command for tilt servo sent during spin-up and spin-down |
| `pan_actuation_range` | list[2] | `270`         | actuation range of pan servo        |
| `tilt_actuation_range`| list[2] | `180`         | actuation range of tilt servo       |
| `pan_min_max_pwr`     | list[2] | `[500,2500]`  | min and max PWM power of pan servo  |
| `tilt_min_max_pwr`    | list[2] | `[500,2500]`  | min and max PWM power of tilt servo |
| `pan_safety_bounds`   | list[2] | `[0,270]`     | safe actuation range of pan servo   |
| `tilt_safety_bounds`  | list[2] | `[0,180]`     | safe actuation range of tilt servo  |

> [!WARNING]
> For the `address` parameter, notice the provided package specifies the address on the I2C bus associated with the `board.SCL_1` and `board.SDA_1` pins. If a user wants to change the associated pins for the I2C bus, they must navigate to `theo_core/scripts/servo_handler_node.py` and change the pins used to construct the `servo_driver` member within the `ServoHandlerNode`'s constructor. The authors are open and would be thankful for contributions that allow a user to change this requirement within the configuration file associated with the node.

#### Parameters for `servo_responder_node`

| Parameter            | Type     | Default           | Description |
|---|---:|---|---|
| `update_timeout` | float | `0.1` | time (in seconds) to carry over previous servo velocity commands before dropping to zero velocities  |

#### Parameters for `sabertooth_node`

| Parameter            | Type     | Default           | Description |
|---|---:|---|---|
| `portname_0` | string  | `/dev/ttyTHS1` | portname for first saberooth module    |
| `portname_1` | string  | `/dev/ttyTHS2` | portname for second saberooth module    |
| `baudrate`   | int32   | `9600`           | set baudrate of both sabertooths        |


### theo_teleop

#### Parameters for `joymapping_node`

| Parameter            | Type     | Default           | Description |
|---|---:|---|---|
| `chassis_max_linspeed`    | float | `1`   | maximum linear speed in meters per second   |
| `chassis_max_angspeed`    | float | `1`   | maximum linear speed in radian per second   |
| `pantilt_maxspeed`        | float | `1`   | maximum servo speed in radian per second    |
| `settings_update_waitime` | float | `0.5` | waittime for updating settings              |


### theo_autoop

#### Parameters for `pid_controller_node`

| Parameter            | Type     | Default           | Description |
|---|---:|---|---|
| `p_linear_gain`                             | float   | `0`        | proporational gain for translational controller             |
| `i_linear_gain`                             | float   | `0`        | integral gain for translational controller                  |
| `d_linear_gain`                             | float   | `0`        | derivative gain for translational controller                |
| `p_angular_gain`                            | float   | `0`        | proporational gain for angular controller                   |
| `i_angular_gain`                            | float   | `0`        | integral gain for angular controller                        |
| `d_angular_gain`                            | float   | `0`        | derivative gain for angular controller                      |
| `p_servo_gain`                              | float   | `0`        | proporational gain for servo controller                     |
| `i_servo_gain`                              | float   | `0`        | integral gain for servo controller                          |
| `d_servo_gain`                              | float   | `0`        | derivative gain for servo controller                        |
| `chassis_linear_error_integrator_threshold` | float   | `1`        | saturation threshold for integrator of translational chassis errors for the associated PID controller in meter-seconds |
| `chassis_angular_error_integrator_threshold`| float   | `1`        | saturation threshold for integrator of angular chassis errors for the associated PID controller in radian-seconds |
| `servo_error_integrator_threshold`          | float   | `1`        | saturation threshold for integrator of servo angular errors for the associated PID controller in radian-seconds        |
| `valid_error_update_dt`                     | float   | `0.1`      | maximum time between error measurements to conduct numerical approximations for the error signal derivatives and integrations in seconds |
| `proximity_met_range_meters`                | float   | `0.05`     | proximity range error to confirm configuration in meters    |
| `proximity_met_direction_degrees`           | float   | `5`        | proximity direction error to confirm configuration in degrees (representative for both chassis and pan-tilt errors, respectively) |
| `rpy_degrees_C0_C`                          | list[3] | `[0,0,0]`  | roll-pitch-yaw angles, in degrees, describing the *active* rotation from the *chassis* frame to the *servo* frame  |
### theo_comm

#### Parameters for `trajectory_transmitter_node`

| Parameter            | Type     | Default           | Description |
|---|---:|---|---|
| `filename`                        | string | `""`    | File path to input CSV file   |
| `delay_time`                      | float  | `0`     | Delay time to start transmitting trajectory after request/configuratoin confirmation   |

### theo_recorder
> [!WARNING]
> The `recorder.launch.py` file, provided in the `theo_recorder` package, will automatically spin up a `rosbag2` instance, recording on all available topics and outputting to the `/mnt/usb` directory. There is no builtin functionality to change this configuration; However, the `recorder_node.launch.py` file is provided to spin up just the recorder, such that a `rosbag2` instance may be spun up by the user aside the provided functionality.


---

## Execution
### Transmitting CSV Trajectory from External Machine to a *Theodwyn* Robot
A CSV of trajectory waypoints may be broadcasted from an external machine to the *theodwyn* robotic system. However, it is required to follow a specific column-data organization for the broadcast functionality included in this repository. To broadcast a trajectory for the *Theodwyn* robot chassis (soley), the CSV data must be organized as the following, 

| Column | Type  | Desciption  |
|---|---:|---|
|  `0`  |  float  | time of the waypoint described by the following column fields in seconds                                            |
|  `1`  |  float  | x position, reference to the *world* frame origin, represented in the *world* frame in meters                       |
|  `2`  |  float  | y position, reference to the *world* frame origin, represented in the *world* frame in meters                       |
|  `3`  |  float  | z position, reference to the *world* frame origin, represented in the *world* frame in meters                       |
|  `4`  |  float  | w scalar part of the quarternion describing the *active* rotation from the *world* to *chassis* frame               |
|  `5`  |  float  | x in vector part of the quarternion describing the *active* rotation from the *world* frame to *chassis* frame      |
|  `6`  |  float  | y in vector part of the quarternion describing the *active* rotation from the *world* frame to *chassis* frame      |
|  `7`  |  float  | z in vector part of the quarternion describing the *active* rotation from the *world* frame to *chassis* frame      |
|  `8`  |  float  | x velocity, as interpreted by a *world* observer, represented in the *world* frame in meters per seconds            |
|  `9`  |  float  | y velocity, as interpreted by a *world* observer, represented in the *world* frame in meters per seconds            |
| `10`  |  float  | z velocity, as interpreted by a *world* observer, represented in the *world* frame in meters per seconds            |
| `11`  |  float  | x angular velocity, as interpreted by a *world* observer, represented in the *chassis* frame in radians per seconds |
| `12`  |  float  | y angular velocity, as interpreted by a *world* observer, represented in the *chassis* frame in radians per seconds |
| `13`  |  float  | z angular velocity, as interpreted by a *world* observer, represented in the *chassis* frame in radians per seconds |

> [!NOTE]
> Although aspects of each of the 6-DOF of motion must be described within the CSV, the *Theodwyn* robot chassis are naturally restricted to locally planar motion. Thus, at the moment, in many cases, the controller will ignore the data provided in some of the broadcasted fields. Although this is the case, we maintain their requrement to enable potential expansions to the *Theodwyn* robotic systems that may be introduced at a later date.


To broadcast a trajectory for the pan-tilt system, the following fields must be included, in addition to the previous 14 fields for the chassis waypoints, and will be instantiated with respect to the time in column `0`,

| Column  | Type  | Desciption  |
|---|---:|---|
| `14` |  float  | pan angle, reference to the *servo* frame, in radians                                |
| `15` |  float  | tilt angle, reference to the *servo* frame, in radians                               |
| `16` |  float  | pan angular velocity, as interpreted by a *chassis* observer, in radians per second  |
| `17` |  float  | tilt angular velocity, as interpreted by a *chassis* observer, in radians per second |

> [!NOTE]
> Users are permitted to include or exclude a header within the transmitted CSV at their leisure. This is because the default exception behavior is to skip CSV lines with unexpected/erroronous data.

#### (1a) Starting Autonomous and Tele-Operation Nodes Onboard *Theodwyn* Robot
The robot must, at some point prior to operations, spin up its ROS nodes onboard. The following in the top-level directory, regardless of when and how it is executed, will do so
```bash
source install/setup.bash                       # ONLY needs to be run once per shell session
```
```bash
ros2 launch theo_core autoop_drive_servo.launch       # configurations may need to be changed in case of unexpected behavior
```

> [!WARNING]
> The user profile on the *Theodwyn* robot is required to be a member of the following user groups for onboard communications during robot operations: `dialout`, `gpio`, `i2c`, `tty`

#### (1b) Starting Trajectory Transmission from External Computer
The external machine/computer will spin up the transmission and vicon receiver nodes, external to the *Theodwyn* Robot. After sourcing the `install/setup.bash`, say, in the current directory, the user has a csv file named `csv_out.csv`. The following can be used to transmit the trajectory, assuming the *Theodwyn* Robot and external machine are on the same local network.
```bash
CSV_FILE=$(realpath "csv_out.csv")
```
```bash
ros2 launch theo_comm transmit_trajectory.launch.py transmit_file_path:=$CSV_FILE
```