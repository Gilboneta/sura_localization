# SURA Localization

This package launches the AUV localization stack around `robot_localization`.

`auv_localization.launch.py` is the generic launch file for any robot. It only
starts `robot_localization` nodes plus optional SURA frame-conversion helpers.
Robot-specific sources such as simulators, pressure adapters, ArUco maps, or
custom sensor bridges should live in a robot-specific launch file.

`cirtesu_auv_localization.launch.py` is the Cirtesu/Stonefish-specific launch.

The important rule is:

```text
robot_localization runs in ENU
the public localization output is converted back to NED
```

This keeps `robot_localization` standard and avoids maintaining a custom NED fork.

## Frame Conventions

```text
world_enu
  x: East
  y: North
  z: Up

world_ned
  x: North
  y: East
  z: Down
```

The EKF state is estimated in `world_enu`. The package then publishes a converted NED odometry for the rest of the robot stack.

## Data Flow

```text
RAW SIM/SENSOR TOPICS                         ENU INTERNAL TOPICS                       PUBLIC OUTPUT

/cirtesub/sensors/imu
  sensor_msgs/Imu
  NED-ish Stonefish frame
        |
        v
  ned_to_enu_imu
        |
        v
/cirtesub/sensors/imu_enu  -------------------------+
  sensor_msgs/Imu                                   |
  frame: cirtesub/IMU                               |
                                                     |
/cirtesub/sensors/gps                               |
  sensor_msgs/NavSatFix                             |
        |                                            |
        v                                            |
  navsat_transform_node <---- /cirtesub/localization/odometry_enu
        |                                            ^
        v                                            |
/cirtesub/sensors/gps/odometry ---------------------+
  nav_msgs/Odometry
  frame: world_enu
                                                     |
/cirtesub/sensors/pressure                          |
  sensor_msgs/FluidPressure                         |
        |                                            |
        v                                            |
  pressure_to_pose                                  |
        |                                            |
        v                                            |
/cirtesub/sensors/pressure/pose --------------------+
  PoseWithCovarianceStamped                         |
  frame: world_enu                                  |
                                                     v
/cirtesub/sensors/dvl/twist ------------------> ekf_node
  TwistWithCovarianceStamped                   robot_localization
                                                frame: world_enu
                                                     |
                                                     v
                                      /cirtesub/localization/odometry_enu
                                        nav_msgs/Odometry
                                        frame: world_enu
                                                     |
                                                     v
                                           enu_to_ned_odometry
                                                     |
                                                     v
                                      /cirtesub/localization/odometry
                                        nav_msgs/Odometry
                                        frame: world_ned
```

## Launched Nodes

`cirtesu_auv_localization.launch.py` starts:

| Node | Package | Purpose |
| --- | --- | --- |
| `world_ned_to_world_enu` | `tf2_ros` | Publishes the static transform between `world_ned` and `world_enu`. |
| `imu_ned_to_enu` | `sura_localization` | Converts the raw Stonefish IMU into ENU before it reaches `robot_localization`. |
| `pressure_to_pose` | `sura_localization` | Converts fluid pressure into a depth pose in `world_enu`. |
| `navsat_transform_node` | `robot_localization` | Converts GPS fixes into ENU odometry. |
| `ekf_filter_node` | `robot_localization` | Fuses GPS, pressure, DVL, and IMU in `world_enu`. |
| `gps_enu_to_ned_odometry` | `sura_localization` | Converts the EKF output from ENU to the public NED odometry. |

`cirtesub_stonefish` should still launch:

| Node | Expected output |
| --- | --- |
| `dvl_to_twist.py` | `/cirtesub/sensors/dvl/twist` |

`pressure_to_pose` is intentionally launched by `sura_localization`, not by `cirtesub_stonefish`, because its output frame must match the localization frame convention.

## Expected Inputs

| Topic | Type | Expected frame/convention | Used by |
| --- | --- | --- | --- |
| `/cirtesub/sensors/gps` | `sensor_msgs/NavSatFix` | `cirtesub/GPS` | `navsat_transform_node` |
| `/cirtesub/sensors/imu` | `sensor_msgs/Imu` | Stonefish/NED convention | `ned_to_enu_imu` |
| `/cirtesub/sensors/pressure` | `sensor_msgs/FluidPressure` | Pressure sensor frame | `pressure_to_pose` |
| `/cirtesub/sensors/dvl/twist` | `geometry_msgs/TwistWithCovarianceStamped` | `cirtesub/DVL` | `ekf_node` |

## Internal Topics

| Topic | Type | Frame | Producer | Consumer |
| --- | --- | --- | --- | --- |
| `/cirtesub/sensors/imu_enu` | `sensor_msgs/Imu` | `cirtesub/IMU` | `ned_to_enu_imu` | `ekf_node`, `navsat_transform_node` |
| `/cirtesub/sensors/pressure/pose` | `geometry_msgs/PoseWithCovarianceStamped` | `world_enu` | `pressure_to_pose` | `ekf_node` |
| `/cirtesub/sensors/gps/odometry` | `nav_msgs/Odometry` | `world_enu` | `navsat_transform_node` | `ekf_node` |
| `/cirtesub/localization/odometry_enu` | `nav_msgs/Odometry` | `world_enu` | `ekf_node` | `enu_to_ned_odometry`, `navsat_transform_node` |

## Public Output

| Topic | Type | Frame | Description |
| --- | --- | --- | --- |
| `/cirtesub/localization/odometry` | `nav_msgs/Odometry` | `world_ned` | Final localization output for the rest of the robot stack. |

## Launch Arguments

| Argument | Default | Description |
| --- | --- | --- |
| `datum_latitude` | required | GPS datum latitude. |
| `datum_longitude` | required | GPS datum longitude. |
| `datum_heading` | required | ENU heading used by `navsat_transform_node`; `0.0` means East. |
| `output_odom_topic` | `/cirtesub/localization/odometry_enu` | Internal ENU EKF output. |
| `odom_frame` | `world_enu` | Odometry frame used by `robot_localization`. |
| `world_frame` | `world_enu` | World frame used by `robot_localization`. |
| `base_link_frame` | `cirtesub/base_link` | Robot base frame. |

Example:

```bash
ros2 launch sura_localization cirtesu_auv_localization.launch.py \
  datum_latitude:=<latitude> \
  datum_longitude:=<longitude> \
  datum_heading:=<heading>
```

## Conversion Rules

### IMU NED to ENU

Used by `imu_ned_to_enu`:

The IMU orientation changes world convention from NED to ENU and sensor-frame
convention from FRD to FLU:

```text
q_out = q_ned_to_enu * q_raw * q_frd_to_flu
```

Angular velocity and linear acceleration are expressed in the IMU frame, so they
only use the sensor-frame conversion:

```text
flu.x =  frd.x
flu.y = -frd.y
flu.z = -frd.z
```

### ENU to NED

Used by `enu_to_ned_odometry`:

```text
ned.x = enu.y
ned.y = enu.x
ned.z = -enu.z
```

The node converts:

- position
- orientation
- linear twist
- angular twist
- pose and twist covariances

## Quick Checks

Check the internal ENU EKF output:

```bash
ros2 topic echo /cirtesub/localization/odometry_enu --once
```

Check the public NED output:

```bash
ros2 topic echo /cirtesub/localization/odometry --once
```

Check the converted IMU:

```bash
ros2 topic echo /cirtesub/sensors/imu_enu --once
```

Check GPS odometry generated by `navsat_transform_node`:

```bash
ros2 topic echo /cirtesub/sensors/gps/odometry --once
```

Check the static world transform:

```bash
ros2 run tf2_ros tf2_echo world_ned world_enu
```

## Notes

- `robot_localization` must only receive data that is consistent with ENU.
- Do not feed the EKF a mix of ENU and NED data.
- `/cirtesub/localization/odometry` is a compatibility output for the NED side of the stack.
- If a new sensor is added to the EKF, first decide whether it is already ENU or needs a converter.
