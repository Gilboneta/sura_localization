# sura_localization

`sura_localization` is a ROS 2 package providing `ros2_control` plugins for AUV attitude estimation and localization using Madgwick and EKF controllers.

Initial controller skeleton:

- `MadgwickController`: placeholder for the Madgwick attitude estimator.
- `EkfController`: placeholder for the EKF localization estimator.

The controller implementations are exported as `controller_interface` plugins.
The algorithms, topics, parameters and hardware interfaces will be added during
development.
