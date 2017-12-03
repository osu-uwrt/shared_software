### Shared Software


__The Underwater Robotics Team__  
_Vehicle Agnostic Packages & Utilities_

---

`shared_hardware` -- Common hardware interfaces for The OSU Underwater Robotics Team
- `scripts` -- Common scripts for setting up various peripherals.
- `imu_state.cpp` -- Node for transforming IMU state messages.
- `udev` -- Contains UDEV rules for our peripherals.

`shared_utils` -- Common custom utilities for making ROS a little easier to deal with.
- `extract_video.cpp` -- Node to extract an AVI from a series of rosbag image messages.
