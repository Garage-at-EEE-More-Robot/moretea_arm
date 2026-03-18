# moretea_arm

## Overview
`moretea_arm` is a ROS 2 action-based arm controller for Feetech STS servos. It executes YAML-defined arm/gripper trajectories through a `play_trajectory` action server.

## Repository Structure
- `action/PlayTrajectory.action` – action interface definition
- `src/play_trajectory_action_node.cpp` – ROS 2 action server (`play_trajectory_server`)
- `src/servo_backend.cpp` – trajectory and gripper execution backend
- `config/*.yaml` – trajectory presets (`rest`, `raise`, `opengripper`, `closegripper`)
- `include/SCServo_Linux/` – vendor servo SDK and static library source