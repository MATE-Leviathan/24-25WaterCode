# Repository Guidance

This repository contains Leviathan ROS 2 robot code. The ROS workspace is
`fishROS_ws`, and packages live under `fishROS_ws/src`.

- Build ROS packages from `fishROS_ws` with `colcon build`.
- Source ROS 2 Humble before building or launching.
- Keep hardware device paths configurable through launch arguments.
- Do not probe Jetson-only serial, camera, or I2C hardware from a non-Jetson
  development machine.
- Existing bag directories and untracked hardware experiments may be local data;
  do not delete or rewrite them unless explicitly asked.
