# Repository Guidelines

## Purpose and current state

This repository documents and diagnoses an iRobot Create 3 robot with a custom payload: Jetson Orin Nano (Ubuntu 22.04, ROS 2 Humble), and Ouster OS0-128 Rev07. `README.md` is the current hardware, frame, connection, and workspace guide. `TODO.md` contains unfinished indoor SLAM and navigation work. `ros2_ws/src/tb4_ouster_autonomy` is a stationary-only ROS package; there is no autonomy launch or automated integration suite yet. Use Mermaid for architecture diagrams in Markdown.

The repository directory and `tb4_ouster_autonomy` package retain legacy names; they do not imply a TurtleBot 4 software or hardware dependency. Use the official [Create 3 docs](https://iroboteducation.github.io/create3_docs/) for the base, [Ouster ROS 2 guide](https://github.com/ouster-lidar/ouster-ros/tree/ros2) for the sensor driver, and [ROS 2 Humble documentation](https://docs.ros.org/en/humble/index.html) for general ROS behavior and package conventions. Use the `humble` branch of Create 3 examples as reference, not an additional bringup stack. Prefer existing ROS packages and ordinary launch/YAML over custom frameworks.

## Files and behavior

- `ros2_ws/src/tb4_ouster_autonomy/tb4_ouster_autonomy/stationary_diagnostics.py` subscribes to ROS telemetry and uses Ouster HTTP GET. It does not publish or configure hardware.
- `ros2_ws/src/tb4_ouster_autonomy/launch/mount_tf.launch.py` publishes only a provisional static TF. Its mount assumptions are in README.
- Retired scripts, captured metadata, and stationary recordings are archived outside this repository at `/home/ivlaborin2/tb4_archive_2026-09-17/`. In particular, the old `test_ouster_os0.py` wrote settings and reinitialized the sensor; do not use it as a status check.
- The sensor last reported firmware 3.2.0, `1024x20`, and 30 cm minimum range; verify again with HTTP GET before relying on these values.

## Development and verification

Build with `/opt/ros/humble/setup.bash` sourced, from `ros2_ws/`, using `colcon build --symlink-install`. On the Jetson, read-only checks include `curl -fsS http://169.254.97.211/api/v1/sensor/config`, `ip -brief addr`, `ros2 topic list -t`, and the installed `stationary_diagnostics` executable. Record actual firmware, addresses, TF frames, and command output when reporting hardware results. Syntax checks alone do not validate hardware behavior.

For new Python code, use four-space indentation, `snake_case` functions/variables, `UPPER_SNAKE_CASE` constants, explicit units and frame names, and small focused modules. Keep each node's responsibility and data ownership clear. Prefer event/action callbacks for transitions; use timers only for measured control, freshness, and costmap needs. Do not copy the old robot's motor controller: Create 3 already controls its wheels.

## Robot and sensor change boundary

The current phase permits read-only host, ROS-topic, and Ouster HTTP GET inspection plus documentation edits. Do not publish `/cmd_vel`, send motion or undock goals, activate autonomous launch, write sensor settings, reinitialize the sensor, or flash firmware unless the user explicitly asks for that operation. In particular, no wheel motion is authorized by a request to inspect or plan. Retain Create 3 safety reflexes and require one final velocity writer when implementing autonomy. The user will choose and apply Ouster settings after receiving a reason for each recommendation.

## Git changes

Preserve existing user changes and untracked files. Use a short imperative commit subject if a commit is requested. For pull requests, state the change, verification commands/results, sensor model/firmware/network observations when relevant, and hardware limitations. Do not claim movement or sensor-stream validation from a read-only metadata query.
