# Sensor and autonomy audit

Reviewed source: `feature/realsense-rplidar-navigation` at `2650ae7b21ee9617734f4e81114923477fbda554`. Changes are isolated on `fix/ecs-sensor-integration`. Review date: 2026-10-25. Public vendor documentation and upstream source are the technical basis; no physical robot was connected to this review environment.

## Findings and implemented changes

| Finding in the reviewed branch | Consequence | Change |
| --- | --- | --- |
| Both Nav2 velocity smoother and explorer publish directly to `/cmd_vel`; moving modes do not launch the existing safety gate | Authority, telemetry watchdogs, and Collision Monitor are bypassed | Shared Nav2 composition publishes `/nav2/cmd_vel`; explorer publishes `/explorer/cmd_vel`; every moving mode launches the gate and Collision Monitor |
| Gate and Collision Monitor reference `/ouster/scan` or `/ouster/cloud_filtered` | Unused safety path observes the wrong sensors | Use `/scan` and `/camera/cloud_filtered`; monitor both |
| Camera crop assumes source Z is vertical | Optical Z is distance forward; distant obstacles can be removed while floor points survive | Transform source cloud into `base_link` at acquisition time, then apply pure height/range/self filters |
| Bumper node broadcasts an identity odometry transform before receiving data and repeatedly re-stamps its last pose with current time | Competing TF authority and stale poses made to look current | Remove TF broadcasting entirely; require the base's real dynamic TF |
| Bumper observations are re-published in `base_link` after the robot moves | Contact obstacle moves with the robot | Anchor contact coordinates in `odom` at contact time and bound event storage |
| Expiring a synthetic cloud does not remove previously marked Nav2 cells | Claimed contact decay does not actually guarantee costmap decay | Keep contact cloud diagnostic-only; gate latches actual contact. Remove synthetic contact marking from local costmap |
| Bumper recovery outputs motion before deadman and telemetry checks, with no verified rear clearance | Recovery can reverse using stale sensors or missing operator authority | Replace reflex backup with a latched stop and explicit resume; preserve Create 3's native reflexes |
| Freshness checks inspect receipt time only; dock and hazard freshness are not checked | Replayed old data or missing base-state streams can appear healthy | Check monotonic receipt age and ROS acquisition age independently for all required streams |
| Non-finite velocity reaches min/max clamping | NaN may become a maximum command rather than a stop | Reject non-finite components before selecting/clamping commands |
| One planar scan contributes one height bin but local VoxelLayer uses `mark_threshold=1` | A planar hit alone may fail to mark an obstacle | Set `mark_threshold=0`; retain D455 observations and cover 1.6 m in the voxel grid |
| Progress checker uses the newer plural key | Humble ignores that selection | Use Humble's `progress_checker_plugin` |
| Explorer compares frontier coordinates in `map` with robot pose in `odom`; ignores map-origin yaw | Incorrect distances and goal headings after SLAM correction or rotated maps | Obtain robot pose from `map→base_link` and rotate grid coordinates by the origin quaternion |
| Explorer proceeds to spin after undock failure/timeout and ignores late action acceptance | Motion can start after a failed action; an accepted old goal can outlive a stop | Require successful undock result and confirmed undocked state; correlate goals by generation and cancel stale accepted requests |
| Launch files contain an absolute home directory and force one DDS profile | Breaks other machines and overwrites offboard configuration | Use user-owned map defaults and respect caller middleware configuration |
| Scripts delete old maps before a new map exists and use broad `pkill` patterns | Failed startup can lose the current map and terminate unrelated work | Preserve timestamped backups and stop only the owned launch process group |
| CI only runs for `main`, and rosdep uses `-r` | Feature branch is untested; dependency errors may be hidden | Run every branch/PR, fail on unresolved dependencies, and exercise final velocity output in Humble |
| `ament_python` is declared as a package dependency | Strict rosdep cannot resolve it | Keep `ament_python` as build type and declare `python3-setuptools` as build dependency |
| README describes sub-millimeter mapping, guaranteed latency/inference, hardware NVENC on Nano, and a VLA watchdog that is absent | Setup guide overstates accuracy and implemented behavior | Remove unsupported claims; label offboard learned control as future work. Nano software encoding follows NVIDIA guidance |

## Architecture decision

The runtime uses data-only components and pure systems, with ROS adapters at the edges. Entities are the robot's authority state and keyed command sources/sensor streams. The safety system does not import ROS, open a session, or publish data. The ROS adapter replaces components and publishes the resulting decision. Point transforms, cropping, contacts, frontier extraction, and map serialization are independently testable functions.

This follows the boundaries in the user's [distributed collaborative autonomy platform](https://github.com/Robin-Lange/distributed-collaborative-autonomy-platform/blob/main/OVERVIEW.md): middleware handles transport; policy owns authority and state freshness; platform adapters handle execution. ECS does not require rewriting the ROS graph in Rust or Bevy.

Fast DDS remains the local transport because Create 3 exposes DDS endpoints. No benchmark demonstrated an advantage from a Zenoh bridge on this single robot. Adding an offboard bridge can be evaluated later without changing the pure systems. Direct DDS compatibility and native Zenoh communication are separate contracts; an RMW environment variable alone is insufficient.

## Physical connections and facts that code cannot verify

| Item | Public documentation / existing configuration | Required verification on the robot |
| --- | --- | --- |
| Create 3 adapter data | Adapter USB-C is USB 2.0 **host**; connect it to the Jetson's device/gadget-capable port; toggle toward USB | Cable data capability, correct port/role, gadget interfaces and actual IPs |
| D455 | Connect to a Jetson USB host port with appropriate USB 3 cabling, not through Create 3's USB 2.0 gadget path | Enumerated camera and negotiated speed (`lsusb -t`), camera serial/firmware and supported profiles |
| RPLIDAR S2 | Slamtec's ROS 2 example uses 1,000,000 baud and DenseBoost | Actual S2 variant, adapter serial, `/dev/rplidar`, scan direction, scan frame and physical scan-plane clearance |
| Create 3 auxiliary power | Adapter raw battery port: max 2 A at 12–16.8 V; USB-C: 5.13 V at 3 A; shared upstream power budget ~28.8 W at 14.4 V | Total compute/sensor/regulator draw including transients and losses. A software power-mode name is not total input power. Do not back-feed the adapter |
| IMU and wheel encoders | Create 3 already fuses onboard odometry; IMU and wheel telemetry remain available | Fresh `/odom`, `/imu`, dynamic TF and clock agreement. No additional EKF is introduced that would double-count the base fusion |
| Sensor extrinsics | Previous branch uses laser `(-0.010,+0.005,0.118) m`; camera `(-0.050,-0.005,0.176) m` | These are carried forward, not independently measured. Check base reference, camera reference point, orientation, sensor yaw and payload envelope |
| TF authority | Base normally publishes odometry TF; `publish_odom_tfs` is configured on the base at application start | Exactly one authority for each TF child. If a prior workaround disabled base TF, restore the correct base configuration before navigation |
| Time synchronization | Create 3 NTP looks for USB peers `192.168.186.1` and `.3` | Configure chrony, check `chronyc tracking`, and compare acquisition ages. Offline local-stratum configuration must be chosen deliberately |
| Offboard networking | Supplied profiles use loopback, USB address, and example private Wi-Fi addresses | Match whitelist to real addresses, avoid overlapping subnets, verify firewall and loss. `maxMessageSize` is not link MTU |

Mount TF supports `laser_{x,y,z,roll,pitch,yaw}` and `camera_{x,y,z,roll,pitch,yaw}`. The D455 wrapper owns camera internal/optical extrinsics. Do not add a second static transform for its optical frames. If a full TurtleBot 4 bringup is also running, avoid duplicating its lidar drivers, velocity publishers, and TF authorities.

The crop assumes a locally level floor and calibrated `base_link` height. It rejects points below 0.04 m; it is not a slope detector or a validated ground-segmentation model. The D455 has limited depth coverage and may miss low, transparent, reflective, or otherwise poorly observed objects. Contact is a stop condition, not evidence that perception alone provided full coverage.

## Verification and practical limits

Pure tests cover source arbitration, watchdogs for every stream, future/stale stamps, non-finite commands, deadman release, pause persistence, bounds, optical transforms, floor/self/ceiling cropping, contact geometry, frontier extraction, map origin rotation, file output, and launch/configuration contracts. ROS tests cover real message conversion, resume, contact latching, and the final Collision Monitor output for healthy motion, a nearby obstacle, obstacle clearance, and camera dropout.

Use the linked Actions run for the final commit to inspect exact test counts and results. CI is x86-64 Ubuntu 22.04/Humble. It cannot verify ARM camera binaries, USB bandwidth, hardware calibration, physical braking, wiring, payload stability, or battery behavior.

Commission with stationary diagnostics first. Inspect `/safety/status`; a stale-telemetry stop is a commissioning failure to resolve, not a reason to disable the watchdog. Confirm the only external `/cmd_vel` publisher is Collision Monitor using `ros2 topic info /cmd_vel --verbose`. Then perform short, supervised, low-speed tests of sensor disconnects, operator stop, and obstacle response in a clear area with access to the robot's stop button. Native undocking is a separate base action; keep its rear path clear.

## Primary sources

1. [Create 3 ROS 2 interface and odometry TF setting](https://iroboteducation.github.io/create3_docs/api/ros2/), [odometry](https://iroboteducation.github.io/create3_docs/api/odometry/), [hazards](https://iroboteducation.github.io/create3_docs/api/hazards/), [safety](https://iroboteducation.github.io/create3_docs/api/safety/).
2. [Create 3 adapter electrical limits and USB role](https://iroboteducation.github.io/create3_docs/hw/adapter/), [Jetson hookup](https://iroboteducation.github.io/create3_docs/hw/jetson_hookup/), [NTP](https://iroboteducation.github.io/create3_docs/setup/compute-ntp/).
3. [Slamtec ROS 2 S2 launch](https://github.com/Slamtec/rplidar_ros/blob/24cc9b6dea97e045bda1408eaa867ce730fd3fc3/launch/rplidar_s2_launch.py). Upstream branch inspected locally at `24cc9b6dea97e045bda1408eaa867ce730fd3fc3`.
4. [RealSense ROS wrapper: frames, point clouds, parameters](https://github.com/realsenseai/realsense-ros/tree/ros2-master), [ROS REP-103 coordinate conventions](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst).
5. Nav2 Humble source inspected at `3c3db59d6969d8ecee8e68468693d006397f4a0c`: [Collision Monitor configuration](https://github.com/ros-navigation/navigation2/blob/3c3db59d6969d8ecee8e68468693d006397f4a0c/nav2_collision_monitor/params/collision_monitor_params.yaml), [stop comparison](https://github.com/ros-navigation/navigation2/blob/3c3db59d6969d8ecee8e68468693d006397f4a0c/nav2_collision_monitor/src/collision_monitor_node.cpp), [controller parameters](https://github.com/ros-navigation/navigation2/blob/3c3db59d6969d8ecee8e68468693d006397f4a0c/nav2_controller/src/controller_server.cpp), [VoxelLayer](https://github.com/ros-navigation/navigation2/blob/3c3db59d6969d8ecee8e68468693d006397f4a0c/nav2_costmap_2d/plugins/voxel_layer.cpp).
6. [Nav2 Humble-to-Iron migration: max_points versus min_points](https://docs.nav2.org/migration/Humble.html). This branch intentionally uses the Humble schema.
7. [NVIDIA Orin Nano software encoding](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/Multimedia/SoftwareEncodeInOrinNano.html).
8. [Eclipse Zenoh ROS 2 DDS bridge](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds), [ROS 2 native Zenoh RMW](https://github.com/ros2/rmw_zenoh).
