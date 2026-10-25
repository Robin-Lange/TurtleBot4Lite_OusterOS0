# Offboard AGX Orin networking

The AGX is optional. Local sensing, safety, SLAM, and navigation run on the robot's Jetson. This repository currently contains no VLA policy, model deployment, bounded remote-command adapter, or `/vla/cmd_vel` heartbeat monitor.

## Private link

Connect the AGX to the private router using a dedicated USB Ethernet adapter. Keep institutional Internet on its existing interface. Identify the private interface explicitly:

```bash
nmcli device status
./scripts/setup_agx_orin_network.sh <private_interface>
```

The helper creates or updates `robot-link` at `192.168.0.10/24`, with IPv4 and IPv6 `never-default` enabled. Verify the subnet does not overlap an existing LAN; change the script and DDS profiles together if it does. No script can guarantee preservation of a route when subnets overlap or another connection manager changes them.

Reserve the robot's Wi-Fi address in the router. The supplied robot DDS whitelist includes `192.168.0.100` and `.151`; replace these with the robot's actual address. Review `config/fastdds_agx_orin.xml` and `config/fastdds_create3.xml` before sourcing:

```bash
source setup_env.sh
```

These XML files configure **peer discovery with multicast and initial unicast peers**, not a Fast DDS Discovery Server. Do not launch a Discovery Server and expect these profiles to become clients automatically. `maxMessageSize=8192` is a Fast DDS transport setting, not the USB/Ethernet MTU. Kernel MTU, socket buffers, and fragmentation are separate settings.

Use diagnostics to measure rates and timestamp age. Wi-Fi throughput, latency, camera JPEG size, and inference performance must be measured on this network. They are not guaranteed by the router or compute model.

## Image transport

If an image-transport plugin is installed and the wrapper advertises it, inspect `/camera/camera/color/image_raw/compressed`. Avoid subscribing to full raw point clouds across Wi-Fi unless needed. Uncompressed 640×480 RGB at 30 Hz has 27.65 MB/s of pixel payload before protocol overhead; whether that is acceptable depends on the measured link.

The Orin Nano does not have NVENC. NVIDIA documents software H.264 encoding for it; do not use the former hardware-NVENC example. See [NVIDIA's Orin Nano software encoding guide](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/Multimedia/SoftwareEncodeInOrinNano.html).

## Future Zenoh or learned control

The local ROS 2/Fast DDS graph remains the supported path. Zenoh's ROS 2 DDS bridge can carry selected ROS traffic over an offboard link, but it adds configuration, type discovery, latency, and failure cases. Native `rmw_zenoh` is a different middleware implementation; changing the RMW does not make the embedded Create 3 DDS endpoint understand it.

Before adding a bridge, measure direct DDS discovery reliability, packet loss, end-to-end command age, and bandwidth. If a bridge is justified, use one explicit boundary, an allowlist, and keep local safety independent of the AGX. A remote policy should submit high-level goals or use a separately validated source adapter with source timestamps, sequence numbers, expiry, authority, and heartbeat. It must never publish directly to `/cmd_vel` or masquerade as an existing local source.

References:

- [Eclipse Zenoh ROS 2 DDS bridge](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds)
- [ROS 2 rmw_zenoh](https://github.com/ros2/rmw_zenoh)
- [Create 3 middleware configuration](https://iroboteducation.github.io/create3_docs/setup/xml-config/)
