# TROUBLESHOOTING

## Setup / build

| Symptom | Cause / fix |
|---------|-------------|
| `Unsupported Ubuntu <ver>` from a script | Only 22.04 (Humble) and 24.04 (Jazzy) are supported. Or `source /opt/ros/<distro>/setup.bash` before running so `$ROS_DISTRO` is set. |
| `ros-<distro>-motion-capture-tracking` not found by apt/rosdep | Uncomment the `motion_capture_tracking` entry in [`crazyswarm2.repos`](../crazyswarm2.repos) and re-run `./scripts/setup.sh` to build it from source. |
| `vcs: command not found` | `sudo apt install python3-vcstool` (also installed by `install_deps.sh`). |
| `colcon build` fails on `crazyflie_interfaces` | Source ROS 2 first; ensure `ros-dev-tools` / `rosidl` packages are installed (`scripts/install_deps.sh`). |
| Missing submodule headers (`crazyflie_tools`, link-cpp) | `cd src/crazyswarm2 && git submodule update --init --recursive`, then rebuild. |
| `natnet_ros2` build fails at "NatNet SDK not found" / `install_sdk.sh` | The SDK is downloaded at build time via `wget` from cloudfront — needs `wget` (installed by `install_deps.sh`) and internet access. Behind a proxy/offline, download `NatNet_SDK_4.4_ubuntu*.tar` manually and extract into `src/natnet_ros2/deps/NatNetSDK/`. |
| Out-of-memory during build (SBC) | `LOW_MEM=1 ./scripts/build.sh`. |
| Package not found after build | Re-source: `source install/setup.bash`. After editing `.msg`/`.srv` rebuild `crazyflie_interfaces` **and** dependents. |
| `install/` missing / `setup.sh` "built" but no workspace | The build aborted before `colcon`. Fixed in current scripts (ROS `setup.bash` + `set -u`); `git pull` and re-run, or build by hand (README → Setup Step 5 "Building by hand"). |
| `ModuleNotFoundError: No module named 'cffirmware'` (on `backend:=sim`) | The simulator needs the firmware bindings. Run `./scripts/setup_sim_firmware.sh` once (README → Setup Step 4). Not needed for hardware backends. |
| `make bindings_python` fails: `git-lfs: not found` / `arm_add_f32.c: No such file or directory` | The CMSIS submodule uses Git LFS. `sudo apt install -y git-lfs && git lfs install`, then in `~/crazyflie-firmware`: `git submodule update --init --recursive --force`, then re-run `make bindings_python`. (`setup_sim_firmware.sh` now installs git-lfs.) |

> **Mismatched ROS distro.** If you built under one distro but source another at
> runtime, you get typesupport / ABI errors. Keep `$ROS_DISTRO` consistent between
> `build.sh` and your runtime shells.

## Crazyradio / drones

| Symptom | Cause / fix |
|---------|-------------|
| Drone never connects | Check `uri` in `config/crazyflies.yaml`; USB permissions ([README → Setup Step 3](../README.md#step-3--crazyradio-usb-permissions-manual-hardware-only)); each drone needs a unique address. |
| Latency / receive-rate warnings; choppy hold | Radio saturated — lower `firmware_logging` rates and mocap rate ([MOCAP §3](MOCAP.md#3-frequency--bandwidth-tuning-240--50-hz)); use one dongle per 1–2 drones. |
| Won't arm | Check `/cf1/status` supervisor bits (tumbled / locked / can't-fly). Place level, retry. |
| Drifts then emergency-lands | Estimator diverging — usually no `/poses` reaching the server (see mocap below). |

## Mocap pipeline

| Symptom | Cause / fix |
|---------|-------------|
| `/<body>/pose` silent | natnet not receiving frames: check `serverIP`/`clientIP` in the natnet launch match Motive's "Local Interface"; firewall off; multicast address/ports match; **Broadcast Frame** enabled in Motive. |
| natnet node up but no topics | It's a LifecycleNode — it must reach **ACTIVE**. Launch with `activate:=true` or transition it manually. |
| `/<body>/pose` flows, `/poses` silent | `DRONES` in `pose_bridge.py` doesn't match the rigid-body names. |
| `/poses` flows, drone still drifts | Rate/QoS mismatch, or marker geometry in `config/motion_capture.yaml` doesn't match the physical layout; orientation flips → make marker patterns asymmetric. |
| Cameras flash white with empty volume | Seeing stray reflections/noise — re-mask in Motive calibration or remove the reflector ([MOCAP §1](MOCAP.md#1-camera-calibration-brief)). |
| Wrong orientation / axes | Motive ground plane not **Z-up**; recalibrate the ground plane. |

## Quick diagnostics

```bash
ros2 topic hz /cf1/pose       # natnet output
ros2 topic hz /poses          # bridge output (~50 Hz expected)
ros2 topic echo /cf1/status --once          # battery / supervisor / link
ros2 run tf2_tools view_frames               # TF tree
ros2 node list ; ros2 topic list
```
