# TROUBLESHOOTING

## Setup / build

| Symptom | Cause / fix |
|---------|-------------|
| `Unsupported Ubuntu <ver>` from a script | Only 22.04 (Humble) and 24.04 (Jazzy) are supported. Or `source /opt/ros/<distro>/setup.bash` before running so `$ROS_DISTRO` is set. |
| `ros-<distro>-motion-capture-tracking` not found by apt/rosdep | Clone it into `src/` and rebuild: `git clone --branch ros2 --recursive https://github.com/IMRCLab/motion_capture_tracking.git src/motion_capture_tracking`, then `./scripts/build.sh`. |
| `vcs: command not found` | `sudo apt install python3-vcstool` (also installed by `install_deps.sh`). |
| `colcon build` fails on `crazyflie_interfaces` | Source ROS 2 first; ensure `ros-dev-tools` / `rosidl` packages are installed (`scripts/install_deps.sh`). |
| Missing submodule headers (`crazyflie_tools`, link-cpp) | `cd src/crazyswarm2 && git submodule update --init --recursive`, then rebuild. |
| `natnet_ros2` build fails at "NatNet SDK not found" / `install_sdk.sh` | The SDK is downloaded at build time via `wget` from cloudfront — needs `wget` (installed by `install_deps.sh`) and internet access. Behind a proxy/offline, download `NatNet_SDK_4.4_ubuntu*.tar` manually and extract into `src/natnet_ros2/deps/NatNetSDK/`. |
| Out-of-memory during build (SBC) | `LOW_MEM=1 ./scripts/build.sh`. |
| Package not found after build | Re-source: `source install/setup.bash`. After editing `.msg`/`.srv` rebuild `crazyflie_interfaces` **and** dependents. |
| `install/` missing / `setup.sh` "built" but no workspace | The build aborted before `colcon`. Fixed in current scripts (ROS `setup.bash` + `set -u`); `git pull` and re-run, or build by hand (README → Setup Step 5 "Building by hand"). |
| `ModuleNotFoundError: No module named 'cffirmware'` (on `backend:=sim`) | The simulator needs the firmware bindings. Run `./scripts/setup_sim_firmware.sh` once (README → Setup Step 4). Not needed for hardware backends. |
| `make bindings_python` fails: `git-lfs: not found` / `arm_add_f32.c: No such file or directory` | The CMSIS submodule uses Git LFS. `sudo apt install -y git-lfs && git lfs install`, then in `~/crazyflie-firmware`: `git submodule update --init --recursive --force`, then re-run `make bindings_python`. (`setup_sim_firmware.sh` now installs git-lfs.) |
| `No module named '_cffirmware'` (sim, even though `import cffirmware` worked in `build/`) | The `setup.py` egg didn't bundle the compiled `.so`. Put the build dir on PYTHONPATH instead: `echo 'export PYTHONPATH=$HOME/crazyflie-firmware/build:$PYTHONPATH' >> ~/.bashrc && source ~/.bashrc`. Verify from your home dir, not `build/`. |

> **Mismatched ROS distro.** If you built under one distro but source another at
> runtime, you get typesupport / ABI errors. Keep `$ROS_DISTRO` consistent between
> `build.sh` and your runtime shells.

## Crazyradio / drones

| Symptom | Cause / fix |
|---------|-------------|
| Drone never connects | Check `uri` in `src/crazyswarm2/crazyflie/config/crazyflies.yaml`; USB permissions ([README → Setup Step 3](../README.md#step-3--crazyradio-usb-permissions-manual-hardware-only)); each drone needs a unique address. |
| Latency / receive-rate warnings; choppy hold | Radio saturated — lower `firmware_logging` rates and mocap rate ([MOCAP §3](MOCAP.md#3-frequency--bandwidth-tuning-240--50-hz)); use one dongle per 1–2 drones. |
| Won't arm | Check `/cf1/status` supervisor bits (tumbled / locked / can't-fly). Place level, retry. |
| Drifts then emergency-lands | Estimator diverging — usually no `/poses` reaching the server (see mocap below). |

## Mocap pipeline

| Symptom | Cause / fix |
|---------|-------------|
| Mocap suddenly "died": `/poses` silent, `ros2 topic info /poses` shows **0 publishers**, mocap node absent from `ros2 node list` | **CONFIRMED on this rig:** a leftover process bound to UDP 1511 (e.g. a stray debug listener). With two `SO_REUSEPORT` sockets on 1511 the kernel delivers each NatNet datagram to only ONE socket — the mocap node starves and hangs **silently** inside libmotioncapture `connect()` (no crash, no publisher). Diagnose: `ss -uanp \| grep :1511` — two sockets = the bug. Fix: kill the orphan, relaunch. Note: ping to the Motive PC proves nothing (unicast works while multicast is starved). |
| Changed the Motive transmission type but the node still gets nothing | Transmission type is read **once at connect**. Fully restart the launch after changing it. The apt `motion_capture_tracking` requires **Multicast** ([MOCAP §4](MOCAP.md#4-multicast-vs-unicast-primer)). |
| Mocap node won't die on Ctrl-C; next launch's connect aborts with SIGABRT | A frozen mocap node ignores SIGINT (blocked in `recv`); the launch escalates to SIGKILL, but check `pgrep -f motion_capture_tracking` for leftovers before relaunching — a leftover can make the next connect abort. |
| Yaw offset / fly-away despite perfect position; preflight GUI shows the red misalignment banner or large dashed `err.yaw` | Motive rigid body was created rotated. Recreate it with the drone's forward axis on global +X ([MOCAP §2](MOCAP.md#2-defining-rigid-bodies)). Position error stays ~1 mm regardless (mocap position is force-fused) — don't let it reassure you. Go/no-go: ±5° fly; 5–15° fix first; >20° do not fly. |
| `/<body>/pose` silent (natnet_ros2 path) | natnet not receiving frames: check `serverIP`/`clientIP` in the natnet launch match Motive's "Local Interface"; firewall off; multicast address/ports match; **Broadcast Frame** enabled in Motive. |
| natnet node up but no topics | It's a LifecycleNode — it must reach **ACTIVE**. Launch with `activate:=true` or transition it manually. |
| `/<body>/pose` flows, `/poses` silent | `DRONES` in `pose_bridge.py` doesn't match the rigid-body names. |
| `/poses` flows, drone still drifts | Rate/QoS mismatch, or marker geometry in `src/crazyswarm2/crazyflie/config/motion_capture.yaml` doesn't match the physical layout; orientation flips → make marker patterns asymmetric. |
| Cameras flash white with empty volume | Seeing stray reflections/noise — re-mask in Motive calibration or remove the reflector ([MOCAP §1](MOCAP.md#1-camera-calibration-brief)). |
| Wrong orientation / axes | Motive ground plane not **Z-up**; recalibrate the ground plane. |

## Preflight GUI

| Symptom | Cause / fix |
|---------|-------------|
| **GUI window never appears** (server/RViz run fine, no obvious error) | Almost always matplotlib on this machine. Run `/usr/bin/python3 -c "import matplotlib.pyplot as p; print(p.get_backend())"` and read the result: **NumPy 1.x-vs-2.x traceback** → user-site NumPy 2.x poisons the apt matplotlib; `pip3 install --user "numpy<2"`. **`ModuleNotFoundError: matplotlib`** → `sudo apt install python3-matplotlib python3-tk`. **Prints `Agg`** → no GUI toolkit; `sudo apt install python3-tk` (with Agg, `plt.show()` returns instantly and the node *exits cleanly* — launch even logs "finished cleanly"). **Prints `TkAgg`/`QtAgg` cleanly** → run the script directly to see the real error: `python3 src/crazyswarm2/crazyflie/scripts/preflight_kalman_plotter.py`. See [README → Setup Step 2](../README.md#step-2--clone-and-run-the-one-shot-setup) for the full extra-deps install block. |
| GUI absent **and** no `preflight` in the launch file (`grep -c preflight src/crazyswarm2/crazyflie/launch/launch.py` prints 0) | Clone predates the preflight GUI (added 2026-07-09). `git pull`, rebuild (`./scripts/build.sh`), re-source. |
| *Nothing* opens (no RViz either); launch exits with `package 'X' not found` | A default-on node's package is missing (`joy`, `rviz2`, `foxglove_bridge`, `motion_capture_tracking`) — one missing package aborts the **entire** launch. Install it (`sudo apt install ros-$ROS_DISTRO-<pkg>`) or disable the node (`teleop:=False`, `rviz:=false`, `foxglove:=False`, `mocap:=False`). |
| Plotter works under `ros2 launch` but crashes with a NumPy error when run manually | The launch file shields the node with `PYTHONNOUSERSITE=1`; your manual shell doesn't. Run it as `PYTHONNOUSERSITE=1 python3 …` or fix the root cause with `pip3 install --user "numpy<2"`. |
| A drone doesn't appear in the GUI | It has `enabled: false` in `crazyflies.yaml`, or the GUI was pointed at a different yaml with `--config`. |
| GUI shows every drone (incl. disabled) plus a yaml warning | The GUI couldn't read `crazyflies.yaml` — it then accepts everything. Fix the path / pass `--config <path>`. |
| Mocap Hz trace in the connectivity panel decays to 0 | Mocap died — see the Mocap pipeline table above (UDP 1511 orphan is the confirmed cause on this rig). |
| kalman telemetry panel empty | The `kalman_preflight` custom log topic was removed/renamed in `crazyflies.yaml` — restore it ([RUNNING §F](RUNNING.md#f-enabling-extra-telemetry-logging-custom-topics)). |

## Color LED

| Symptom | Cause / fix |
|---------|-------------|
| Deck never lights green on connect | Server log says `No Color LED deck detected … skipping LED set` — deck missing/loose, or the firmware doesn't expose the `colorLedBot` params. |
| `led.sh` / `color_led` warn that no `…colorLedBot.wrgb8888` params were found | `firmware_params: query_all_values_on_connect` must be `True` in `config/server.yaml` (default in this repo — don't set it back to `False`); also confirm the drone actually connected and carries the deck. |
| `color_led` exits with `/crazyflie_server/set_parameters not available after 10s` | The server isn't running (`ros2 launch crazyflie launch.py`) or your shell is on a different `ROS_DOMAIN_ID`. |
| Color command "succeeds" but nothing changes under `backend:=sim` | Expected — LED control is hardware-only; the sim backend declares no firmware params. |
| `color_led_cflib.py` can't open the radio link | The crazyflie_server is still running and owns the dongle — stop the launch first. Also check the in-file `URI` (marked `# EDIT ME`) matches your drone. |

## Quick diagnostics

```bash
ros2 topic hz /cf1/pose       # onboard pose (firmware logging)
ros2 topic hz /poses          # mocap output (~50 Hz expected)
ros2 topic info /poses        # publisher count — 0 = mocap node dead/starved
ss -uanp | grep :1511         # two sockets on 1511 = orphan starving the mocap node
pgrep -f motion_capture_tracking   # leftover frozen mocap process?
ros2 topic echo /cf1/status --once          # battery / supervisor / link
ros2 run tf2_tools view_frames               # TF tree
ros2 node list ; ros2 topic list
```
